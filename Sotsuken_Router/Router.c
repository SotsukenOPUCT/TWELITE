
/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <string.h>
#include <jendefs.h>
#include <AppHardwareApi.h>
#include "AppQueueApi_ToCoNet.h"
#include "config.h"
#include "ccitt8.h"
#include "Interrupt.h"
#include "Router.h"
#include "Version.h"
#include "utils.h"
#include "flash.h"
#include "common.h"

// Serial options
#include "serial.h"
#include "fprintf.h"
#include "sprintf.h"
#include "Interactive.h"
#include "sercmd_gen.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/

// Select Modules (define befor include "ToCoNet.h")
#define ToCoNet_USE_MOD_NWK_LAYERTREE
#define ToCoNet_USE_MOD_NBSCAN
#define ToCoNet_USE_MOD_NBSCAN_SLAVE
#define ToCoNet_USE_MOD_NWK_MESSAGE_POOL
#define ToCoNet_USE_MOD_DUPCHK

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"
#include "app_event.h"

#define TOCONET_DEBUG_LEVEL 0

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

// アプリケーションごとの振る舞いを記述するための関数テーブル
tsCbHandler *psCbHandler = NULL;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vInitHardware(int f_warm_start);
static void vSerialInit( uint32 u32Baud, tsUartOpt *pUartOpt );

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

// Local data used by the tag during operation
tsAppData_Ro sAppData;
tsFILE sSerStream;
tsSerialPortSetup sSerPort;

// Timer object
tsTimerContext sTimerApp;
static bool_t bVwd = FALSE;

tsToCoNet_NwkLyTr_Context *pc;

// Layer 設定
int layer = 2;

// Sleep 時間設定(ms)
int sleep = 57000;



/****************************************************************************/
/***        Local Definitions                                           ***/
/****************************************************************************/

#define DO1   18          // デジタル出力 1
#define DO2   19          // デジタル出力 2
#define DO3   4          // デジタル出力 3
#define DO4   9          // デジタル出力 4

/****************************************************************************/
/***        ユーザー定義関数                                              ***/
/****************************************************************************/

/*
vProcessEvCore
ユーザーイベント関数
cold start内のToCoNet_Event_Register_State_Machine(vProcessEvCore)で定義
*/

static tsToCoNet_NwkLyTr_Config sNwkLayerTreeConfig;
static tsToCoNet_Nwk_Context* pContextNwk;

static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
switch (pEv->eState) {
	// アイドル状態
	case E_STATE_IDLE:

		if (eEvent == E_EVENT_START_UP) {
			if (u32evarg & EVARG_START_UP_WAKEUP_RAMHOLD_MASK) {
				
				A_PRINTF(LB "Wake Up!");
				
				// 再送回数の指定
				sAppData.sNwkLayerTreeConfig.u8TxRetryCtUp = 3;
			
				//レイヤー数の設定
				sAppData.sNwkLayerTreeConfig.u8Layer = layer; 
				//	NBビーコン方式
				sNwkLayerTreeConfig.u8Second_To_Beacon = 3; // set NB beacon interval
				sNwkLayerTreeConfig.u8StartOpt = TOCONET_MOD_LAYERTREE_STARTOPT_NB_BEACON; // set NB beacon mode
			
				//中継器として始動
				sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_ROUTER;
				sAppData.sNwkLayerTreeConfig.u16TxMaxDelayUp_ms = 100;
				sAppData.pContextNwk = ToCoNet_NwkLyTr_psConfig(&sAppData.sNwkLayerTreeConfig);
				ToCoNet_Nwk_bInit(sAppData.pContextNwk);
				ToCoNet_Nwk_bStart(sAppData.pContextNwk);
			}else{
				// 再送回数の指定
				sAppData.sNwkLayerTreeConfig.u8TxRetryCtUp = 6;
			
				//レイヤー数の設定
				sAppData.sNwkLayerTreeConfig.u8Layer = layer; 
				//	NBビーコン方式
				sNwkLayerTreeConfig.u8Second_To_Beacon = 3; // set NB beacon interval
				sNwkLayerTreeConfig.u8StartOpt = TOCONET_MOD_LAYERTREE_STARTOPT_NB_BEACON; // set NB beacon mode
				sNwkLayerTreeConfig.u8LayerOptions = 1;
			
				//中継器として始動
				sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_ROUTER;
				sAppData.pContextNwk = ToCoNet_NwkLyTr_psConfig(&sAppData.sNwkLayerTreeConfig);
				A_PRINTF(LB "Start Router (Layer %d)", sAppData.sNwkLayerTreeConfig.u8Layer);
				ToCoNet_Nwk_bInit(sAppData.pContextNwk);
				ToCoNet_Nwk_bStart(sAppData.pContextNwk);
				}
				ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
		}
		break;

	// 稼働状態
	case E_STATE_RUNNING:

		if(eEvent == E_EVENT_TX){
			// set UART Rx port as interrupt source
			vAHI_DioSetDirection(u32DioPortWakeUp, 0); // set as input

			// set DIO wakeup source
			(void)u32AHI_DioInterruptStatus(); // clear interrupt register
			vAHI_DioWakeEnable(u32DioPortWakeUp, 0); // enable ports
			vAHI_DioWakeEdge(u32DioPortWakeUp, 0); // set edge (rising)
		}

		static bool_t count_RX = 0;
		static bool_t count_TX = 0;

		if(eEvent == E_ORDER_KICK_RX){
			// データ受信時に DO4 を Hi に
        	count_RX = 100;
        	vPortSetHi(DO4);
		}

		if(eEvent == E_ORDER_KICK_TX){
			// データ送信時に DO4 を Hi に
        	count_TX = 101;
        	vPortSetHi(DO4);

			A_PRINTF(LB "Sleep %d s", sleep / 1000);
		}

		// 4ms 周期のシステムタイマ通知
    	if (eEvent == E_EVENT_TICK_TIMER) {
        	if (count_RX > 0) {
            	count_RX--;
        	} else {
            	// DO4 が Hi なら Lo に
            	if (!bPortRead(DO4)) {
                	vPortSetLo(DO4);
            	}
			}

       		if (count_TX > 1) {
            	count_TX--;
        	} else {
				if(count_TX == 1){
				count_TX--;
            	// DO4 が Hi なら Lo に
            	if (!bPortRead(DO4)) {
                	vPortSetLo(DO4);
            	}
				ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP); 
				}
        	}		
		}

		break;
	
	// Sleep への移行状態
	case E_STATE_APP_SLEEP:

		// スリープ状態に移行
		ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, sleep, FALSE, FALSE); 
		break;

	default:
		break;
	}
	return;
}

//  シリアルポートからの入力を処理

void input_from_keyboard(void){
    // handle UART command
	while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
		int16 i16Char;

		i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

		vfPrintf(&sSerStream, "\n\r# [%c] --> ", i16Char);
	    SERIAL_vFlush(sSerStream.u8Device);

		switch(i16Char) {


		case 'i': //データ表示
			_C {
				A_PRINTF(LB "* App ID:%08x Long Addr:%08x Short Addr %04x LID %02d",
				sToCoNet_AppContext.u32AppId, ToCoNet_u32GetSerial(), sToCoNet_AppContext.u16ShortAddress,
				sAppData.sFlash.sData.u8id);
			}
			break;
			
		case 'n': //ネットワーク状態表示
			_C {
				A_PRINTF("Nwk Conf"
			 			LB"Layer = %d"
			 			LB"Sublayer = %d"
			 			LB"State = %d"
						LB"Type = %d"
			 			LB"Access Point = %08X"LB,
						pc->sInfo.u8Layer,
						pc->sInfo.u8LayerSub,
						pc->sInfo.u8State,
						pc->sInfo.u8NwkTypeId,
						pc->u32AddrHigherLayer
					);
			}
			break;	

		case'+': //レイヤー(階層)を一つずつ増やして中継器としてネットワーク開始

			_C {
				sAppData.sNwkLayerTreeConfig.u8Layer++;
				A_PRINTF(LB "Layer Increase");
				sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_ROUTER;
				sAppData.pContextNwk = ToCoNet_NwkLyTr_psConfig(&sAppData.sNwkLayerTreeConfig);
				if (sAppData.pContextNwk) {
					A_PRINTF(LB "Start (layer %d)", sAppData.sNwkLayerTreeConfig.u8Layer);
					ToCoNet_Nwk_bInit(sAppData.pContextNwk);
					ToCoNet_Nwk_bStart(sAppData.pContextNwk);
				} else {
					;
				}	
			}
			break;

		case'-': //レイヤー(階層)を一つずつ減らして中継器としてネットワーク開始
			_C {
				sAppData.sNwkLayerTreeConfig.u8Layer--;
				A_PRINTF(LB "Layer Decrease");
				sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_ROUTER;
				sAppData.pContextNwk = ToCoNet_NwkLyTr_psConfig(&sAppData.sNwkLayerTreeConfig);
				if (sAppData.pContextNwk) {
					A_PRINTF(LB "Start (layer %d)", sAppData.sNwkLayerTreeConfig.u8Layer);
					ToCoNet_Nwk_bInit(sAppData.pContextNwk);
					ToCoNet_Nwk_bStart(sAppData.pContextNwk);
				} else {
					;
				}
			}
			break;

		case 's': //再起動
			_C {
				ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, 100, FALSE, FALSE); 
			}
			break;

		case 't': // パケット送信
			_C {
				tsTxDataApp sTx;
				memset(&sTx, 0, sizeof(sTx)); // 必ず０クリアしてから使う！
				uint8 *q =  sTx.auData;
	
				sTx.u32SrcAddr = ToCoNet_u32GetSerial();
			
				sTx.u32DstAddr = TOCONET_NWK_ADDR_NEIGHBOUR_ABOVE;

				sTx.u8Cmd = 0; // 0..7 の値を取る。パケットの種別を分けたい時に使用する
				sTx.u8Len = q - sTx.auData; // パケットのサイズ
				sTx.u8CbId = sAppData.u16frame_count & 0xFF; // TxEvent で通知される番号、送信先には通知されない
				sTx.u8Seq = sAppData.u16frame_count & 0xFF; // シーケンス番号(送信先に通知される)
				sTx.u8Retry = 0x82; //3回送る
				sTx.u16DelayMin = 1000;

				// SPRINTF でメッセージを作成
				SPRINTF_vRewind();
				vfPrintf(SPRINTF_Stream, "%08x", ToCoNet_u32GetSerial());
				memcpy(sTx.auData, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
				sTx.u8Len = SPRINTF_u16Length();

				if(ToCoNet_Nwk_bTx(sAppData.pContextNwk, &sTx)){
					A_PRINTF(LB"Tx Processing (AddrHigherLayer : %08x)"LB,pc->u32AddrHigherLayer);
				}else{
					A_PRINTF(LB"Tx Error"LB);
				};
			
			}
			break;

		case 'r': //ネットワーク再始動
			_C {
				ToCoNet_Nwk_bInit(sAppData.pContextNwk);
				ToCoNet_Nwk_bStart(sAppData.pContextNwk);
			}
			break;
		}
	}
}

/****************************************************************************/
/***        コールバック関数（記述必須）                                               ***/
/****************************************************************************/

/*
 * cbAppColdStart 
 * 電源が投入された直後、もしくはスリープ復帰時に呼び出し
 * 引数 bAfterAhiInit 
 * AHI(Application Hardwaare Interface)の初期化フラグ（初期化前はFALSE）
 */

void cbAppColdStart(bool_t bAfterAhiInit) {
	if (!bAfterAhiInit) { 
		// before AHI init, very first of code.
		
		// Register modules
		ToCoNet_REG_MOD_ALL();
	} else {
		// disable brown out detect
		vAHI_BrownOutConfigure(0,//0:2.0V 1:2.3V
				FALSE,
				FALSE,
				FALSE,
				FALSE);

		// clear application context
		memset(&sAppData, 0x00, sizeof(sAppData));

		// SPRINTF
		SPRINTF_vInit128();

		// フラッシュメモリからの読み出し
		//   フラッシュからの読み込みが失敗した場合、ID=15 で設定する
		sAppData.bFlashLoaded = Config_bLoad(&sAppData.sFlash);
	

		// TWELITE NET configuration
		sToCoNet_AppContext.u32AppId = sAppData.sFlash.sData.u32appid;;
		sToCoNet_AppContext.u8Channel = sAppData.sFlash.sData.u8ch;
		sToCoNet_AppContext.u32ChMask = sAppData.sFlash.sData.u32chmask;

		sToCoNet_AppContext.bRxOnIdle = TRUE;
		sToCoNet_AppContext.u8TxMacRetry = 1;


		// Register user PRSEV.
		ToCoNet_Event_Register_State_Machine(vProcessEvCore);
		
		// Other Hardware
		vInitHardware(FALSE);
		Interactive_vInit();

		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(TOCONET_DEBUG_LEVEL);
		// START UP MESSAGE
		A_PRINTF(LB "Router:%08x" LB,ToCoNet_u32GetSerial());

	}
}
/**
 * スリープ復帰時の処理
 */
static bool_t bWakeupByButton;

void cbAppWarmStart(bool_t bStart) {
	if (!bStart) {

		bWakeupByButton = FALSE;

	} else {
		// Initialize hardware
		vInitHardware(TRUE);
	}
}

// メイン処理
void cbToCoNet_vMain(void) {
	/* handle serial input */
	input_from_keyboard();
}


/**
 * 子機または中継機を経由したデータを受信する。
 *
 * - アドレスを取り出して、内部のデータベースへ登録（メッセージプール送信用）
 * - UART に指定書式で出力する
 *   - 出力書式\n
 *     ::(受信元ルータまたは親機のアドレス):(シーケンス番号):(送信元アドレス):(LQI)<CR><LF>
 *
 * @param pRx 受信データ構造体
 */

 void cbToCoNet_vRxEvent(tsRxDataApp *pRx) {

	//　受信パケットの詳細を表示
	A_PRINTF(LB"SrcAddr:%08x"
			 LB"DstAddr:%08x"
			 LB"Len:%d"
			 LB"LQI:%d"LB
			 LB"%s"LB,
			 pRx->u32SrcAddr,
			 pRx->u32DstAddr,
			 pRx->u8Len,
			 pRx->u8Lqi,
			 pRx->auData
	);

	// パケットを上位へ転送する

		tsTxDataApp sTx;
		memset(&sTx, 0, sizeof(sTx));
		uint8 *q = sTx.auData;

			sTx.u32SrcAddr = ToCoNet_u32GetSerial();
			
			sTx.u32DstAddr = TOCONET_NWK_ADDR_NEIGHBOUR_ABOVE;

			sTx.u8Cmd = 0; // 0..7 の値を取る。パケットの種別を分けたい時に使用する
			sTx.u8Len = q - sTx.auData; // パケットのサイズ
			sTx.u8CbId = sAppData.u16frame_count & 0xFF; // TxEvent で通知される番号、送信先には通知されない
			sTx.u8Seq = sAppData.u16frame_count & 0xFF; // シーケンス番号(送信先に通知される)
			sTx.u8Retry = 0x82; //3回送る
			sTx.u16DelayMin = 2000;
			
			SPRINTF_vRewind();
			vfPrintf(SPRINTF_Stream, LB"%s -> %08x",pRx->auData,ToCoNet_u32GetSerial());
			memcpy(sTx.auData, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
			sTx.u8Len = SPRINTF_u16Length();

		//　送信処理に関する表示
		if (ToCoNet_Nwk_bTx(sAppData.pContextNwk, &sTx)) {
			A_PRINTF(LB"Tx Processing (AddrHigherLayer : %08x)"LB,pc->u32AddrHigherLayer);
		} else {
			A_PRINTF("Tx Error"LB);
		}

	// DO4 を一定時間 Hi にする
    ToCoNet_Event_Process(E_ORDER_KICK_RX, 0, vProcessEvCore);

}

/**
 * 送信完了時のイベント
 * @param u8CbId
 * @param bStatus
 */

void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
		//　送信処理結果の表示
		A_PRINTF(LB"Tx Result : %s", bStatus ? "OK" : "NG");
		ToCoNet_Event_Process(E_EVENT_TX, u8CbId, vProcessEvCore);
	// DO4 を一定時間 Hi にする
    ToCoNet_Event_Process(E_ORDER_KICK_TX, 0, vProcessEvCore);
	return;
}

/**
 * ネットワークイベント。
 * - E_EVENT_TOCONET_NWK_START\n
 *   ネットワーク開始時のイベントを vProcessEvCore に伝達
 *
 * @param eEvent
 * @param u32arg
 */

void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg) {
	switch (eEvent) {
	case E_EVENT_TOCONET_NWK_START:
		//　上位ノードへの接続を通知
		A_PRINTF(LB"Connect");
		//　接続先ノードのアドレスを表示
		A_PRINTF(LB"Access Point : %08x"LB,pc->u32AddrHigherLayer);

		break;

	case E_EVENT_TOCONET_NWK_DISCONNECT:
		//　接続先ノードが無いことを通知
		A_PRINTF(LB"Disconnect"LB);
		//　ネットワーク再始動を実施
		ToCoNet_Nwk_bInit(sAppData.pContextNwk);
		ToCoNet_Nwk_bStart(sAppData.pContextNwk);

		break;

	case E_EVENT_TOCONET_NWK_SCAN_COMPLETE:
		//　NB Scan 終了を通知
		A_PRINTF(LB"Scan Complete"LB);
		break;
	
	default:
		break;
	}
}


/**
 * ハードウェア割り込みの遅延実行部
 *
 * @param u32DeviceId 割り込み源
 * @param u32ItemBitmap	割り込みパラメータ
 */
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap) {
}

/**
 * ハードウェア割り込み
 * - 処理なし
 *
 * @param u32DeviceId
 * @param u32ItemBitmap
 * @return
 */
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	return FALSE;
}

/****************************************************************************/
/***        初期化                                              ***/
/****************************************************************************/

/**
 * ハードウェアの初期化
 * @param f_warm_start
 */
PRIVATE void vInitHardware(int f_warm_start) {
	// インタラクティブモードの初期化
	Interactive_vInit();

	// 使用ポートの設定
    vPortAsOutput(DO4);
    vPortSetLo(DO4);

	// Serial Port の初期化
	{
		tsUartOpt sUartOpt;
		memset(&sUartOpt, 0, sizeof(tsUartOpt));
		uint32 u32baud = UART_BAUD;

		// BPS ピンが Lo の時は 38400bps
		vPortAsInput(PORT_BAUD);
		if (sAppData.bFlashLoaded && (bPortRead(PORT_BAUD) || IS_APPCONF_OPT_UART_FORCE_SETTINGS() )) {
			u32baud = sAppData.sFlash.sData.u32baud_safe;
			sUartOpt.bHwFlowEnabled = FALSE;
			sUartOpt.bParityEnabled = UART_PARITY_ENABLE;
			sUartOpt.u8ParityType = UART_PARITY_TYPE;
			sUartOpt.u8StopBit = UART_STOPBITS;

			// 設定されている場合は、設定値を採用する (v1.0.3)
			switch(sAppData.sFlash.sData.u8parity & APPCONF_UART_CONF_PARITY_MASK) {
			case 0:
				sUartOpt.bParityEnabled = FALSE;
				break;
			case 1:
				sUartOpt.bParityEnabled = TRUE;
				sUartOpt.u8ParityType = E_AHI_UART_ODD_PARITY;
				break;
			case 2:
				sUartOpt.bParityEnabled = TRUE;
				sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
				break;
			}

			// ストップビット
			if (sAppData.sFlash.sData.u8parity & APPCONF_UART_CONF_STOPBIT_MASK) {
				sUartOpt.u8StopBit = E_AHI_UART_2_STOP_BITS;
			} else {
				sUartOpt.u8StopBit = E_AHI_UART_1_STOP_BIT;
			}

			// 7bitモード
			if (sAppData.sFlash.sData.u8parity & APPCONF_UART_CONF_WORDLEN_MASK) {
				sUartOpt.u8WordLen = 7;
			} else {
				sUartOpt.u8WordLen = 8;
			}

			vSerialInit(u32baud, &sUartOpt);
		} else {
			vSerialInit(u32baud, NULL);
		}

	}

	// 外部ウォッチドッグタイマー用
	vPortSetLo(11);				// 外部のウォッチドッグを有効にする。
	vPortSet_TrueAsLo(9, bVwd);	// VWDをいったんHiにする。
	vPortAsOutput(11);			// DIO11を出力として使用する。
	vPortAsOutput(9);			// DIO9を出力として使用する。
}

/**
 * UART の初期化
 */

void vSerialInit( uint32 u32Baud, tsUartOpt *pUartOpt ) {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer[1024];
	static uint8 au8SerialRxBuffer[512];

	/* Initialise the serial port to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
	sSerPort.u32BaudRate = UART_BAUD;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
	sSerPort.u8SerialPort = UART_PORT;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInit(&sSerPort);

	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART_PORT;
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
