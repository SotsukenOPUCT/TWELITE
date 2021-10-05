
/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <string.h>
#include <AppHardwareApi.h>
#include "AppQueueApi_ToCoNet.h"
#include "config.h"
#include "ccitt8.h"
#include "Interrupt.h"
#include "Parent.h"
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
#include "AddrKeyAry.h"

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

//	アプリケーションごとの振る舞いを記述するための関数テーブル
tsCbHandler *psCbHandler = NULL;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vInitHardware(int f_warm_start);
static void vSerialInit();

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

// Local data used by the tag during operation
tsAppData_Pa sAppData;
tsFILE sSerStream, sSerStream1;
tsSerialPortSetup sSerPort, sSerPort1;

// Timer object
tsTimerContext sTimerApp;
static bool_t bVwd = FALSE;
uint32 rdate = 0x80000000;

tsToCoNet_NwkLyTr_Context *pc;

/****************************************************************************/
/***        Local Definitions                                           ***/
/****************************************************************************/

#define LED   9          // LED

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/*
vProcessEvCore
ユーザーイベント関数
cold start内のToCoNet_Event_Register_State_Machine(vProcessEvCore)で定義
*/
static tsToCoNet_NwkLyTr_Config sNwkLayerTreeConfig;
static tsToCoNet_Nwk_Context *pContextNwk;

static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
switch (pEv->eState) {
	// アイドル状態
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			//レイヤー数の設定
			sNwkLayerTreeConfig.u8Layer = 1;	//0
			//	NBビーコン方式
			sNwkLayerTreeConfig.u8Second_To_Beacon = 3; // set NB beacon interval
			sNwkLayerTreeConfig.u8StartOpt = TOCONET_MOD_LAYERTREE_STARTOPT_NB_BEACON; // set NB beacon mode
			//始動
			sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_ROUTER;
			sAppData.pContextNwk = ToCoNet_NwkLyTr_psConfig(&sAppData.sNwkLayerTreeConfig);
			A_PRINTF(LB "Start Parent (Layer %d)", sAppData.sNwkLayerTreeConfig.u8Layer);
			ToCoNet_Nwk_bInit(sAppData.pContextNwk);
			ToCoNet_Nwk_bStart(sAppData.pContextNwk);
			}
			ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
		break;

	// 稼働状態
	case E_STATE_RUNNING:
		if(eEvent == E_EVENT_NEW_STATE){
			//A_PRINTF(LB "[E_STATE_RUNNING]");
		}
		if(eEvent == E_EVENT_TX){
			//レイヤー数の設定
			sAppData.sNwkLayerTreeConfig.u8Layer = 1;
			//	NBビーコン方式
			sNwkLayerTreeConfig.u8Second_To_Beacon = 3; // set NB beacon interval
			sNwkLayerTreeConfig.u8StartOpt = TOCONET_MOD_LAYERTREE_STARTOPT_NB_BEACON; // set NB beacon mode

			//Routerとして始動
			sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_ROUTER;
			sAppData.pContextNwk = ToCoNet_NwkLyTr_psConfig(&sAppData.sNwkLayerTreeConfig);
			A_PRINTF(LB "Tx layer %d", sAppData.sNwkLayerTreeConfig.u8Layer);
			ToCoNet_Nwk_bInit(sAppData.pContextNwk);
			ToCoNet_Nwk_bStart(sAppData.pContextNwk);
		}
		if(eEvent == E_ORDER_KICK_WAIT){
			A_PRINTF(LB "Start Tx Wait");
			// 待機状態へ移行
			ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_A);
		}
		break;

	static bool_t count = 0;
	// 送信待機
	case E_STATE_APP_WAIT_A:

		if (eEvent == E_EVENT_NEW_STATE) {
			//　待機時間を設定（秒）
			count = 60;
		}

		if (eEvent == E_EVENT_TICK_SECOND) {
        	if (count > 0) {
				count--;
				//　残り時間の表示
				A_PRINTF("\r" "Tx Wait : %d s ",count);
        	} else {
				A_PRINTF(LB);
				ToCoNet_Event_SetState(pEv, E_STATE_APP_TX);
			}
		}
		break;
	// 送信
	case E_STATE_APP_TX:
		if (eEvent == E_EVENT_NEW_STATE) {
			A_PRINTF(LB"[E_STATE_APP_TX]");
			memset(&sNwkLayerTreeConfig, 0, sizeof(sNwkLayerTreeConfig));
			sNwkLayerTreeConfig.u8StartOpt = TOCONET_MOD_LAYERTREE_STARTOPT_NB_BEACON;
			//sAppData.sNwkLayerTreeConfig.u8LayerOptions = 1;
			sAppData.sNwkLayerTreeConfig.u8Layer = 10;
			sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_ROUTER;
			sAppData.pContextNwk = ToCoNet_NwkLyTr_psConfig(&sAppData.sNwkLayerTreeConfig);
			A_PRINTF(LB "Router Layer %d", sAppData.sNwkLayerTreeConfig.u8Layer);
			ToCoNet_Nwk_bInit(sAppData.pContextNwk);
			ToCoNet_Nwk_bStart(sAppData.pContextNwk);
		}
		ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
		break;

	default:
		break;
	}

	// 1秒毎にいろいろテスト
	if (eEvent == E_EVENT_TICK_SECOND) {
        // DO4 の Lo / Hi をトグル
        //bPortRead(LED) ? vPortSetHi(LED) : vPortSetLo(LED);

		//Serial1をテスト
		vfPrintf(&sSerStream1,"Hello\r\n");
    }
}

/*input_from_keyboard
 * シリアルポートからの入力を処理します。
 * - シリアルポートからの入力は uart.c/serial.c により管理される FIFO キューに値が格納されます。
 *   このキューから１バイト値を得るのが SERIAL_i16RxChar() です。
 * Keyboardから"t"を入力するとbroadcastで"Test"を送信
 */
void input_from_keyboard(void)
{
	tsToCoNet_NwkLyTr_Context *pc = (void*)sAppData.pContextNwk;
	// handle UART command
	while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
		int16 i16Char;

		i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

		vfPrintf(&sSerStream, "\n\r# [%c] --> ", i16Char);
	    SERIAL_vFlush(sSerStream.u8Device);

		switch(i16Char) {

		case 'i': //データ表示
			_C {
				A_PRINTF(LB "App ID:%08x Long Addr:%08x Short Addr %04x LID %02d",
				sToCoNet_AppContext.u32AppId, ToCoNet_u32GetSerial(), sToCoNet_AppContext.u16ShortAddress,
				sAppData.sFlash.sData.u8id);
			}
			break;

		case 'n': //ネットワーク状態表示
			_C {
				A_PRINTF("Nwk Conf"
			 			LB"layer = %d"
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
					A_PRINTF(LB "Start (Layer %d)", sAppData.sNwkLayerTreeConfig.u8Layer);
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
					A_PRINTF(LB "Start (Layer %d)", sAppData.sNwkLayerTreeConfig.u8Layer);
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
				sAppData.sNwkLayerTreeConfig.u8Layer = 10;

				sTx.u32SrcAddr = ToCoNet_u32GetSerial();
				sTx.u32DstAddr = TOCONET_NWK_ADDR_NEIGHBOUR_ABOVE;

				sTx.u8Cmd = 0; // 0..7 の値を取る。パケットの種別を分けたい時に使用する
				sTx.u8Len = q - sTx.auData; // パケットのサイズ
				sTx.u8CbId = sAppData.u16frame_count & 0xFF; // TxEvent で通知される番号、送信先には通知されない
				sTx.u8Seq = sAppData.u16frame_count & 0xFF; // シーケンス番号(送信先に通知される)
				sTx.u8Retry = 0x82; //3回送る

				// SPRINTF でメッセージを作成
				SPRINTF_vRewind();
				vfPrintf(SPRINTF_Stream, "Test");
				memcpy(sTx.auData, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
				sTx.u8Len = SPRINTF_u16Length();

				A_PRINTF(LB "Router Layer %d", sAppData.sNwkLayerTreeConfig.u8Layer);
				ToCoNet_Nwk_bInit(sAppData.pContextNwk);
				ToCoNet_Nwk_bStart(sAppData.pContextNwk);

				//送信要求
				if (ToCoNet_Nwk_bTx(sAppData.pContextNwk, &sTx)) {
					A_PRINTF(LB"Tx Processing (AddrHigherLayer : %08x)"LB,pc->u32AddrHigherLayer);
				} else {	
					A_PRINTF(LB"Tx Error"LB);
				}

				sAppData.sNwkLayerTreeConfig.u8Layer = 1;

				A_PRINTF(LB "Router Layer %d", sAppData.sNwkLayerTreeConfig.u8Layer);
				ToCoNet_Nwk_bInit(sAppData.pContextNwk);
				ToCoNet_Nwk_bStart(sAppData.pContextNwk);

			}
			break;

		case 'r': //ネットワーク再始動
			_C {
				ToCoNet_Nwk_bInit(sAppData.pContextNwk);
				ToCoNet_Nwk_bStart(sAppData.pContextNwk);
			}
			break;

		case 'e': //シリアル通信のテスト用
			_C {
				if (!bPortRead(LED)) {
                	vPortSetLo(LED);
					A_PUTCHAR('A');
            	} else {
					vPortSetHi(LED);
					A_PUTCHAR('B');
					
				}
			}
			break;
		}
	}
}

void input_from_esp32(void) {

	while (!SERIAL_bRxQueueEmpty(sSerPort1.u8SerialPort)) {
		// UART1のFIFOキューから１バイトずつ取り出して処理する。
		int16 i16Char;

		i16Char = SERIAL_i16RxChar(sSerPort1.u8SerialPort);

		//vfPrintf(&sSerStream1, "\n\r# [%c] --> ", i16Char);
	    SERIAL_vFlush(sSerStream1.u8Device);

		switch(i16Char){
			case 'h':
			_C {
				if (!bPortRead(LED)) {
                	vPortSetLo(LED);
					A_PUTCHAR('A');
            	} else {
					vPortSetHi(LED);
					A_PUTCHAR('B');
					
				}
			}
			break;
		}
	}
		
}

void router(){
	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx)); // 必ず０クリアしてから使う！
	uint8 *q =  sTx.auData;

	sTx.u32SrcAddr = ToCoNet_u32GetSerial();

	sTx.u32DstAddr = TOCONET_NWK_ADDR_NEIGHBOUR_ABOVE;
	sTx.u8Len = SPRINTF_u16Length();
	sTx.u8Cmd = 0; // 0..7 の値を取る。パケットの種別を分けたい時に使用する
	sTx.u8Len = q - sTx.auData; // パケットのサイズ
	sTx.u8CbId = sAppData.u16frame_count & 0xFF; // TxEvent で通知される番号、送信先には通知されない
	sTx.u8Seq = sAppData.u16frame_count & 0xFF; // シーケンス番号(送信先に通知される)
	sTx.u8Retry = 0x82; //3回送る

	// SPRINTF でメッセージを作成
	SPRINTF_vRewind();	// 内部ポインタを先頭に巻き戻す
	vfPrintf(SPRINTF_Stream, LB"%08x",ToCoNet_u32GetSerial());
	// vfPrintf(SPRINTF_Stream, LB"%s -> %08x",rdate,ToCoNet_u32GetSerial());
	memcpy(sTx.auData, SPRINTF_pu8GetBuff(), SPRINTF_u16Length());
	sTx.u8Len = SPRINTF_u16Length();

	//　送信処理に関する表示
	if (ToCoNet_Nwk_bTx(sAppData.pContextNwk, &sTx)) {
		A_PRINTF(LB"Tx Processing (AddrHigherLayer : %08x)"LB, pc->u32AddrHigherLayer);
	} else {
		A_PRINTF("Tx Error"LB);
		}

	}

/****************************************************************************/
/***        コールバック関数（記述必須）                                    ***/
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
		// clear application context
		memset(&sAppData, 0x00, sizeof(sAppData));

		// SPRINTF
		SPRINTF_vInit128();

		// フラッシュメモリからの読み出し
		// フラッシュからの読み込みが失敗した場合、ID=15 で設定する
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

		//ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(TOCONET_DEBUG_LEVEL);

		// START UP MESSAGE
		A_PRINTF(LB "Parent:%08x" LB,ToCoNet_u32GetSerial());
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
		A_PRINTF( LB"Wake Up");
	}
}

/**
 * メイン処理
 * - シリアルポートの処理
 */
void cbToCoNet_vMain(void) {
	/* handle serial input */
	input_from_keyboard();
	input_from_esp32();
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

	rdate = pRx;

	ToCoNet_Event_Process(E_ORDER_KICK_WAIT, 0, vProcessEvCore);
}

/**
 * 送信完了時のイベント
 * @param u8CbId
 * @param bStatus
 */
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
		//　送信処理結果の表示
		A_PRINTF(LB"Tx Result : %s",
		bStatus ? "OK" : "NG");
		ToCoNet_Event_Process(E_EVENT_TX, u8CbId, vProcessEvCore);
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

		// Local Event Machine にこのイベントを通知
		ToCoNet_Event_Process(eEvent, u32arg, vProcessEvCore);

		//　上位ノードへの接続を通知
		A_PRINTF(LB"Connect");
		//　接続先ノードのアドレスを表示
		A_PRINTF(LB"Access Point : %08x"LB, pc->u32AddrHigherLayer);

		router();
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
/***       初期化                                          				　***/
/****************************************************************************/

/**
 * ハードウェアの初期化
 * @param f_warm_start
 */
PRIVATE void vInitHardware(int f_warm_start) {
	// インタラクティブモードの初期化
	Interactive_vInit();

	// 使用ポートの設定
    vPortAsOutput(LED);
    vPortSetHi(LED);

	// Serial Port の初期化
	vSerialInit();

	// 受信したときにDO1を光らせる
	bAHI_DoEnableOutputs(TRUE);
	vAHI_DoSetDataOut( 0x01<<1, 0 );

	// 外部ウォッチドッグタイマー用
	vPortSetLo(11);				// 外部のウォッチドッグを有効にする。
	vPortSet_TrueAsLo(9, bVwd);	// VWDをいったんHiにする。
	vPortAsOutput(11);			// DIO11を出力として使用する。
	vPortAsOutput(9);			// DIO9を出力として使用する。
}

/**
 * UARTの初期化
 */
void vSerialInit() {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer0[1024];
	static uint8 au8SerialRxBuffer0[512];

	static uint8 au8SerialTxBuffer1[1024];
	static uint8 au8SerialRxBuffer1[512];

	/* Initialise the serial port (0) to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer0;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer0;
	sSerPort.u32BaudRate = UART_BAUD;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer0);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer0);
	sSerPort.u8SerialPort = UART0_PORT;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInit(&sSerPort);

	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART0_PORT;


	/* Initialise the serial port (1) to be used for ESP32 output */
	/* TweliteのUART1のピンはTxが2（基板の表示は14）, RXが19（基板の表示は15） */
	sSerPort1.pu8SerialRxQueueBuffer = au8SerialRxBuffer1;
	sSerPort1.pu8SerialTxQueueBuffer = au8SerialTxBuffer1;
	sSerPort1.u32BaudRate = UART_BAUD;
	sSerPort1.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort1.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort1.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer1);
	sSerPort1.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer1);
	sSerPort1.u8SerialPort = UART1_PORT;
	sSerPort1.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInit(&sSerPort1);

	sSerStream1.bPutChar = SERIAL_bTxChar;
	sSerStream1.u8Device = UART1_PORT;

}



/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
