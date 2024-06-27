/*******************************************************************************
* Copyright (C) 2018 Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#include <stdio.h>
#include <string.h>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "gcr_regs.h"
#include "wsf_types.h"
#include "wsf_os.h"
#include "util/bstream.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "wsf_assert.h"
#include "hci_api.h"
#include "hci_vs.h"
#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "sec_api.h"
#include "dm_api.h"
#include "l2c_api.h"
#include "smp_api.h"
#include "att_api.h"
#include "app_api.h"
#include "app_main.h"
#include "app_db.h"
#include "app_ui.h"
#include "svc_ch.h"
#include "svc_core.h"
#include "svc_dis.h"
#include "ll_init_api.h"
#include "util/calc128.h"


#include "wsf_types.h"
#include "wsf_os.h"

#if WDXS_INCLUDED  == TRUE
#include "wsf_efs.h"
#include "svc_wdxs.h"
#include "wdxs/wdxs_api.h"
#include "wdxs/wdxs_main.h"
#include "wdxs_file.h"
#endif

#include "ble_api.h"
#include "ble_service.h"
#include "app_interface_process.h"

#include "Peripherals.h"
#include "tmr.h"
#include "trng.h"
#include "app_flash.h"
#include "main.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

#define LL_IMPL_REV             0x2303

#define LL_MEMORY_FOOTPRINT     0xc152

#define DISPATCH_BLE(iter)	for(int ble_dispatch = 0; ble_dispatch < iter; ble_dispatch++){wsfOsDispatcher();}

uint8_t LlMem[LL_MEMORY_FOOTPRINT];

/*! Enumeration of client characteristic configuration descriptors */
enum
{
  DATS_GATT_SC_CCC_IDX,           /*! GATT service, service changed characteristic */
  DATS_WP_DAT_CCC_IDX,            /*! ARM Ltd. proprietary service, data transfer characteristic */
  DATS_NUM_CCC_IDX
};

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/
/*! configurable parameters for advertising */
/* These intervals directly impact energy usage during the non-connected/advertising mode */
static const appAdvCfg_t datsAdvCfg =
{
  { 1000,     0,     0},                  /*! Advertising durations in ms */
  {   96,   200,     0}                   /*! Advertising intervals in 0.625 ms units */
};


/*! configurable parameters for slave */
static const appSlaveCfg_t datsSlaveCfg =
{
  1,                                      /*! Maximum connections */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t datsUpdateCfg =
{
  0,                                    /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
  6,                                    /*! Minimum connection interval in 1.25ms units */
  12,                                    /*! Maximum connection interval in 1.25ms units */
  4,                                    /*! Connection latency */
  50,                                   /*! Supervision timeout in 10ms units */
  5                                     /*! Number of update attempts before giving up */
};

/*! ATT configurable parameters (increase MTU) */
static const attCfg_t datsAttCfg =
{
  15,                               /* ATT server service discovery connection idle timeout in seconds */
  241,                              /* desired ATT MTU */
  ATT_MAX_TRANS_TIMEOUT,            /* transcation timeout in seconds */
  4                                 /* number of queued prepare writes supported by server */
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! advertising data, discoverable mode */
static const uint8_t datsAdvDataDisc[] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! manufacturer specific data */
  3,                                      /*! length */
  DM_ADV_TYPE_MANUFACTURER,               /*! AD type */
  UINT16_TO_BYTES(HCI_ID_ARM)             /*! company ID */
};

/*! scan data, discoverable mode */
static const uint8_t datsScanDataDisc[] =
{
  /*! device name */
  20,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'M',
  'A',
  'X',
  'R',
  'E',
  'F',
  'D',
  'E',
  'S',
  '1',
  '0',
  '6',
  '/',
  'H',
  'S',
  'P',
  '4',
  '.',
  '0'
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t datsCccSet[DATS_NUM_CCC_IDX] =
{
  /* cccd handle          value range               security level */
  {GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},   /* DATS_GATT_SC_CCC_IDX */
  {HRS_HRM_CH_CCC_HDL,     ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}    /* DATS_WP_DAT_CCC_IDX */
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
static struct
{
  wsfHandlerId_t    handlerId;        /* WSF handler ID */
} datsCb;

bool_t resetFlag;

static dmConnId_t g_connID;

static svc_cv_t fp_svc = NULL;
static ble_state_cb_t fp_ble_state = NULL;

static ble_state_t g_ble_state = BLE_STATE_IDLE;

LlRtCfg_t _ll_cfg = {
    /* Device */
    /*compId*/                  LL_COMP_ID_ARM,
    /*implRev*/                 LL_IMPL_REV,
    /*btVer*/                   LL_VER_BT_CORE_SPEC_5_0,
    /*_align32 */               0, // padding for alignment

    /* Advertiser */
    /*maxAdvSets*/              4, // 4 Extended Advertising Sets
    /*maxAdvReports*/           8,
    /*maxExtAdvDataLen*/        LL_MAX_ADV_DATA_LEN,
    /*defExtAdvDataFrag*/       64,
    /*auxDelayUsec*/            0,

    /* Scanner */
    /*maxScanReqRcvdEvt*/       4,
    /*maxExtScanDataLen*/       LL_MAX_ADV_DATA_LEN,

    /* Connection */
    /*maxConn*/                 2,
    /*numTxBufs*/               16,
    /*numRxBufs*/               16,
    /*maxAclLen*/               512,
    /*defTxPwrLvl*/             0,
    /*ceJitterUsec*/            0,

    /* DTM */
    /*dtmRxSyncMs*/             10000,

    /* PHY */
    /*phy2mSup*/                TRUE,
    /*phyCodedSup*/             FALSE,
    /*stableModIdxTxSup*/       FALSE,
    /*stableModIdxRxSup*/       FALSE
};

const BbRtCfg_t _bb_cfg = {
    /*clkPpm*/                  20,
    /*rfSetupDelayUsec*/        BB_RF_SETUP_DELAY_US,
    /*defaultTxPower*/          5,
    /*maxScanPeriodMsec*/       BB_MAX_SCAN_PERIOD_MS,
    /*schSetupDelayUsec*/       BB_SCH_SETUP_DELAY_US
};

static uint8_t serviceReadCallback(dmConnId_t connId, uint16_t handle, uint8_t operation,
		uint16_t offset, attsAttr_t *pAttr);

static uint8_t serviceWriteCallback(dmConnId_t connId, uint16_t handle, uint8_t operation,
    	  uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr);

static void DatsHandlerInit(wsfHandlerId_t handlerId);

static void DatsHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

static void DatsStart(void);

static void datsDmCback(dmEvt_t *pDmEvt);

static void datsAttCback(attEvt_t *pEvt);

static void datsCccCback(attsCccEvt_t *pEvt);

static void datsSetup(dmEvt_t *pMsg);

static void datsProcMsg(dmEvt_t *pMsg);

static void datsWsfBufDiagnostics(WsfBufDiag_t *pInfo);

void SysTick_Handler(void)
{
    WsfTimerUpdate(WSF_MS_PER_TICK);
}

void ble_init()
{
	  wsfHandlerId_t handlerId;

#ifndef ENABLE_SDMA
	  uint32_t memUsed;

	  /* Enable coded PHY */
	  if(MXC_GCR->revision != 0xA1) {
	    _ll_cfg.phyCodedSup = TRUE;
	  }

	  /* Initialize link layer. */
	  LlInitRtCfg_t ll_init_cfg =
	  {
	      .pBbRtCfg     = &_bb_cfg,
	      .wlSizeCfg    = 4,
	      .rlSizeCfg    = 4,
	      .plSizeCfg    = 4,
	      .pLlRtCfg     = &_ll_cfg,
	      .pFreeMem     = LlMem,
	      .freeMemAvail = LL_MEMORY_FOOTPRINT
	  };

	  memUsed = LlInitControllerExtInit(&ll_init_cfg);
	  if(memUsed != LL_MEMORY_FOOTPRINT)
	  {
		  pr_info("mem_used: 0x%x LL_MEMORY_FOOTPRINT: 0x%x\n", memUsed,
	          LL_MEMORY_FOOTPRINT);
	  }
#endif

	handlerId = WsfOsSetNextHandler(HciHandler);
	HciHandlerInit(handlerId);

	SecInit();
	SecAesInit();
	SecCmacInit();
	SecEccInit();

	handlerId = WsfOsSetNextHandler(DmHandler);
	DmDevVsInit(0);

	DmAdvInit();
	DmConnInit();
	DmConnSlaveInit();

	DmSecInit();
	DmSecLescInit();
	DmPrivInit();
	DmHandlerInit(handlerId);

	handlerId = WsfOsSetNextHandler(L2cSlaveHandler);
	L2cSlaveHandlerInit(handlerId);
	L2cInit();
	L2cSlaveInit();

	handlerId = WsfOsSetNextHandler(AttHandler);
	AttHandlerInit(handlerId);
	AttsInit();
	AttsIndInit();

	handlerId = WsfOsSetNextHandler(SmpHandler);
	SmpHandlerInit(handlerId);
	SmprInit();
	SmprScInit();
	HciSetMaxRxAclLen(256);

	handlerId = WsfOsSetNextHandler(AppHandler);
	AppHandlerInit(handlerId);

	handlerId = WsfOsSetNextHandler(DatsHandler);
	DatsHandlerInit(handlerId);

	/* Creating GAP and GATT services */
	SvcCoreAddGroup();

	/* Creating MRD104 data exchange services */
	RegisterServiceCallback(serviceReadCallback, serviceWriteCallback);
	RegisterService();

	DatsStart();
}

void ble_service_callback(svc_cv_t svc_callback)
{
	fp_svc = svc_callback;
}

void ble_state_callback(ble_state_cb_t state_callback)
{
	fp_ble_state = state_callback;
}

void ble_send_notify(uint8_t * p_data, uint16_t len)
{
	if (AttsCccEnabled(g_connID, DATS_WP_DAT_CCC_IDX))
	{
		DISPATCH_BLE(1000);

		/* send notification */
	    AttsHandleValueNtf(g_connID, HRS_HRM_HDL, len, p_data);
	}
}

void ble_send_data(uint8_t * p_data, uint16_t len)
{
	if(g_connID){
		AttsSetAttr(HRS_CP_HDL, len, p_data);
	}
}

void ble_start_adv()
{
	AppAdvStart(APP_MODE_AUTO_INIT);
}

void ble_stop_adv()
{
	AppAdvStop();
}

ble_state_t ble_get_state()
{
	return g_ble_state;
}

void ble_connection_close()
{
	AppConnClose(g_connID);
}

static uint8_t serviceWriteCallback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                          	  uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{

  if(NULL != fp_svc){
	  fp_svc(CALLBACK_WRITE, pValue, len);
  }

  return ATT_SUCCESS;
}

static uint8_t serviceReadCallback(dmConnId_t connId, uint16_t handle, uint8_t operation,
							uint16_t offset, attsAttr_t *pAttr)
{
	if(NULL != fp_svc){
		fp_svc(CALLBACK_READ, NULL, 0);
	}

	return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t   *pMsg;
  uint16_t  len;


    len = DmSizeOfEvt(pDmEvt);

    if ((pMsg = WsfMsgAlloc(len)) != NULL)
    {
      memcpy(pMsg, pDmEvt, len);
      WsfMsgSend(datsCb.handlerId, pMsg);
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Application ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsAttCback(attEvt_t *pEvt)
{

}

/*************************************************************************************************/
/*!
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsCccCback(attsCccEvt_t *pEvt)
{
  appDbHdl_t    dbHdl;

  /* if CCC not set from initialization and there's a device record */
  if ((pEvt->handle != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl((dmConnId_t) pEvt->hdr.param)) != APP_DB_HDL_NONE))
  {
    /* store value in device database */
    AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set up advertising and other procedures that need to be performed after
 *          device reset.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsSetup(dmEvt_t *pMsg)
{

  /* set advertising and scan response data for discoverable mode */
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(datsAdvDataDisc), (uint8_t *) datsAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(datsScanDataDisc), (uint8_t *) datsScanDataDisc);


  /* set advertising and scan response data for connectable mode */
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(datsAdvDataDisc), (uint8_t *) datsAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(datsScanDataDisc), (uint8_t *) datsScanDataDisc);

  /* Enable coded PHY */
  if(MXC_GCR->revision != 0xA1) {
    DmSetDefaultPhy(0, HCI_PHY_LE_1M_BIT | HCI_PHY_LE_2M_BIT | HCI_PHY_LE_CODED_BIT,
      HCI_PHY_LE_1M_BIT | HCI_PHY_LE_2M_BIT | HCI_PHY_LE_CODED_BIT);
  } else {
    DmSetDefaultPhy(0, HCI_PHY_LE_1M_BIT | HCI_PHY_LE_2M_BIT, HCI_PHY_LE_1M_BIT | HCI_PHY_LE_2M_BIT);
  }

  AppAdvStart(APP_MODE_AUTO_INIT);
}

/*************************************************************************************************/
/*!
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsProcMsg(dmEvt_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;

  switch(pMsg->hdr.event)
  {
    case DM_RESET_CMPL_IND:
      DmSecGenerateEccKeyReq();
      datsSetup(pMsg);
      uiEvent = APP_UI_RESET_CMPL;

      if(NULL != fp_ble_state)
    	  fp_ble_state(BLE_STATE_IDLE);
      g_ble_state = BLE_STATE_IDLE;

      break;

    case DM_ADV_START_IND:
      uiEvent = APP_UI_ADV_START;

      if(NULL != fp_ble_state)
    	  fp_ble_state(BLE_STATE_ADV_START);
      g_ble_state = BLE_STATE_ADV_START;
      break;

#ifndef BTLE_APP_IGNORE_EXT_EVENTS
    case DM_ADV_SET_START_IND:
      uiEvent = APP_UI_ADV_SET_START_IND;

      break;
#endif /* BTLE_APP_IGNORE_EXT_EVENTS */

    case DM_ADV_STOP_IND:
      uiEvent = APP_UI_ADV_STOP;
      if(NULL != fp_ble_state)
    	  fp_ble_state(BLE_STATE_ADV_STOP);
      g_ble_state = BLE_STATE_ADV_STOP;
      break;

#ifndef BTLE_APP_IGNORE_EXT_EVENTS
     case DM_ADV_SET_STOP_IND:
      uiEvent = APP_UI_ADV_SET_STOP_IND;
      break;
#endif /* BTLE_APP_IGNORE_EXT_EVENTS */

    case DM_CONN_OPEN_IND:
      uiEvent = APP_UI_CONN_OPEN;
      g_connID = (dmConnId_t) pMsg->hdr.param;
      HciVsSetConnTxPower(g_connID, 127);

      if(NULL != fp_ble_state)
    	  fp_ble_state(BLE_STATE_CONN_OPEN);
      g_ble_state = BLE_STATE_CONN_OPEN;
      break;

    case DM_CONN_CLOSE_IND:
    	pr_info("Connection closed status 0x%x, reason 0x%x", pMsg->connClose.status, pMsg->connClose.reason);
      switch (pMsg->connClose.reason)
      {
        case HCI_ERR_CONN_TIMEOUT:      pr_info(" TIMEOUT\n");         break;
        case HCI_ERR_LOCAL_TERMINATED:  pr_info(" LOCAL TERM\n");      break;
        case HCI_ERR_REMOTE_TERMINATED: pr_info(" REMOTE TERM\n");     break;
        case HCI_ERR_CONN_FAIL:         pr_info(" FAIL ESTABLISH\n");  break;
        case HCI_ERR_MIC_FAILURE:       pr_info(" MIC FAILURE\n");     break;
      }
      uiEvent = APP_UI_CONN_CLOSE;
      if(NULL != fp_ble_state)
    	  fp_ble_state(BLE_STATE_CONN_CLOSE);
      g_ble_state = BLE_STATE_CONN_CLOSE;
      break;

    case DM_SEC_COMPARE_IND:
      AppHandleNumericComparison(&pMsg->cnfInd);
      break;

    case DM_ADV_NEW_ADDR_IND:
      break;

    case DM_VENDOR_SPEC_IND:
      break;

    default:
      break;
  }

  if (uiEvent != APP_UI_NONE)
  {
    AppUiAction(uiEvent);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void DatsHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("DatsHandlerInit");

  /* store handler ID */
  datsCb.handlerId = handlerId;

  /* Set configuration pointers */
  pAppSlaveCfg = (appSlaveCfg_t *) &datsSlaveCfg;

  pAppAdvCfg = (appAdvCfg_t *) &datsAdvCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &datsUpdateCfg;
  pAttCfg = (attCfg_t *) &datsAttCfg;


  /* Initialize application framework */
  AppSlaveInit();

  resetFlag = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Callback for WSF buffer diagnostic messages.
 *
 *  \param  pInfo     Diagnostics message
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsWsfBufDiagnostics(WsfBufDiag_t *pInfo)
{
  if (pInfo->type == WSF_BUF_ALLOC_FAILED)
  {
    APP_TRACE_INFO2("Dats got WSF Buffer Allocation Failure - Task: %d Len: %d",
                     pInfo->param.alloc.taskId, pInfo->param.alloc.len);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void DatsHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("Dats got evt 0x%x", pMsg->event);

    if (pMsg->event >= DM_CBACK_START && pMsg->event <= DM_CBACK_END)
    {
      /* process advertising and connection-related messages */
      AppSlaveProcDmMsg((dmEvt_t *) pMsg);

      /* process security-related messages */
      AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);


    }

    /* perform profile and user interface-related operations */
    datsProcMsg((dmEvt_t *) pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void DatsStart(void)
{
  /* Register for stack callbacks */
  DmRegister(datsDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, datsDmCback);
  AttRegister(datsAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(DATS_NUM_CCC_IDX, (attsCccSet_t *) datsCccSet, datsCccCback);

  WsfBufDiagRegister(datsWsfBufDiagnostics);

  /* Reset the device */
  DmDevReset();
}

//BLE Main Functions are moved here
uint32_t SystemHeapSize=WSF_BUF_SIZE;
uint32_t SystemHeap[WSF_BUF_SIZE/4];
uint32_t SystemHeapStart;

//volatile uint8_t ble_sent_evt = 0;

static uint8_t ble_rx_buff[20] = {0};
static uint16_t ble_rx_len = 0;

/*! Default pool descriptor. */
static wsfBufPoolDesc_t mainPoolDesc[WSF_BUF_POOLS] =
{
  {  16,  8 },
  {  32,  4 },
  {  64,  4 },
  { 128,  4 },
  { 256,  4 },
  { 512,  4 }
};

static bool_t myTrace(const uint8_t *pBuf, uint32_t len)
{
    extern uint8_t wsfCsNesting;

    if (wsfCsNesting == 0)
    {
        fwrite(pBuf, len, 1, stdout);
        return TRUE;
    }

    return FALSE;
}

// /* Currently we are using RTC to send notification packets. If that works, these function can be removed in the future */
// void BLE_Timer_Handler()
// {
//     // Clear interrupt
//     TMR_IntClear(BLE_TIMER);

//     ble_sent_evt = 1;
// }

// void BLE_Timer_Init()
// {
// 	// Declare variables
// 	tmr_cfg_t tmr;

// 	/* 12.5ms */
// //	uint32_t period_ticks =  PeripheralClock/(4 * 60);
// #define MS_TO_TICK(ms, prescaler)	((ms) * (PeripheralClock / (prescaler * 1000)))
// 	uint32_t period_ticks =  MS_TO_TICK(17, 2);

// 	TMR_Disable(BLE_TIMER);

// 	TMR_Init(BLE_TIMER, TMR_PRES_2, 0);

// 	tmr.mode = TMR_MODE_CONTINUOUS;
// 	tmr.cmp_cnt = period_ticks; //SystemCoreClock*(1/interval_time);
// 	tmr.pol = 0;
// 	TMR_Config(BLE_TIMER, &tmr);

// }

// void BLE_Timer_Enable(uint8_t en)
// {
// 	TMR_Enable(BLE_TIMER);
// }

/*************************************************************************************************/
/*!
 *  \brief  Initialize WSF.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfInit(void)
{
    uint32_t bytesUsed;
    /* setup the systick for 1MS timer*/
    SysTick->LOAD = (SystemCoreClock / 1000) * WSF_MS_PER_TICK;
    SysTick->VAL = 0;
    SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);

    WsfTimerInit();

    SystemHeapStart = (uint32_t)&SystemHeap;
    memset(SystemHeap, 0, sizeof(SystemHeap));
    pr_info("SystemHeapStart = 0x%x\n", SystemHeapStart);
    pr_info("SystemHeapSize = 0x%x\n", SystemHeapSize);
    bytesUsed = WsfBufInit(WSF_BUF_POOLS, mainPoolDesc);
    pr_info("bytesUsed = 0x%x\n", bytesUsed);
    UNUSED(bytesUsed);

    WsfTraceRegisterHandler(myTrace);
    WsfTraceEnable(TRUE);
}


/*
 * In two-chip solutions, setting the address must wait until the HCI interface is initialized.
 * This handler can also catch other Application events, but none are currently implemented.
 * See ble-profiles/sources/apps/app/common/app_ui.c for further details.
 *
 */
void SetAddress(uint8_t event)
{
	uint8_t bdAddr[6] = {0x00, 0x00, 0x00, 0x80, 0x18, 0x00};

	if(event == APP_UI_RESET_CMPL)
	{
		if(app_flash_read_data(APP_FLASH_MAC_ADDR, &bdAddr[0], 6) == 0)
		{

		}
		else{
		    TRNG_Init(NULL);

		    TRNG_Read(MXC_TRNG, &bdAddr[0], 3);

		    app_flash_write_data(APP_FLASH_MAC_ADDR, &bdAddr[0], 6);
		}

		HciVsSetBdAddr(bdAddr);
	}

}

void callback(svc_cb_type_t type, uint8_t * p_value, uint16_t len){

	if(CALLBACK_WRITE == type)
	{
		if(len > sizeof(ble_rx_buff)){
			len = 20;
		}
		memcpy(ble_rx_buff, p_value, len);
		ble_rx_len = len;
		app_interface_process_cmd(APP_INTERFACE_DEV_BLE, ble_rx_buff, ble_rx_len);

	}


	if(CALLBACK_READ == type)
	{
		app_main_evt_post(EVT_BLE_READ_CB);
		pr_info("Read\n");
	};
}

void ble_state_cb(ble_state_t state)
{
	switch(state)
	{
	case BLE_STATE_IDLE:
		pr_info("BLE_STATE_IDLE \n");
		break;

	case BLE_STATE_ADV_START:
		pr_info("BLE_STATE_ADV_START \n");
		break;

	case BLE_STATE_ADV_STOP:
		pr_info("BLE_STATE_ADV_STOP \n");
		break;

	case BLE_STATE_CONN_OPEN:
		app_main_evt_post(EVT_BLE_CONN_OPEN);
		pr_info("BLE_STATE_CONN_OPEN \n");
		break;

	case BLE_STATE_CONN_CLOSE:
		app_main_evt_post(EVT_BLE_CONN_CLOSE);
		pr_info("BLE_STATE_CONN_CLOSE \n");
		break;
	default:
		break;
	}
}
