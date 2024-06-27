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


#include <stdlib.h>
#include <stdint.h>

#include "wsf_types.h"
#include "att_api.h"
#include "att_uuid.h"
#include "wsf_trace.h"
#include "util/bstream.h"
//#include "svc_hrs.h"
#include "svc_ch.h"
#include "svc_cfg.h"

#include "ble_service.h"


/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef HRS_SEC_PERMIT_READ
#define HRS_SEC_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

/*! Characteristic write permissions */
#ifndef HRS_SEC_PERMIT_WRITE
#define HRS_SEC_PERMIT_WRITE SVC_SEC_PERMIT_WRITE
#endif

/**************************************************************************************************
 Static Variables
**************************************************************************************************/

/* UUIDs */
static const uint8_t svcHrmUuid[] = {UINT16_TO_BYTES(ATT_UUID_HR_MEAS)};
//static const uint8_t svcSlUuid[] = {UINT16_TO_BYTES(ATT_UUID_HR_SENSOR_LOC)};
static const uint8_t svcCpUuid[] = {UINT16_TO_BYTES(ATT_UUID_HR_CP)};


/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Heart rate service declaration */
//static const uint8_t hrsValSvc[] = {UINT16_TO_BYTES(ATT_UUID_HEART_RATE_SERVICE)};
static const uint8_t hrsValSvc[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E};
static const uint16_t hrsLenSvc = sizeof(hrsValSvc);

/* Heart rate measurement characteristic */
//static const uint8_t hrsValHrmCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(HRS_HRM_HDL), UINT16_TO_BYTES(ATT_UUID_HR_MEAS)};
static const uint8_t hrsValHrmCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(HRS_HRM_HDL), 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
static const uint16_t hrsLenHrmCh = sizeof(hrsValHrmCh);

/* Heart rate measurement */
/* Note these are dummy values */
static const uint8_t hrsValHrm[] = {0};
static const uint16_t hrsLenHrm = sizeof(hrsValHrm);

/* Heart rate measurement client characteristic configuration */
static uint8_t hrsValHrmChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hrsLenHrmChCcc = sizeof(hrsValHrmChCcc);

///* Body sensor location characteristic */
//static const uint8_t hrsValSlCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(HRS_SL_HDL), UINT16_TO_BYTES(ATT_UUID_HR_SENSOR_LOC)};
//static const uint16_t hrsLenSlCh = sizeof(hrsValSlCh);
//
///* Body sensor location */
//static uint8_t hrsValSl[] = {CH_BSENSOR_LOC_WRIST};
//static const uint16_t hrsLenSl = sizeof(hrsValSl);

/* Control point characteristic */
//static const uint8_t hrsValCpCh[] = {ATT_PROP_READ | ATT_PROP_WRITE, UINT16_TO_BYTES(HRS_CP_HDL), UINT16_TO_BYTES(ATT_UUID_HR_CP)};
static const uint8_t hrsValCpCh[] = {ATT_PROP_READ | ATT_PROP_WRITE, UINT16_TO_BYTES(HRS_CP_HDL), 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};
static const uint16_t hrsLenCpCh = sizeof(hrsValCpCh);

/* Control point */
/* Note these are dummy values */
static uint8_t hrsValCp[20] = {0};
static uint16_t hrsLenCp = sizeof(hrsValCp);

/* Attribute list for HRS group */
static const attsAttr_t hrsList[] =
{
  {
    attPrimSvcUuid,
    (uint8_t *) hrsValSvc,
    (uint16_t *) &hrsLenSvc,
    sizeof(hrsValSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) hrsValHrmCh,
    (uint16_t *) &hrsLenHrmCh,
    sizeof(hrsValHrmCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    svcHrmUuid,
    (uint8_t *) hrsValHrm,
    (uint16_t *) &hrsLenHrm,
    sizeof(hrsValHrm),
    0,
    0
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hrsValHrmChCcc,
    (uint16_t *) &hrsLenHrmChCcc,
    sizeof(hrsValHrmChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE)
  },
//  {
//    attChUuid,
//    (uint8_t *) hrsValSlCh,
//    (uint16_t *) &hrsLenSlCh,
//    sizeof(hrsValSlCh),
//    0,
//    ATTS_PERMIT_READ
//  },
//  {
//    svcSlUuid,
//    hrsValSl,
//    (uint16_t *) &hrsLenSl,
//    sizeof(hrsValSl),
//    0,
//	ATTS_PERMIT_READ
//  },
  {
    attChUuid,
    (uint8_t *) hrsValCpCh,
    (uint16_t *) &hrsLenCpCh,
    sizeof(hrsValCpCh),
    0,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  {
    svcCpUuid,
    (uint8_t *) hrsValCp,
    (uint16_t *) &hrsLenCp,
    sizeof(hrsValCp),
    ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN,
	ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
//	ATTS_PERMIT_WRITE
  }
};

/* HRS group structure */
static attsGroup_t svcHrsGroup =
{
  NULL,
  (attsAttr_t *) hrsList,
  NULL,
  NULL,
  HRS_START_HDL,
  HRS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void RegisterService(void)
{
  AttsAddGroup(&svcHrsGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void RemoveService(void)
{
  AttsRemoveGroup(HRS_START_HDL);
}

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  readCback   Read callback function.
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void RegisterServiceCallback(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
	svcHrsGroup.readCback = readCback;
	svcHrsGroup.writeCback = writeCback;
}

