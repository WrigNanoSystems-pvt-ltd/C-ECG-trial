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

#ifndef DRIVERS_BLE_SERVICE_H_
#define DRIVERS_BLE_SERVICE_H_

/**************************************************************************************************
  Macros
**************************************************************************************************/
/** \name Heart Rate Error Codes
 *
 */
/**@{*/
#define HRS_ERR_CP_NOT_SUP          0x80    /*!< \brief Control Point value not supported */
/**@}*/

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Heart Rate Service Handles
 *
 */
/**@{*/
#define HRS_START_HDL               0x20              /*!< \brief Start handle. */
#define HRS_END_HDL                 (HRS_MAX_HDL - 1) /*!< \brief End handle. */


/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Heart Rate Service Handles */
enum
{
  HRS_SVC_HDL = HRS_START_HDL,      /*!< \brief Heart rate service declaration */
  HRS_HRM_CH_HDL,                   /*!< \brief Heart rate measurement characteristic */
  HRS_HRM_HDL,                      /*!< \brief Heart rate measurement */
  HRS_HRM_CH_CCC_HDL,               /*!< \brief Heart rate measurement client characteristic configuration */
//  HRS_SL_CH_HDL,                    /*!< \brief Body sensor location characteristic */
//  HRS_SL_HDL,                       /*!< \brief Body sensor location */
  HRS_CP_CH_HDL,                    /*!< \brief Heart rate control point characteristic */
  HRS_CP_HDL,                       /*!< \brief Heart rate control point */
  HRS_MAX_HDL                       /*!< \brief Maximum handle. */
};
/**@}*/

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void RegisterService(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void RemoveService(void);

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
void RegisterServiceCallback(attsReadCback_t readCback, attsWriteCback_t writeCback);

#endif /* DRIVERS_BLE_SERVICE_H_ */
