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

#ifndef DRIVERS_BLE_API_H_
#define DRIVERS_BLE_API_H_

typedef enum {
	CALLBACK_WRITE	=	0u,
	CALLBACK_READ,

	CALLBACK_MAX
} svc_cb_type_t;

typedef enum {
	BLE_STATE_IDLE	=	0u,
	BLE_STATE_ADV_START,
	BLE_STATE_ADV_STOP,
	BLE_STATE_CONN_OPEN,
	BLE_STATE_CONN_CLOSE,

	BLE_STATE_MAX
} ble_state_t;

/*! \var typedef svc_cv_t
 * \brief A type defined for callback function
 *
 * \param event type - Which callback type it is
 * \param pValue	 - pValue memory address
 * \param len		 - length of the data
 */
typedef void (*svc_cv_t)(svc_cb_type_t event, uint8_t * pValue, uint16_t len);

/*! \var typedef ble_state_cb_t
 * \brief A type defined for callback function for BLE GAP states
 *
 * \param state type - Which states the BLE MCU is at
 */
typedef void (*ble_state_cb_t)(ble_state_t state);

/*************************************************************************************************/
/*!
 *  \brief  Initialize all the services.
 *
 *  \return None.
 */
/*************************************************************************************************/
void ble_init();

/*************************************************************************************************/
/*!
 *  \brief  Register a callback for MRD104 data exchange services.
 *	\detail If callback type is read, pValue and len parameters are useless. Otherwise,
 *			they keep the memory address and length of the received data.
 *  \return None.
 */
/*************************************************************************************************/
void ble_service_callback(svc_cv_t svc_callback);

/*************************************************************************************************/
/*!
 *  \brief  Register a callback for MRD104 GAP states
 *	\detail It passes the BLE Gap state as a parameter
 *  \return None.
 */
/*************************************************************************************************/
void ble_state_callback(ble_state_cb_t state_callback);

/*************************************************************************************************/
/*!
 *  \brief  Sending notification to the GATT client
 *	\detail
 *  \return None.
 */
/*************************************************************************************************/
void ble_send_notify(uint8_t * p_data, uint16_t len);

void ble_send_data(uint8_t * p_data, uint16_t len);

/*************************************************************************************************/
/*!
 *  \brief  Starting BLE advertisement
 *	\detail
 *  \return None.
 */
/*************************************************************************************************/
void ble_start_adv();

/*************************************************************************************************/
/*!
 *  \brief  Stoping BLE advertisement
 *	\detail
 *  \return None.
 */
/*************************************************************************************************/
void ble_stop_adv();

void ble_connection_close();

ble_state_t ble_get_state();

/******************************************************************************************/
//BLE API Main Functions are moved here

#define DISPATCH_BLE(iter)	for(int ble_dispatch = 0; ble_dispatch < iter; ble_dispatch++){wsfOsDispatcher();}


/* Size of buffer for stdio functions */
#define WSF_BUF_POOLS       6
#define WSF_BUF_SIZE        0x1048

#define NUM_PACKS_PER_CHUNK	7


// void BLE_Timer_Handler();
// void BLE_Timer_Init();
// void BLE_Timer_Enable(uint8_t en);
void WsfInit(void);
void SetAddress(uint8_t event);
void callback(svc_cb_type_t type, uint8_t * p_value, uint16_t len);
void ble_state_cb(ble_state_t state);
#endif /* DRIVERS_BLE_API_H_ */
