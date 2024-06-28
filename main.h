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

#ifndef _MAIN_H_
#define _MAIN_H_


/*******************************      INCLUDES    ****************************/
#include "max32665.h"

/*******************************      DEFINES     ****************************/


#define EVT_TEMP_SENSOR_SAMPLE			(1u << 0u)

#define EVT_BLE_CONN_OPEN				(1u << 1u)
#define EVT_BLE_CONN_CLOSE				(1u << 2u)

#define EVT_BUTTON_SINGLE_PRESS			(1u << 3u)
#define EVT_BUTTON_DOUBLE_PRESS			(1u << 4u)
#define EVT_BUTTON_TRIPLE_PRESS			(1u << 5u)
#define EVT_BUTTON_LONG_PRESS			(1u << 6u)

#define EVT_STATUS_LED_ON_OFF			(1u << 7u)

#define EVT_FLASH_LOGGING_STARTED		(1u << 8u)
#define EVT_FLASH_LOGGING_FINISHED		(1u << 9u)

#define EVT_MEASUREMENT_START			(1u << 10u)
#define EVT_MEASUREMENT_STOP			(1u << 11u)

#define EVT_SENSORHUB_OUT_FIFO_READ		(1u << 12u)

#define EVT_BLE_WRITE_CB				(1u << 13u)
#define EVT_BLE_READ_CB					(1u << 14u)

#define EVT_USB_WRITE_CB				(1u << 15u)

#define EVT_TEMP_MEASUREMENT_NOTIFY 	(1u << 16u)
#define EVT_TEMP_SENSOR_UPDATE 			(1u << 17u)
#define EVT_RTC_UPDATE					(1u << 18u)
#define EVT_SPO2_APP_SETTINGS_UPDATE	(1u << 19u)

#define EVT_LOC_FIND_STARTED            (1u << 20u)
#define EVT_LOC_FIND_IN_PROGRESS        (1u << 21u)
#define EVT_LOC_FIND_STOPPED            (1u << 22u)

/*SpO2 State Machine*/
enum
{
	SPO2_SM_NOT_INITIALIZED,
	SPO2_SM_ACTIVE,
	SPO2_SM_WAITING,
	SPO2_SM_DISABLED,
};

extern uint8_t spo2_meas_selected_pd;

void app_main_evt_post(unsigned int evt);
int spo2_state_machine(uint8_t restart);

#endif /* _MAIN_H_ */
