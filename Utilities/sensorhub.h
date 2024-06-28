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

#ifndef DRIVERS_SENSORHUB_SENSORHUB_H_
#define DRIVERS_SENSORHUB_SENSORHUB_H_


#include "sh_defs.h"

// Sensor/Algo indicies
typedef enum {
	SH_ACC_SENSOR			=	0x00,
	SH_MAX86178,
	SH_TEMP_SENSOR,
	SH_NUM_SENSORS
} sensor_idx_t;

typedef enum {
	SH_NUM_ALGOS,
} algo_idx_t;

sensor_t *mxm_sh_get_sensor_instance(int sid);

void mxm_sh_sync_active_sensors(void);

int mxm_sh_get_num_sensors(void);

int mxm_sh_init();

uint8_t mxm_sh_sensor_enable(uint8_t sensor_idx, uint8_t mode, uint8_t ext_data_mode);

void mxm_sh_execute_once(void);


#endif /* DRIVERS_SENSORHUB_SENSORHUB_H_ */
