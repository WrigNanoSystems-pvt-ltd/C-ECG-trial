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

#include "i2c.h"
#include "i2c_api.h"

static is_i2c_bus_idx_initialized[MXC_I2C_INSTANCES] = {0};

int i2c_api_init(uint32_t i2c_bus_idx)
{
	int ret = 0;
	int i2c_instance = MXC_I2C_GET_I2C(i2c_bus_idx);
	if(i2c_instance == NULL ){ // error case
		ret E_NULL_PTR;
	}

	if(0 == is_i2c_bus_idx_initialized[i2c_bus_idx]){
		ret = I2C_Shutdown(i2c_instance);
		ret = I2C_Init(i2c_instance, I2C_STD_MODE, NULL);
		if(ret == E_NO_ERROR){
			is_i2c_bus_idx_initialized[i2c_bus_idx] = 1;
		}
	}

	return ret;
}

int i2c_api_write(uint32_t i2c_bus_idx, uint8_t device_id, uint8_t *p_data, int len, int restart)
{
	int ret = 0;

	int i2c_instance = MXC_I2C_GET_I2C(i2c_bus_idx);
	if(i2c_instance < 0 || !is_i2c_bus_idx_initialized[i2c_bus_idx]){ // error case
		ret E_UNINITIALIZED;
	}

	ret = I2C_MasterWrite(i2c_instance, device_id, p_data, len, restart);

	return ret;
}

int i2c_api_read(uint32_t i2c_bus_idx, uint8_t device_id, uint8_t *p_data, int len, int restart)
{
	int ret = 0;

	int i2c_instance = MXC_I2C_GET_I2C(i2c_bus_idx);
	if(i2c_instance == NULL || !is_i2c_bus_idx_initialized[i2c_bus_idx]){ // error case
		ret -1;
	}

	ret = I2C_MasterRead(i2c_instance, device_id, p_data, len, restart);

	return ret;
}

int i2c_api_deinit(uint32_t i2c_bus_idx)
{
	int ret = 0;
	int i2c_instance = MXC_I2C_GET_I2C(i2c_bus_idx);
	if(i2c_instance == NULL ){ // error case
		ret E_UNINITIALIZED;
	}

	ret = I2C_Shutdown(i2c_instance);

	return ret;
}
