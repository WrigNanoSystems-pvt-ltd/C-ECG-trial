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

#ifndef DRIVERS_I2C_API_I2C_API_H_
#define DRIVERS_I2C_API_I2C_API_H_

#include <stdint.h>

#if defined(FTHR) || defined(FTHR2)
#warning I2C API MAX3266FTHR Defined
	#define MRD104_I2C_BUS MXC_I2C0_BUS0
	#define SECONDARY_I2C_BUS MXC_I2C0_BUS0
#elif MRD104
#warning I2C API MRD104 Board Defined
	#define MRD104_I2C_BUS MXC_I2C0_BUS0
	#define SECONDARY_I2C_BUS MXC_I2C1_BUS0
#elif MRD106
	#warning I2C API MRD106 Board Defined
	#define MRD104_I2C_BUS MXC_I2C0_BUS0
	#define SECONDARY_I2C_BUS MXC_I2C1_BUS0
#endif

int i2c_api_init(uint32_t i2c_bus_idx);

int i2c_api_write(uint32_t i2c_bus_idx, uint8_t device_id, uint8_t *p_data, int len, int restart);

int i2c_api_read(uint32_t i2c_bus_idx, uint8_t device_id, uint8_t *p_data, int len, int restart);

int i2c_api_deinit(uint32_t i2c_bus_idx);

#endif /* DRIVERS_I2C_API_I2C_API_H_ */
