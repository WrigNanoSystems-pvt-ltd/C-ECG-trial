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
#ifndef MAX30210_PLATFORM_H
#define MAX30210_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#if defined(FTHR) || defined(FTHR2)
#define TEMP_SENSOR_I2C_BUS 0
#elif MRD106
#define TEMP_SENSOR_I2C_BUS 1
#elif MRD104
#define TEMP_SENSOR_I2C_BUS 1
#endif

#ifndef MAX30210_I2C_ADDR1
#define MAX30210_I2C_ADDR1 UINT8_C(0x80)
#endif

#ifndef MAX30210_I2C_ADDR2
#define MAX30210_I2C_ADDR2 UINT8_C(0x82)
#endif

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

int max30210_platform_read_register(uint8_t i2c_address, uint8_t reg, uint8_t * buf, int buf_len);
int max30210_platform_write_registers(uint8_t i2c_address, uint8_t reg, uint8_t const * data, int data_len);
int max30210_platform_write_register(uint8_t i2c_address, uint8_t reg, uint8_t data);
int max30210_platform_change_bits(uint8_t i2c_address, uint8_t reg, uint8_t mask, uint8_t bits);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* MAX30210_PLATFORM_H */
