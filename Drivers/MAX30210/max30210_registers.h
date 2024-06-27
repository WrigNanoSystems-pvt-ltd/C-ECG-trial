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
#ifndef MAX30210_REGISTERS_H
#define MAX30210_REGISTERS_H

#include <stdint.h>

#define MAX30210_DEFAULT_I2C_ADDR UINT8_C(0x80)

#define MAX30210_REG_STATUS                UINT8_C(0x00)
#define MAX30210_REG_INTERRUPT_ENABLE      UINT8_C(0x02)

#define MAX30210_REG_FIFO_WRITE_POINTER    UINT8_C(0x04)
#define MAX30210_REG_FIFO_READ_POINTER     UINT8_C(0x05)
#define MAX30210_REG_FIFO_OVERFLOW_COUNTER UINT8_C(0x06)
#define MAX30210_REG_FIFO_DATA_COUNTER     UINT8_C(0x07)
#define MAX30210_REG_FIFO_DATA             UINT8_C(0x08)
#define MAX30210_REG_FIFO_CONFIGURATION_1  UINT8_C(0x09)
#define MAX30210_REG_FIFO_CONFIGURATION_2  UINT8_C(0x0A)
	#define MAX30210_BIT_TEMP_RDY      UINT8_C(1 << 0)
	#define MAX30210_BIT_FIFO_RO       UINT8_C(1 << 1)
	#define MAX30210_BIT_FIFO_STAT_CLR UINT8_C(1 << 3)
	#define MAX30210_BIT_FLUSH_FIFO    UINT8_C(1 << 4)

#define MAX30210_REG_SYSTEM_CONF           UINT8_C(0x11)
#define MAX30210_REG_PIN_CONF              UINT8_C(0x12)

#define MAX30210_REG_ALARM_HIGH_SETUP      UINT8_C(0x20)
#define MAX30210_REG_ALARM_LOW_SETUP       UINT8_C(0x21)
#define MAX30210_REG_ALARM_HIGH_MSB        UINT8_C(0x22)
#define MAX30210_REG_ALARM_HIGH_MSB        UINT8_C(0x22)
#define MAX30210_REG_ALARM_HIGH_LSB        UINT8_C(0x23)
#define MAX30210_REG_ALARM_LOW_MSB         UINT8_C(0x24)
#define MAX30210_REG_ALARM_LOW_LSB         UINT8_C(0x25)
#define MAX30210_REG_INC_FAST_THRESH       UINT8_C(0x26)
#define MAX30210_REG_DEC_FAST_THRESH       UINT8_C(0x27)
#define MAX30210_REG_ALARM_LOW_LSB         UINT8_C(0x25)
#define MAX30210_REG_TEMP_CONF1			   UINT8_C(0x28)
#define MAX30210_REG_TEMP_CONF2			   UINT8_C(0x29)

#define MAX30210_REG_TEMP_CONV			   UINT8_C(0x2A)
	#define MAX30210_BIT_CONVERT_T     UINT8_C(1 << 0)
	#define MAX30210_BIT_CONVERT_AUTO  UINT8_C(1 << 1)

#define MAX30210_REG_TEMP_DATA_MSB		   UINT8_C(0x2B)
#define MAX30210_REG_TEMP_DATA_LSB		   UINT8_C(0x2C)
#define MAX30210_REG_TEMP_SLOPE_MSB		   UINT8_C(0x2D)
#define MAX30210_REG_TEMP_SLOPE_LSB		   UINT8_C(0x2E)

#define MAX30210_REG_PART_ID_1             UINT8_C(0x30)
#define MAX30210_REG_PART_ID_2             UINT8_C(0x31)
#define MAX30210_REG_PART_ID_3             UINT8_C(0x32)
#define MAX30210_REG_PART_ID_4             UINT8_C(0x33)
#define MAX30210_REG_PART_ID_5             UINT8_C(0x34)
#define MAX30210_REG_PART_ID_6             UINT8_C(0x35)
#define MAX30210_REG_IDENTIFIER            UINT8_C(0xFF)

#endif  /* MAX30210_REGISTERS_H */
