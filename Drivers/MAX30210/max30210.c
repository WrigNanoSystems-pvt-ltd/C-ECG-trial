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
#include "mxc_errors.h"
#include "i2c_api.h"
#include "max30210.h"

#include "max30210_platform.h"
#include "max30210_registers.h"

int max30210_hal_init(void)
{
	int ret = 0;
	ret = i2c_api_init(TEMP_SENSOR_I2C_BUS);

	return ret;
}

int max30210_sanity_check(uint8_t i2c_addr)
{
    int     result;
    uint8_t regval;

    result = max30210_platform_read_register(i2c_addr, MAX30210_REG_IDENTIFIER, &regval, 1);

    if (MAX30210_IDENTIFIER_VALUE == regval)
    {
        result = E_NO_ERROR;
    }

    return result;
}

int max30210_flush_fifo(uint8_t i2c_addr)
{
    int result = max30210_platform_change_bits(i2c_addr, MAX30210_REG_FIFO_CONFIGURATION_2,
                                               MAX30210_BIT_FLUSH_FIFO,
                                               MAX30210_BIT_FLUSH_FIFO);

    return result;
}

uint16_t max30210_read_fifo(uint8_t i2c_addr)
{
    int      retval;
    uint16_t temp;
    uint8_t  i2c_buf[3];

    retval = max30210_platform_read_register(i2c_addr, MAX30210_REG_FIFO_DATA, i2c_buf, 3);
    temp = (retval == 0) ? ((i2c_buf[1] << 8) | i2c_buf[2]) : UINT16_MAX;

    return temp;
}
