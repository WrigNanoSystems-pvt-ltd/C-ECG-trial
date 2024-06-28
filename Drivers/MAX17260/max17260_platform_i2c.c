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
#include "max17260_platform.h"
#include "i2c.h"
#include "i2c_api.h"


#include <stddef.h>

#if defined(FTHR) || defined(FTHR2)
#define MAX17260_I2C_BUS 0
#elif MRD106
#define MAX17260_I2C_BUS 1
#elif MRD104
#define MAX17260_I2C_BUS 1
#endif
/****************************************************************************/
/*  GLOBAL FUNCTION DEFINITIONS                                             */
/****************************************************************************/

int max17260_platform_read_register(uint8_t reg, uint16_t* buf)
{
    int     err  = E_NO_ERROR;
    uint8_t i2c_buf[2];

    if (NULL == buf)
    {
        err = E_BAD_PARAM;
    }

    if (E_NO_ERROR == err)
    {
        err = i2c_api_write(MAX17260_I2C_BUS, MAX17260_I2C_ADDR, &reg, 1, 1);
    }

    if (err > 0)
    {
        err = i2c_api_read(MAX17260_I2C_BUS,  \
                             MAX17260_I2C_ADDR, \
                             i2c_buf,           \
                             sizeof(i2c_buf),   \
                             0);
    }

    if (err > 0)
    {
        *buf = (i2c_buf[1] << 8) | i2c_buf[0];
    }

    return err;
}

bool max17260_platform_write_register(uint8_t reg, uint16_t data)
{
    int     retval;
    uint8_t lsb = (data & 0x00FF);
    uint8_t msb = (data & 0xFF00) >> 8;

    uint8_t i2c_buf[] = {reg, lsb, msb};

    retval = i2c_api_write(MAX17260_I2C_BUS,  \
                             MAX17260_I2C_ADDR, \
                             i2c_buf,           \
                             sizeof(i2c_buf),   \
                             0);

    return (retval == sizeof(i2c_buf));
}

bool max17260_platform_change_bits(uint8_t reg, uint16_t mask, uint16_t bits)
{
    int      retval;
    bool     change_ok;
    uint16_t data;

    retval = max17260_platform_read_register(reg, &data);

    if (sizeof(data) != retval)
    {
        change_ok = false;
    }
    else
    {
        data &= ~mask;
        data |= (bits & mask);

        change_ok = max17260_platform_write_register(reg, data);
    }

    return change_ok;
}

