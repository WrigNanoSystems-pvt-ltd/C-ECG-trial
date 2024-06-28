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
#include "i2c.h"
#include "i2c_api.h"
#include <stddef.h>
#include "max30210_platform.h"



int max30210_platform_init()
{
	return i2c_api_init(TEMP_SENSOR_I2C_BUS);
}

int max30210_platform_read_register(uint8_t i2c_address, uint8_t reg, uint8_t * buf, int buf_len)
{
    int err = E_NO_ERROR;

    if (NULL == buf)
    {
        err = E_NULL_PTR;
    }

    if (buf_len <= 0)
    {
        err = E_BAD_PARAM;
    }

    if (E_NO_ERROR == err)
    {
    	//TODO: change i2c calls with i2c api
        err = 	i2c_api_write(TEMP_SENSOR_I2C_BUS, i2c_address, &reg, sizeof(uint8_t), 1);
    }

    if (err > 0)
    {
        err = i2c_api_read(TEMP_SENSOR_I2C_BUS, i2c_address, buf, buf_len, 0);
    }

    if(err == buf_len){
		return E_NO_ERROR;
	}else{
		return E_COMM_ERR;
	}
}

int max30210_platform_write_registers(uint8_t i2c_address, uint8_t reg, uint8_t const * data, int data_len)
{
    int  retval;
    bool write_ok = true;
    int  idx      = 0;

    while (true == write_ok && idx < data_len)
    {
        retval = max30210_platform_write_register(i2c_address,reg + idx, *(data + idx));

        if (retval != E_NO_ERROR)
        {
            write_ok = false;
        }

        idx++;
    }

    return retval;
}

int max30210_platform_write_register(uint8_t i2c_address, uint8_t reg, uint8_t data)
{
    int     retval;
    uint8_t i2c_buf[] = {reg, data};

    retval = i2c_api_write(TEMP_SENSOR_I2C_BUS, i2c_address, i2c_buf, sizeof(i2c_buf), 0);

    if(retval == sizeof(i2c_buf)){
    	return E_NO_ERROR;
    }else{
    	return E_COMM_ERR;
    }
}

int max30210_platform_change_bits(uint8_t i2c_address, uint8_t reg, uint8_t mask, uint8_t bits)
{
    uint8_t data;
    int     retval;
    bool    change_ok;

    retval = max30210_platform_read_register(i2c_address,reg, &data, sizeof(uint8_t));

    if (E_NO_ERROR != retval)
    {
        change_ok = false;
    }
    else
    {
        data = data & ~mask;
        data = data | (bits & mask);

        change_ok = max30210_platform_write_register(i2c_address, reg, data);
    }

    return change_ok;
}
