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
#include "mxc_delay.h"
#include "sensorhub.h"

#include <stddef.h>
#include <float.h>
#include "max30210.h"
#include "max30210_platform.h"
#include "max30210_registers.h"

#ifndef UNUSED_PARAM
#   define UNUSED_PARAM(x) (void)(x)
#endif  /* UNUSED_PARAM */

static int max30210_init(void);
static int max30210_dump_reg(uint8_t * buf, int size);
static int max30210_read_reg(uint8_t * buf, int size);
static int max30210_write_reg(uint8_t * reg_addr, uint8_t * val);
static int max30210_enable(int mode);
static int max30210_sync_data(void);
static int max30210_execute_once(void * params);
static int max30210_pop_data(sensor_data_t ** sensor_data);

#ifdef TEMP_RAW_DATA
#define MAX30210_RAW_DATA_SIZE	2
static uint8_t max30210_temp[MAX30210_RAW_DATA_SIZE] = {0};
#else
static float max30210_temp = 0.0;
#endif

extern sensor_t max30210;

sensor_t max30210 = {
    .num_dump_regs = 36,

    .init         = max30210_init,
    .dump_reg     = max30210_dump_reg,
    .read_reg     = max30210_read_reg,
    .write_reg    = max30210_write_reg,
    .enable       = max30210_enable,
    .sync_data    = max30210_sync_data,
    .execute_once = max30210_execute_once,
    .pop_data     = max30210_pop_data,

	.num_items  = 0,
    .report_buf = {
        .buf = (uint8_t *)&max30210_temp
    }
};

static int max30210_init(void)
{
    int result = max30210_hal_init();

    if (E_NO_ERROR == result){
    	max30210_sanity_check(MAX30210_I2C_ADDR1);
    }

    if (E_NO_ERROR == result)
    {
        result = max30210_sync_data();
    }

    return result;
}

static int max30210_dump_reg(uint8_t * buf, int size)
{
    static int const     ENTRY_COUNT = 7;
    static uint8_t const reg_table[][2] = {{0x00, 0x00},
    									   {0x02, 0x02},
                                           {0x04, 0x0A},
                                           {0x11, 0x12},
                                           {0x20, 0x2E},
                                           {0x30, 0x35},
                                           {0xFF, 0xFF}};

    int  result  = E_NO_ERROR;
    bool read_ok = true;

    uint8_t reg;
    uint8_t val;
    uint8_t end;

    int addr;
    int regval;

    int entry = 0;
    int err   = 0;
    int iter  = 0;

    if (size != max30210.num_dump_regs)
    {
        result  = E_BAD_PARAM;
        read_ok = false;
    }

    while (true == read_ok && entry < ENTRY_COUNT)
    {
        reg = reg_table[entry][0];
        end = reg_table[entry][1];

        while (true == read_ok && reg < end)
        {
            err = max30210_platform_read_register(MAX30210_I2C_ADDR1, reg, &val, 1);

            if (err !=  E_NO_ERROR)
            {
                read_ok = false;
            }
            else
            {
                addr   = 2 * iter;
                regval = 2 * iter + 1;

                buf[addr]   = reg;
                buf[regval] = val;

                iter++;
                reg++;
            }
        }

        entry++;
    }

    return result;
}

static int max30210_read_reg(uint8_t * reg_addr, int len)
{
    int  retval;
    bool read_ok = true;
    int  idx     = 0;

    /* OT07 does not support auto increment register address. */
    /* Because of that, we are doing in the code. */
    uint8_t base_reg_addr = 0;

    if (NULL == reg_addr)
    {
        retval  = E_NULL_PTR;
        read_ok = false;
    }

    if (len <= 0)
    {
        retval  = E_BAD_PARAM;
        read_ok = false;
    }

    if(true == read_ok)
    {
    	base_reg_addr = *reg_addr;
    }

    while (true == read_ok && idx < len)
    {
        retval = max30210_platform_read_register(MAX30210_I2C_ADDR1, reg_addr[idx], &reg_addr[idx], 1);

        if (retval != E_NO_ERROR)
        {
            read_ok = false;
        }

        idx++;
        base_reg_addr++;

        reg_addr[idx] = base_reg_addr;
    }


    return retval;
}

static int max30210_write_reg(uint8_t * reg_addr, uint8_t * val)
{
    int result = max30210_platform_write_register(MAX30210_I2C_ADDR1,*reg_addr, *val);
    return result;
}

static int max30210_enable(int mode)
{
    UNUSED_PARAM(mode);
    return E_NO_ERROR;
}

static int max30210_sync_data(void)
{
    int result;
    uint8_t sync_config = MAX30210_BIT_FIFO_RO | MAX30210_BIT_FIFO_STAT_CLR;

    max30210_flush_fifo(MAX30210_I2C_ADDR1);
    result = max30210_platform_write_register(MAX30210_I2C_ADDR1,
    										  MAX30210_REG_FIFO_CONFIGURATION_2,
                                              sync_config);

    return result;
}

static int max30210_execute_once(void * params)
{
    int      result;
    uint8_t  regval;
    uint16_t fifo_val;

    UNUSED_PARAM(params);

    max30210_platform_read_register(MAX30210_I2C_ADDR1, MAX30210_REG_FIFO_DATA_COUNTER, &regval, 1);

    if (0 != regval)
    {
        fifo_val = max30210_read_fifo(MAX30210_I2C_ADDR1);

        if (UINT16_MAX == fifo_val)
        {
            result = E_COMM_ERR;
        }
        else
        {
#ifdef TEMP_RAW_DATA
        	memcpy(&max30210.report_buf.buf[0], &fifo_val, MAX30210_RAW_DATA_SIZE);
#else
        	*(float *)(max30210.report_buf.buf) = fifo_val * 0.005;
#endif
            result = E_SUCCESS;
        }
    }
    else
    {
        max30210_platform_read_register(MAX30210_I2C_ADDR1,MAX30210_REG_TEMP_CONV, &regval, 1);

        if (0 == (regval & MAX30210_BIT_CONVERT_T))
        {
            regval |= MAX30210_BIT_CONVERT_T;
            max30210_platform_write_register(MAX30210_I2C_ADDR1, MAX30210_REG_TEMP_CONV, regval);
        }

        result = E_BUSY;
    }

    return result;
}

static int max30210_pop_data(sensor_data_t ** sensor_data)
{
    int      result;
    uint8_t  regval;
    uint16_t fifo_val;

    UNUSED_PARAM(sensor_data);

    max30210_platform_read_register(MAX30210_I2C_ADDR1, MAX30210_REG_FIFO_DATA_COUNTER, &regval, 1);

    if (0 == regval)
    {
        result = E_NONE_AVAIL;
    }
    else
    {
        fifo_val = max30210_read_fifo(MAX30210_I2C_ADDR1);

        if (UINT16_MAX == fifo_val)
        {
            result = E_COMM_ERR;
        }
        else
        {
#ifdef TEMP_RAW_DATA
        	memcpy(&max30210.report_buf.buf[0], &fifo_val, MAX30210_RAW_DATA_SIZE);
#else
        	*(float *)(max30210.report_buf.buf) = fifo_val * 0.005;
#endif
            result = E_SUCCESS;
        }
    }

    return result;
}
