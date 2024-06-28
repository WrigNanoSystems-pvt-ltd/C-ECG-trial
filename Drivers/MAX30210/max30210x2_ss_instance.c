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

static int max30210x2_init(void);
static int max30210x2_dump_reg(uint8_t * buf, int size);
static int max30210x2_read_reg(uint8_t * buf, int size);
static int max30210x2_write_reg(uint8_t * reg_addr, uint8_t * val);
static int max30210x2_enable(int mode);
static int max30210x2_sync_data(void);
static int max30210x2_execute_once(void * params);
static int max30210x2_pop_data(sensor_data_t ** sensor_data);
static int max30210x2_get_config(void *data, uint8_t params, uint8_t *tx_buf, int tx_buf_sz);
static int max30210x2_set_config(void *data, uint8_t *params, int param_sz);

#define NUM_OF_TEMP_SENSORS 2

#ifdef TEMP_RAW_DATA
#define MAX30210_RAW_DATA_SIZE	2
static uint8_t max30210x2_temp[NUM_OF_TEMP_SENSORS][MAX30210_RAW_DATA_SIZE] = {0};
#else
static float max30210x2_temp[NUM_OF_TEMP_SENSORS] = 0.0;
#endif

static uint8_t temp_sensor_i2c_address_list[] = {MAX30210_I2C_ADDR1, MAX30210_I2C_ADDR2};


extern sensor_t max30210x2;

sensor_t max30210x2 = {
    .num_dump_regs = 36 * NUM_OF_TEMP_SENSORS,

    .init         = max30210x2_init,
    .dump_reg     = max30210x2_dump_reg,
    .read_reg     = max30210x2_read_reg,
    .write_reg    = max30210x2_write_reg,
    .enable       = max30210x2_enable,
    .sync_data    = max30210x2_sync_data,
    .execute_once = max30210x2_execute_once,
    .pop_data     = max30210x2_pop_data,
	.get_cfg 	  = max30210x2_get_config,
	.set_cfg 	  = max30210x2_set_config,

    .report_buf = {
        .buf = (uint8_t *)&max30210x2_temp
    }
};

static int max30210x2_init(void)
{
    int result = max30210_hal_init();



    for(int i = 0; i< NUM_OF_TEMP_SENSORS; i++){
    	if (E_NO_ERROR == result){
			result = max30210_sanity_check(temp_sensor_i2c_address_list[i]);
		}
    	if(E_NO_ERROR != result){
    		break;
    	}
    }

    if (E_NO_ERROR == result)
    {
        result = max30210x2_sync_data();
    }


    return result;
}

static int max30210x2_dump_reg(uint8_t * buf, int size)
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

    if (size < max30210x2.num_dump_regs*2)
    {
        result  = E_BAD_PARAM;
        read_ok = false;
    }


    for(int i = 0; i< NUM_OF_TEMP_SENSORS; i++){
		int entry = 0;

		while (true == read_ok && entry < ENTRY_COUNT)
		{
			reg = reg_table[entry][0];
			end = reg_table[entry][1];

			while (true == read_ok && reg < end)
			{
				err = max30210_platform_read_register(temp_sensor_i2c_address_list[i], reg, &val, 1);

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





		if(E_NO_ERROR != err){
			break;
		}
	}




    return err;
}

static int max30210x2_read_reg(uint8_t * reg_addr, int len)
{
    int  retval;
    bool read_ok = true;

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


    for(int i = 0; i< NUM_OF_TEMP_SENSORS; i++){

    	base_reg_addr = *reg_addr;
        int  idx     = 0;


		while (true == read_ok && idx < len)
		{
			retval = max30210_platform_read_register(temp_sensor_i2c_address_list[i], reg_addr[idx], &reg_addr[idx], 1);

			if (retval < 0)
			{
				read_ok = false;
			}

			idx++;
			base_reg_addr++;

			reg_addr[idx] = base_reg_addr;
		}
    }

    return (retval >= 0);
}

static int max30210x2_write_reg(uint8_t * reg_addr, uint8_t * val)
{
    bool result = max30210_platform_write_register(MAX30210_I2C_ADDR1,*reg_addr, *val);
    return (result ? 0 : -1);
}

static int max30210x2_enable(int mode)
{
    UNUSED_PARAM(mode);
    return E_NO_ERROR;
}

static int max30210x2_sync_data(void)
{
    int result;
    uint8_t sync_config = MAX30210_BIT_FIFO_RO | MAX30210_BIT_FIFO_STAT_CLR;


    for(int i = 0; i< NUM_OF_TEMP_SENSORS; i++){

    	max30210_flush_fifo(temp_sensor_i2c_address_list[i]);
		result = max30210_platform_write_register(temp_sensor_i2c_address_list[i],
												  MAX30210_REG_FIFO_CONFIGURATION_2,
												  sync_config);

    	if(E_NO_ERROR != result){
    		break;
    	}
    }
    return result;
}

static int max30210x2_execute_once(void * params)
{
    int      result[NUM_OF_TEMP_SENSORS];
    uint8_t  regval;
    uint16_t fifo_val;

    UNUSED_PARAM(params);

    for(int i = 0; i< NUM_OF_TEMP_SENSORS; i++){

		max30210_platform_read_register(temp_sensor_i2c_address_list[i], MAX30210_REG_FIFO_DATA_COUNTER, &regval, 1);

		if (0 != regval)
		{
			fifo_val = max30210_read_fifo(temp_sensor_i2c_address_list[i]);

			if (UINT16_MAX == fifo_val)
			{
				result[i] = E_COMM_ERR;
			}
			else
			{
	#ifdef TEMP_RAW_DATA
				memcpy(&max30210x2.report_buf.buf[i*MAX30210_RAW_DATA_SIZE], &fifo_val, MAX30210_RAW_DATA_SIZE);
	#else
				*(float *)(max30210x2.report_buf.buf[i]) = fifo_val * 0.005;
	#endif
				max30210x2.num_items |= (1<<i);
				result[i] = E_SUCCESS;
			}

		}
		else
		{
			max30210_platform_read_register(temp_sensor_i2c_address_list[i],MAX30210_REG_TEMP_CONV, &regval, 1);

			if (0 == (regval & MAX30210_BIT_CONVERT_T))
			{
				regval |= MAX30210_BIT_CONVERT_T;
				max30210_platform_write_register(temp_sensor_i2c_address_list[i], MAX30210_REG_TEMP_CONV, regval);
			}
			max30210x2.num_items &= !(1<<i);

			result[i] = E_BUSY;
		}


	}

    for(int i = 0; i< NUM_OF_TEMP_SENSORS; i++){

		if(E_NO_ERROR != result[i]){
			return E_COMM_ERR;
		}
    }

    return E_NO_ERROR;
}

static int max30210x2_pop_data(sensor_data_t ** sensor_data)
{
    int      result;
    uint8_t  regval;
    uint16_t fifo_val;

    UNUSED_PARAM(sensor_data);



    for(int i = 0; i< NUM_OF_TEMP_SENSORS; i++){

    	max30210_platform_read_register(temp_sensor_i2c_address_list[i], MAX30210_REG_FIFO_DATA_COUNTER, &regval, 1);

		if (0 == regval)
		{
			result = E_NONE_AVAIL;
		}
		else
		{
			fifo_val = max30210_read_fifo(temp_sensor_i2c_address_list[i]);

			if (UINT16_MAX == fifo_val)
			{
				result = E_COMM_ERR;
			}
			else
			{
	#ifdef TEMP_RAW_DATA
				memcpy(&max30210x2.report_buf.buf[i*MAX30210_RAW_DATA_SIZE], &fifo_val, MAX30210_RAW_DATA_SIZE);
	#else
				*(float *)(max30210x2.report_buf.buf[i]) = fifo_val * 0.005;
	#endif
				result = E_SUCCESS;
			}
		}


		if(E_NO_ERROR != result){
			break;
		}
	}




    return result;
}

/**data: the pointer for the array that contains the register addresses those will be read
 * params : the # of the sensor
 * tx_buf: read values from sensor
 * tx_buffer_size: the length of registers to be read
 */
static int max30210x2_get_config(void *data, uint8_t params, uint8_t *tx_buf, int tx_buf_sz)
{
    int      result;
    uint8_t  regval;
    uint16_t fifo_val;


    if(params >= NUM_OF_TEMP_SENSORS){
		return E_BAD_PARAM;
    }

    for(int idx = 0; idx< tx_buf_sz; idx++){

    	result = max30210_platform_read_register(temp_sensor_i2c_address_list[params], ((uint8_t*) data)[idx], &tx_buf[idx], 1);

    	if(E_NO_ERROR != result){
    		E_COMM_ERR;
		}
    }



    return result;
}

/**data: the pointer for the array that contains the register addresses those will be written
 * params[0] : the # of the sensor
 * *params[1] : the pointer to the data that will be written
 *
 * param_sz: the length of registers to be written
 */
static int max30210x2_set_config(void *data, uint8_t *params, int param_sz){
    int      result;
    uint8_t  regval;
    uint16_t fifo_val;

    uint8_t * local_data = (uint8_t *)data;

    if(params[0] >= NUM_OF_TEMP_SENSORS){
		return E_BAD_PARAM;
    }

    uint8_t selected_i2c_address =  temp_sensor_i2c_address_list[params[0]];

    for(int idx = 0; idx< param_sz; idx++){

    	result = max30210_platform_write_register(selected_i2c_address, local_data[idx], params[idx+1]);

    	if(E_NO_ERROR != result){
    		E_COMM_ERR;
		}
    }

    return result;
}

