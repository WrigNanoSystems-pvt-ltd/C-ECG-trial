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
#include "i2c_api.h"

#include <stddef.h>

#include "max20356_platform.h"
#include "mxc_errors.h"
#include "i2c_api.h"
//#include "adc.h"

#define MAX20356_ADC_CH MXC_ADC_CH_0
#define MAX20356_ADC_MON MXC_ADC_MONITOR_0
//#define MAX20356_SECURE_WRITES_ENABLED


static int get_right_most_set_bit(uint32_t num)
{
  if(num==0)
  {  return 0;
  }
  else
  {
    int pos = 0;
    for (int i = 0; i < 32; i++) {
        if (!(num & (1 << i)))
            pos++;
        else
            break;
    }
    return pos;
  }
}

/****************************************************************************/
/*  GLOBAL FUNCTION DEFINITIONS                                             */
/****************************************************************************/

int max20356_platform_read_register(max20356_driver_t driver, \
                                    uint8_t reg,              \
                                    uint8_t* buf,             \
                                    int buf_len)
{
    int     err = E_NO_ERROR;
    int ret_val = 0;
    uint8_t addr;

    if (NULL == buf || buf_len <= 0)
    {
        err = E_BAD_PARAM;
    }

    switch (driver)
    {
        case MAX20356_DRIVER_ADC:
            addr = MAX20356_ADC_I2C_ADDR;
            break;
        case MAX20356_DRIVER_PMIC:
            addr = MAX20356_PMIC_I2C_ADDR;
            break;
        case MAX20356_DRIVER_FUEL_GAUGE:
            addr = MAX20356_FUEL_GAUGE_I2C_ADDR;
            break;
        default:
            err = E_BAD_PARAM;
    }

    int data_w_length = 1;
    if (E_NO_ERROR == err)
    {
    	ret_val = i2c_api_write(MAX20356_I2C_BUS, addr, &reg, data_w_length, 0);
    }

    if (ret_val == data_w_length)
    {
    	ret_val = i2c_api_read(MAX20356_I2C_BUS, addr, buf, buf_len, 0);
    }else{
    	return E_COMM_ERR;
    }

    if(ret_val == buf_len){
    	err = E_NO_ERROR;
    }else{
    	err = E_COMM_ERR;
    }

    return err;
}

bool max20356_platform_write_registers(max20356_driver_t driver, \
                                       uint8_t reg,              \
                                       uint8_t const* data,      \
                                       int data_len)
{
    int  retval   = 0;
    bool write_ok = true;
    int  iter     = 0;

    if (NULL == data || data_len <= 0)
    {
        write_ok = false;
    }

    while (true == write_ok && iter < data_len)
    {
        retval = max20356_platform_write_register(driver,     \
                                                  reg + iter, \
                                                  *(data + iter));

        if (0 != retval)
        {
            write_ok = false;
        }

        iter++;
    }

    return retval;
}

bool max20356_platform_write_register(max20356_driver_t driver, \
                                      uint8_t reg,              \
                                      uint8_t data)
{
    int     retval = 1;
    uint8_t addr;

#if !defined(MAX20356_SECURE_WRITES_ENABLED)
    uint8_t i2c_buf[] = {reg, data};
#else
    uint8_t csum1 = (MAX20356_PMIC_I2C_ADDR + reg + data) % 255;
    uint8_t csum2 = ((3 * MAX20356_PMIC_I2C_ADDR) + (2 * reg) + data) % 255;

    uint8_t i2c_buf[] = {reg, data, csum1, csum2};
#endif

    switch (driver)
    {
        case MAX20356_DRIVER_ADC:
            addr = MAX20356_ADC_I2C_ADDR;
            break;
        case MAX20356_DRIVER_PMIC:
            addr = MAX20356_PMIC_I2C_ADDR;
            break;
        case MAX20356_DRIVER_FUEL_GAUGE:
            addr = MAX20356_FUEL_GAUGE_I2C_ADDR;
            break;
        default:
            retval = 0;
    }

    if (0 != retval)
    {
        retval = i2c_api_write(MAX20356_I2C_BUS, addr, i2c_buf, sizeof(i2c_buf), 0);
    }

    return (retval == sizeof(i2c_buf));
}

bool max20356_platform_change_bits(max20356_driver_t driver, uint8_t reg, uint8_t mask, uint8_t bits)
{
    int     retval;
    bool    change_ok;
    uint8_t data;

    retval = max20356_platform_read_register(driver, reg, &data, sizeof(data));

    if (0 != retval)
    {
        change_ok = false;
    }
    else
    {
        data &= ~mask;
        data |= bits << get_right_most_set_bit(mask);

        change_ok = max20356_platform_write_register(driver, reg, data);
    }

    return change_ok;
}

bool max20356_platform_init_adc()
{
	/*mxc_gpio_cfg_t *adcpin = &gpio_cfg_adc0;
    if (MXC_ADC_Init() != E_NO_ERROR) {
        printf("Error Bad Parameter\n");

        while(1);
    }

    switch(MAX20356_ADC_CH)
    {
    case MXC_ADC_CH_0:
    	adcpin = &gpio_cfg_adc0;
    	break;
    case MXC_ADC_CH_1:
    	adcpin = &gpio_cfg_adc1;
    	break;
    case MXC_ADC_CH_2:
    	adcpin = &gpio_cfg_adc2;
    	break;
    case MXC_ADC_CH_3:
    	adcpin = &gpio_cfg_adc3;
    	break;
    case MXC_ADC_CH_4:
    	adcpin = &gpio_cfg_adc4;
    	break;
    case MXC_ADC_CH_5:
    	adcpin = &gpio_cfg_adc5;
    	break;
    case MXC_ADC_CH_6:
    	adcpin = &gpio_cfg_adc6;
    	break;
    case MXC_ADC_CH_7:
    	adcpin = &gpio_cfg_adc6;
    	break;
    default:
    	break;
    }
    MXC_GPIO_Config(adcpin);
*/
    /* Set up LIMIT0 to monitor high and low trip points */
/*    MXC_ADC_SetDataAlignment(0);
    MXC_ADC_SetMonitorChannel(MAX20356_ADC_MON, MAX20356_ADC_CH);
    MXC_ADC_SetMonitorHighThreshold(MAX20356_ADC_MON, 0x3FF);
    MXC_ADC_SetMonitorLowThreshold(MAX20356_ADC_MON, 0x0);
    MXC_ADC_EnableMonitor(MAX20356_ADC_MON);*/

	return -1;

}

int max20356_platform_read_adc()
{
/*	uint16_t adcVal = 0;
	MXC_ADC_StartConversion(MAX20356_ADC_CH);
	MXC_ADC_GetData(&adcVal);

	return adcVal;*/
	return -1;
}
