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

#include "max20356.h"

#include <stdint.h>
#include <stdlib.h>
#include <limits.h>
#include <float.h>

#include "mxc_delay.h"

#include "mxc_errors.h"
#include "max17260.h"


#include "max20356_platform.h"
#include "max20356_registers.h"
#include "gpio.h"
#include "i2c_api.h"


/****************************************************************************/
/*  STATIC FUNCTION DECLERATIONS                                            */
/****************************************************************************/

static int max20356_sanity_check(void);
static int max20356_defaults(void);
static void max20356_reg_lock(uint8_t lock_bits);
static void max20356_reg_unlock(uint8_t lock_bits);




/****************************************************************************/
/*  GLOBAL FUNCTION DEFINITIONS                                             */
/****************************************************************************/

int max20356_init(void)
{
    int result = 0;
    //TODO:speed parameter and error handling should be added for i2c api init
    i2c_api_init(MAX20356_I2C_BUS);
    result = max20356_sanity_check();

    if (E_SUCCESS == result)
    {
        result = max20356_defaults();
    }

    max17260_init();

    return result;
}

int max20356_set_out_voltage(max20356_output_t out, uint8_t lvl)
{
    int err = E_SUCCESS;
    int reg;

    switch (out)
    {
        case MAX20356_OUTPUT_BK1:
            reg = MAX20356_PMIC_REG_BUCK_1_VSET;
            break;

        case MAX20356_OUTPUT_BK2:
            reg = MAX20356_PMIC_REG_BUCK_2_VSET;
            break;

        case MAX20356_OUTPUT_BK3:
            reg = MAX20356_PMIC_REG_BUCK_3_VSET;
            break;

        case MAX20356_OUTPUT_BBOUT:
            reg = MAX20356_PMIC_REG_BBST_VSET;
            break;

        case MAX20356_OUTPUT_LDO1:
            reg = MAX20356_PMIC_REG_LDO1_VSET;
            break;

        case MAX20356_OUTPUT_LDO2:
            reg = MAX20356_PMIC_REG_LDO2_VSET;
            break;

        case MAX20356_OUTPUT_LDO3:
            reg = MAX20356_PMIC_REG_LDO3_VSET;
            break;

        default:
            err = E_BAD_PARAM;
    }

    if (E_SUCCESS == err)
    {
        max20356_platform_change_bits(MAX20356_DRIVER_PMIC,     \
                                      reg,                      \
                                      MAX20356_MSK_VOLTAGE_LVL, \
                                      lvl);
    }

    return err;
}

int max20356_output_config(max20356_output_t out, uint8_t mode)
{
    int err = E_SUCCESS;
    int reg;

    switch (out)
    {
        case MAX20356_OUTPUT_BK1:
            reg = MAX20356_PMIC_REG_BUCK_1_ENA;
            break;

        case MAX20356_OUTPUT_BK2:
            reg = MAX20356_PMIC_REG_BUCK_2_ENA;
            break;

        case MAX20356_OUTPUT_BK3:
            reg = MAX20356_PMIC_REG_BUCK_3_ENA;
            break;

        case MAX20356_OUTPUT_BBOUT:
            reg = MAX20356_PMIC_REG_BBST_ENA;
            break;

        case MAX20356_OUTPUT_LDO1:
            reg = MAX20356_PMIC_REG_LDO1_ENA;
            break;

        case MAX20356_OUTPUT_LDO2:
            reg = MAX20356_PMIC_REG_LDO2_ENA;
            break;

        case MAX20356_OUTPUT_LSW1:
            reg = MAX20356_PMIC_REG_LSW_1_ENA;
            break;

        case MAX20356_OUTPUT_LSW2:
            reg = MAX20356_PMIC_REG_LSW_2_ENA;
            break;

        case MAX20356_OUTPUT_LSW3:
            reg = MAX20356_PMIC_REG_LSW_3_ENA;
            break;
        default:
            err = E_BAD_PARAM;
    }

    if (E_SUCCESS == err)
    {
        max20356_platform_change_bits(MAX20356_DRIVER_PMIC, \
                                      reg,                  \
                                      MAX20356_MSK_OUT_CFG, \
                                      mode);
    }

    return err;
}

int max20356_get_chip_version(uint8_t *p_chipver)
{
    int ret = 0;

    ret = max20356_platform_read_register(MAX20356_DRIVER_PMIC,      \
                                          MAX20356_PMIC_REG_CHIP_ID, \
                                          p_chipver,                 \
                                          1);

    return ret;
}

int max20356_read_register(uint8_t reg, uint8_t *byt){
	int ret = 0;

	ret = max20356_platform_read_register(MAX20356_DRIVER_PMIC,     \
										  reg, 						\
										  byt,                 		\
										  1);

	return ret;


}


int max20356_drive_led(uint8_t led, uint8_t step_count)
{
    int     err = E_NO_ERROR;
    uint8_t cfg;

    if (step_count > MAX20356_LED_STEP_LIMIT)
    {
        err = E_BAD_PARAM;
    }
    else
    {
        cfg = step_count | MAX20356_LED_BIT_ON;
        max20356_platform_write_register(MAX20356_DRIVER_PMIC, led, cfg);
    }

    return err;
}

int max20356_led_off(uint8_t led)
{
    max20356_platform_write_register(MAX20356_DRIVER_PMIC, led, 0x00);

    return E_SUCCESS;
}

unsigned max20356_get_voltage(max20356_output_t out)
{
    unsigned voltage = UINT_MAX;
    int      err     = E_NO_ERROR;

    uint8_t  ch_ivmon;
    uint8_t  ch_adc;
    unsigned adc_limit;
    uint8_t  regval;
    uint8_t  avg;
    uint16_t adcVal;

    switch (out)
    {
        case MAX20356_OUTPUT_BAT:
            ch_ivmon  = MAX20356_IVMON_CH_BAT;
            adc_limit = MAX20356_ADC_IVMON_VOLTAGE_LIMIT;
            break;
        case MAX20356_OUTPUT_SYS:
            ch_ivmon  = MAX20356_IVMON_CH_SYS;
            adc_limit = MAX20356_ADC_IVMON_VOLTAGE_LIMIT;
            break;
        case MAX20356_OUTPUT_BK1:
            ch_ivmon  = MAX20356_IVMON_CH_BK1;
            adc_limit = MAX20356_ADC_IVMON_VOLTAGE_LIMIT;
            break;
        case MAX20356_OUTPUT_BK2:
            ch_ivmon  = MAX20356_IVMON_CH_BK2;
            adc_limit = MAX20356_ADC_IVMON_VOLTAGE_LIMIT;
            break;
        case MAX20356_OUTPUT_BK3:
            ch_ivmon  = MAX20356_IVMON_CH_BK3;
            adc_limit = MAX20356_ADC_IVMON_VOLTAGE_LIMIT;
            break;
        case MAX20356_OUTPUT_LDO1:
            ch_ivmon  = MAX20356_IVMON_CH_L1OUT;
            adc_limit = MAX20356_ADC_IVMON_VOLTAGE_LIMIT;
            break;
        case MAX20356_OUTPUT_LDO2:
            ch_ivmon  = MAX20356_IVMON_CH_L2OUT;
            adc_limit = MAX20356_ADC_IVMON_VOLTAGE_LIMIT;
            break;
        case MAX20356_OUTPUT_LDO3:
            ch_ivmon  = MAX20356_IVMON_CH_L3OUT;
            adc_limit = MAX20356_ADC_IVMON_VOLTAGE_LIMIT;
            break;
        case MAX20356_OUTPUT_BBOUT:
            ch_ivmon  = MAX20356_IVMON_CH_BBOUT;
            adc_limit = MAX20356_ADC_IVMON_VOLTAGE_LIMIT;
            break;
        case MAX20356_OUTPUT_THM:
            ch_ivmon    = MAX20356_IVMON_CH_THM;
            adc_limit = MAX20356_ADC_CHGIN_VOLTAGE_LIMIT;
            break;
        default:
            err = E_BAD_PARAM;
    }

    if (E_NO_ERROR == err)
    {
    	 max20356_platform_change_bits(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_IVMON_CFG, MAX20356_IVMON_CTRL, ch_ivmon);
    	 adcVal = max20356_platform_read_adc();
    	 voltage = (2*adcVal * adc_limit) / MAX20356_ADC_RESOLUTION;

    }

    return voltage;
}

int max20356_is_usb_connected(uint8_t *p_is_connected)
{
	int ret = 0;
	uint8_t reg_val = 0;

	ret = max20356_platform_read_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_STATUS0, &reg_val, 1);

	if(0 == ret)
	{
		if(reg_val & MAX20356_MSK_STATUS_OF_CHARGER){
			*p_is_connected = 1;
		}
		else{
			*p_is_connected = 0;
		}
	}

	return ret > 0 ? 0 : -1;
}

int max20356_send_power_off_cmd()
{
	int ret = 0;

	max20356_reg_unlock(MAX20356_LCK_ALL);
	ret = max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_PWR_CMD, MAX20356_PWR_CMD_OFF_MODE);

	return ret > 0 ? 0 : -1;
}

/****************************************************************************/
/*  STATIC FUNCTION DEFINITIONS                                             */
/****************************************************************************/

static int max20356_sanity_check(void)
{

    return 0;
}

static int max20356_defaults(void)
{
	//TODO: Add extra check to the voltage values
    unsigned bk3;
    unsigned bk2;
    unsigned bk1;

    unsigned lsw1;
    unsigned ldo2;

    unsigned bbst;

    uint8_t status;
    uint8_t ivmoncfg;

    int err = E_NO_ERROR;

    max20356_reg_unlock(MAX20356_LCK_ALL);

    //Checking if USB is present
    max20356_platform_read_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_STATUS1, &status, 1);
    mxc_delay(200000);
    status &= MAX20356_USB_OK;
    if(status == 0)
    {
    	//Disable LDO 2 if usb is not present
    	max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_LDO2_ENA, 0xE0);
    }
    else
    {
    	max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_LDO2_ENA, 0xE1);
    }
    mxc_delay(200000);

    uint8_t mpc3_cfg = 0x14;
    max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_MPC_3, mpc3_cfg);
    //LDO2 MPC3 Interrupt setting
    max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_INT_1, MAX20356_USB_OK_INT);
    max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_USBOKITRCFG, MAX20356_USB_OK_INT_MPC3);
    max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_INTMASK_1, MAX20356_USB_OK_INT);
    mxc_delay(200000);
    //Buck 1 Enable
    max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_BUCK_1_ENA, 0xE1);
    mxc_delay(200000);

    //Buck 2 Enable
    max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_BUCK_2_ENA, 0xE1);
    mxc_delay(200000);

    //LSW 1 Enable
    max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_LSW_1_ENA, 0xE1);
    mxc_delay(200000);

    //BBst Enable
    max20356_platform_write_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_BBST_ENA, 0xE1);
    mxc_delay(200000);
    max20356_reg_lock(MAX20356_LCK_ALL);

    return err;
}

static void max20356_reg_lock(uint8_t lock_bits)
{
    uint8_t msk = 0xFF & ~lock_bits;

    max20356_platform_write_register(MAX20356_DRIVER_PMIC,       \
                                     MAX20356_PMIC_REG_LOCK_MSK_1, \
                                     msk);

    max20356_platform_write_register(MAX20356_DRIVER_PMIC,          \
                                     MAX20356_PMIC_REG_LOCK_UNLOCK_1, \
                                     MAX20356_PWD_LOCK);
}

static void max20356_reg_unlock(uint8_t lock_bits)
{
    uint8_t msk = 0xFF & ~lock_bits;

    max20356_platform_write_register(MAX20356_DRIVER_PMIC,       \
                                     MAX20356_PMIC_REG_LOCK_MSK_1, \
                                     msk);
    mxc_delay(200000);
    max20356_platform_write_register(MAX20356_DRIVER_PMIC,          \
                                     MAX20356_PMIC_REG_LOCK_UNLOCK_1, \
                                     MAX20356_PWD_UNLOCK);
    mxc_delay(200000);
}

int max20356_usb_power(int onOff)
{
	uint8_t pwr = 0xE1;
 	max20356_reg_unlock(MAX20356_LCK_ALL);
 	if(onOff != 0)
 	{
 		max20356_platform_write_register(MAX20356_DRIVER_PMIC,MAX20356_PMIC_REG_LDO2_ENA, pwr);
 	}
 	else
 	{
 		pwr = 0xE0;
 		max20356_platform_write_register(MAX20356_DRIVER_PMIC,MAX20356_PMIC_REG_LDO2_ENA, pwr);
 	}
 	max20356_reg_lock(MAX20356_LCK_ALL);

 	return 0;
}

