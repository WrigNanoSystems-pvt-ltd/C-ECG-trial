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
#ifndef DRIVERS_MAX20356_H
#define DRIVERS_MAX20356_H

#include <stdint.h>

#define MAX20356_VOLTAGE_LVL_1V8  UINT8_C(0x32)
#define MAX20356_VOLTAGE_LVL_5V   UINT8_C(0x32)

#define MAX20356_OUT_MODE_DISABLED  UINT8_C(0x00)
#define MAX20356_OUT_MODE_ENABLED   UINT8_C(0x01)

#define MAX20356_LED_BLUE   UINT8_C(0x7A)
#define MAX20356_LED_RED    UINT8_C(0x7B)
#define MAX20356_LED_GREEN  UINT8_C(0x7C)

#define MAX20356_LED_STEP_LIMIT  UINT8_C(24)

#define MAX20356_ERR_BK1_VOLTAGE   (-5000)
#define MAX20356_ERR_BK2_VOLTAGE   (-5001)
#define MAX20356_ERR_BK3_VOLTAGE   (-5002)
#define MAX20356_ERR_LDO1_VOLTAGE  (-5003)
#define MAX20356_ERR_LDO2_VOLTAGE  (-5004)
#define MAX20356_ERR_BBST_VOLTAGE  (-5005)

/****************************************************************************/
/*  STATUS POSITIONS                                                */
/****************************************************************************/
#define MAX20356_USB_OK (1 << 3)


/****************************************************************************/
/*  REGISTER SETTTING VALUES                                                */
/****************************************************************************/

#define MAX20356_PWR_CMD_OFF_MODE	UINT8_C(0xB2)
#define MAX20356_PWR_FACTORY_MODE	UINT8_C(0x52)
/****************************************************************************/
/*  SECURITY FUNCTION LOCK BITS                                             */
/****************************************************************************/
//LOCK MSK 1
#define MAX20356_LCK_BK1  UINT8_C(1 << 0)
#define MAX20356_LCK_BK2  UINT8_C(1 << 1)
#define MAX20356_LCK_BK3  UINT8_C(1 << 2)
#define MAX20356_LCK_BB   UINT8_C(1 << 3)
#define MAX20356_LCK_LD1  UINT8_C(1 << 4)
#define MAX20356_LCK_LD2  UINT8_C(1 << 5)
#define MAX20356_LCK_LD3  UINT8_C(1 << 6)
#define MAX20356_LCK_LD4  UINT8_C(1 << 7)

//LOCK MSK 2
#define MAX20356_LCK_BK1_SEQ  UINT8_C(1 << 0)
#define MAX20356_LCK_BK2_SEQ  UINT8_C(1 << 1)
#define MAX20356_LCK_BK3_SEQ  UINT8_C(1 << 2)
#define MAX20356_LCK_BB_SEQ   UINT8_C(1 << 3)
#define MAX20356_LCK_LD1_SEQ  UINT8_C(1 << 4)
#define MAX20356_LCK_LD2_SEQ  UINT8_C(1 << 5)
#define MAX20356_LCK_LD3_SEQ  UINT8_C(1 << 6)
#define MAX20356_LCK_LD4_SEQ  UINT8_C(1 << 7)

//LOCK MSK 3
#define MAX20356_LCK_CHG  	UINT8_C(1 << 0)
#define MAX20356_LCK_LIM  	UINT8_C(1 << 1)
#define MAX20356_LCK_GMDRP  UINT8_C(1 << 2)
#define MAX20356_LCK_WD  	UINT8_C(1 << 3)
#define MAX20356_LCK_LSW1  	UINT8_C(1 << 5)
#define MAX20356_LCK_LSW2  	UINT8_C(1 << 6)
#define MAX20356_LCK_LSW3  	UINT8_C(1 << 7)

#define MAX20356_LCK_ALL  UINT8_C(0xFF)

/****************************************************************************/
/*  LOCK PASSWORDS                                                          */
/****************************************************************************/

#define MAX20356_PWD_LOCK    UINT8_C(0xAA)
#define MAX20356_PWD_UNLOCK  UINT8_C(0x55)

/****************************************************************************/
/*  REGISTER MASKS                                                          */
/****************************************************************************/

#define MAX20356_MSK_OUT_CFG      UINT8_C(0x03)
#define MAX20356_MSK_VOLTAGE_LVL  UINT8_C(0x3F)
#define MAX20356_MSK_STATUS_OF_CHARGER	UINT8_C(0x05)
#define MAX20356_IVMON_DIVIDER UINT8_C(3 << 5)
#define MAX20356_IVMON_CTRL UINT8_C(0x0F)

/****************************************************************************/
/*  VOLTAGE MONITOR CHANNELS                                                */
/****************************************************************************/

#define MAX20356_IVMON_CH_BAT    	UINT8_C(0x02)
#define MAX20356_IVMON_CH_SYS    	UINT8_C(0x03)
#define MAX20356_IVMON_CH_BK1    	UINT8_C(0x04)
#define MAX20356_IVMON_CH_BK2    	UINT8_C(0x05)
#define MAX20356_IVMON_CH_BK3    	UINT8_C(0x06)
#define MAX20356_IVMON_CH_L1OUT  	UINT8_C(0x07)
#define MAX20356_IVMON_CH_L2OUT  	UINT8_C(0x08)
#define MAX20356_IVMON_CH_L3OUT  	UINT8_C(0x09)
#define MAX20356_IVMON_CH_BBOUT  	UINT8_C(0x0A)
#define MAX20356_IVMON_CH_THM  		UINT8_C(0x0B)
#define MAX20356_IVMON_CH_GND0  	UINT8_C(0x0C)
#define MAX20356_IVMON_CH_LOUT_RTC  UINT8_C(0x0D)
#define MAX20356_IVMON_CH_GND1  	UINT8_C(0x0F)




/****************************************************************************/
/*  ADC CHANNELS                                                            */
/****************************************************************************/
//TODO: Some are obsolate
#define MAX20356_ADC_CH_HDIN    UINT8_C(0x00)
#define MAX20356_ADC_CH_IVMON   UINT8_C(0x01)
#define MAX20356_ADC_CH_CHGIN   UINT8_C(0x03)
#define MAX20356_ADC_CH_CPOUT   UINT8_C(0x04)
#define MAX20356_ADC_CH_BSTOUT  UINT8_C(0x05)

#define MAX20356_ADC_BIT_ENABLE  UINT8_C(1 << 0)
#define MAX20356_LED_BIT_ON      UINT8_C(1 << 5)

#define MAX20356_ADC_IVMON_VOLTAGE_LIMIT  (1220)  /* mV */
#define MAX20356_ADC_CHGIN_VOLTAGE_LIMIT  (8250)  /* mV */
#define MAX20356_ADC_RESOLUTION           (1024)

#define MAX20356_BK1_LOWER_LIMIT   (1710)
#define MAX20356_BK1_UPPER_LIMIT   (1890)

#define MAX20356_BK2_LOWER_LIMIT   (1710)
#define MAX20356_BK2_UPPER_LIMIT   (1890)

#define MAX20356_BK3_LOWER_LIMIT   (3135)
#define MAX20356_BK3_UPPER_LIMIT   (3465)

#define MAX20356_BBST_LOWER_LIMIT  (4750)
#define MAX20356_BBST_UPPER_LIMIT  (5250)

#define MAX20356_POWER_OFF 0xB2
#define MAX20356_USB_OK_INT (1 << 3)
#define MAX20356_USB_OK_INT_MPC3 (1 << 3)
#define MAX20356_EN (1 << 0)
#define MAX20356_USB_OK_INT_F (1<<7)
#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

typedef enum
{
    MAX20356_OUTPUT_BAT,
    MAX20356_OUTPUT_SYS,
    MAX20356_OUTPUT_BK1,
    MAX20356_OUTPUT_BK2,
    MAX20356_OUTPUT_BK3,
    MAX20356_OUTPUT_LDO1,
    MAX20356_OUTPUT_LDO2,
	MAX20356_OUTPUT_LDO3,
	MAX20356_OUTPUT_BBOUT,
	MAX20356_OUTPUT_THM,
    MAX20356_OUTPUT_GND0,
    MAX20356_OUTPUT_LOUT_RTC,
    MAX20356_OUTPUT_GND1,
	MAX20356_OUTPUT_LSW1,
	MAX20356_OUTPUT_LSW2,
	MAX20356_OUTPUT_LSW3,

} max20356_output_t;

typedef enum
{
	MAX20356_IVMON_DIV_1,
	MAX20356_IVMON_DIV_2,
	MAX20356_IVMON_DIV_3,
	MAX20356_IVMON_DIV_4
} max20356_ivmon_div_t;

int max20356_init(void);
int max20356_set_out_voltage(max20356_output_t out, uint8_t lvl);
int max20356_output_config(max20356_output_t out, uint8_t mode);
int max20356_get_chip_version(uint8_t *p_chipver);
int max20356_read_register(uint8_t reg, uint8_t *byt);
int max20356_drive_led(uint8_t led, uint8_t step_count);
int max20356_led_off(uint8_t led);
unsigned max20356_get_voltage(max20356_output_t out);
int max20356_is_usb_connected(uint8_t *p_is_connected);
int max20356_send_power_off_cmd();
int max20356_usb_power(int onOff);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* DRIVERS_MAX20356_H */
