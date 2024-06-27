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


#include <stdint.h>

#include "app_led.h"
#include "app_led_wrapper.h"

#include "../GUI/gui.h"
#include "usb.h"
#include "max17260.h"
#include "ble_api.h"
#include "app_gui.h"
#include "mrd106.h"

static mrd_t *m_mrd = NULL;
//static unsigned char prev_LED_condition;
static unsigned char UpdateLEDFlag = 0;

static led_color_type_t led_colour;
static led_blink_type_t led_blink;

extern uint8_t programming_me15;
extern uint8_t programmed_me15;

extern uint8_t power_but_evt;

void solid_LED(unsigned char colour){

	if(led_colour != colour || led_blink != LED_BLINK_SOLID){
	    led_colour = colour;
	    led_blink = LED_BLINK_SOLID;
		UpdateLEDFlag = 1;
	}



}

void blink_LED_fast(unsigned char colour){
	if(led_colour != colour || led_blink != LED_BLINK_FAST){
	    led_colour = colour;
	    led_blink = LED_BLINK_FAST;

		UpdateLEDFlag = 1;
	}

}

void blink_LED_slow(unsigned char colour){

	if(led_colour != colour || led_blink != LED_BLINK_SLOW){
	    led_colour = colour;
	    led_blink = LED_BLINK_SLOW;
		UpdateLEDFlag = 1;
	}

}

unsigned char check_RED_conditions(){

    uint8_t batt_lev = m_mrd->getBatteryPercentage();
    if(m_mrd)
    {
		if (batt_lev > 100) {
				batt_lev = 100;
		}

		if (batt_lev < 0) {
			batt_lev = 0;
		}

		if ((batt_lev) <= 10) {
			blink_LED_slow(LED_COLOR_RED);
			return 1;
		}

		/*if(!programmed_me15)
		{
			blink_LED_fast(LED_COLOR_RED);
			return 1;
		}*/
    }
    return 0;
}

unsigned char check_GREEN_conditions(){

	return 0;

}

unsigned char check_BLUE_conditions(){

    ble_state_t ble_stat = ble_get_state();

	if(BLE_STATE_CONN_OPEN ==ble_stat){
		if(adapter_is_reporting()){
			if(0)//m_mrd->getFlashLogEnable())
			{
				return 0;
			}
			else
			{
				blink_LED_fast(LED_COLOR_BLUE);
			}
		} else{
			solid_LED(LED_COLOR_BLUE);
		}
		return 1;
	} else{
		return 0;
	}

	return 0;
}

unsigned char check_MSBL_YELLOW_condition(){


	// static Timer yellowTimer;

	// if(SSBootloaderComm::FlashingCompleted){
	// 	if(prev_LED_condition!=FAST_YELLOW){
	// 		prev_LED_condition = FAST_YELLOW;
	// 		blink_LED_fast(YELLOW);
	// 		yellowTimer.reset();
	// 		yellowTimer.start();
	// 	} else{
	// 		if( (yellowTimer.read_ms() > 2500) ){
	// 			yellowTimer.stop();
	// 			SSBootloaderComm::FlashingCompleted = false;
	// 		}
	// 	}
	// 	return 1;
	// } else{
	// 	return 0;
	// }

    return 0;
}

unsigned char check_MAGENTA_conditions(){

	if(power_but_evt){
		blink_LED_fast(LED_COLOR_MAGENTA);
		return 1;
	}

    return 0;
}

unsigned char check_CYAN_conditions(){
	
	if(m_mrd->getFlashLogEnable()){
		blink_LED_slow(LED_COLOR_CYAN);
		return 1;
	}

	if(usb_get_status()){
		solid_LED(LED_COLOR_CYAN);
		return 1;
	}

	return 0;
}

unsigned char check_YELLOW_conditions(){

    uint8_t batt_lev = m_mrd->getBatteryPercentage();

	if(batt_lev>100){
		batt_lev = 100;
	}

	if(batt_lev < 0){
		batt_lev = 0;
	}

	if(m_mrd == NULL){
		solid_LED(LED_COLOR_YELLOW);
		return 1;
	}

	/*if(programming_me15){
		blink_LED_fast(LED_COLOR_YELLOW);
		return 1;
	}*/

	if( ((batt_lev) > 10) && ((batt_lev) <= 50) ){
			blink_LED_slow(LED_COLOR_YELLOW);
		return 1;
	}
    else{
		return 0;
	}
}

void check_WHITE_conditions()
{
    uint8_t batt_lev = m_mrd->getBatteryPercentage();

	if(batt_lev>100){
		batt_lev = 100;
	}

	if(batt_lev < 0){
		batt_lev = 0;
	}

	if(batt_lev > 50)
	{
		blink_LED_slow(LED_COLOR_WHITE);
	}

}

void check_LED_conditions(){

	unsigned char ret;
	m_mrd = getMRD();
	if(m_mrd)
	{
	ret = check_MAGENTA_conditions();
	if(ret == 0){
		ret = check_MSBL_YELLOW_condition();
		if(ret == 0){
			ret = check_BLUE_conditions();
			if(ret == 0){
				ret = check_RED_conditions();
				if(ret == 0){
					ret = check_YELLOW_conditions();
					if(ret == 0){
						ret = check_CYAN_conditions();
						if(ret == 0){
							ret = check_GREEN_conditions();
							if(ret == 0)
							{
								check_WHITE_conditions();
							}
						}
					}
				}
			}
		}
	}

	if(UpdateLEDFlag)
	{
		/* Apply LED status, LEDs are on state */
		if(m_mrd->getStatusLedEnable()){
			app_led_blink(led_colour, led_blink);
			UpdateLEDFlag = 0;
		}
		else{
			/* LEDs are off state, apply only magenta fast blink */
			if(led_colour == LED_COLOR_MAGENTA && led_blink == LED_BLINK_FAST){
				app_led_blink(led_colour, led_blink);
				UpdateLEDFlag = 0;
			}
		}

	}

	if(!m_mrd->getStatusLedEnable() && led_colour != LED_COLOR_MAGENTA && led_blink != LED_BLINK_FAST){
		app_led_blink_stop();
		app_led_off();
		led_colour = LED_COLOR_MAX;
		led_blink = LED_BLINK_MAX;
	}
	}
}
