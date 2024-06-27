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

#include "app_led.h"
#include "led.h"
#include "tmr.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "mrd106.h"

#include "main.h"



#define LED_SLOW_BLINK_INTERVAL_MS		(1000)
#define LED_FAST_BLINK_INTERVAL_MS		(250)

static mrd_t *m_mrd = NULL;

const uint8_t color_maps[LED_COLOR_MAX][NUMBER_OF_LED] =
{
		{1, 0, 0}, {0, 0, 1}, {0, 1, 0},
		{1, 1, 0}, {0, 1, 1}, {1, 0, 1},
		{1, 1, 1}, {0, 0, 0}
};

static led_color_type_t g_led_color = LED_COLOR_MAX;

/* Currently we are using RTC to send notification packets. If that works, these function can be removed in the future */
void app_led_timer_handle()
{
	static uint8_t is_off_state = 0;

	if(!is_off_state){
		app_led_off();
		is_off_state = 1;
	}
	else{
		app_led_on(g_led_color);
		is_off_state = 0;
	}
}

void app_led_timer_run(uint32_t interval)
{
	m_mrd->updateTimerPeriod(LED_TIMER, interval, LED_TIMER_PS);
	m_mrd->enableTimer(LED_TIMER, 1);

}

void app_led_blink_stop()
{
	m_mrd->enableTimer(LED_TIMER, 0);
}

int app_led_on(led_color_type_t color)
{
	int index = 0;
	int ret = 0;

	if(color >= LED_COLOR_MAX){
		ret = E_BAD_PARAM;
	}

	if(E_NO_ERROR == ret){
		for(index = 0; index < NUMBER_OF_LED; index++){
			if(color_maps[color][index]){
				LED_On(index);
			}
			else{
				LED_Off(index);
			}
		}
	}

	return ret;
}

int app_led_off()
{
	int ret = 0;
	int index = 0;

	for(index = 0; index < NUMBER_OF_LED; index++){
		LED_Off(index);
	}

	return ret;
}

int app_led_blink(led_color_type_t color, led_blink_type_t blink)
{
	int ret = 0;

	if(m_mrd == NULL)
	{
		m_mrd = getMRD();
	}

	if(color >= LED_COLOR_MAX || blink >= LED_BLINK_MAX){
		ret = E_BAD_PARAM;
	}

	if(E_NO_ERROR == ret){

		app_led_off();
		app_led_blink_stop();
		g_led_color = color;
		ret = app_led_on(g_led_color);

		switch(blink)
		{
		case LED_BLINK_SOLID:
			break;
		case LED_BLINK_FAST:
			app_led_timer_run(LED_FAST_BLINK_INTERVAL_MS);
			break;
		case LED_BLINK_SLOW:
			app_led_timer_run(LED_SLOW_BLINK_INTERVAL_MS);
			break;

		case LED_BLINK_MAX:
			break;

		}
	}

	return ret;
}






