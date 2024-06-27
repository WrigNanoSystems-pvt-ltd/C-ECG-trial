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

/*******************************      INCLUDES    ****************************/
#include <stddef.h>
#include "mxc_config.h"
#include "mxc_assert.h"
#include "button.h"
#include "button_gesture_handler.h"
#include "main.h"
#include "gpio.h"


/*******************************      DEFINES     ****************************/


/******************************* Type Definitions ****************************/
typedef struct {
	gpio_cfg_t 		  gpio;
	button_callback_t cb;
} buttons_struct_t;

/******************************* 	Variables 	  ****************************/
#if defined(FTHR) || defined(FTHR2)
static buttons_struct_t buttons[] = {
	{ {PORT_1, PIN_6, GPIO_FUNC_IN, GPIO_PAD_PULL_UP},  NULL}, //P1.6 is not connected for MAX32666FTHR
	{ {PORT_1, PIN_10, GPIO_FUNC_IN, GPIO_PAD_PULL_UP},  NULL}
};
#elif MRD104
static buttons_struct_t buttons[] = {
	{ {PORT_1, PIN_6, GPIO_FUNC_IN, GPIO_PAD_PULL_UP},  NULL},
	{ {PORT_1, PIN_4, GPIO_FUNC_IN, GPIO_PAD_PULL_UP},  NULL}
};
#elif MRD106
static buttons_struct_t buttons[] = {
	{ {PORT_1, PIN_6, GPIO_FUNC_IN, GPIO_PAD_PULL_UP},  NULL},//P1.6 is not connected for MAX32666FTHR
	{ {PORT_1, PIN_4, GPIO_FUNC_IN, GPIO_PAD_PULL_UP},  NULL}
};
#endif

static const unsigned int num_pbs = sizeof(buttons) / sizeof(buttons[0]);

static const unsigned int int_pol_vals[BUTTON_MAX_INT] = {0, 0, 1, 1, 2};

/******************************* Static Functions ****************************/
void GPIO0_IRQHandler(void)
{
    GPIO_Handler(PORT_0);
}
void GPIO1_IRQHandler(void)
{
    GPIO_Handler(PORT_1);
}
void GPIO2_IRQHandler(void)
{
    GPIO_Handler(PORT_2);
}
void GPIO3_IRQHandler(void)
{
    GPIO_Handler(PORT_3);
}

/*Button callbacks*/
void single_press()
{
	app_main_evt_post(EVT_BUTTON_SINGLE_PRESS);
}

void double_press()
{
	app_main_evt_post(EVT_BUTTON_DOUBLE_PRESS);
}

void triple_press()
{
	app_main_evt_post(EVT_BUTTON_TRIPLE_PRESS);
}

void long_press()
{
	app_main_evt_post(EVT_BUTTON_LONG_PRESS);
}

/******************************* Public Functions ****************************/
int button_init(void)
{
    int retval = E_NO_ERROR;
    unsigned int i;
    
    GPIO_Init();

    // Enable pushbutton inputs
    for (i = 0; i < num_pbs; i++) {
        if (GPIO_Config(&buttons[i].gpio) != E_NO_ERROR) {
            retval = E_UNKNOWN;
        }
    }

    button_gesture_handler_init(BUTTON_1);
    button_gesture_handler_save_callback(SINGLE_PRESS, single_press);
    button_gesture_handler_save_callback(DOUBLE_PRESS, double_press);
    button_gesture_handler_save_callback(TRIPLE_PRESS, triple_press);
    button_gesture_handler_save_callback(SINGLE_LONG_PRESS, long_press);
    button_int_enable(BUTTON_1);


    return retval;
}

int button_register_cb(unsigned int pb, button_callback_t callback, button_int_sel_t type)
{
    MXC_ASSERT(pb < num_pbs);

	gpio_cfg_t *gpio = &buttons[pb].gpio;
	
	// Keep Callback
	buttons[pb].cb = callback;
	
    if (callback) {
        // Register callback
        GPIO_RegisterCallback(gpio, callback, (void*)pb);


        if(BUTTON_HIGH_LEVEL_INT == type || BUTTON_LOW_LEVEL_INT == type){
        	// Configure and enable interrupt
        	GPIO_IntConfig(gpio, GPIO_INT_LEVEL, int_pol_vals[type]);
        }
        else{
        	// Configure and enable interrupt
        	GPIO_IntConfig(gpio, GPIO_INT_EDGE, int_pol_vals[type]);
        }

        GPIO_IntEnable(gpio);
        NVIC_EnableIRQ((IRQn_Type)MXC_GPIO_GET_IRQ(gpio->port));
    } else {
        // Disable interrupt and clear callback
        GPIO_IntDisable(gpio);
        GPIO_RegisterCallback(gpio, NULL, NULL);
    }

    return E_NO_ERROR;
}

void button_int_enable(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    GPIO_IntEnable(&buttons[pb].gpio);
}

void button_int_disable(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    GPIO_IntDisable(&buttons[pb].gpio);
}

void button_int_clear(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    GPIO_IntClr(&buttons[pb].gpio);
}

int button_get(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    return !GPIO_InGet(&buttons[pb].gpio);
}

button_callback_t button_get_cb(unsigned int pb)
{
    MXC_ASSERT(pb < num_pbs);
    return buttons[pb].cb;
}
