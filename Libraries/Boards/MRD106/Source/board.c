/*******************************************************************************
 * Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
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
 *
 * $Date: 2019-08-20 14:12:31 -0500 (Tue, 20 Aug 2019) $
 * $Revision: 45542 $
 *
 ******************************************************************************/

#include <stdio.h>
#include "mxc_config.h"
#include "mxc_sys.h"
#include "mxc_assert.h"
#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "led.h"
#include "spixfc.h"
#include "icc.h"

#include "mxc_delay.h"

#include "Peripherals.h"

#include "mrd106_isr.h"

#include "app_led.h"
#include "app_led_wrapper.h"

#define GPIO_VSSEL_1v8			(0x00000000)


/***** Global Variables *****/
mxc_uart_regs_t * ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const gpio_cfg_t pb_pin[] = {
    {PORT_1, PIN_4, GPIO_FUNC_IN, GPIO_PAD_PULL_UP}
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(gpio_cfg_t));

const gpio_cfg_t led_pin[] = {
    {PORT_0, PIN_23, GPIO_FUNC_OUT, GPIO_PAD_NONE},
    {PORT_0, PIN_24, GPIO_FUNC_OUT, GPIO_PAD_NONE},
    {PORT_0, PIN_26, GPIO_FUNC_OUT, GPIO_PAD_NONE},
	{PORT_0, PIN_21, GPIO_FUNC_OUT, GPIO_PAD_NONE},
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(gpio_cfg_t));

gpio_cfg_t reset_pin = {PORT_0, PIN_21, GPIO_FUNC_OUT, GPIO_PAD_NONE};
gpio_cfg_t mfio_pin = {PORT_0, PIN_9, GPIO_FUNC_OUT, GPIO_PAD_NONE};
gpio_cfg_t lis2dh12_irq = {PORT_0, PIN_25, GPIO_FUNC_IN, GPIO_PAD_NONE}; //It is used for ADXL367
gpio_cfg_t pmic_interrupt_pin = {PORT_1, PIN_10, GPIO_FUNC_IN, GPIO_PAD_NONE};
gpio_cfg_t afe_interrupt_pin = {PORT_0, PIN_13, GPIO_FUNC_IN, GPIO_PAD_PULL_UP};

/***** File Scope Variables *****/
const uart_cfg_t uart_cfg = {
    .parity = UART_PARITY_DISABLE,
    .size   = UART_DATA_SIZE_8_BITS,
    .stop   = UART_STOP_1,
    .flow   = UART_FLOW_CTRL_DIS,
    .pol    = UART_FLOW_POL_DIS,
    .baud   = CONSOLE_BAUD,
    .clksel = UART_CLKSEL_SYSTEM
};

const sys_cfg_uart_t uart_sys_cfg = {
    .map = MAP_B,
    .flow = Disable
};

const sys_cfg_spixfc_t spixfc_sys_cfg = NULL;   // There is no special system configuration parameters for SPIXC

const spixfc_cfg_t mx66_spixfc_cfg = {
    0, //mode
    0, //ssel_pol
    10000000 //baud
};

/******************************************************************************/
void mxc_assert(const char *expr, const char *file, int line)
{
	pr_info("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);
    while (1);
}

/******************************************************************************/
int Board_Init(void)
{
    int err;

    mxc_delay(MXC_DELAY_SEC(2));

	MXC_GPIO0->vssel = GPIO_VSSEL_1v8;
	MXC_GPIO1->vssel = GPIO_VSSEL_1v8 | (PIN_0 | PIN_1 |PIN_2);

    if ((err = Console_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    if ((err = LED_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    err = SPI_Line_Release();
    if(err != E_NO_ERROR){
	   return err;
    }


	GPIO_Config(&reset_pin);
	GPIO_OutClr(&reset_pin); // keep reset pin low for ME15 till the power up sequence completed.
	
    GPIO_OutSet(&reset_pin);

    ICC_Enable();
	GPIO_Config(&lis2dh12_irq);

    max20356_init();

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Init(void)
{
    int err;

    if ((err = UART_Init(ConsoleUart, &uart_cfg, &uart_sys_cfg)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Shutdown(void)
{
    int err;
    
    if ((err = UART_Shutdown(ConsoleUart)) != E_NO_ERROR) {
        return err;
    }
    
    return E_NO_ERROR;
}

/******************************************************************************/
void NMI_Handler(void)
{
    __NOP();
}

/******************************************************************************/
int MX66_Board_Init(void)
{
  return SPIXFC_Init(MX66_SPI, &mx66_spixfc_cfg, &spixfc_sys_cfg);

}

/******************************************************************************/
int MX66_Board_Read(uint8_t* read, unsigned len, unsigned deassert, spixfc_width_t width)
{

    spixfc_req_t req = {MX66_SSEL,deassert,0,NULL,read, width,len,0,0,NULL};

    return SPIXFC_Trans(MX66_SPI, &req);
}

/******************************************************************************/
int MX66_Board_Write(const uint8_t* write, unsigned len, unsigned deassert, spixfc_width_t width)
{

    spixfc_req_t req = {MX66_SSEL,deassert,0,write,NULL, width,len,0,0,NULL};

    return SPIXFC_Trans(MX66_SPI, &req);
}

/******************************************************************************/
int MX66_Clock(unsigned len, unsigned deassert)
{
    return SPIXFC_Clocks(MX66_SPI, len, MX66_SSEL, deassert);
}

int SPI_Line_Release()
{
	int err = 0;

	/* Setting spi pins as high-Z */
	gpio_cfg_t cs_ppg;
	gpio_cfg_t cs_acc;
	gpio_cfg_t spi_mosi;
	gpio_cfg_t spi_miso;
	gpio_cfg_t spi_clk;

	cs_ppg.func = GPIO_FUNC_IN;
	cs_ppg.pad = GPIO_PAD_NONE;
	cs_ppg.port = PORT_0;
	cs_ppg.mask = PIN_16;

	cs_acc.func = GPIO_FUNC_IN;
	cs_acc.pad = GPIO_PAD_NONE;
	cs_acc.port = PORT_0;
	cs_acc.mask = PIN_22;

	spi_mosi.func = GPIO_FUNC_IN;
	spi_mosi.pad = GPIO_PAD_NONE;
	spi_mosi.port = PORT_0;
	spi_mosi.mask = PIN_9;

	spi_miso.func = GPIO_FUNC_IN;
	spi_miso.pad = GPIO_PAD_NONE;
	spi_miso.port = PORT_0;
	spi_miso.mask = PIN_10;

	spi_clk.func = GPIO_FUNC_IN;
	spi_clk.pad = GPIO_PAD_NONE;
	spi_clk.port = PORT_0;
	spi_clk.mask = PIN_11;

	if ((err = GPIO_Config(&cs_ppg)) != E_NO_ERROR) {
		MXC_ASSERT_FAIL();
		return err;
	}

	if ((err = GPIO_Config(&cs_acc)) != E_NO_ERROR) {
		MXC_ASSERT_FAIL();
	    return err;
	}

	if ((err = GPIO_Config(&spi_mosi)) != E_NO_ERROR) {
		MXC_ASSERT_FAIL();
	    return err;
	}

	if ((err = GPIO_Config(&spi_miso)) != E_NO_ERROR) {
		MXC_ASSERT_FAIL();
	    return err;
	}

	if ((err = GPIO_Config(&spi_clk)) != E_NO_ERROR) {
		MXC_ASSERT_FAIL();
		return err;
	}

	return err;
}

