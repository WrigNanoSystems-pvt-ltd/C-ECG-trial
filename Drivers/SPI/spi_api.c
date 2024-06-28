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


#include <stdlib.h>
#include <string.h>
#include "spi_api.h"
#include "spi.h"

#include "board.h"

#include "Peripherals.h"

#define MAX_NUMBER_OF_INSTANCE	5

typedef struct{
	uint8_t is_used;
	gpio_cfg_t gpio_cs;
} spi_cs_pins_t;

static spi_cs_pins_t cs_pins[MAX_NUMBER_OF_INSTANCE];

static sys_cfg_spi_t spi_sys;
static spi_req_t req;

static void spi_select(int spi_index);
static void spi_deselect(int spi_index);

static uint8_t g_is_inited = 0;

int spi_api_init(uint32_t speed)
{
	int ret = 0;
	uint32_t index = 0;

	if(0 == g_is_inited){
		for(index = 0; index < MAX_NUMBER_OF_INSTANCE; index++){
			cs_pins[index].is_used = 0;
			cs_pins[index].gpio_cs.func = 0;
			cs_pins[index].gpio_cs.mask= 0;
			cs_pins[index].gpio_cs.pad = 0;
			cs_pins[index].gpio_cs.port = 0;
		}

	    spi_sys.map = MAP_A;
	    spi_sys.ss0 = Disable;
	    spi_sys.ss1 = Disable;
	    spi_sys.ss2 = Disable;
	    spi_sys.num_io = 0;

	    ret = SPI_Init(SPI1, 0, speed, spi_sys);
	    pr_info("SPI_Init return value %d \n", ret);

	    if(0 == ret){
	        ret = SPI_Enable(SPI1);
	        pr_info("SPI_Enable return value %d \n", ret);
	    }


	    g_is_inited = 1;
	}

	return ret;
}

int spi_api_open(uint32_t * spi_index, uint32_t cs_port, uint32_t cs_pin)
{
	int ret = 0;
	int index = 0;
	uint8_t empty_slot_exist = 0;

	if(cs_port <= PORT_4 && cs_pin <= PIN_31){
		ret = 0;
	}
	else{
		ret = -1;
	}

	if(0 == ret){
		for(index = 0; index < MAX_NUMBER_OF_INSTANCE; index++){
			if(0 == cs_pins[index].is_used){
				cs_pins[index].is_used = 1;
				empty_slot_exist = 1;
				break;
			}
		}

		if(0 == empty_slot_exist){
			ret = -1;
		}
	}

	if(0 == ret){
		cs_pins[index].gpio_cs.port = cs_port;
		cs_pins[index].gpio_cs.mask = cs_pin;
		cs_pins[index].gpio_cs.func = GPIO_FUNC_OUT;
		cs_pins[index].gpio_cs.pad = GPIO_PAD_NONE;

		ret = GPIO_Config(&cs_pins[index].gpio_cs);

		if(0 == ret){
			*spi_index = index;
			spi_deselect(index);
		}

	}

	return ret;
}


int spi_api_read(int spi_index, uint8_t * p_send, uint8_t * p_receive, uint32_t tx_len, uint32_t rx_len)
{
	int ret = 0;
	uint8_t rx_data[tx_len + rx_len];

	if(spi_index > MAX_NUMBER_OF_INSTANCE || NULL == p_receive || NULL == p_send){
		ret = -1;
	}

	if(0 == ret){
		memset(&req, 0, sizeof(req));

		req.bits = 8;
		req.callback = NULL;
		req.deass = 0;
		req.len = tx_len + rx_len;
		req.rx_data = rx_data;
		req.rx_num = 0;
		req.ssel = 0;
		req.ssel_pol = SPI17Y_POL_LOW;
		req.tx_data = p_send;
		req.tx_num = 0;
		req.width = SPI17Y_WIDTH_1;

		spi_select(spi_index);

		ret = SPI_MasterTrans(SPI1, &req);

		spi_deselect(spi_index);

		if(0 == ret){
			memcpy(p_receive, &rx_data[tx_len], rx_len);
		}
	}

	return ret;
}

int spi_api_write(int spi_index, uint8_t const * p_send, uint32_t tx_len)
{
	int ret = 0;
	uint8_t rx_data[tx_len];

	if(spi_index > MAX_NUMBER_OF_INSTANCE || NULL == p_send ){
		ret = -1;
	}

	if(0 == ret){
		memset(&req, 0, sizeof(req));

		req.bits = 8;
		req.callback = NULL;
		req.deass = 0;
		req.len = tx_len;
		req.rx_data = rx_data;
		req.rx_num = 0;
		req.ssel = 0;
		req.ssel_pol = SPI17Y_POL_LOW;
		req.tx_data = p_send;
		req.tx_num = 0;
		req.width = SPI17Y_WIDTH_1;

		spi_select(spi_index);

		ret = SPI_MasterTrans(SPI1, &req);

		spi_deselect(spi_index);
	}

	return ret;
}

int spi_api_disable()
{
	int ret = 0;
	int index = 0;

	SPI_Disable(SPI1);
	SPI_Shutdown(SPI1);

	SPI_Line_Release();

	for(index = 0; index < MAX_NUMBER_OF_INSTANCE; index++){
		if(cs_pins[index].is_used){
			cs_pins[index].gpio_cs.func = GPIO_FUNC_IN;
			cs_pins[index].gpio_cs.pad = GPIO_PAD_NONE;

			ret += GPIO_Config(&cs_pins[index].gpio_cs);

			break;
		}
	}

	g_is_inited = 0;

	return ret;
}

static void spi_select(int spi_index)
{
	GPIO_OutClr(&cs_pins[spi_index].gpio_cs);
}

static void spi_deselect(int spi_index)
{
	GPIO_OutSet(&cs_pins[spi_index].gpio_cs);
}


