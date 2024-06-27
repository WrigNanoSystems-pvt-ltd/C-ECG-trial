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

#include "app_gui.h"

#include <stdint.h>

#include "Peripherals.h"

#define MAX_NUMBER_OF_INTERFACE		(5u)

extern app_gui_cmd_interface_t sens_int;
extern app_gui_cmd_interface_t sens_nrf;
extern app_gui_cmd_interface_t nim_int;
extern app_gui_cmd_interface_t algo_int;

app_gui_cmd_interface_t  *interfaces[MAX_NUMBER_OF_INTERFACE] =
{
		[0] = &sens_nrf,
		[1] = &sens_int,
		[2] = &nim_int,
		[3] = &algo_int,
};

int app_gui_init()
{
	int ret = 0;

	for(int index = 0; index < ARRAY_SIZE(interfaces); index++)
	{
		if(NULL != interfaces[index])
			ret |= interfaces[index]->init();
	}

	return ret;
}

int app_gui_connected()
{
	int ret = 0;

	for(int index = 0; index < ARRAY_SIZE(interfaces); index++)
	{
		if(NULL != interfaces[index])
			ret |= interfaces[index]->connected();
	}

	return ret;
}

int app_gui_disconnected()
{
	int ret = 0;

	for(int index = 0; index < ARRAY_SIZE(interfaces); index++)
	{
		if(NULL != interfaces[index])
			ret |= interfaces[index]->disconnected();
	}

	return ret;
}

int app_gui_start()
{
	int ret = 0;

	for(int index = 0; index < ARRAY_SIZE(interfaces); index++)
	{
		if(NULL != interfaces[index])
			ret |= interfaces[index]->start();
	}

	return ret;
}

int app_gui_stop()
{
	int ret = 0;

	for(int index = 0; index < ARRAY_SIZE(interfaces); index++)
	{
		if(NULL != interfaces[index])
			ret |= interfaces[index]->stop();
	}

	return ret;
}

int app_gui_exec(uint8_t command[20], uint8_t response[20])
{
	int ret = 0;

	for(int index = 0; index < ARRAY_SIZE(interfaces); index++)
	{
		if(NULL != interfaces[index])
		{
			ret = interfaces[index]->exec(command, response);
			if(0 <= ret)
			{
				break;
			}
		}

	}

	return ret;
}

int app_gui_reset()
{
	int ret = 0;

	for(int index = 0; index < ARRAY_SIZE(interfaces); index++)
	{
		if(NULL != interfaces[index])
			ret |= interfaces[index]->reset();
	}


	return ret;
}
