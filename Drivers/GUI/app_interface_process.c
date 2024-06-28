/*
 * app_interface_process.c
 *
 *  Created on: 1 Tem 2021
 *      Author: Erdem.Simsek
 */


#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app_interface_process.h"
#include "app_interface_ble_process.h"


static command_parser interfaces[APP_INTERFACE_MAX];

int app_interface_process_init()
{
	int ret = 0;

	memset(interfaces, 0u, sizeof(interfaces));

	interfaces[APP_INTERFACE_DEV_BLE] = app_interface_ble_process_cmd;

	return ret;
}

int app_interface_process_cmd(app_interface_dev_t dev, uint8_t const * const p_cmd, uint16_t len)
{
	int ret = 0;

	if((NULL == interfaces[dev]) || (NULL == p_cmd) || (0 == len))
		ret = -1;

	if(0 == ret)
	{
		ret = interfaces[dev](p_cmd, len);
	}


	return ret;
}

int app_interface_process_reset()
{
	int ret = 0;

	ret |= app_interface_ble_reset();

	return ret;
}
