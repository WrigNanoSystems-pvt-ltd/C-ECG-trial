/*
 * app_interface_ble_process.c
 *
 *  Created on: 1 Tem 2021
 *      Author: Erdem.Simsek
 */


#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app_interface_ble_process.h"
#include "gui.h"
#include "MWA/app_mwa.h"

static uint8_t is_first_cmd_recevied = 0;
static app_interface_ble_gui_t gui = APP_INTERFACE_BLE_MRD104GUI;
static const char wellness_app_identifier[] = "get_device_info\n";

int app_interface_ble_process_init()
{
	int ret = 0;


	return ret;
}


int app_interface_ble_process_cmd(uint8_t const * const p_cmd, uint16_t len)
{
	int ret = 0;

	if(0 == is_first_cmd_recevied)
	{
		is_first_cmd_recevied = 1;

		/* If first command is "get_device_info", set the gui to the Wellnes App*/
		if(0 == memcmp(wellness_app_identifier, p_cmd, len))
			gui = APP_INTERFACE_BLE_WELLNESSAPP;
	}
	switch(gui)
	{
		case APP_INTERFACE_BLE_MRD104GUI:
			ble_data_handler(p_cmd, len);
			break;
		case APP_INTERFACE_BLE_WELLNESSAPP:
			mwa_parse_cmd(p_cmd);
			break;

		default:
			break;
	}

	return ret;
}


int app_interface_ble_reset()
{
	int ret = 0;

	/* Default GUI is MRD104 GUI */
	gui = APP_INTERFACE_BLE_MRD104GUI;
	is_first_cmd_recevied = 0;

	return ret;
}

app_interface_ble_gui_t app_interface_ble_selected()
{
	return gui;
}
