/*
 * app_interface_ble_process.h
 *
 *  Created on: 1 Tem 2021
 *      Author: Erdem.Simsek
 */

#ifndef DRIVERS_GUI_APP_INTERFACE_BLE_PROCESS_H_
#define DRIVERS_GUI_APP_INTERFACE_BLE_PROCESS_H_

typedef enum{
	APP_INTERFACE_BLE_MRD104GUI = 0,
	APP_INTERFACE_BLE_WELLNESSAPP,

	APP_INTERFACE_BLE_MAX
} app_interface_ble_gui_t;

int app_interface_ble_process_init();

int app_interface_ble_process_cmd(uint8_t const * const p_cmd, uint16_t len);

int app_interface_ble_reset();

app_interface_ble_gui_t app_interface_ble_selected();

#endif /* DRIVERS_GUI_APP_INTERFACE_BLE_PROCESS_H_ */
