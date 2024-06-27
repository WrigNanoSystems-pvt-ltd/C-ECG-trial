/*
 * app_interface_process.h
 *
 *  Created on: 1 Tem 2021
 *      Author: Erdem.Simsek
 */

#ifndef DRIVERS_GUI_APP_INTERFACE_PROCESS_H_
#define DRIVERS_GUI_APP_INTERFACE_PROCESS_H_


typedef enum{
	APP_INTERFACE_DEV_BLE = 0,

	APP_INTERFACE_MAX
} app_interface_dev_t;

typedef int (*command_parser)(uint8_t const * const p_cmd, uint16_t len);

int app_interface_process_init();

int app_interface_process_cmd(app_interface_dev_t dev, uint8_t const * const p_cmd, uint16_t len);

int app_interface_process_reset();


#endif /* DRIVERS_GUI_APP_INTERFACE_PROCESS_H_ */
