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

#ifndef DRIVERS_GUI_APP_GUI_H_
#define DRIVERS_GUI_APP_GUI_H_

#include <stdint.h>

#define COMMAND_PACKETSIZE			(20)
#define RESPONSE_PACKET_SIZE		(20)

#define LOG_FILENAME_LEN	(8u)

typedef enum{
	APP_ALGO_STATE_AH_SH_NOT_ENABLED = 0u,
	APP_ALGO_STATE_AH_ENABLED,
	APP_ALGO_STATE_SH_ENABLED,

	ALGO_STATE_MAX
} app_algo_state_t;

typedef struct {
	int (*init)(void);
	int (*connected)(void);
	int (*disconnected)(void);
	int (*reset)(void);
	int (*start)(void);
	int (*stop)(void);
	int (*exec)(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
} app_gui_cmd_interface_t;

struct flash_log_time{
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t reserved;
};

app_algo_state_t app_gui_get_ah_state();

void app_gui_set_ah_state(app_algo_state_t state);

int gui_is_enabled_flashlogging();

int gui_get_flash_logfile_name(char file_name[LOG_FILENAME_LEN]);

int gui_get_flash_log_start_time(struct flash_log_time * p_start_time);

int gui_get_flash_log_stop_time(struct flash_log_time * p_end_time);

int gui_get_flash_log_header(char *headerPtr);

uint64_t gui_get_flash_start_wall_time();

uint64_t gui_get_flash_stop_wall_time();

/****************************************************/
int app_gui_init();

int app_gui_reset();

int app_gui_connected();

int app_gui_disconnected();

int app_gui_start();

int app_gui_stop();

int app_gui_exec(uint8_t command[20], uint8_t response[20]);

#endif /* DRIVERS_GUI_APP_GUI_H_ */
