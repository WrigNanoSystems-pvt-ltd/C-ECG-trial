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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "app_gui.h"
#include "windows_gui_packet_defs.h"
#include "Peripherals.h"
#include "utils.h"
#include "main.h"

#include "mrd106.h"

static mrd_t *m_mrd = NULL;

struct cmd_func_table{
	uint8_t group_id;
	uint8_t field_id;
	int (*cmd)(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
};

struct wall_clock_time{
	uint64_t epoch_time_start;
	uint64_t rtc_timestamp;
};

enum{
	CMD_GRUOP_ID_POS	=	0u,
	CMD_FIELD_POS,
	CMD_WR_RD_OPERATION_POS,
	CMD_OPTION_BYTE,
	CMD_ARGS_BYTE1,
	CMD_ARGS_BYTE2,
	CMD_ARGS_BYTE3,
	CMD_ARGS_BYTE4,
	CMD_ARGS_BYTE5,
	CMD_ARGS_BYTE6,
};

static int nim_cmd_fw_ver(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nim_cmd_enable_flashlog(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nim_cmd_flashlog_write(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nim_cmd_is_busy(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nim_cmd_is_full(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nim_cmd_enable_fclk(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);

static int nim_int_init(void);
static int nim_int_connected(void);
static int nim_int_disconnected(void);
static int nim_int_reset(void);
static int nim_int_start();
static int nim_int_stop();
static int nim_int_exec(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);

static struct cmd_func_table cmd_func_table[] =
{
		{.group_id = TGT_NIM,	.field_id = NIM_FWVER,				.cmd = &nim_cmd_fw_ver				},
		{.group_id = TGT_NIM,	.field_id = NIM_ENABLE_FLASHLOG,	.cmd = &nim_cmd_enable_flashlog	},
		{.group_id = TGT_NIM,	.field_id = NIM_FLASHLOG_WRITE,		.cmd = &nim_cmd_flashlog_write		},
		{.group_id = TGT_NIM,	.field_id = NIM_ISBUSY,				.cmd = &nim_cmd_is_busy			},
		{.group_id = TGT_NIM,	.field_id = NIM_ISFULL,				.cmd = &nim_cmd_is_full			},
		{.group_id = TGT_NIM,	.field_id = NIM_ENABLE_FCLK,		.cmd = &nim_cmd_enable_fclk		},

};

//static int is_flash_logging_enabled = 0;
static char fileName[LOG_FILENAME_LEN];
static struct flash_log_time start_time;
static struct flash_log_time end_time;
static struct wall_clock_time wall_clock;
static char header[126];
static int headerCursor;
app_gui_cmd_interface_t nim_int =
{
		.init = nim_int_init,
		.connected = nim_int_connected,
		.disconnected = nim_int_disconnected,
		.reset = nim_int_reset,
		.start = nim_int_start,
		.stop = nim_int_stop,
		.exec = nim_int_exec,
};

int gui_get_flash_logfile_name(char file_name[LOG_FILENAME_LEN])
{
	memcpy(file_name, fileName, LOG_FILENAME_LEN);

	return 0;
}

int gui_get_flash_log_start_time(struct flash_log_time * p_start_time)
{
	if(NULL == p_start_time)
		return -1;

	memcpy(p_start_time, &start_time, sizeof(struct flash_log_time));
	return 0;
}

int gui_get_flash_log_stop_time(struct flash_log_time * p_end_time)
{
	if(NULL == p_end_time)
		return -1;

	memcpy(p_end_time, &end_time, sizeof(struct flash_log_time));
	return 0;
}

int gui_get_flash_log_header(char *headerPtr)
{
	memset(headerPtr, 0, sizeof(header));
	memcpy(headerPtr, header, sizeof(header));
}


uint64_t gui_get_flash_start_wall_time()
{
	/* There is a time difference between enabling external flash logging and enabling sensor measurement
	 * To compensate this time difference, application adds elapsed time to start time.*/
	return wall_clock.epoch_time_start + (utils_get_time_ms() - wall_clock.rtc_timestamp);
}

uint64_t gui_get_flash_stop_wall_time()
{
	return wall_clock.epoch_time_start + (utils_get_time_ms() - wall_clock.rtc_timestamp);
}

static int nim_int_init(void)
{
	int ret = 0;
	m_mrd = getMRD();
	if(m_mrd == NULL)
	{
		ret = -1;
	}

	return ret;
}

static int nim_int_connected(void)
{
	int ret = 0;

	return ret;
}

static int nim_int_disconnected(void)
{
	int ret = 0;

	return ret;
}

static int nim_int_reset(void)
{
	int ret = 0;

	return ret;
}

static int nim_int_start()
{
	int ret = 0;

	return ret;
}

static int nim_int_stop()
{
	int ret = 0;

	return ret;
}

static int nim_int_exec(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	int ind = 0;

	ret = -1;

	ret = -1;
	for(ind = 0; ind < ARRAY_SIZE(cmd_func_table); ind++)
	{
		if(command[CMD_GRUOP_ID_POS] == cmd_func_table[ind].group_id && command[CMD_FIELD_POS] == cmd_func_table[ind].field_id)
		{
			ret = 0;
			break;
		}
	}

	if(0 == ret)
	{
		memcpy(response, command, COMMAND_PACKETSIZE);
		ret = cmd_func_table[ind].cmd(command, response);
	}

	return ret;
}

static int nim_cmd_fw_ver(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	return 0;
}

static int nim_cmd_enable_flashlog(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{

	if(command[CMD_WR_RD_OPERATION_POS]) /*Read*/
	{
		response[0] = MSG_ENABLE_FLASHLOG;
		response[1] = m_mrd->getFlashLogEnable();
	}
	else /*Write*/
	{
		//is_flash_logging_enabled = command[CMD_OPTION_BYTE];
		m_mrd->setFlashLogEnable(command[CMD_OPTION_BYTE]);
		if(m_mrd->getFlashLogEnable())
		{
			start_time.year = ((uint16_t)command[4] << 8) | ((uint16_t)command[5]);
			start_time.month = command[6];
			start_time.day = command[7];
			start_time.hour = command[8];
			start_time.min = command[9];
			start_time.sec = command[10];

			struct tm t;
			time_t t_of_day;

			memset(&t, 0, sizeof(struct tm));

			t.tm_year = start_time.year-1900;  // Year - 1900
			t.tm_mon = start_time.month - 1;           // Month, where 0 = jan
			t.tm_mday = start_time.day;          // Day of the month
			t.tm_hour = start_time.hour;
			t.tm_min = start_time.min;
			t.tm_sec = start_time.sec;
			t.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
			t_of_day = mktime(&t);

			/* Multiply it with 1000 to convert resolution to ms from second */
			wall_clock.epoch_time_start = (uint64_t)((uint64_t)t_of_day * 1000);
			wall_clock.rtc_timestamp = utils_get_time_ms();

			memcpy(fileName, &command[12], LOG_FILENAME_LEN);
		}
		else
		{
			end_time.year = ((uint16_t)command[4] << 8) | ((uint16_t)command[5]);
			end_time.month = command[6];
			end_time.day = command[7];
			end_time.hour = command[8];
			end_time.min = command[9];
			end_time.sec = command[10];
			if(m_mrd->getFlashLogEnable())
			{
				app_main_evt_post(EVT_MEASUREMENT_STOP);
			}
		}

	}

	return 0;
}

static int nim_cmd_flashlog_write(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	memcpy(header+headerCursor, command+2, 18);
	headerCursor = headerCursor+18;
	if(headerCursor == 126)
	{
		headerCursor = 0;
	}
	return 0;
}

static int nim_cmd_is_busy(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	response[0] = MSG_NIM_ISBUSY;
	response[1] = 0;
	return 0;
}

static int nim_cmd_is_full(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	response[0] = MSG_NIM_ISFULL;
	response[1] = 0;
	return 0;
}

static int nim_cmd_enable_fclk(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	return 0;
}
