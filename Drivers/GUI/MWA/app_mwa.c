/*
 * app_mwa.c
 *
 *  Created on: 2 Tem 2021
 *      Author: Erdem.Simsek
 */

#include <stdio.h>
#include <string.h>

#include "utils.h"
#include "version.h"
#include "mxc_errors.h"
#include "ble_api.h"

#include "algohub_config_api.h"
#include "sh_comm.h"
#include "app_gui.h"
#include "main.h"
#include "algohub_sensorhub_manager.h"

#include "app_mwa.h"

#define CONSOLE_STR_BUFF_SIZE		(1536u)
#define COMMAND_OUTPUT_SIZE			(1024u)

typedef struct {
	const char * command;
	void (*cmd_func)(const char *msg);
	const char * help;
} cmdWAPInterface_t;

static void stop_sensors(const char *msg);
static void stop(const char *msg);
static void restart(const char *msg);
static void get_device_info(const char *msg);
static void get_battery_level(const char *msg);
static void set_cfg_report1(const char *msg);
static void set_cfg_report2(const char *msg);
static int check_for_command(const char *str);
static int is_command_found(const char* cmd_table, const char* rx_cmd);
static void set_cfg_lcd_time(const char *msg);
static void get_cfg_sh_dhparams(const char *msg);
static void set_cfg_sh_dhlpublic(const char *msg);
static void get_cfg_sh_dhrpublic(const char *msg);
static void get_cfg_sh_auth(const char *msg);
static void set_cfg_wearablesuite_initialhr(const char *msg);
static void set_cfg_wearablesuite_personheight(const char *msg);
static void set_cfg_wearablesuite_personweight(const char *msg);
static void set_cfg_wearablesuite_personage(const char *msg);
static void set_cfg_wearablesuite_persongender(const char *msg);
static void set_cfg_wearablesuite_scdenable(const char *msg);
static void set_cfg_blepower(const char *msg);
static void set_cfg_event_mode(const char *msg);
static void set_cfg_wearablesuite_algomode(const char *msg);
static void set_cfg_flash_log(const char *msg);
static void set_cfg_stream_bin(const char *msg);
static void read_ppg_9(const char *msg);

static int get_command_len(const char * const cmd);
static int parse_public_str(const char *ptr_ch, const char *cmd, uint8_t *data, int data_sz);

const static cmdWAPInterface_t CMDTABLE_WAINTERFACE[] =
{
		{ "stop_sensors\n"             		,     	stop_sensors              	,        	"Stops all sensors."                                         	},
		{ "stop\n"                     		,     	stop                      	,        	"Stops the algorithm and all sensors."                       	},
		{ "restart\n"                  		,     	restart                   	,        	"Restarts MAX32630."                                         	},
		{ "get_device_info\n"          		,     	get_device_info           	,        	"Gets the information about the device."                     	},
		{ "get_battery_level\n"        		,     	get_battery_level         	,        	"Gets the information about the battery level."              	},
		{ "set_cfg report 1\n"         		,     	set_cfg_report1			 	,        	"Sets the report mode to 1."                                 	},
		{ "set_cfg report 2\n"         		,     	set_cfg_report2			 	,        	"Sets the report mode to 2."                                 	},
		{ "set_cfg lcd time\n"				,		set_cfg_lcd_time			,			""																},
		{ "get_cfg sh_dhparams"				,		get_cfg_sh_dhparams			,			""																},
		{ "set_cfg sh_dhlpublic"			,		set_cfg_sh_dhlpublic		,			""																},
		{ "get_cfg sh_dhrpublic"			,		get_cfg_sh_dhrpublic		,			""																},
		{ "get_cfg sh_auth"					,		get_cfg_sh_auth				,			""																},
		{ "set_cfg wearablesuite initialhr" ,		set_cfg_wearablesuite_initialhr		,	""																},
		{ "set_cfg wearablesuite personheight"	,	set_cfg_wearablesuite_personheight	,	""																},
		{ "set_cfg wearablesuite personweight"	,	set_cfg_wearablesuite_personweight	,	""																},
		{ "set_cfg wearablesuite personage"	,		set_cfg_wearablesuite_personage		,	""																},
		{ "set_cfg wearablesuite persongender"	,	set_cfg_wearablesuite_persongender	,	""																},
		{ "set_cfg wearablesuite scdenable"	,		set_cfg_wearablesuite_scdenable		,	""																},
		{ "set_cfg blepower"				,		set_cfg_blepower			,			""																},
		{ "set_cfg event_mode"				,		set_cfg_event_mode			,			""																},
		{ "set_cfg wearablesuite algomode"	,		set_cfg_wearablesuite_algomode		,	""																},
		{ "set_cfg flash log"				,		set_cfg_flash_log			,			""																},
		{ "set_cfg stream bin"				,		set_cfg_stream_bin			,			""																},
		{ "read ppg 9"						,		read_ppg_9					,			""																}

};

static char cmd_tx[COMMAND_OUTPUT_SIZE];
static char cmd_rx[CONSOLE_STR_BUFF_SIZE];
static int data_len;
static int lcd_time_val = 0;

int mwa_parse_cmd(uint8_t const * const cmd)
{
	bool is_cmd_found = false;
	uint8_t index = 0;
	static int cmd_rx_pos = 0;

	memcpy(&cmd_rx[cmd_rx_pos], cmd, strlen((const char *)cmd));

	if(check_for_command(cmd_rx))
	{
		for(index = 0; index < ARRAY_SIZE(CMDTABLE_WAINTERFACE); index++){

			if(is_command_found(CMDTABLE_WAINTERFACE[index].command, cmd_rx)){
			//if(starts_with(CMDTABLE_WAINTERFACE[index].command, cmd_rx)){
				if(NULL != CMDTABLE_WAINTERFACE[index].cmd_func){
					// Delete the new line character
					int cmd_rx_len = get_command_len(cmd_rx);
					cmd_rx[cmd_rx_len] = '\0';
					CMDTABLE_WAINTERFACE[index].cmd_func(cmd_rx);
					is_cmd_found = true;
					break;
				}
			}
		}

		if(!is_cmd_found){
			data_len = snprintf(cmd_tx, sizeof(cmd_tx)-1, "\r\n%s err=-255\r\n", cmd);
		}

		memset(cmd_rx, 0, sizeof(cmd_rx));
		cmd_rx_pos = 0;
	}
	else
	{
		cmd_rx_pos += strlen((const char *)cmd);
	}


	return data_len;
}


static void stop_sensors(const char *msg)
{

}

static void stop(const char *msg)
{
	int resp_len = 0;

	app_main_evt_post(EVT_MEASUREMENT_STOP);
	resp_len = snprintf(cmd_tx, sizeof(cmd_tx)-1, "\r\n%s err=0\r\n", msg);

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void restart(const char *msg)
{

}

static void get_device_info(const char *msg)
{
	int resp_len = 0;

	uint8_t algover[3];
	ah_sh_read_software_version(algover);

	//Command
	resp_len = snprintf(cmd_tx, sizeof(cmd_tx)-1, "%s", msg);

	//Platform
	resp_len += snprintf(cmd_tx + resp_len, sizeof(cmd_tx)- resp_len, " platform=", PLATFORM_NAME);

	//FW Ver
	resp_len += snprintf(cmd_tx + resp_len, sizeof(cmd_tx)- resp_len, " firmware_ver=%s_%u_%u.%u", PLATFORM_TYPE, VER_MAJ, VER_MIN, VER_PATCH);

	//Adding sensors
	resp_len += snprintf(cmd_tx + resp_len, sizeof(cmd_tx)- resp_len, " sensors=ppg ");

	//Adding partname
	resp_len += snprintf(cmd_tx + resp_len, sizeof(cmd_tx)- resp_len, " part_name_ppg=max86178");

	//Adding algo ver
	resp_len += snprintf(cmd_tx + resp_len, sizeof(cmd_tx)- resp_len, " algo_ver_ppg=5.1.2");

	//Adding hub fw version
	resp_len += snprintf(cmd_tx + resp_len, sizeof(cmd_tx)- resp_len, " hub_firm_ver=%u.%u.%u", algover[0], algover[1], algover[2]);

	//Addign fw algos
	resp_len += snprintf(cmd_tx + resp_len, sizeof(cmd_tx)- resp_len, "  fw_algos=agc,aec,wspo2,whrm");

	resp_len += snprintf(cmd_tx + resp_len, sizeof(cmd_tx)- resp_len, "   err=0\r\n");

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void get_battery_level(const char *msg)
{

}

static void set_cfg_report1(const char *msg)
{

}

static void set_cfg_report2(const char *msg)
{

}

static void set_cfg_lcd_time(const char *msg)
{
	int resp_len = 0;
	int lcd_time_count = 0;
	char lcd_time_str[30];

	while(1)
    {
        if(msg[17+lcd_time_count] == '\0')
        {
            lcd_time_str[lcd_time_count] = '\0';
            break;
        }
        lcd_time_str[lcd_time_count] = msg[17+lcd_time_count];
        lcd_time_count++;
    }

    sscanf(lcd_time_str,"%d",&lcd_time_val);

    lcd_time_count=0;

    resp_len = snprintf(cmd_tx, sizeof(cmd_tx)-1, "\r\n%s err=0\r\n", msg);

    ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void get_cfg_sh_dhparams(const char *msg)
{
	int resp_len = 0;
	int status = 0;

	uint8_t rxBuff[6+1];  // first byte is status
	char outstr[2*sizeof(rxBuff)];
	int str_idx = 0;

	status = sh_get_dhparams(&rxBuff[0], sizeof(rxBuff));

    if (status == 0){

    	for (int i = 0; i < sizeof(rxBuff)-1; i++)
    		str_idx += snprintf(outstr + str_idx, sizeof(outstr) - str_idx - 1, "%02X", rxBuff[i+1]);

    	resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s value=%s err=%d\r\n", msg, outstr, 0);

	} else{
		resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
	}

    ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_sh_dhlpublic(const char *msg)
{
	uint8_t LOCALPUBLIC[12];
	int resp_len = 0;
	int ret = 0;

	ret = parse_public_str(msg, "set_cfg sh_dhlpublic", LOCALPUBLIC, sizeof(LOCALPUBLIC));
	if (ret) {
		resp_len = snprintf(cmd_tx,sizeof(cmd_tx),"%s err=%d\r\n", msg, -254);
	}
	else
	{
		int status = sh_set_dhlocalpublic(&LOCALPUBLIC[0], sizeof(LOCALPUBLIC));
	    if (status == 0){
	    	resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
		}
		else{
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
		}
	}

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}


static void get_cfg_sh_dhrpublic(const char *msg)
{
	int status = 0;
	int resp_len = 0;
	uint8_t rxBuff[12+1];  // first byte is status
	char outstr[2*sizeof(rxBuff)];
	int str_idx = 0;

	status = sh_get_dhremotepublic( &rxBuff[0], sizeof(rxBuff) );
    if (status == 0){
    	for (int i = 0; i < sizeof(rxBuff)-1; i++)
    		str_idx += snprintf(outstr + str_idx, sizeof(outstr) - str_idx - 1, "%02X", rxBuff[i+1]);

    	resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s value=%s err=%d\r\n", msg, outstr, 0);

	}else{

		resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
	}

    ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void get_cfg_sh_auth(const char *msg)
{
	uint8_t rxBuff[32+1];  // first byte is status
	char outstr[2*sizeof(rxBuff)];
	int str_idx = 0;
	int status = 0;
	int resp_len = 0;

	status = sh_get_authentication( &rxBuff[0], sizeof(rxBuff) );
	if(status == 0) {

    	for (int i = 0; i < sizeof(rxBuff)-1; i++)
    		str_idx += snprintf(outstr + str_idx, sizeof(outstr) - str_idx - 1, "%02X", rxBuff[i+1]);

    	resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s value=%s err=%d\r\n", msg, outstr, 0);

    }
	else{
    	resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
    }

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_wearablesuite_initialhr(const char *msg)
{
	int ret;
	int status = 0;
	int resp_len = 0;
	uint32_t val = 0;

	ret = (parse_cmd_data(msg, "set_cfg wearablesuite initialhr", &val, 1, true) != 1);
	if (ret) {
		resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -254);
	}
	else
	{
		status = ah_set_cfg_wearablesuite_initialhr(val);
		if (status == 0){
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
		}
		else{
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
		}
	}

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_wearablesuite_personheight(const char *msg)
{
	int ret;
	int status = 0;
	int resp_len = 0;
	uint32_t val = 0;

	ret = (parse_cmd_data(msg, "set_cfg wearablesuite personheight", &val, 1, true) != 1);
	if (ret) {
		resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -254);
	}
	else
	{
		status = ah_set_cfg_wearablesuite_personheight(val);
		if (status == 0)
		{
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
		}
		else
		{
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
		}
	}

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_wearablesuite_personweight(const char *msg)
{
	uint32_t val;
	int ret = 0;
	int status = 0;
	int resp_len = 0;


	ret = (parse_cmd_data(msg, "set_cfg wearablesuite personweight", &val, 1, true) != 1);
	if (ret) {
		resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -254);
	}
	else
	{
		status = ah_set_cfg_wearablesuite_personweight(val);
		if (status == 0)
		{
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
		}
		else
		{
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
		}
	}

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_wearablesuite_personage(const char *msg)
{
	uint32_t val;
	int ret = 0;
	int status = 0;
	int resp_len = 0;

	ret = (parse_cmd_data(msg, "set_cfg wearablesuite personage", &val, 1, true) != 1);
	if (ret) {
		resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -254);
	}
	else
	{
		status = ah_set_cfg_wearablesuite_personage(val);
		if (status == 0){
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
		}
		else{
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);

		}
	}

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_wearablesuite_persongender(const char *msg)
{
	uint32_t val;
	int ret = 0;
	int status = 0;
	int resp_len = 0;

	ret = (parse_cmd_data(msg, "set_cfg wearablesuite persongender", &val, 1, true) != 1);
	if (ret) {
		resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -254);
	}
	else
	{
		status = ah_set_cfg_wearablesuite_persongender(val);
		if (status == 0){
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
		}
		else{
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
		}
	}

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_wearablesuite_scdenable(const char *msg)
{
	uint32_t val;
	int ret = 0;
	int status = 0;
	int resp_len = 0;

	ret = (parse_cmd_data(msg, "set_cfg wearablesuite scdenable", &val, 1, true) != 1);
	if (ret) {
		resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -254);
	}
	else
	{
		status = ah_set_cfg_wearablesuite_scdenable(val);
		if (status == 0){
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
		}
		else{
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
		}
	}

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_blepower(const char *msg)
{
	int resp_len;
	resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_event_mode(const char *msg)
{
	int resp_len;
	resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_wearablesuite_algomode(const char *msg)
{
	uint32_t val;
	int ret = 0;
	int status = 0;
	int resp_len = 0;

	ret = (parse_cmd_data(msg, "set_cfg wearablesuite algomode", &val, 1, true) != 1);
	if (ret) {
		resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -254);
	}
	else
	{
		status = ah_set_cfg_wearablesuite_algomode(val);
		if (status == 0){
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
		}
		else{
			resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
		}
	}

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_flash_log(const char *msg)
{
	int resp_len;
	resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, -1);
	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void set_cfg_stream_bin(const char *msg)
{
	int resp_len;
	resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);
	ble_send_notify((uint8_t *)cmd_tx, resp_len);
}

static void read_ppg_9(const char *msg)
{
	int resp_len = 0;

	app_gui_set_ah_state(APP_ALGO_STATE_SH_ENABLED);
	ah_sh_get_spi_ownership(AH_SH_SPI_OWNER_SENSORUHB);
	app_main_evt_post(EVT_MEASUREMENT_START);

	resp_len = snprintf(cmd_tx, sizeof(cmd_tx), "\r\n%s err=%d\r\n", msg, 0);

	ble_send_notify((uint8_t *)cmd_tx, resp_len);
	// enable measurement
}

static int check_for_command(const char *str)
{
	int len = strlen(str);
	int command_found = 0;

	for(int ind = 0; ind < len; ind++)
		if(str[ind] == '\n')
			command_found = 1;

	return command_found;
}

static int is_command_found(const char* cmd_table, const char* rx_cmd)
{
	int cmd_table_len = strlen(cmd_table);
	int rx_cmd_len = strlen(rx_cmd);

	if(rx_cmd_len < cmd_table_len)
		return 0;

	for(int ind = 0; ind < (cmd_table_len - 1); ind++)
		if(cmd_table[ind] != rx_cmd[ind])
			return 0;

	return 1;
}

static int parse_public_str(const char *ptr_ch, const char *cmd, uint8_t *data, int data_sz)
{
	char ascii_byte[] = { 0, 0, 0 };
	const char* sptr = ptr_ch + strlen(cmd);
	int found = 0;
	int ssfound;
	unsigned int val32;

	//Eat spaces after cmd
	while (*sptr == ' ') { sptr++; }
	if (*sptr == '\0')
		return -1;

	while (found < data_sz) {
		if (*sptr == '\0')
			break;
		ascii_byte[0] = *sptr++;
		ascii_byte[1] = *sptr++;
		ssfound = sscanf(ascii_byte, "%x", &val32);
		if (ssfound != 1)
			break;
		*(data + found) = (uint8_t)val32;
		found++;
	}

	if (found < data_sz)
		return -1;
	return 0;
}

static int get_command_len(const char * const cmd)
{
	int len = 0;
	while(cmd[len] != '\n')
		len++;

	return len;
}
