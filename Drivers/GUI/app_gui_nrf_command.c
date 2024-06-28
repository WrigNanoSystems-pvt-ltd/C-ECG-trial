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
#include <string.h>
#include <stdbool.h>

#include "app_gui.h"
#include "windows_gui_packet_defs.h"
#include "Peripherals.h"

#include "algohub_sensorhub_manager.h"
#include "gui.h"

#include "version.h"
#include "main.h"
#include "max86178.h"

#include "mrd106.h"
#include "utils.h"



struct cmd_func_table{
	uint8_t target_dev;
	uint8_t command;
	int (*cmd)(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
};

enum{
	TARGET_DEVICE_ID,
	TARGET_MESSAGE,
};

static int nrf_int_init(void);
static int nrf_int_connected(void);
static int nrf_int_disconnected(void);
static int nrf_int_reset(void);
static int nrf_int_start(void);
static int nrf_int_stop(void);
static int nrf_int_exec(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);

static int nrf_int_firmware_version(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nrf_int_enanble_sensors(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nrf_int_config_vdd(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nrf_int_use_acc(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nrf_int_get_battery(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nrf_int_status_led(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nrf_int_enable_temp(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nrf_int_temp_period(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nrf_int_set_time(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nrf_int_spo2_app_conf(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int nrf_int_loc_find(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);

static struct cmd_func_table cmds[] = {
	{.target_dev = TGT_NRF,		.command = NRF_FWVER,			.cmd = nrf_int_firmware_version		},
	{.target_dev = TGT_NRF,		.command = NRF_ENABLE_SENSORS,	.cmd = nrf_int_enanble_sensors		},
	{.target_dev = TGT_NRF,		.command = NRF_CONFIG_VDD,		.cmd = nrf_int_config_vdd			},
	{.target_dev = TGT_NRF,		.command = NRF_USEACC,			.cmd = nrf_int_use_acc				},
	{.target_dev = TGT_NRF,		.command = NRF_BATTERY,			.cmd = nrf_int_get_battery			},
	{.target_dev = TGT_NRF,		.command = NRF_STATUS_LED,		.cmd = nrf_int_status_led			},
	{.target_dev = TGT_NRF,		.command = NRF_ENABLE_TEMP,		.cmd = nrf_int_enable_temp			},
	{.target_dev = TGT_NRF,		.command = NRF_TEMP_PERIOD,		.cmd = nrf_int_temp_period			},
	{.target_dev = TGT_NRF,		.command = NRF_SET_TIME,		.cmd = nrf_int_set_time				},
	{.target_dev = TGT_NRF,		.command = NRF_SPO2_APP_CONF,	.cmd = nrf_int_spo2_app_conf		},
    {.target_dev = TGT_NRF,     .command = NRF_LOC_FIND,        .cmd = nrf_int_loc_find            },
};

static os64wrapper_cfg_t adapConfig;
int optimum_fifo_threshold;
bool setFrameRdyIrq;


static mrd_t *m_mrd = NULL;

//extern uint8_t temp_sensor_sampling_freq_ind;
extern uint64_t rtc_time_update_value;
extern uint8_t spo2_meas_selected_pd;
extern uint8_t spo2_meas_selected_meas_int;
app_gui_cmd_interface_t sens_nrf =
{
		.init = nrf_int_init,
		.connected = nrf_int_connected,
		.disconnected = nrf_int_disconnected,
		.reset = nrf_int_reset,
		.start = nrf_int_start,
		.stop = nrf_int_stop,
		.exec = nrf_int_exec,
};

static int nrf_int_init(void)
{
	int ret = 0;
	m_mrd = getMRD();
	if(m_mrd == NULL)
	{
		ret = -1;
	}
	return ret;
}

static int nrf_int_connected(void)
{
	int ret = 0;

	return ret;
}

static int nrf_int_disconnected(void)
{
	int ret = 0;

	return ret;
}

static int nrf_int_reset(void)
{
	int ret = 0;

	return ret;
}

static int nrf_int_start(void)
{
	int ret = 0;

	return ret;
}

static int nrf_int_stop(void)
{
	int ret = 0;

	return ret;
}

static int nrf_int_exec(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	int ind = 0;

	ret = -1;

	for(ind = 0; ind < ARRAY_SIZE(cmds); ind++)
	{
		if(command[TARGET_DEVICE_ID] == cmds[ind].target_dev && command[TARGET_MESSAGE] == cmds[ind].command)
		{
			ret = 0;
			break;
		}
	}

	if(0 == ret)
	{
		memcpy(response, command, COMMAND_PACKETSIZE);
		ret = cmds[ind].cmd(command, response);
	}

	return ret;
}

static int nrf_int_firmware_version(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	response[0] = MSG_NRF_FWVER;
	response[1] = VER_MAJ;
	response[2] = VER_MIN;
	response[3] = (VER_YEAR & 0xFF00) >> 8;
	response[4] = VER_YEAR & 0xFF;
	response[5] = VER_MTH;
	response[6] = VER_DAY;
	response[7] = VER_PATCH;
	response[8] = VER_BUILD;

	ret = ah_sh_read_software_version(&response[9]);

	return ret;
}

static int nrf_int_enanble_sensors(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	ah_sh_spi_ownwers_t spi_owner = ah_sh_spi_ownership_status();

    uint8_t ch;
    uint8_t ppg;
    uint8_t ecg;
    uint8_t iq;
    uint8_t ecgfaster;
    uint8_t biozFaster;
    uint8_t diodes;
	uint8_t temp;

	if(AH_SH_SPI_OWNER_HOST == spi_owner)
	{

		if(0 == command[2]) // writeparam
		{
			if(command[3])
			{
				max86178_is_enabled(&temp);

				if(temp)
				{
					adapter_set_sensor_running();

					max86178_get_num_of_channel(&ch);
					max86178_is_ppg_enabled(&ppg);
					max86178_is_ecg_enabled(&ecg);
					max86178_is_iq_enabled(&iq);
					max86178_is_ecg_faster_than_ppg(&ecgfaster);
					max86178_is_bioz_faster_than_ppg(&biozFaster);
					max86178_get_num_photo_diodes(&diodes);

					if(0 == diodes)
					{
						ppg = 0;
						ch = 0;
					}


					adapConfig.UseAcc = adapter_get_acc_state();
					adapConfig.ecgEn = ecg;
					adapConfig.iqEn = iq;
					adapConfig.os64NumCh_enabled = ch;
					adapConfig.os64NumPhotoDiodes_enabled = diodes;
					adapConfig.os64_isEcgFasterThanPpg = ecgfaster;
					adapConfig.os64ppgFrameFirstChanTag = 0;
					adapConfig.ppgEn = ppg;

					adapter_init(&adapConfig, &optimum_fifo_threshold, &setFrameRdyIrq );

					if(ppg)
					{
						max86178_set_frame_ready_int(1);
						max86178_set_a_full_int(0);
					}
					else
					{

					if(!ppg && ecg)
					{
						max86178_set_fifo_a_full(optimum_fifo_threshold);
						max86178_set_frame_ready_int(0);
						max86178_set_a_full_int(1);
					}

					if(iq)
					{
						max86178_set_fifo_a_full(optimum_fifo_threshold);
						max86178_set_frame_ready_int(setFrameRdyIrq);
						max86178_set_a_full_int(1);

					}
					}
					adapter_set_periodic_packets(1);
					app_main_evt_post(EVT_MEASUREMENT_START);
				}
				else
				{
					adapter_clear_sensor_running();
					app_main_evt_post(EVT_MEASUREMENT_STOP);
				}
			}
			else{
				adapter_clear_sensor_running();
				if(m_mrd->getFlashLogEnable())
				{
					adapter_set_event(EVT_SENSORSTOP);
				}
				else
				{
					app_main_evt_post(EVT_MEASUREMENT_STOP);
				}

			}
		}
		else
		{
			response[0] = MSG_ENABLE_SENSORS;
			response[1] = adapter_is_reporting();
		}
	}
	else if (AH_SH_SPI_OWNER_SENSORUHB == spi_owner)
	{
		if(0 == command[2]) // writeparam
		{
			if(command[3]) // enable measurement
			{
				/* Default configuration for sensorhub */
				adapConfig.UseAcc = adapter_get_acc_state();
				adapConfig.ecgEn = 0;
				adapConfig.iqEn = 0;
				adapConfig.os64NumCh_enabled = 3;
				adapConfig.os64NumPhotoDiodes_enabled = 2;
				adapConfig.os64_isEcgFasterThanPpg = 0;
				adapConfig.os64ppgFrameFirstChanTag = 0;
				adapConfig.ppgEn = 1;

				adapter_set_sensor_running();

				adapter_init(&adapConfig, &optimum_fifo_threshold, &setFrameRdyIrq );

				adapter_set_periodic_packets(1);
				app_main_evt_post(EVT_MEASUREMENT_START);
			}
			else // disable measurement
			{
				adapter_clear_sensor_running();
				app_main_evt_post(EVT_MEASUREMENT_STOP);
			}
		}
		else // read param
		{
			response[0] = MSG_ENABLE_SENSORS;
			response[1] = adapter_is_reporting();
		}
	}
	else
	{
		ret = -1;
	}


	return ret;
}

static int nrf_int_config_vdd(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	response[0] = MSG_NRF_CONFIG_VDD;
	response[1] = 0;

	return ret;
}

static int nrf_int_use_acc(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(0 == command[2])
	{
		adapter_set_acc_state(command[3]);
	}
	else
	{
		response[0] = MSG_NRF_USEACC;
		response[1] = adapter_get_acc_state();
	}

	return ret;
}

static int nrf_int_get_battery(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	response[0] = MSG_NRF_BATTERY;
	response[1] = ((m_mrd->getChargingStatus() & 0x01) << 7) | (m_mrd->getBatteryPercentage() & 0x7F);

	return ret;
}

static int nrf_int_status_led(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(0 == command[2])
	{
		m_mrd->setStatusLedEnable(command[3]);
	}
	else
	{
		response[0] = MSG_STATUS_LED;
		response[1] = m_mrd->getStatusLedEnable();
	}

	return ret;
}

static int nrf_int_enable_temp(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(0 == command[2])
	{
		m_mrd->setTempSensorEnable(command[3]);
	}
	else
	{
		response[0] = MSG_STATUS_LED;
		response[1] = m_mrd->getTempSensorEnable();
	}

	return ret;
}

static int nrf_int_temp_period(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(0 == command[2])
	{
		m_mrd->setTempSensorFreq(command[3]);
		app_main_evt_post(EVT_TEMP_SENSOR_UPDATE);
	}
	else
	{
		response[0] = MSG_TEMP_PERIOD;
		response[1] = m_mrd->getTempSensorFreq();
	}

	return ret;
}

static int nrf_int_set_time(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	rtc_time_update_value = 0;
	if(0 == command[2])
	{
		for(int i = 0; i<8; i++){
			rtc_time_update_value +=  ((uint64_t)command[10-i]&0xFF)<<(0+i*8);
		}
		app_main_evt_post(EVT_RTC_UPDATE);
	}
	else
	{
		uint64_t rtc_time = utils_get_time_ms();
		response[0] = MSG_SET_TIME;
		for(int i = 0; i<8; i++){
			response[1+i] =  ( rtc_time>>(0+i*8) )& 0xFF;
		}
	}

	return ret;
}

static int nrf_int_spo2_app_conf(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]){

	int ret = 0;
	if(0 == command[2])
	{
		if(command[3] == 0xFF || command[3] < SPO2_APP_SETTING_MEAS_INTERVAL_LIMIT){
			spo2_meas_selected_meas_int= command[3];
		}

		if(command[4] < SPO2_APP_SETTING_USE_PD_MAX){
			spo2_meas_selected_pd = command[4];
		}
		app_main_evt_post(EVT_SPO2_APP_SETTINGS_UPDATE);
	}
	else
	{

		response[0] = MSG_SPO2_APP_CONF;
		response[1] = spo2_meas_selected_meas_int;
		response[2] = spo2_meas_selected_pd;

		return ret;
	}

}

static int nrf_int_loc_find(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
    int ret = 0;

    if(0 == command[2])
    {
        if(command[3] == 1)
        {
            app_main_evt_post(EVT_LOC_FIND_STARTED);
            adapConfig.UseAcc = adapter_get_acc_state();
            adapConfig.ecgEn = false;
            adapConfig.iqEn = false;
            adapConfig.os64NumCh_enabled = 2;
            adapConfig.os64NumPhotoDiodes_enabled = 2;
            adapConfig.os64_isEcgFasterThanPpg = false;
            adapConfig.os64ppgFrameFirstChanTag = 0;
            adapConfig.ppgEn = true;
            adapter_init(&adapConfig, &optimum_fifo_threshold, &setFrameRdyIrq );
            max86178_set_frame_ready_int(1);
            max86178_set_a_full_int(0);
        }
        else
        {
            app_main_evt_post(EVT_LOC_FIND_STOPPED);
        }


    }
    else
    {
        response[0] = MSG_LOC_FIND;
    }
    return ret;
}


