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

#include "app_gui.h"
#include "windows_gui_packet_defs.h"
#include "Peripherals.h"

#include "algohub_config_api.h"
#include "sensorhub_config_api.h"
#include "algohub_sensorhub_manager.h"
#include "max86178.h"
#include "sh_comm.h"

#define ALGO_WHRM_GRN0_CH_DEFAULT_SLOT		ALGOHUB_DEFAULT_WHRM0_CH_SLOT
#define ALGO_WHRM_GRN1_CH_DEFAULT_SLOT		ALGOHUB_DEFAULT_WHRM1_CH_SLOT
#define ALGO_SPO2_IR_CH_DEFAULT_SLOT		ALGOHUB_DEFAULT_IR_CH_SLOT
#define ALGO_SPO2_RED_CH_DEFAULT_SLOT		ALGOHUB_DEFAULT_RED_CH_SLOT
#define ALGO_WHRM_SPO2_MAX_SLOT_NUMBER		(2u)

#define ALGO_WHRM_GRN0_CH_DEFAULT_PD		ALGOHUB_DEFAULT_WHRM0_CH_PD
#define ALGO_WHRM_GRN1_CH_DEFAULT_PD		ALGOHUB_DEFAULT_WHRM1_CH_PD
#define ALGO_SPO2_IR_CH_DEFAULT_PD			ALGOHUB_DEFAULT_IR_CH_PD
#define ALGO_SPO2_RED_CH_DEFAULT_PD			ALGOHUB_DEFAULT_RED_CH_PD


#define ALGO_DEFAULT_FULL_ADC_RANGE			(32.0f)
#define ALGO_PD_CURRENT_ROUNDING_FIX		(0.5f)

#define DEFAULT_ALGO_MODE					(0u)
#define DEFAULT_AFE_CONTROL					(1u)

#define DEFAULT_WHRM_INT_INTEG_TIME			(3u)
#define DEFAULT_SPO2_INT_INTEG_TIME			(3u)
#define DEFAULT_MIN_INTEG_TIME				(0u)
#define DEFAULT_MAX_INTEG_TIME				(3u)

#define DEFAULT_WHRM_INT_AVG_SAMPLE			(0u)
#define DEFAULT_SPO2_INIT_AVG_SAMPLE		(2u)
#define DEFAULT_MIN_AVG_SAMPLE				(0u)
#define DEFAULT_MAX_AVG_SAMPLE				(4u)
#define DEFAULT_SPO2_MIN_AVG_SAMPLE			(2)
#define DEFAULT_SPO2_MAX_AVG_SAMPLE			(4)

#define DEFAULT_MIN_PD_CURR					(4u)
#define DEFAULT_INIT_PD_CURR				(10u)
#define DEFAULT_TARGET_PD_CURR_PERIOD		(18u)

#define DEFAULT_MOTION_DETECT_THS			(5u)

#define DEFAULT_TARGET_PD_CURR				(10u)

#define DEFAULT_INIT_DAC_OFFSET_PPG1		(0u)
#define DEFAULT_INIT_DAC_OFFSET_PPG2		(0u)

#define DEFAULT_WHRM_CHANNEL_CURR			(10u)
#define DEFAULT_SPO2_IR_CHANNEL_CURR		(20u)
#define DEFAULT_SPO2_RED_CHANNEL_CURR		(20u)

#define DEFAULT_SCD_SETTING					(0u)
#define DEFAULT_ALGO_ENABLE					(0u)

enum{
	CMD_GRUOP_ID_POS	=	0u,
	CMD_FIELD_POS,
	CMD_SUBFIELD_POS,
	CMD_WR_RD_OPERATION_POS,
	CMD_OPTION_BYTE,
	CMD_ARGS_BYTE1,
	CMD_ARGS_BYTE2,
	CMD_ARGS_BYTE3,
	CMD_ARGS_BYTE4,
	CMD_ARGS_BYTE5,
	CMD_ARGS_BYTE6,
};

enum{
	AVG_SAMPLING_25_TO_1 = 1,
	AVG_SAMPLING_50_TO_2 = 2,
	AVG_SAMPLING_100_TO_4 = 4,
	AVG_SAMPLING_200_TO_8 = 8,
	AVG_SAMPLING_400_TO_16 = 16,
};

typedef struct __attribute__((packed)){
	uint8_t algo_mode;
	uint8_t afe_control;
	struct {
		uint8_t slot[2];
		uint8_t pd[2];
	} whrm_ch;

	struct {
		uint8_t slot[2];
		uint8_t pd[2];
	} spo2_ch;

	uint8_t scd_setting;
	app_algo_state_t algo_state;

	struct {
		uint8_t int_integration_time[ALGO_WHRM_SPO2_MAX_SLOT_NUMBER];
		uint8_t min_integration_time;
		uint8_t max_integration_time;
		uint8_t int_avg_sampling[ALGO_WHRM_SPO2_MAX_SLOT_NUMBER];
		uint8_t min_avg_sampling[ALGO_WHRM_SPO2_MAX_SLOT_NUMBER];
		uint8_t max_avg_sampling[ALGO_WHRM_SPO2_MAX_SLOT_NUMBER];
		uint8_t min_pd_curr;
		uint8_t init_pd_curr;
		uint8_t target_pd_curr_period;
		uint8_t motion_detect_ths;
		uint8_t target_pd_curr;

		struct {
			uint8_t ppg1[ALGO_WHRM_SPO2_MAX_SLOT_NUMBER];
			uint8_t ppg2[ALGO_WHRM_SPO2_MAX_SLOT_NUMBER];
		} init_dac_offset;

		uint8_t led_curr[ALGO_WHRM_SPO2_MAX_SLOT_NUMBER];
	} control[ALGO_AFE_TYPE_MAX];

} algohub_gui_settings_t;

typedef int (*cmd_func)(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);

struct group_cmd_table{
	uint8_t group_family_id;
	uint8_t field_id;
	uint8_t subfield_id;
	cmd_func func;
};

static int algohub_cmd_algomode_selection(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_afe_control(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_init_integration_time(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_min_integration_time(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_max_integration_time(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_init_avg_sampling(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_min_avg_sampling(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_max_avg_sampling(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_min_pd_current(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_init_pd_current(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_target_pd_current_period(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_motion_detect_ths(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_init_dac_offset(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_target_pd_current(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_whrm_ch_sel(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_spo2_ch_sel(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_scd_setting(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_enable(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_spo2_int_sampling_avg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_spo2_min_sampling_avg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int algohub_cmd_spo2_max_sampling_avg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);

static int algo_int_init(void);
static int algo_int_connected(void);
static int algo_int_disconnected(void);
static int algo_int_reset(void);
static int algo_int_start(void);
static int algo_int_stop(void);
static int algo_int_exec(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);


static const algohub_gui_settings_t default_conf =
{
		/* Common settings between different AEC/AGC Control */
		.algo_mode = DEFAULT_ALGO_MODE,
		.afe_control = DEFAULT_AFE_CONTROL,
		.whrm_ch.slot[0] = ALGO_WHRM_GRN0_CH_DEFAULT_SLOT,
		.whrm_ch.slot[1] = ALGO_WHRM_GRN1_CH_DEFAULT_SLOT,
		.whrm_ch.pd[0] = ALGO_WHRM_GRN0_CH_DEFAULT_PD,
		.whrm_ch.pd[1] = ALGO_WHRM_GRN1_CH_DEFAULT_PD,
		.spo2_ch.slot[0] = ALGO_SPO2_IR_CH_DEFAULT_SLOT,
		.spo2_ch.slot[1] = ALGO_SPO2_RED_CH_DEFAULT_SLOT,
		.spo2_ch.pd[0] = ALGO_SPO2_IR_CH_DEFAULT_PD,
		.spo2_ch.pd[1] = ALGO_SPO2_RED_CH_DEFAULT_PD,
		.scd_setting = DEFAULT_SCD_SETTING,
		.algo_state = APP_ALGO_STATE_AH_ENABLED,

		.control[ALGO_AFE_TYPE_NONE] =
		{
				.int_integration_time[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_WHRM_INT_INTEG_TIME,
				.int_integration_time[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INT_INTEG_TIME,
				.int_integration_time[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INT_INTEG_TIME,
				.min_integration_time = DEFAULT_MIN_INTEG_TIME,
				.max_integration_time = DEFAULT_MAX_INTEG_TIME,

				.int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_WHRM_INT_AVG_SAMPLE,
				.int_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INIT_AVG_SAMPLE,
				.int_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INIT_AVG_SAMPLE,

				.min_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_MIN_AVG_SAMPLE,
				.min_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MIN_AVG_SAMPLE,
				.min_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MIN_AVG_SAMPLE,

				.max_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_MAX_AVG_SAMPLE,
				.max_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MAX_AVG_SAMPLE,
				.max_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MAX_AVG_SAMPLE,

				.min_pd_curr = DEFAULT_MIN_PD_CURR,
				.init_pd_curr = DEFAULT_INIT_PD_CURR,
				.target_pd_curr_period = DEFAULT_TARGET_PD_CURR_PERIOD,
				.motion_detect_ths = DEFAULT_MOTION_DETECT_THS,
				.target_pd_curr = DEFAULT_TARGET_PD_CURR,
				.init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG1,
				.init_dac_offset.ppg2[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG2,
				.init_dac_offset.ppg1[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG1,
				.init_dac_offset.ppg2[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG2,
				.init_dac_offset.ppg1[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG1,
				.init_dac_offset.ppg2[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG2,

				.led_curr[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_WHRM_CHANNEL_CURR,
				.led_curr[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_IR_CHANNEL_CURR,
				.led_curr[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_RED_CHANNEL_CURR,
		},

		.control[ALGO_AFE_TYPE_AEC] =
		{
				.int_integration_time[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_WHRM_INT_INTEG_TIME,
				.int_integration_time[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INT_INTEG_TIME,
				.int_integration_time[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INT_INTEG_TIME,
				.min_integration_time = DEFAULT_MIN_INTEG_TIME,
				.max_integration_time = DEFAULT_MAX_INTEG_TIME,

				.int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_WHRM_INT_AVG_SAMPLE,
				.int_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INIT_AVG_SAMPLE,
				.int_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INIT_AVG_SAMPLE,


				.min_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_MIN_AVG_SAMPLE,
				.min_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MIN_AVG_SAMPLE,
				.min_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MIN_AVG_SAMPLE,

				.max_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_MAX_AVG_SAMPLE,
				.max_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MAX_AVG_SAMPLE,
				.max_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MAX_AVG_SAMPLE,

				.min_pd_curr = DEFAULT_MIN_PD_CURR,
				.init_pd_curr = DEFAULT_INIT_PD_CURR,
				.target_pd_curr_period = DEFAULT_TARGET_PD_CURR_PERIOD,
				.motion_detect_ths = DEFAULT_MOTION_DETECT_THS,
				.target_pd_curr = DEFAULT_TARGET_PD_CURR,
				.init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG1,
				.init_dac_offset.ppg2[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG2,
				.init_dac_offset.ppg1[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG1,
				.init_dac_offset.ppg2[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG2,
				.init_dac_offset.ppg1[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG1,
				.init_dac_offset.ppg2[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG2,

				.led_curr[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_WHRM_CHANNEL_CURR,
				.led_curr[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_IR_CHANNEL_CURR,
				.led_curr[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_RED_CHANNEL_CURR,
		},

		.control[ALGO_AFE_TYPE_AGC] =
		{
				.int_integration_time[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_WHRM_INT_INTEG_TIME,
				.int_integration_time[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INT_INTEG_TIME,
				.int_integration_time[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INT_INTEG_TIME,
				.min_integration_time = DEFAULT_MIN_INTEG_TIME,
				.max_integration_time = DEFAULT_MAX_INTEG_TIME,

				.int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_WHRM_INT_AVG_SAMPLE,
				.int_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INIT_AVG_SAMPLE,
				.int_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_INIT_AVG_SAMPLE,


				.min_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_MIN_AVG_SAMPLE,
				.min_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MIN_AVG_SAMPLE,
				.min_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MIN_AVG_SAMPLE,

				.max_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_MAX_AVG_SAMPLE,
				.max_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MAX_AVG_SAMPLE,
				.max_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_MAX_AVG_SAMPLE,

				.min_pd_curr = DEFAULT_MIN_PD_CURR,
				.init_pd_curr = DEFAULT_INIT_PD_CURR,
				.target_pd_curr_period = DEFAULT_TARGET_PD_CURR_PERIOD,
				.motion_detect_ths = DEFAULT_MOTION_DETECT_THS,
				.target_pd_curr = DEFAULT_TARGET_PD_CURR,
				.init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG1,
				.init_dac_offset.ppg2[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG2,
				.init_dac_offset.ppg1[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG1,
				.init_dac_offset.ppg2[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG2,
				.init_dac_offset.ppg1[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG1,
				.init_dac_offset.ppg2[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_INIT_DAC_OFFSET_PPG2,

				.led_curr[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = DEFAULT_WHRM_CHANNEL_CURR,
				.led_curr[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = DEFAULT_SPO2_IR_CHANNEL_CURR,
				.led_curr[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = DEFAULT_SPO2_RED_CHANNEL_CURR,
		}
};

static algohub_gui_settings_t gui_conf;


static const struct group_cmd_table cmd_func_table[] =
{
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_MODE_SELECTION,	.subfield_id = 0xFF, 								.func = &algohub_cmd_algomode_selection			},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AFE_CONTROL,		.subfield_id = 0xFF, 								.func = &algohub_cmd_afe_control				},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_INITIAL_INTEGRATION_TIME,	.func = &algohub_cmd_init_integration_time		},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_MIN_INTEGRATION_TIME,		.func = &algohub_cmd_min_integration_time		},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_MAX_INTEGRATION_TIME,		.func = &algohub_cmd_max_integration_time		},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_INIT_SAMPLING_FS,			.func = &algohub_cmd_init_avg_sampling			},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_MIN_SAMPLING_FS,			.func = &algohub_cmd_min_avg_sampling			},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_MAX_SAMPLING_FS,			.func = &algohub_cmd_max_avg_sampling			},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_MIN_PD_CURRENT,				.func = &algohub_cmd_min_pd_current				},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_INIT_PD_CURRENT,			.func = &algohub_cmd_init_pd_current			},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_TARGET_PD_CURRENT_PERIOD,	.func = &algohub_cmd_target_pd_current_period	},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_MOTION_DETECT_THRS,			.func = &algohub_cmd_motion_detect_ths			},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AEC_SETTINGS,		.subfield_id = ALGO_AEC_DAC_OFFSET,					.func = &algohub_cmd_init_dac_offset			},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_AGC_SETTINGS,		.subfield_id = ALGO_AGC_TARGET_PD_CURRENT,			.func = &algohub_cmd_target_pd_current			},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_LED_PD_SELECTION,	.subfield_id = ALGO_LED_PD_WHRM_LED_PD_SELECTION,	.func = &algohub_cmd_whrm_ch_sel				},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_LED_PD_SELECTION,	.subfield_id = ALGO_LED_PD_WSPO2_LED_PD_SELECTION,	.func = &algohub_cmd_spo2_ch_sel				},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_SCS_SETTINGS,		.subfield_id = ALGO_SCD_SETTING,					.func = &algohub_cmd_scd_setting				},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_MODE,				.subfield_id = 0xFF,								.func = &algohub_cmd_enable						},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_SPO2_SETTINGS,	.subfield_id = ALGOHUB_SPO2_INIT_SAMPLING_FS,		.func = &algohub_cmd_spo2_int_sampling_avg		},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_SPO2_SETTINGS,	.subfield_id = ALGOHUB_SPO2_MIN_SAMPLING_FS,		.func = &algohub_cmd_spo2_min_sampling_avg		},
		{.group_family_id = TGT_ALGO_CONF,	.field_id = ALGO_CONF_SPO2_SETTINGS,	.subfield_id = ALGOHUB_SPO2_MAX_SAMPLING_FS,		.func = &algohub_cmd_spo2_max_sampling_avg		},

};

static const uint8_t gui_to_algo[] = {AVG_SAMPLING_25_TO_1, AVG_SAMPLING_50_TO_2, AVG_SAMPLING_100_TO_4, AVG_SAMPLING_200_TO_8, AVG_SAMPLING_400_TO_16};

app_gui_cmd_interface_t algo_int =
{
		.init = algo_int_init,
		.connected = algo_int_connected,
		.disconnected = algo_int_disconnected,
		.reset = algo_int_reset,
		.start = algo_int_start,
		.stop = algo_int_stop,
		.exec = algo_int_exec,
};

app_algo_state_t app_gui_get_ah_state()
{
	return gui_conf.algo_state;
}

void app_gui_set_ah_state(app_algo_state_t state)
{
	gui_conf.algo_state = state;
}

static int algo_int_init(void)
{
	int ret = 0;

	gui_conf = default_conf;

	return ret;
}

static int algo_int_connected(void)
{
	int ret = 0;

	gui_conf = default_conf;

	return ret;
}

static int algo_int_disconnected(void)
{
	int ret = 0;


	return ret;
}

static int algo_int_reset(void)
{
	int ret = 0;

	return ret;
}

static int algo_int_start(void)
{
	int ret = 0;

	if(APP_ALGO_STATE_AH_ENABLED == gui_conf.algo_state)
	{
		uint8_t afe_type = gui_conf.afe_control;

		/* Setting AFE Controller */
		switch(afe_type)
		{
			case ALGO_AFE_TYPE_NONE:
				ret |= ah_set_cfg_wearablesuite_afeenable(0);
				break;
			case ALGO_AFE_TYPE_AEC:
				ret |= ah_set_cfg_wearablesuite_afeenable(1);
				ret |= ah_set_cfg_wearablesuite_autopdcurrentenable(1);
				break;
			case ALGO_AFE_TYPE_AGC:
				ret |= ah_set_cfg_wearablesuite_afeenable(1);
				ret |= ah_set_cfg_wearablesuite_autopdcurrentenable(0);
				break;
		}

		/* Setting initial integration time */
		ret |= ah_set_cfg_wearablesuite_initintoption(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_conf.control[afe_type].int_integration_time[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]);
		/* Spo2 AFE controller tries to maximize this value even if you provide an initial value. For that reason, the application sets maximum value as initial value. */
		ret |= ah_set_cfg_wearablesuite_maxtintoption(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_conf.control[afe_type].int_integration_time[ALGO_SPO2_IR_CH_DEFAULT_SLOT]);
		ret |= ah_set_cfg_wearablesuite_maxtintoption(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_conf.control[afe_type].int_integration_time[ALGO_SPO2_RED_CH_DEFAULT_SLOT]);

		/* Setting initial averaging sample */
		ret |= ah_set_cfg_wearablesuite_initfsmpoption(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]]);
		/* Spo2 AFE controller tries to maximize this value even if you provide an initial value. For that reason, the application sets maximum value as initial value. */

		ret |= ah_set_cfg_wearablesuite_initfsmpoption(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]]);
		ret |= ah_set_cfg_wearablesuite_minfsmpoption(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].min_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT]]);
		ret |= ah_set_cfg_wearablesuite_maxfsmpoption(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].max_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT]]);

		ret |= ah_set_cfg_wearablesuite_initfsmpoption(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]]);
		ret |= ah_set_cfg_wearablesuite_minfsmpoption(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].min_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT]]);
		ret |= ah_set_cfg_wearablesuite_maxfsmpoption(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].max_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT]]);

		/* Setting min_pd_curr */
		uint16_t min_pd_curr = 0;
		min_pd_curr = (uint16_t)(((float)gui_conf.control[afe_type].min_pd_curr / ALGO_DEFAULT_FULL_ADC_RANGE) * 1000);
		ret |= ah_set_cfg_wearablesuite_minpdcurrent(min_pd_curr);

		/* Setting initial pd current */
		uint16_t init_pd_curr = (uint16_t)(((float)gui_conf.control[afe_type].init_pd_curr / ALGO_DEFAULT_FULL_ADC_RANGE) * 1000);
		ret |= ah_set_cfg_wearablesuite_initialpdcurrent(init_pd_curr);

		/* Setting target pd current update period*/
		uint16_t target_pd_curr_upda_period = gui_conf.control[afe_type].target_pd_curr_period * 100;
		ret |= ah_set_cfg_wearablesuite_targetpdperiod(target_pd_curr_upda_period);

		/* Setting motion threshold */
		uint16_t motion_ths = gui_conf.control[afe_type].motion_detect_ths * 10;
		ret |= ah_set_cfg_wearablesuite_motionthreshold(motion_ths);

		ret |= ah_set_cfg_wearablesuite_initdacoffppg1(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]);
		ret |= ah_set_cfg_wearablesuite_initdacoffppg2(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]);

		uint8_t max_hr_dacoffset = gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] >= gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] \
					? gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] : gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT];
		ret |= ah_set_cfg_wearablesuite_maxdacoffset(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, max_hr_dacoffset);

		ret |= ah_set_cfg_wearablesuite_initdacoffppg1(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_SPO2_IR_CH_DEFAULT_SLOT]);
		ret |= ah_set_cfg_wearablesuite_initdacoffppg2(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_SPO2_IR_CH_DEFAULT_SLOT]);

		ret |= ah_set_cfg_wearablesuite_initdacoffppg1(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_SPO2_RED_CH_DEFAULT_SLOT]);
		ret |= ah_set_cfg_wearablesuite_initdacoffppg2(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_SPO2_RED_CH_DEFAULT_SLOT]);

		/* Setting target pd current */
		uint16_t target_pd_curr = (uint16_t)(((float)gui_conf.control[afe_type].target_pd_curr / ALGO_DEFAULT_FULL_ADC_RANGE) * 1000);
		ret |= ah_set_cfg_wearablesuite_targetpdcurrent(target_pd_curr);

		/*No need to send channel selection command they are fixed */
		ret |= ah_set_cfg_wearablesuite_scdenable(gui_conf.scd_setting);

		/* Applying sensor register */
		if(ALGO_AFE_TYPE_AEC == gui_conf.afe_control || ALGO_AFE_TYPE_AGC == gui_conf.afe_control)
		{
			max86178_set_leds_current(ALGO_SPO2_IR_CH_DEFAULT_SLOT, MAX86178_LED_DRIVER_A, gui_conf.control[afe_type].led_curr[ALGO_SPO2_IR_CH_DEFAULT_SLOT]);
			max86178_set_leds_current(ALGO_SPO2_RED_CH_DEFAULT_SLOT, MAX86178_LED_DRIVER_A, gui_conf.control[afe_type].led_curr[ALGO_SPO2_RED_CH_DEFAULT_SLOT]);
		}

		/* Setting LED Currents and informing algohub with new LED current configurations */
		uint8_t led_curr = 0;
		float curr_per_step = 0;
		uint16_t curr = 0;

		max86178_get_leds_current(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, MAX86178_LED_DRIVER_A, &led_curr);
		max86178_get_led_current_steps(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, &curr_per_step);
		curr = (led_curr * curr_per_step) * 10;
		ret |= ah_set_cfg_wearablesuite_initledcurr(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, curr);

		max86178_get_leds_current(ALGO_SPO2_IR_CH_DEFAULT_SLOT, MAX86178_LED_DRIVER_A, &led_curr);
		max86178_get_led_current_steps(ALGO_SPO2_IR_CH_DEFAULT_SLOT, &curr_per_step);
		curr = (led_curr * curr_per_step) * 10;
		ret |= ah_set_cfg_wearablesuite_initledcurr(ALGO_SPO2_IR_CH_DEFAULT_SLOT, curr);

		max86178_get_leds_current(ALGO_SPO2_RED_CH_DEFAULT_SLOT, MAX86178_LED_DRIVER_A, &led_curr);
		max86178_get_led_current_steps(ALGO_SPO2_RED_CH_DEFAULT_SLOT, &curr_per_step);
		curr = (led_curr * curr_per_step) * 10;
		ret |= ah_set_cfg_wearablesuite_initledcurr(ALGO_SPO2_RED_CH_DEFAULT_SLOT, curr);

		/* Setting integration time */
		max86178_set_integration_time(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_conf.control[afe_type].int_integration_time[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]);
		max86178_set_integration_time(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_conf.control[afe_type].int_integration_time[ALGO_SPO2_IR_CH_DEFAULT_SLOT]);
		max86178_set_integration_time(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_conf.control[afe_type].int_integration_time[ALGO_SPO2_RED_CH_DEFAULT_SLOT]);

		/* Setting averaging */
		max86178_set_average_sample(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]]);
		max86178_set_average_sample(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].int_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT]]);
		max86178_set_average_sample(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].int_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT]]);

		/* Setting initial dac offset */
		max86178_set_dac_offset(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, ALGO_WHRM_GRN0_CH_DEFAULT_PD, gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]);
		max86178_set_dac_offset(ALGO_WHRM_GRN1_CH_DEFAULT_SLOT, ALGO_WHRM_GRN1_CH_DEFAULT_PD, gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_WHRM_GRN1_CH_DEFAULT_SLOT]);
		max86178_set_dac_offset(ALGO_SPO2_IR_CH_DEFAULT_SLOT, ALGO_SPO2_IR_CH_DEFAULT_PD, gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_SPO2_IR_CH_DEFAULT_SLOT]);
		max86178_set_dac_offset(ALGO_SPO2_RED_CH_DEFAULT_SLOT, ALGO_SPO2_RED_CH_DEFAULT_PD, gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_SPO2_RED_CH_DEFAULT_SLOT]);
	}

	if(APP_ALGO_STATE_SH_ENABLED == gui_conf.algo_state)
	{
		uint8_t afe_type = gui_conf.afe_control;

		/* Setting AFE Controller */
		switch(afe_type)
		{
			case ALGO_AFE_TYPE_NONE:
				ret |= sh_set_cfg_wearablesuite_afeenable(0);
				break;
			case ALGO_AFE_TYPE_AEC:
				ret |= sh_set_cfg_wearablesuite_afeenable(1);
				ret |= sh_set_cfg_wearablesuite_autopdcurrentenable(1);
				break;
			case ALGO_AFE_TYPE_AGC:
				ret |= sh_set_cfg_wearablesuite_afeenable(1);
				ret |= sh_set_cfg_wearablesuite_autopdcurrentenable(0);
				break;
		}

		/* Setting initial integration time */
		ret |= sh_set_cfg_wearablesuite_initintoption(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_conf.control[afe_type].int_integration_time[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]);
		/* Spo2 AFE controller tries to maximize this value even if you provide an initial value. For that reason, the application sets maximum value as initial value. */
		ret |= sh_set_cfg_wearablesuite_maxtintoption(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_conf.control[afe_type].int_integration_time[ALGO_SPO2_IR_CH_DEFAULT_SLOT]);
		ret |= sh_set_cfg_wearablesuite_maxtintoption(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_conf.control[afe_type].int_integration_time[ALGO_SPO2_RED_CH_DEFAULT_SLOT]);

		/* Setting initial averaging sample */
		ret |= sh_set_cfg_wearablesuite_initfsmpoption(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]]);

		/* Spo2 AFE controller tries to maximize this value even if you provide an initial value. For that reason, the application sets maximum value as initial value. */
		ret |= sh_set_cfg_wearablesuite_initfsmpoption(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].int_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT]]);
		ret |= sh_set_cfg_wearablesuite_minfsmpoption(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].min_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT]]);
		ret |= sh_set_cfg_wearablesuite_maxfsmpoption(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].max_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT]]);

		ret |= sh_set_cfg_wearablesuite_initfsmpoption(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].int_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT]]);
		ret |= sh_set_cfg_wearablesuite_minfsmpoption(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].min_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT]]);
		ret |= sh_set_cfg_wearablesuite_maxfsmpoption(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_to_algo[gui_conf.control[afe_type].max_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT]]);

		/* Setting min_pd_curr */
		uint16_t min_pd_curr = 0;
		min_pd_curr = (uint16_t)(((float)gui_conf.control[afe_type].min_pd_curr / ALGO_DEFAULT_FULL_ADC_RANGE) * 1000);
		ret |= sh_set_cfg_wearablesuite_minpdcurrent(min_pd_curr);

		/* Setting initial pd current */
		uint16_t init_pd_curr = (uint16_t)(((float)gui_conf.control[afe_type].init_pd_curr / ALGO_DEFAULT_FULL_ADC_RANGE) * 1000);
		ret |= sh_set_cfg_wearablesuite_initialpdcurrent(init_pd_curr);

		/* Setting target pd current update period*/
		uint16_t target_pd_curr_upda_period = gui_conf.control[afe_type].target_pd_curr_period * 100;
		ret |= sh_set_cfg_wearablesuite_targetpdperiod(target_pd_curr_upda_period);

		/* Setting motion threshold */
		uint16_t motion_ths = gui_conf.control[afe_type].motion_detect_ths * 10;
		ret |= sh_set_cfg_wearablesuite_motionthreshold(motion_ths);

		/* Setting DAC Offset */
		ret |= sh_set_cfg_wearablesuite_initdacoffppg1(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]);
		ret |= sh_set_cfg_wearablesuite_initdacoffppg2(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT]);

		uint8_t max_hr_dacoffset = gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] >= gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] \
								? gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] : gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT];
		ret |= sh_set_cfg_wearablesuite_maxdacoffset(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, max_hr_dacoffset);

		ret |= sh_set_cfg_wearablesuite_initdacoffppg1(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_SPO2_IR_CH_DEFAULT_SLOT]);
		ret |= sh_set_cfg_wearablesuite_initdacoffppg2(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_SPO2_IR_CH_DEFAULT_SLOT]);

		ret |= sh_set_cfg_wearablesuite_initdacoffppg1(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg1[ALGO_SPO2_RED_CH_DEFAULT_SLOT]);
		ret |= sh_set_cfg_wearablesuite_initdacoffppg2(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_conf.control[afe_type].init_dac_offset.ppg2[ALGO_SPO2_RED_CH_DEFAULT_SLOT]);

		/* Readback LED currents and inform the sensorhub */
		if(ALGO_AFE_TYPE_NONE == gui_conf.afe_control)
		{
			/* All sensor registers will be cleared upon running sensorhub mode.
			 * Because of that, application loose LED current values written by GUI in No control mode
			 * For that reason, application reads back LED currents from sensor registers  over sensorhub
			 * and inform sensorhub to start with these led currents. In MRD104 implementation, green, ir, red channels
			 * are fixed */
			static const uint8_t green_led_addr = 0x25;
			static const uint8_t ir_led_addr = 0x2D;
			static const uint8_t red_led_addr = 0x35;
			uint32_t led_curr = 0;

			ret |= sh_get_reg(SH_SENSORIDX_MAX86178, green_led_addr, &led_curr);
			gui_conf.control[afe_type].led_curr[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = led_curr * (0.5);

			ret |= sh_get_reg(SH_SENSORIDX_MAX86178, ir_led_addr, &led_curr);
			gui_conf.control[afe_type].led_curr[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = led_curr * (0.5);

			ret |= sh_get_reg(SH_SENSORIDX_MAX86178, red_led_addr, &led_curr);
			gui_conf.control[afe_type].led_curr[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = led_curr * (0.5);

		}

		/* Setting initial led current */
		ret |= sh_set_cfg_wearablesuite_initledcurr(ALGO_WHRM_GRN0_CH_DEFAULT_SLOT, gui_conf.control[afe_type].led_curr[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] * 10);
		ret |= sh_set_cfg_wearablesuite_initledcurr(ALGO_SPO2_IR_CH_DEFAULT_SLOT, gui_conf.control[afe_type].led_curr[ALGO_SPO2_IR_CH_DEFAULT_SLOT] * 10);
		ret |= sh_set_cfg_wearablesuite_initledcurr(ALGO_SPO2_RED_CH_DEFAULT_SLOT, gui_conf.control[afe_type].led_curr[ALGO_SPO2_RED_CH_DEFAULT_SLOT] * 10);

		/* Setting target pd current */
		uint16_t target_pd_curr = (uint16_t)(((float)gui_conf.control[afe_type].target_pd_curr / ALGO_DEFAULT_FULL_ADC_RANGE) * 1000);
		ret |= sh_set_cfg_wearablesuite_targetpdcurrent(target_pd_curr);

		/*No need to send channel selection command they are fixed */
		ret |= sh_set_cfg_wearablesuite_scdenable(gui_conf.scd_setting);
	}

	return ret;
}

static int algo_int_stop(void)
{
	int ret = 0;

	return ret;
}

static int algo_int_exec(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	int ind = 0;

	ret = -1;
	for(ind = 0; ind < ARRAY_SIZE(cmd_func_table); ind++)
	{
		if(command[CMD_GRUOP_ID_POS] == cmd_func_table[ind].group_family_id && command[CMD_FIELD_POS] == cmd_func_table[ind].field_id && command[CMD_SUBFIELD_POS] == cmd_func_table[ind].subfield_id)
		{
			ret = 0;
			break;
		}
	}

	if(0 == ret)
	{
		memcpy(response, command, COMMAND_PACKETSIZE);
		ret = cmd_func_table[ind].func(command, response);
	}

	return ret;
}

static int algohub_cmd_algomode_selection(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.algo_mode = command[CMD_OPTION_BYTE];
		ret = 1;
	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.algo_mode;
	}

	return ret;
}

static int algohub_cmd_afe_control(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.afe_control = command[CMD_OPTION_BYTE];

		ret =1;

	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.afe_control;
	}

	return ret;
}

static int algohub_cmd_init_integration_time(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[ALGO_AFE_TYPE_NONE].int_integration_time[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AGC].int_integration_time[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];

		ret = 1;
	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.control[gui_conf.afe_control].int_integration_time[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT];
	}

	return ret;
}

static int algohub_cmd_min_integration_time(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[gui_conf.afe_control].min_integration_time = command[CMD_OPTION_BYTE];

		ret = 1;
	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.control[gui_conf.afe_control].min_integration_time;
	}

	return ret;
}

static int algohub_cmd_max_integration_time(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[gui_conf.afe_control].max_integration_time = command[CMD_OPTION_BYTE];

		ret = 1;
	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.control[gui_conf.afe_control].max_integration_time;
	}

	return ret;
}

static int algohub_cmd_init_avg_sampling(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[ALGO_AFE_TYPE_NONE].int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AGC].int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];

		ret = 1;
	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.control[gui_conf.afe_control].int_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT];
	}

	return ret;
}

static int algohub_cmd_min_avg_sampling(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[gui_conf.afe_control].min_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		ret = 1;
	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.control[gui_conf.afe_control].min_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT];
	}

	return ret;
}

static int algohub_cmd_max_avg_sampling(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[gui_conf.afe_control].max_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		ret = 1;
	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.control[gui_conf.afe_control].max_avg_sampling[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT];
	}

	return ret;
}

static int algohub_cmd_min_pd_current(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[gui_conf.afe_control].min_pd_curr = command[CMD_ARGS_BYTE1];
		ret = 1;
	}
	else
	{
		response[CMD_ARGS_BYTE1] = gui_conf.control[gui_conf.afe_control].min_pd_curr;
	}

	return ret;
}

static int algohub_cmd_init_pd_current(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[gui_conf.afe_control].init_pd_curr = command[CMD_ARGS_BYTE1];
		ret = 1;
	}
	else
	{
		response[CMD_ARGS_BYTE1] = gui_conf.control[gui_conf.afe_control].init_pd_curr;
	}

	return ret;
}

static int algohub_cmd_target_pd_current_period(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[gui_conf.afe_control].target_pd_curr_period = command[CMD_ARGS_BYTE1];
		ret = 1;
	}
	else
	{
		response[CMD_ARGS_BYTE1] =  gui_conf.control[gui_conf.afe_control].target_pd_curr_period;
	}

	return ret;
}

static int algohub_cmd_motion_detect_ths(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[gui_conf.afe_control].motion_detect_ths = command[CMD_ARGS_BYTE1];
		ret = 1;
	}
	else
	{
		response[CMD_ARGS_BYTE1] = gui_conf.control[gui_conf.afe_control].motion_detect_ths;
	}

	return ret;
}

static int algohub_cmd_init_dac_offset(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[ALGO_AFE_TYPE_NONE].init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = command[CMD_ARGS_BYTE1];
		gui_conf.control[ALGO_AFE_TYPE_NONE].init_dac_offset.ppg2[ALGO_WHRM_GRN1_CH_DEFAULT_SLOT] = command[CMD_ARGS_BYTE2];

		gui_conf.control[ALGO_AFE_TYPE_AGC].init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT] = command[CMD_ARGS_BYTE1];
		gui_conf.control[ALGO_AFE_TYPE_AGC].init_dac_offset.ppg2[ALGO_WHRM_GRN1_CH_DEFAULT_SLOT] = command[CMD_ARGS_BYTE2];

		ret = 1;
	}
	else
	{
		response[CMD_ARGS_BYTE1] = gui_conf.control[gui_conf.afe_control].init_dac_offset.ppg1[ALGO_WHRM_GRN0_CH_DEFAULT_SLOT];
		response[CMD_ARGS_BYTE2] = gui_conf.control[gui_conf.afe_control].init_dac_offset.ppg2[ALGO_WHRM_GRN1_CH_DEFAULT_SLOT];
	}

	return ret;
}

static int algohub_cmd_target_pd_current(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.control[gui_conf.afe_control].target_pd_curr = command[CMD_ARGS_BYTE1];
		ret = 1;
	}
	else
	{
		response[CMD_ARGS_BYTE1] = gui_conf.control[gui_conf.afe_control].target_pd_curr;
	}

	return ret;
}

static int algohub_cmd_whrm_ch_sel(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		if(command[CMD_ARGS_BYTE1])
		{
			gui_conf.whrm_ch.slot[0] = command[CMD_ARGS_BYTE2];
			gui_conf.whrm_ch.pd[0] = command[CMD_ARGS_BYTE3];

		}
		else
		{
			gui_conf.whrm_ch.slot[1] = command[CMD_ARGS_BYTE2];
			gui_conf.whrm_ch.pd[1] = command[CMD_ARGS_BYTE3];
		}
		ret = 1;
	}
	else
	{
		if(command[CMD_ARGS_BYTE1])
		{
			response[CMD_ARGS_BYTE2] = gui_conf.whrm_ch.slot[0];
			response[CMD_ARGS_BYTE3] = gui_conf.whrm_ch.pd[0];
		}
		else
		{
			response[CMD_ARGS_BYTE2] = gui_conf.whrm_ch.slot[1];
			response[CMD_ARGS_BYTE3] = gui_conf.whrm_ch.pd[1] ;
		}

	}

	return ret;
}

static int algohub_cmd_spo2_ch_sel(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		if(0 == command[CMD_ARGS_BYTE1])
		{
			gui_conf.spo2_ch.slot[0] = command[CMD_ARGS_BYTE2];
			gui_conf.spo2_ch.pd[0] = command[CMD_ARGS_BYTE3];

		}

		if(1 == command[CMD_ARGS_BYTE1])
		{
			gui_conf.spo2_ch.slot[1] = command[CMD_ARGS_BYTE2];
			gui_conf.spo2_ch.pd[1] = command[CMD_ARGS_BYTE3];
		}
		ret = 1;
	}
	else
	{
		if(0 == command[CMD_ARGS_BYTE1])
		{
			response[CMD_ARGS_BYTE2] = gui_conf.spo2_ch.slot[0];
			response[CMD_ARGS_BYTE3] = gui_conf.spo2_ch.pd[0];
		}

		if(1 == command[CMD_ARGS_BYTE1])
		{
			response[CMD_ARGS_BYTE2] = gui_conf.spo2_ch.slot[1];
			response[CMD_ARGS_BYTE3] = gui_conf.spo2_ch.pd[1];
		}
	}

	return ret;
}

static int algohub_cmd_scd_setting(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.scd_setting = command[CMD_OPTION_BYTE];
		ret = 1;
	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.scd_setting;
	}

	return ret;
}

static int algohub_cmd_enable(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{
		gui_conf.algo_state = command[CMD_OPTION_BYTE];

		switch(gui_conf.algo_state)
		{
			case APP_ALGO_STATE_AH_SH_NOT_ENABLED:
				ret = ah_sh_get_spi_ownership(AH_SH_SPI_OWNER_HOST);
				break;

			case APP_ALGO_STATE_AH_ENABLED:
				ret = ah_sh_get_spi_ownership(AH_SH_SPI_OWNER_HOST);
				break;

			case APP_ALGO_STATE_SH_ENABLED:
				ret = ah_sh_get_spi_ownership(AH_SH_SPI_OWNER_SENSORUHB);
				break;
			case ALGO_STATE_MAX:
				break;
		}
		ret = 1;
	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.algo_state;
	}

	return ret;
}


static int algohub_cmd_spo2_int_sampling_avg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{

		gui_conf.control[ALGO_AFE_TYPE_NONE].int_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AEC].int_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AGC].int_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];

		gui_conf.control[ALGO_AFE_TYPE_NONE].int_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AEC].int_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AGC].int_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];

	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.control[ALGO_AFE_TYPE_NONE].int_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT];
	}

	return ret;
}


static int algohub_cmd_spo2_min_sampling_avg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{

		gui_conf.control[ALGO_AFE_TYPE_NONE].min_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AEC].min_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AGC].min_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];

		gui_conf.control[ALGO_AFE_TYPE_NONE].min_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AEC].min_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AGC].min_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];

	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.control[ALGO_AFE_TYPE_NONE].min_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT];
	}

	return ret;
}

static int algohub_cmd_spo2_max_sampling_avg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;

	if(command[CMD_WR_RD_OPERATION_POS])
	{

		gui_conf.control[ALGO_AFE_TYPE_NONE].max_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AEC].max_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AGC].max_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];

		gui_conf.control[ALGO_AFE_TYPE_NONE].max_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AEC].max_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];
		gui_conf.control[ALGO_AFE_TYPE_AGC].max_avg_sampling[ALGO_SPO2_RED_CH_DEFAULT_SLOT] = command[CMD_OPTION_BYTE];

	}
	else
	{
		response[CMD_OPTION_BYTE] = gui_conf.control[ALGO_AFE_TYPE_NONE].max_avg_sampling[ALGO_SPO2_IR_CH_DEFAULT_SLOT];
	}

	return ret;
}

