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

#ifndef DRIVERS_GUI_WINDOWS_GUI_PACKET_DEFS_H_
#define DRIVERS_GUI_WINDOWS_GUI_PACKET_DEFS_H_


#define COMMAND_PACKETSIZE			(20)
#define RESPONSE_PACKET_SIZE		(20)

/* Group IDs */
enum {
	TGT_NRF = 0,
	TGT_NIM,
	TGT_I2C,
	TGT_SENS_OPT,
	TGT_SENS_ACC,
	TGT_DBG,
	TGT_UNUSED,

	TGT_ALGO_CONF = 0x27,
	TGT_TEMP_AMB_CONF = 0x28,
	TGT_TEMP_SKIN_CONF = 0x29,

};

// NRF specific commands
enum {
	NRF_FWVER = 0,
	NRF_ENABLE_SENSORS,
	NRF_CONFIG_VLED,
	NRF_CONFIG_VDD,
	NRF_RTCPRESCALE,
	NRF_USEACC,
	NRF_CONFIG_VDDLDO,
	NRF_CONFIG_BBST,
	NRF_BATTERY,
	NRF_NUMPD,
	NRF_CONNPARAMS,
	NRF_CONFIG_PMIC,
	NRF_STATUS_LED = 0x20,
	NRF_ENABLE_TEMP = 0x21,
	NRF_TEMP_PERIOD = 0x22,
    NRF_LOC_FIND = 0x23,
	NRF_SET_TIME = 0x24,
	NRF_SPO2_APP_CONF = 0X25,
};

// Target Device Commands
enum {
	READREG = 0,
	RMW,	// read-modify-write (modify bit range)
	REGBLOCKREAD,
};

// NIM specific commands
enum {
	NIM_FWVER = 0,
	NIM_ENABLE_FLASHLOG,
	NIM_FLASHLOG_WRITE,
	NIM_ISBUSY,
	NIM_ISFULL,
	NIM_ENABLE_FCLK,
};

/* Message Packet Formats */
enum {
	MSG_NRF_FWVER = 0,
	MSG_NIM_FWVER,
	MSG_ENABLE_SENSORS,
	MSG_ENABLE_FLASHLOG,
	MSG_REGVAL,
	MSG_NRF_CONFIG_VLED,
	MSG_NRF_CONFIG_VDD,
	MSG_NRF_RTCPRESCALE,
	MSG_NRF_USEACC,
	MSG_NIM_ISBUSY,
	MSG_NIM_ISFULL,
	MSG_NRF_CONFIG_VDDLDO,
	MSG_NRF_CONFIG_BBST,
	MSG_NRF_BATTERY,
	MSG_NRF_NUMPD,
	MSG_NRF_CONNPARAMS,
	MSG_NRF_CONFIG_PMIC,
	MSG_NIM_ENABLE_FCLK,
	MSG_REGBLOCKREAD,
	DBG_MSG,

	MSG_STATUS_LED  = 0x40,
	MSG_ENABLE_TEMP = 0x41,
	MSG_TEMP_PERIOD = 0x42,
    MSG_LOC_FIND = 0x43,
	MSG_SET_TIME = 0x44,
	MSG_SPO2_APP_CONF = 0X45,

};


/* Algohub Group's Field IDs */
enum{
	ALGO_CONF_BASE = 0,
	ALGO_CONF_MODE_SELECTION,
	ALGO_CONF_AFE_CONTROL,
	ALGO_CONF_AEC_SETTINGS,
	ALGO_CONF_AGC_SETTINGS,
	ALGO_CONF_LED_PD_SELECTION,
	ALGO_CONF_SCS_SETTINGS,
	ALGO_CONF_SPO2_SETTINGS,

	ALGO_CONF_MODE = 0xFE,
};

enum{
	ALGO_AFE_TYPE_NONE = 0,
	ALGO_AFE_TYPE_AEC,
	ALGO_AFE_TYPE_AGC,
	ALGO_AFE_TYPE_MAX
};

/* Algohub Group AEC Sub Field IDs */
enum{
	ALGO_AEC_INITIAL_INTEGRATION_TIME = 0,
	ALGO_AEC_MIN_INTEGRATION_TIME,
	ALGO_AEC_MAX_INTEGRATION_TIME,
	ALGO_AEC_INIT_SAMPLING_FS,
	ALGO_AEC_MIN_SAMPLING_FS,
	ALGO_AEC_MAX_SAMPLING_FS,
	ALGO_AEC_MIN_PD_CURRENT,
	ALGO_AEC_INIT_PD_CURRENT,
	ALGO_AEC_TARGET_PD_CURRENT_PERIOD,
	ALGO_AEC_MOTION_DETECT_THRS,
	ALGO_AEC_DAC_OFFSET,
};

/* Algohub Group Spo2 Sub Field IDs */
enum{
	ALGOHUB_SPO2_INIT_SAMPLING_FS = 0,
	ALGOHUB_SPO2_MIN_SAMPLING_FS,
	ALGOHUB_SPO2_MAX_SAMPLING_FS,
};

/* Algohub Group AGC Sub Field IDs */
enum{
	ALGO_AGC_TARGET_PD_CURRENT = 0x0B,
};

/* Algohub Group LED_PD Sub Field IDs */
enum{
	ALGO_LED_PD_WHRM_LED_PD_SELECTION = 0,
	ALGO_LED_PD_WSPO2_LED_PD_SELECTION,
};


/* Algohub Group SCD Field IDs */
enum{
	ALGO_SCD_SETTING = 0,
};

/* SpO2 App Settigns Enums*/
enum{
	SPO2_APP_SETTING_USE_PD1 = 0,
	SPO2_APP_SETTING_USE_PD2,
	SPO2_APP_SETTING_USE_PD_MAX,
};

enum{
	SPO2_APP_SETTING_MEAS_INTERVAL_30_MIN = 0,
	SPO2_APP_SETTING_MEAS_INTERVAL_60_MIN,
	SPO2_APP_SETTING_MEAS_INTERVAL_90_MIN,
	SPO2_APP_SETTING_MEAS_INTERVAL_LIMIT,
	SPO2_APP_SETTING_MEAS_CONTINIOUS = 0XFF,
};





#endif /* DRIVERS_GUI_WINDOWS_GUI_PACKET_DEFS_H_ */
