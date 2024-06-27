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

#ifndef DRIVERS_GUI_GUI_H_
#define DRIVERS_GUI_GUI_H_

/*
 * os64adapter.h
 *
 *  Created on: Aug 12, 2020
 *      Author: Yagmur.Gok
 */

#ifndef WEARABLES_MBED_OS64SOURCE_OS64ADAPTER_H_
#define WEARABLES_MBED_OS64SOURCE_OS64ADAPTER_H_


#include "queue.h"

#define SENSORQUEUESIZE	4*60	/* ECG @ 1024 Sps and PPG @ 20 Fps: there will be 51 ECG samples for every 1 PPG frame */
#define SENSORBLEBUFSIZE	SENSORQUEUESIZE*3
#define SENSORBUFSIZE	160
#define LOGBUFSIZE 		160
#define PACKETSIZE		20
#define NOTIQUEUESIZE	1000

#define ALGO_PACK_LOW_SNR_FLAG_POS				(0x00u)
#define ALGO_PACK_MOTION_FLAG_POS				(0x01u)
#define ALGO_PACK_LOW_PI_FLAG_POS				(0x02u)
#define ALGO_PACK_UNRELIABLE_R_FLAG_POS			(0x03u)

// events to be handled in main superloop
enum {
	EVT_SENSORINIT = 0,
	EVT_SENSORSTART,
	EVT_SENSORSTOP,
	EVT_FLASHLOGSTART,
	EVT_FLASHLOGSTOP,
	EVT_I2CREAD,
	EVT_I2CRMW,
	EVT_I2CBLOCKREAD,
	EVT_OPTREAD,
	EVT_OPTRMW,
	EVT_OPTBLOCKREAD,
	EVT_ACCREAD,
	EVT_ACCRMW,
	EVT_ACCBLOCKREAD,
	EVT_OPTINT,
	EVT_ACCINT,
	EVT_INIT,
	EVT_NIMFWVER,
	EVT_NIMRSPFWVER,
	EVT_NIMDATALOG,
	EVT_NIMDATATEST,
	EVT_TIMER1,
	EVT_TIMER2,
	EVT_TIMER3,
	EVT_TIMER4,
	EVT_RTCPRESCALE,
	EVT_PMIC_INT,
	EVT_PMIC_PFN,
	EVT_PMIC_INDIR_WRITE,
	EVT_PMIC_INDIR_READ_RDY,
	EVT_UPDATECONNPARAMS,
	EVT_PERIODICDATAREADY,
	EVT_TESTCONFIGSENSORS,
	EVT_GETNUMPD,
	EVT_FCLKSTART,
	EVT_FCLKSTOP,
	EVT_NIMREADFLASH,
	EVT_NIMRSPREADFLASH,
	EVT_I2CRMWSTREAM,
	EVT_OPTRMWSTREAM,
	EVT_ACCRMWSTREAM,
	EVT_USERSETTINGSGCHANGENOTI,
	NUM_EVTS,
};

// notification packet codes
enum {
	NOTI_SNSR0 = 0,
	NOTI_SNSR1,
	NOTI_SNSR2,
	NOTI_PERIODIC,
	NOTI_FLASHLOG,
	NOTI_DUMMY0,	/* not used; placeholder so that parser can work with OS64 */
	NOTI_I2CSETTINGSCHANGE,
	NOTI_OPTSETTINGSCHANGE,
	NOTI_ACCSETTINGSCHANGE,
	NOTI_DUMMY1,	/* not used; placeholder so that parser can work with OS64 */
	NOTI_SNSR3,
	NOTI_ECG,
	NOTI_TIMESTAMP0,
	NOTI_TIMESTAMP1,
	NOTI_IQ0,
	NOTI_IQ1,
	NOTI_ALGO,
	NOTI_TEMP,

    NOTI_LOC_FIND = 0x16,
	NOTI_ACC= 0x17,

	NOTI_ALGO_CONF = 0x27,
	NOTI_TEMP_CONF = 0x28,
	//NOTI_BIOZ = 0xE,

	NOTI_STOP_PACK = 0xFE,
	NOTI_DONT_CARE_PACK = 0xFF,
};

// sensor queue positions
enum {
	POS_QUEUE_ECG = 0,
	POS_QUEUE_IQ,
	POS_QUEUE_TIME,
	POS_QUEUE_OPT,
	NUM_QUEUES,
};

enum {
	TAG_PPG_MAX = 0x08, //	don't stop at MEAS 9 (0x08) since OVF or dark measurements might occur
	TAG_ECG = 0x0B,	// really 6 bits incl fast bit
	TAG_TIME = 0x0E,	// really 5 bits per the datasheet
	TAG_I = 0x09,
	TAG_Q = 0x0A,
	TAG_UTILITY = 0x0F,
};

#define TAG_PPG_MEAS1 0  // indicate start of a frame
#define ACCEL_NUM_BYTES_PER_SAMPLE	6
#define OS64_NUMBYTES_PER_ITEM	3

typedef struct{

	bool ppgEn;
	bool ecgEn;
	bool iqEn;
	bool os64_isEcgFasterThanPpg;
	bool UseAcc;
	uint8_t os64NumCh_enabled;
	uint8_t os64NumPhotoDiodes_enabled;
	uint8_t os64ppgFrameFirstChanTag;

}os64wrapper_cfg_t;


//typedef union {
//	struct {
//		uint32_t val:20;
//		uint32_t tag:4;
//		uint32_t:8;
//	};
//	uint32_t raw;
//} ppg_data_t;
//
//typedef union {
//	struct {
//		uint32_t val:18;
//		uint32_t tag:6;
//		uint32_t:8;
//	};
//	uint32_t raw;
//} ecg_data_t;
//
//typedef struct {
//	int16_t x;
//	int16_t y;
//	int16_t z;
//} accel_report_t;

typedef struct{
	uint8_t algo_mode;
	uint8_t hr;
	uint8_t hr_conf;
	uint8_t rr_msb;
	uint8_t rr_lsb;
	uint8_t rr_conf;
	uint8_t spo2;
	uint8_t spo2_conf;
	uint8_t r_val_msb;
	uint8_t r_val_lsb;
	uint8_t spo2_comp;
	uint8_t spo2_state;
	uint8_t activity;
	uint8_t scd_state;
	uint8_t flag_region;
} algo_packet_t;

extern uint8_t gUseAcc;
extern bool gPpgEn;
extern bool gEcgEn;
extern bool gIqEn;

int gui_set_msg_queue(queue_t* queue);
int gui_queue_reset();

int adapter_set_report_queue(queue_t* queue);
int adapter_init(const os64wrapper_cfg_t* os64config,
		          int* optimum_fifo_threshold , bool* setFrameRdyIrq );
int adapter_execute_push ( queue_t* tagQueue , queue_t* accQueue );

int adapter_execute_pop(void );
int adapter_read_frame(uint8_t buf[20]);

void format_data_algo(algo_packet_t * p_packet);
void format_data_temp(uint8_t *report_buf, uint64_t rtc_time);
void format_data_loc_find(uint8_t *buffer, uint16_t size);

void format_stop_packet();

void format_dont_care_packet();

// test functions
void adapter_check_queues(int*ecgItemCnt, int* ppgItemCnt, int* timeItemCnt, int* iqItemCnt);
int check_ble_packet_queue(void);

void ble_data_handler(uint8_t const * const p_rxdata, uint16_t rx_len);
void ble_sanity_handler();

int adapter_is_reporting();

void adapter_set_sensor_running();

void adapter_clear_sensor_running();

void adapter_set_acc_state(int en);

int adapter_get_acc_state();

void adapter_set_periodic_packets(int en);

void adapter_set_event(int evt);

int is_adapter_reporting_started();

int adapter_is_acc_enabled();

int get_sensorstop_status();

int get_periodicpacket_status();

void set_periodicpacket_status(uint8_t e);

#define DEBUG_PRINT(x,y)

#endif /* WEARABLES_MBED_OS64SOURCE_OS64ADAPTER_H_ */


#endif /* DRIVERS_GUI_GUI_H_ */
