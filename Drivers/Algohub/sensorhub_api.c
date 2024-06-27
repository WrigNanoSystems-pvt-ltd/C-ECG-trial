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

#include "Peripherals.h"

#include "mxc_errors.h"
#include "mxc_delay.h"

#include "sensorhub_api.h"
#include "sh_comm.h"

#define SS_PACKET_COUNTERSIZE					1

/* 3 Slots Measurement 2 PD */
#define SSMAX86178_MODE1_DATASIZE				18

/* 3 Axis */
#define SSACCEL_MODE1_DATASIZE					6

/* Sensorhub reporting mode 1 */
#define SSWHRM_WSPO2_SUITE_MODE1_DATASIZE		20

/* Sensorhub reporting mode 2 */
#define SSWHRM_WSPO2_SUITE_MODE2_DATASIZE		58

#define MAX_NUMBER_OF_TRANSACTION_IN_BYTE		(255)

typedef void (*parser) (void * , uint8_t *);

typedef enum{
	TIMESTAMP_DATA			=	0,
	MAX86178_MODE1_DATA,
	WHRM_SPO2_SUITE_MODE1,
	WHRM_SPO2_SUTITE_MODE2,
	ACCEL_MODE1_DATA,
	DATA_FORMAT_MAX
} data_format_t;

typedef union {
	struct {
		uint8_t max86178_enabled:1;
		uint8_t accel_enabled   :1;
		uint8_t whrm_wspo2_suite_enabled_mode1 :1;
		uint8_t whrm_wspo2_suite_enabled_mode2 :1;
		uint8_t timestamp_enabled:1;

		uint8_t reserved:3;
	};
	uint8_t status_vals;
} algo_sensor_status_t;

typedef struct{
	data_format_t data_type;
	int data_size;
	parser fp_parse;
} ss_instance_parser_t;

/* Data parser functions */
static void data_time_stamper(void * p_time_stamp, uint8_t* data_ptr);
static void accel_data_rx(void * p_accel_data, uint8_t* data_ptr);
static void max86178_data_rx(void * p_ppg_data, uint8_t* data_ptr);
static void whrm_wspo2_suite_data_rx_mode1(void * p_algo_data_mode1, uint8_t* data_ptr);
static void whrm_wspo2_suite_data_rx_mode2(void * p_algo_data_mode2, uint8_t* data_ptr);

static algo_sensor_status_t g_algo_sensor_stat;

static ss_instance_parser_t parsers[SH_NUM_CURRENT_ALGOS] =
{
		{TIMESTAMP_DATA,		 0, 									data_time_stamper					},
		{MAX86178_MODE1_DATA, 	 SSMAX86178_MODE1_DATASIZE,				max86178_data_rx					},
		{WHRM_SPO2_SUITE_MODE1,	 SSWHRM_WSPO2_SUITE_MODE1_DATASIZE,		whrm_wspo2_suite_data_rx_mode1		},
		{WHRM_SPO2_SUTITE_MODE2, SSWHRM_WSPO2_SUITE_MODE2_DATASIZE,		whrm_wspo2_suite_data_rx_mode2		},
		{ACCEL_MODE1_DATA,		 SSACCEL_MODE1_DATASIZE,				accel_data_rx},
};

int sensorhub_interface_init()
{
	int ret = 0;

	sh_init_hwcomm_interface();

	mxc_delay(MXC_DELAY_SEC(1));

	ret = sh_set_data_type(SS_DATATYPE_BOTH, true);

	if(0 == ret){
		ret = sh_set_fifo_thresh(1);
	}

	return ret;
}

int sensorhub_enable_sensors()
{
	int ret = 0;

	ret = sh_sensor_enable_(SH_SENSORIDX_ACCEL, 1, SH_INPUT_DATA_DIRECT_SENSOR);
	g_algo_sensor_stat.accel_enabled = 1;
	pr_info("SENSOR_ENABLED_ACCEL Ret %d \n", ret);

	if(0 == ret){
		/* Enabling OS6X with host supplies data */
		ret = sh_sensor_enable_(SH_SENSORIDX_MAX86178, 1, SH_INPUT_DATA_DIRECT_SENSOR);
		g_algo_sensor_stat.max86178_enabled =1;
		pr_info("SENSOR_ENABLED_OS64 Ret %d \n", ret);
	}

	return ret;
}

int sensorhub_disable_sensor()
{
	int ret = 0;

	ret = sh_sensor_disable(SH_SENSORIDX_ACCEL);
	g_algo_sensor_stat.accel_enabled = 0;
	pr_info("SENSOR_DISABLED_ACCEL Ret %d \n", ret);

	if(0 == ret){
		/* Enabling OS6X with host supplies data */
		ret = sh_sensor_disable(SH_SENSORIDX_MAX86178);
		g_algo_sensor_stat.max86178_enabled = 0;
		pr_info("SENSOR_DISABLED_OS64 Ret %d \n", ret);
	}

	return ret;
}

int sensorhub_enable_algo(sensorhub_report_mode_t mode)
{
	int ret = 0;

	ret = sh_enable_algo_(SS_ALGOIDX_WHRM_WSPO2_SUITE_OS6X , (int) mode);

	if(SENSORHUB_MODE_BASIC == mode){
		g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode1 = 1;
	}
	else{
		g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode2 = 1;
	}


	return ret;
}

int sensorhub_disable_algo()
{
	int ret = 0;

	ret = sh_disable_algo(SS_ALGOIDX_WHRM_WSPO2_SUITE_OS6X);

	g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode1 = 0;
	g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode2 = 0;

	return ret;
}

int sensorhub_get_output_sample_number(int * p_number_of_sample)
{
	int ret = 0;

	if(NULL == p_number_of_sample)
	{
		ret = E_BAD_PARAM;
	}

	if(E_NO_ERROR == ret)
	{
		ret = sh_num_avail_samples(p_number_of_sample);
	}

	return ret;
}

int sensorhub_get_result(sensorhub_output *  p_result)
{
	int ret = 0;
	int num_bytes_to_read = 0;
	uint8_t hubStatus = 0;
	uint8_t databuf[150];
	uint8_t *ptr;

	if(NULL == p_result)
	{
		ret = E_BAD_PARAM;
	}

	if(E_NO_ERROR == ret)
	{
		ret = sh_get_sensorhub_status(&hubStatus);
		pr_info("sh_get_sensorhub_status ret %d \n", ret);
	}

	if(hubStatus & SS_MASK_STATUS_DATA_RDY){
		pr_info("SS_MASK_STATUS_DATA_RDY\n");
	}
	else{
		ret = E_NONE_AVAIL;
	}

	if(hubStatus & SS_MASK_STATUS_FIFO_OUT_OVR){
		pr_info("SS_MASK_STATUS_FIFO_OUT_OVR\n");
	}

	if(0 == ret){

		memset(p_result, 0, sizeof(sensorhub_output));
		memset(databuf, 0, sizeof(databuf));

		num_bytes_to_read += SS_PACKET_COUNTERSIZE;

		if(g_algo_sensor_stat.max86178_enabled)
			num_bytes_to_read += SSMAX86178_MODE1_DATASIZE;

		if(g_algo_sensor_stat.accel_enabled)
			num_bytes_to_read += SSACCEL_MODE1_DATASIZE;

		if(g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode1)
			num_bytes_to_read += SSWHRM_WSPO2_SUITE_MODE1_DATASIZE;

		if(g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode2)
			num_bytes_to_read += SSWHRM_WSPO2_SUITE_MODE2_DATASIZE;

		ptr  = &databuf[2];

		WAIT_MS(5);

		ret = sh_read_fifo_data(1, num_bytes_to_read, databuf, sizeof(databuf));

		if(g_algo_sensor_stat.accel_enabled){
			parsers[ACCEL_MODE1_DATA].fp_parse(&p_result->acc_data, ptr);
			ptr += parsers[ACCEL_MODE1_DATA].data_size;
		}

		if(g_algo_sensor_stat.max86178_enabled){
			parsers[MAX86178_MODE1_DATA].fp_parse(&p_result->ppg_data, ptr);
			ptr += parsers[MAX86178_MODE1_DATA].data_size;
		}

		if(g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode1){
			parsers[WHRM_SPO2_SUITE_MODE1].fp_parse(&p_result->algo_data, ptr);
			ptr += parsers[WHRM_SPO2_SUITE_MODE1].data_size;
		}

		if(g_algo_sensor_stat.whrm_wspo2_suite_enabled_mode2){
			parsers[WHRM_SPO2_SUTITE_MODE2].fp_parse(&p_result->algo_data, ptr);
			ptr += parsers[WHRM_SPO2_SUTITE_MODE2].data_size;
		}
	}
	else{
		ret = E_NONE_AVAIL;
	}

	return ret;
}

int sensorhub_get_version(uint8_t algoVersion[3])
{
	int ret = 0;
	ret = sh_get_algo_version(algoVersion);

	return ret;
}

int sensorhub_reset_sensor_configuration()
{
	int ret = 0;

	ret |= sh_set_reg(SH_SENSORIDX_MAX86178, 0x10, 0x01, 1);

	ret |= sh_set_reg(SH_SENSORIDX_MAX86178, 0x10, (0x01 << 1), 1);
	ret |= sh_set_reg(SH_SENSORIDX_MAX86178, 0x15, (0x1E), 1);
	ret |= sh_set_reg(SH_SENSORIDX_MAX86178, 0x18, (0x80), 1);
	ret |= sh_set_reg(SH_SENSORIDX_MAX86178, 0x19, (0x9F), 1);
	ret |= sh_set_reg(SH_SENSORIDX_MAX86178, 0x1A, (0x3F), 1);
	ret |= sh_set_reg(SH_SENSORIDX_MAX86178, 0x1C, (0x1 << 5), 1);
	ret |= sh_set_reg(SH_SENSORIDX_MAX86178, 0x1D, (0x05), 1);
	ret |= sh_set_reg(SH_SENSORIDX_MAX86178, 0x1E, (0x1F), 1);
	ret |= sh_set_reg(SH_SENSORIDX_MAX86178, 0x86, (0x01 << 6), 1);

	return ret;
}


static void data_time_stamper(void * p_time_stamp, uint8_t* data_ptr)
{

}

static void accel_data_rx(void * p_accel_data, uint8_t* data_ptr)
{
	accel_data * sample = (accel_data  *)p_accel_data;

	sample->x = (data_ptr[0] << 8) | data_ptr[1];
	sample->y = (data_ptr[2] << 8) | data_ptr[3];
	sample->z = (data_ptr[4] << 8) | data_ptr[5];
}

static void max86178_data_rx(void * p_ppg_data, uint8_t* data_ptr)
{
	max86178_data * sample = (max86178_data *)p_ppg_data;

	sample->led1 = (data_ptr[0] << 16) | (data_ptr[1] << 8) | data_ptr[2];	// LED1, PD1
	sample->led2 = (data_ptr[3] << 16) | (data_ptr[4] << 8) | data_ptr[5];
	sample->led3 = (data_ptr[6] << 16) | (data_ptr[7] << 8) | data_ptr[8];  // LED3, PD1
	sample->led4 = (data_ptr[9] << 16) | (data_ptr[10] << 8) | data_ptr[11]; 	// LED1, PD2
	sample->led5 = (data_ptr[12] << 16) | (data_ptr[13] << 8) | data_ptr[14];
	sample->led6 = (data_ptr[15] << 16) | (data_ptr[16] << 8) | data_ptr[17]; 	// LED3, PD2
}

static void whrm_wspo2_suite_data_rx_mode1(void * p_algo_data_mode1, uint8_t* data_ptr)
{
	whrm_wspo2_suite_sensorhub_data *sample = (whrm_wspo2_suite_sensorhub_data *) p_algo_data_mode1;

	sample->current_operating_mode = data_ptr[0];

	/* WHRM Data */
	sample->hr = (data_ptr[1] << 8) | data_ptr[2];
	sample->hr_conf = data_ptr[3];
	sample->rr = (data_ptr[4] << 8) | data_ptr[5];
	sample->rr_conf = data_ptr[6];
	sample->activity_class = data_ptr[7];

	/* WSPO2 Data */
	sample->r = (data_ptr[8] << 8) | data_ptr[9];
	sample->spo2_conf = data_ptr[10];
	sample->spo2 = (data_ptr[11] << 8) | data_ptr[12]; // already x10;
	sample->percentComplete = data_ptr[13];
	sample->lowSignalQualityFlag = data_ptr[14];
	sample->motionFlag = data_ptr[15];
	sample->lowPiFlag = data_ptr[16];
	sample->unreliableRFlag = data_ptr[17];
	sample->spo2State = data_ptr[18];
	sample->scd_contact_state = data_ptr[19];

}

static void whrm_wspo2_suite_data_rx_mode2(void * p_algo_data_mode2, uint8_t* data_ptr)
{
	whrm_wspo2_suite_sensorhub_data *sample = (whrm_wspo2_suite_sensorhub_data *) p_algo_data_mode2;

	sample->current_operating_mode = data_ptr[0];

	/* WHRM Data */
	sample->hr = (data_ptr[1] << 8) | data_ptr[2];
	sample->hr_conf = data_ptr[3];
	sample->rr = (data_ptr[4] << 8) | data_ptr[5];
	sample->rr_conf = data_ptr[6];
	sample->activity_class = data_ptr[7];

	/* WSPO2 Data */
	sample->r = (data_ptr[8] << 8) | data_ptr[9];
	sample->spo2_conf = data_ptr[10];
	sample->spo2 = (data_ptr[11] << 8) | data_ptr[12]; // already x10;
	sample->percentComplete = data_ptr[13];
	sample->lowSignalQualityFlag = data_ptr[14];
	sample->motionFlag = data_ptr[15];
	sample->lowPiFlag = data_ptr[16];
	sample->unreliableRFlag = data_ptr[17];
	sample->spo2State = data_ptr[18];
	sample->scd_contact_state = data_ptr[19];

	/* Extended report */
	sample->walk_steps = (data_ptr[20] << 24) | (data_ptr[21]  << 16) | (data_ptr[22] << 8) | data_ptr[23];
	sample->run_steps = (data_ptr[24] << 24) | (data_ptr[25] << 16) | (data_ptr[26] << 8) | data_ptr[27];
	sample->kcal = (data_ptr[28] << 24) | (data_ptr[29] << 16) | (data_ptr[30] << 8) | data_ptr[31];
	sample->totalActEnergy = (data_ptr[32] << 24) | (data_ptr[33] << 16) | (data_ptr[34] << 8) | data_ptr[35];

	int ind = SH_PPG_SIGNAL_GREEN0;
	int idx = 35;
	for(ind = SH_PPG_SIGNAL_GREEN0; ind < SH_PPG_SIGNAL_MAX; ++ind)
	{
		uint16_t curr = (data_ptr[idx+1] << 8) | data_ptr[idx+2];
		uint8_t int_time = data_ptr[idx+3];
		uint8_t avg = data_ptr[idx+4];
		uint8_t dac_off = data_ptr[idx+5];

		sample->afe_requests[ind].led_curr.is_led_curr_update_requested = (curr >> 15);
		sample->afe_requests[ind].led_curr.requested_led_curr = (curr & 0x7FFF);

		sample->afe_requests[ind].int_time.is_int_time_update_requested = (int_time >> 7);
		sample->afe_requests[ind].int_time.requested_int_time = (int_time & 0x7F);

		sample->afe_requests[ind].sample_averaging.is_averaging_update_requested = (avg >> 7);
		sample->afe_requests[ind].sample_averaging.requested_avg = (avg & 0x7F);

		sample->afe_requests[ind].dac_offset.is_dac_offset_update_requested = (dac_off >> 7);
		sample->afe_requests[ind].dac_offset.dac_offset = (dac_off & 0x7F);

		idx += 5;
	}


	sample->hrm_afe_state = data_ptr[56];
	sample->is_high_motion = data_ptr[57];
}
