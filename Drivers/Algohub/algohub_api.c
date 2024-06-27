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

#include "algohub_api.h"
#include "algohub_config_api.h"
#include "algohub_sensorhub_manager.h"

#include "mxc_delay.h"
#include "mxc_errors.h"

#include "sh_comm.h"
#include "mrd106.h"
#include "mrd106_helper.h"
#include "gui.h"
#include "queue.h"
#include "main.h"
#include "windows_gui_packet_defs.h"


extern queue_t queue_algo;
algohub_feed_data_t algohub_batch_in[ALGOHUB_INPUT_BATCH_SIZE];
algohub_report_t algohub_batch_out[ALGOHUB_INPUT_BATCH_SIZE];
algo_packet_t algo_ble_packet = {0};
static uint8_t algohub_fifo_written = 0;
uint16_t algohub_write_status = 0;

#define ALGOHUB_REPORT_SIZE_IN_MODE_1 (24u)
#define ALGOHUB_REPORT_SIZE_IN_MODE_2 (24u)

typedef enum
{
	ALGOHUB_OPERATING_MODE_1 = 0,
	ALGOHUB_OPERATING_MODE_2,
	ALGOHUB_OPERATING_MODE_MAX,
} algohub_operating_mode_t;

static const uint8_t algohub_reporting_size[ALGOHUB_OPERATING_MODE_MAX] = {ALGOHUB_REPORT_SIZE_IN_MODE_1, ALGOHUB_REPORT_SIZE_IN_MODE_2};

static volatile algohub_operating_mode_t algohub_curr_mode = ALGOHUB_OPERATING_MODE_1;
static uint8_t algohub_soft_ver[3];

static const uint8_t PPG_SIGNAL_CH_PD_TABLE[ALGOHUB_PPG_MAX][2] = {
		{ALGOHUB_DEFAULT_WHRM0_CH_SLOT,ALGOHUB_DEFAULT_WHRM0_CH_PD},
		{ALGOHUB_DEFAULT_WHRM1_CH_SLOT,ALGOHUB_DEFAULT_WHRM1_CH_PD},
		{ALGOHUB_DEFAULT_IR_CH_SLOT,ALGOHUB_DEFAULT_IR_CH_PD},
		{ALGOHUB_DEFAULT_RED_CH_SLOT,ALGOHUB_DEFAULT_RED_CH_PD}};

int algohub_init()
{
	int ret = 0;

	sh_init_hwcomm_interface();

	mxc_delay(MXC_DELAY_SEC(1));

	ret = sh_set_data_type(SS_DATATYPE_BOTH, false);

	if (0 == ret)
	{
		ret = sh_set_fifo_thresh(1);
	}

	if (0 == ret)
	{
		uint8_t algohub_soft_ver_len;
		ret = sh_get_ss_fw_version(algohub_soft_ver, &algohub_soft_ver_len);
	}

	return ret;
}

int algohub_enable()
{
	int ret = 0;

	ret = sh_sensor_enable_(SH_SENSORIDX_ALGOHUB, 1, SH_INPUT_DATA_FROM_HOST);

	return ret;
}

int algohub_disable()
{
	int ret = 0;

	ret = sh_sensor_enable_(SH_SENSORIDX_ALGOHUB, 0, SH_INPUT_DATA_FROM_HOST);

	return ret;
}

int algohub_feed_data(const algohub_feed_data_t *const p_data)
{
	int ret = 0;
	int i = 2;
	int num_wr_bytes;
	uint8_t tx_buf_feed[2 + 24];

	if (NULL == p_data)
	{
		ret = E_BAD_PARAM;
	}

	if (E_NO_ERROR == ret)
	{
		tx_buf_feed[i++] = (p_data->ppg_data_in.green1 >> 16);
		tx_buf_feed[i++] = (p_data->ppg_data_in.green1 >> 8);
		tx_buf_feed[i++] = (p_data->ppg_data_in.green1);

		tx_buf_feed[i++] = (p_data->ppg_data_in.green2 >> 16);
		tx_buf_feed[i++] = (p_data->ppg_data_in.green2 >> 8);
		tx_buf_feed[i++] = (p_data->ppg_data_in.green2);

		tx_buf_feed[i++] = (p_data->ppg_data_in.ir >> 16);
		tx_buf_feed[i++] = (p_data->ppg_data_in.ir >> 8);
		tx_buf_feed[i++] = (p_data->ppg_data_in.ir);

		tx_buf_feed[i++] = (p_data->ppg_data_in.red >> 16);
		tx_buf_feed[i++] = (p_data->ppg_data_in.red >> 8);
		tx_buf_feed[i++] = (p_data->ppg_data_in.red);

		tx_buf_feed[i++] = 0;
		tx_buf_feed[i++] = 0;
		tx_buf_feed[i++] = 0;

		tx_buf_feed[i++] = 0;
		tx_buf_feed[i++] = 0;
		tx_buf_feed[i++] = 0;

		tx_buf_feed[i++] = (p_data->acc_data_in.x >> 8);
		tx_buf_feed[i++] = (p_data->acc_data_in.x);

		tx_buf_feed[i++] = (p_data->acc_data_in.y >> 8);
		tx_buf_feed[i++] = (p_data->acc_data_in.y);

		tx_buf_feed[i++] = (p_data->acc_data_in.z >> 8);
		tx_buf_feed[i++] = (p_data->acc_data_in.z);

		ret = sh_feed_to_input_fifo(tx_buf_feed, sizeof(tx_buf_feed), &num_wr_bytes);
	}

	return ret;
}

int algohub_feed_batch_data(const algohub_feed_data_t *const p_data, uint8_t number_of_inputs)
{
	int ret = 0;
	int i = 2;
	int num_wr_bytes;
	uint8_t tx_buf_feed[2 + (24 * number_of_inputs)];

	if (NULL == p_data)
	{
		ret = E_BAD_PARAM;
	}

	if (E_NO_ERROR == ret)
	{
		for (int j = 0; j < number_of_inputs; j++)
		{
			tx_buf_feed[i++] = (p_data[j].ppg_data_in.green1 >> 16);
			tx_buf_feed[i++] = (p_data[j].ppg_data_in.green1 >> 8);
			tx_buf_feed[i++] = (p_data[j].ppg_data_in.green1);

			tx_buf_feed[i++] = (p_data[j].ppg_data_in.green2 >> 16);
			tx_buf_feed[i++] = (p_data[j].ppg_data_in.green2 >> 8);
			tx_buf_feed[i++] = (p_data[j].ppg_data_in.green2);

			tx_buf_feed[i++] = (p_data[j].ppg_data_in.ir >> 16);
			tx_buf_feed[i++] = (p_data[j].ppg_data_in.ir >> 8);
			tx_buf_feed[i++] = (p_data[j].ppg_data_in.ir);

			tx_buf_feed[i++] = (p_data[j].ppg_data_in.red >> 16);
			tx_buf_feed[i++] = (p_data[j].ppg_data_in.red >> 8);
			tx_buf_feed[i++] = (p_data[j].ppg_data_in.red);

			tx_buf_feed[i++] = 0;
			tx_buf_feed[i++] = 0;
			tx_buf_feed[i++] = 0;

			tx_buf_feed[i++] = 0;
			tx_buf_feed[i++] = 0;
			tx_buf_feed[i++] = 0;

			tx_buf_feed[i++] = (p_data[j].acc_data_in.x >> 8);
			tx_buf_feed[i++] = (p_data[j].acc_data_in.x);

			tx_buf_feed[i++] = (p_data[j].acc_data_in.y >> 8);
			tx_buf_feed[i++] = (p_data[j].acc_data_in.y);

			tx_buf_feed[i++] = (p_data[j].acc_data_in.z >> 8);
			tx_buf_feed[i++] = (p_data[j].acc_data_in.z);
		}
		ret = sh_feed_to_input_fifo(tx_buf_feed, sizeof(tx_buf_feed), &num_wr_bytes);
	}

	return ret;
}

int algohub_read_outputfifo(const algohub_report_t *p_result, const int numberResultToBeRead)
{
	int ret = 0;
	int index = 0;
	uint8_t hubStatus;
	uint8_t databuf[256];
	volatile int algo_report_size = 0;

	if (algohub_curr_mode >= ALGOHUB_OPERATING_MODE_MAX)
		ret = E_BAD_STATE;

	if (E_NO_ERROR == ret)
		ret = sh_get_sensorhub_status(&hubStatus);

	if (hubStatus & SS_MASK_STATUS_DATA_RDY)
		ret = 0;
	else
		ret = E_NONE_AVAIL;

	if (E_NO_ERROR == ret)
	{
		algo_report_size = algohub_reporting_size[algohub_curr_mode];

		for (index = 0; index < numberResultToBeRead; index++)
		{
			memset(databuf, 0, sizeof(databuf));
			ret |= sh_read_fifo_data(1, algo_report_size, databuf, sizeof(databuf));

			((algohub_report_t *)(p_result + index))->test_led = (databuf[1] << 16) + (databuf[2] << 8) + (databuf[3]);

			((algohub_report_t *)(p_result + index))->algo.isAfeRequestExist = (databuf[4] & 0x80) >> 7u;
			((algohub_report_t *)(p_result + index))->algo.algoMode = (databuf[4] & 0x7F);

			((algohub_report_t *)(p_result + index))->hr = (databuf[5] << 8) | (databuf[6]);
			((algohub_report_t *)(p_result + index))->hrConfidence = databuf[7];

			((algohub_report_t *)(p_result + index))->rr = (databuf[8] << 8) | (databuf[9]);
			((algohub_report_t *)(p_result + index))->rrConfidence = databuf[10];
			((algohub_report_t *)(p_result + index))->activityClass = databuf[11];
			((algohub_report_t *)(p_result + index))->r = (databuf[12] << 8) | (databuf[13]);
			((algohub_report_t *)(p_result + index))->spo2Confidence = databuf[14];
			((algohub_report_t *)(p_result + index))->spo2 = (databuf[15] << 8) | databuf[16];
			((algohub_report_t *)(p_result + index))->spo2PercentComplete = databuf[17];
			((algohub_report_t *)(p_result + index))->spo2LowSignalQualityFlag = databuf[18];
			((algohub_report_t *)(p_result + index))->spo2MotionFlag = databuf[19];
			((algohub_report_t *)(p_result + index))->spo2LowPiFlag = databuf[20];
			((algohub_report_t *)(p_result + index))->spo2UnreliableRFlag = databuf[21];
			((algohub_report_t *)(p_result + index))->spo2State = databuf[22];
			((algohub_report_t *)(p_result + index))->scdSontactState = databuf[23];
			((algohub_report_t *)(p_result + index))->algoRet = databuf[24];
		}
	}

	return ret;
}

int algohub_reset_configs()
{
	int ret = 0;

	ret = ah_set_cfg_wearablesuite_reset_algo_config();

	mxc_delay(MXC_DELAY_MSEC(20));

	return ret;
}

int algohub_notify_afe_request_applied()
{
	return ah_set_cfg_wearablesuite_clear_aferequest(1);
}

int algohub_read_software_version(uint8_t version[3])
{
	memcpy(version, algohub_soft_ver, sizeof(algohub_soft_ver));

	return 0;
}

int algohub_get_aferequest(uint8_t afe_reqs[20])
{
	return ah_get_cfg_wearablesuite_aferequest(afe_reqs);
}

void algohub_parse_aferequest(uint8_t afe_reqs[20], algohub_afe_reqs_t *afe)
{
	int index = 0;

	uint8_t int_req = 0;
	uint8_t avg_req = 0;
	uint8_t dac_req = 0;
	uint16_t led_req = 0;

	memset(afe, 0, sizeof(algohub_afe_reqs_t) * ALGOHUB_PPG_MAX);

	for (index = 0; index < ALGOHUB_PPG_MAX; index++)
	{
		led_req = ((uint16_t)afe_reqs[(index * 5) + 0] << 8) + (uint16_t)afe_reqs[(index * 5) + 1];
		int_req = afe_reqs[(index * 5) + 2];
		avg_req = afe_reqs[(index * 5) + 3];
		dac_req = afe_reqs[(index * 5) + 4];

		((algohub_afe_reqs_t *)(afe + index))->led_curr_adjustment.is_led_curr_update_requested = led_req >> 15;
		((algohub_afe_reqs_t *)(afe + index))->led_curr_adjustment.curr = (led_req & 0x7FFF) / 10;

		/* Algohub never makes requests to set LED current to zero.
		 * While calculating requested led current, the fractions might be lost due to conversion.
		 * For that reason, we are setting 1mA as minimum led current */
		if (((algohub_afe_reqs_t *)(afe + index))->led_curr_adjustment.curr == 0)
			((algohub_afe_reqs_t *)(afe + index))->led_curr_adjustment.curr = 1;

		((algohub_afe_reqs_t *)(afe + index))->int_time_adjustment.is_int_time_update_requested = int_req >> 7;
		((algohub_afe_reqs_t *)(afe + index))->int_time_adjustment.integration_time = int_req & 0x7F;

		((algohub_afe_reqs_t *)(afe + index))->avg_smp_adjustment.is_avg_smp_update_requested = avg_req >> 7;
		((algohub_afe_reqs_t *)(afe + index))->avg_smp_adjustment.avg_smp = avg_req & 0x7F;

		((algohub_afe_reqs_t *)(afe + index))->dac_offset_adjustment.is_dac_offset_update_requested = dac_req >> 7;
		((algohub_afe_reqs_t *)(afe + index))->dac_offset_adjustment.dac_offset = dac_req & 0x7F;
	}

	return;
}

int algohub_ppg_signal_meas_ch_no(algohub_ppg_singal_type_t ppg_signal)
{
	int ret = 0;
	if (ppg_signal >= ALGOHUB_PPG_MAX)
		ret = -1;

	ret = PPG_SIGNAL_CH_PD_TABLE[ppg_signal][0];
	return ret;
}

int algohub_ppg_signal_pd_no(algohub_ppg_singal_type_t ppg_signal)
{
	int ret = 0;
	if (ppg_signal >= ALGOHUB_PPG_MAX)
		ret = -1;

	ret = PPG_SIGNAL_CH_PD_TABLE[ppg_signal][1];
	return ret;
}

int algohub_send_spi_release_request()
{
	return sh_spi_release();
}

int algohub_notify_spi_released()
{
	return sh_spi_use();
}

int algohub_sync(uint64_t time)
{
	return sh_comm_sync(time);
}

int algohub_process(uint64_t current_time)
{
	mrd_t *mrd = getMRD();
	sensor_t *acc = mrd->getSensor(SH_ACC_SENSOR);
	bool algohub_idle = !(algohub_sync(current_time) > 0);
	int ret = 0;
	if (algohub_fifo_written)
	{
		memset(&algohub_batch_out, 0, sizeof(algohub_batch_out));

		ret = algohub_read_outputfifo(algohub_batch_out, ALGOHUB_INPUT_BATCH_SIZE);

		int algohub_output_index = ALGOHUB_INPUT_BATCH_SIZE - 1;

		pr_info("AlgohubReadRet %d \n", ret);
		if (0 == ret)
		{
			if (algohub_batch_out[algohub_output_index].algo.isAfeRequestExist)
			{
				/* Save afe request flag for further process */
				is_afe_request_made++; // = 1;
			}

			algohub_batch_out[algohub_output_index].rr /= 10;

			algo_ble_packet.algo_mode = algohub_batch_out[algohub_output_index].algo.algoMode;
			algo_ble_packet.hr = algohub_batch_out[algohub_output_index].hr / 10;
			algo_ble_packet.hr_conf = algohub_batch_out[algohub_output_index].hrConfidence;
			algo_ble_packet.rr_msb = algohub_batch_out[algohub_output_index].rr >> 8;
			algo_ble_packet.rr_lsb = algohub_batch_out[algohub_output_index].rr;
			algo_ble_packet.rr_conf = algohub_batch_out[algohub_output_index].rrConfidence;
			algo_ble_packet.spo2 = algohub_batch_out[algohub_output_index].spo2 / 10;
			algo_ble_packet.spo2_conf = algohub_batch_out[algohub_output_index].spo2Confidence;
			algo_ble_packet.r_val_msb = algohub_batch_out[algohub_output_index].r >> 8;
			algo_ble_packet.r_val_lsb = algohub_batch_out[algohub_output_index].r;
			algo_ble_packet.spo2_comp = (algohub_batch_out[algohub_output_index].spo2PercentComplete & 0x80) >> 7;
			if (0 == algo_ble_packet.spo2)
			{
				algo_ble_packet.spo2_comp = 0;
			}
			algo_ble_packet.spo2_state = algohub_batch_out[algohub_output_index].spo2State;
			algo_ble_packet.activity = algohub_batch_out[algohub_output_index].activityClass;
			algo_ble_packet.scd_state = algohub_batch_out[algohub_output_index].scdSontactState;
			algo_ble_packet.flag_region = ((uint8_t)algohub_batch_out[algohub_output_index].spo2LowSignalQualityFlag << ALGO_PACK_LOW_SNR_FLAG_POS) |
										  ((uint8_t)algohub_batch_out[algohub_output_index].spo2UnreliableRFlag << ALGO_PACK_UNRELIABLE_R_FLAG_POS) |
										  ((uint8_t)algohub_batch_out[algohub_output_index].spo2LowPiFlag << ALGO_PACK_LOW_PI_FLAG_POS) |
										  ((uint8_t)algohub_batch_out[algohub_output_index].spo2MotionFlag << ALGO_PACK_MOTION_FLAG_POS);
		}
		format_data_algo(&algo_ble_packet);
		algohub_fifo_written--; // = 0;
	}
	else
	{
		uint16_t algo_ppg_queue_len = queue_len(&queue_algo);
		uint16_t algo_ppg_queue_len_temp = algo_ppg_queue_len;

		if (!algohub_idle)
		{
			algohub_write_status = algohub_feed_batch_data(algohub_batch_in, 0);
			if (algohub_write_status == 0)
			{
				algohub_fifo_written++; //
			}
		}
		else if (algo_ppg_queue_len >= (ALGOHUB_INPUT_FRAME_SIZE * ALGOHUB_INPUT_BATCH_SIZE))
		{

			for (int jj = 0; jj < ALGOHUB_INPUT_BATCH_SIZE; jj++)
			{
				/* Read only one frame data */
				algo_ppg_queue_len = ALGOHUB_INPUT_FRAME_SIZE;

				memset(&algohub_batch_in, 0, sizeof(algohub_batch_in));

				/* Read only one frame data */
				algo_ppg_queue_len = ALGOHUB_INPUT_FRAME_SIZE;

				uint32_t ppg_data[ALGOHUB_INPUT_FRAME_SIZE];
				int16_t acc_data[3];

				queue_front_n(&queue_algo, ppg_data, algo_ppg_queue_len, sizeof(ppg_data));
				queue_pop_n(&queue_algo, algo_ppg_queue_len);

				queue_front_n(acc->queue, acc_data, 1, sizeof(acc_data));

				algohub_batch_in[jj].ppg_data_in.green1 = ppg_data[0] & 0x000FFFFF;
				algohub_batch_in[jj].ppg_data_in.green2 = ppg_data[1] & 0x000FFFFF;
				if(spo2_meas_selected_pd == SPO2_APP_SETTING_USE_PD1){
					algohub_batch_in[jj].ppg_data_in.ir = ppg_data[0] & 0x000FFFFF;
					algohub_batch_in[jj].ppg_data_in.red = ppg_data[2] & 0x000FFFFF;
				}
				else{
					algohub_batch_in[jj].ppg_data_in.ir = ppg_data[1] & 0x000FFFFF;
					algohub_batch_in[jj].ppg_data_in.red = ppg_data[3] & 0x000FFFFF;
				}

				double accel_count_to_mg = 0;
				acc->get_cfg((void *)&accel_count_to_mg, 1, 0, 0); // Get mg conversion rate

				algohub_batch_in[jj].acc_data_in.x = acc_data[0] * accel_count_to_mg;
				algohub_batch_in[jj].acc_data_in.y = acc_data[1] * accel_count_to_mg;
				algohub_batch_in[jj].acc_data_in.z = acc_data[2] * accel_count_to_mg;
			}

			/* If PPG and ECG fired at the same time, please wait for first ECG data transmission*/
			if (is_adapter_reporting_started())
			{
				int ret = 0;
				algohub_write_status = algohub_feed_batch_data(algohub_batch_in, ALGOHUB_INPUT_BATCH_SIZE);
				pr_info("AlgohubFeedRet %d \n", algohub_write_status);
				if (0 != ret)
				{
					WAIT_MS(1);
					algohub_write_status = algohub_feed_batch_data(algohub_batch_in, ALGOHUB_INPUT_BATCH_SIZE);
				}

				algohub_fifo_written++; //
			}

			algo_ppg_queue_len = queue_len(&queue_algo);


			if (algo_ppg_queue_len > 0 && algo_ppg_queue_len < ALGOHUB_INPUT_FRAME_SIZE)
			{
				queue_pop_n(&queue_algo, algo_ppg_queue_len);
			}
		}
	}

	return 0;
}
