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

#include "sensorhub.h"
#include "sh_defs.h"

/* Sensor Instances */
extern sensor_t lis2dh12_accel;
extern sensor_t max86178;
extern sensor_t max30208;
extern sensor_t max30210;
extern sensor_t max30210x2;
extern sensor_t adxl367_accel;

//extern volatile uint32_t lis2dh12_wakeup_event;
//extern volatile int lis2dh12_irq_events_cnt;

sensor_t *ss_sensors[SH_NUM_SENSORS] = {
#ifdef LIS2DH12
		[SH_ACC_SENSOR] = &lis2dh12_accel,
#elif ENABLE_ADXL367
		[SH_ACC_SENSOR] = &adxl367_accel,
#endif
		[SH_MAX86178] = &max86178,
#if defined(ENABLE_MAX30208)
		[SH_TEMP_SENSOR] = &max30208,
#elif defined(ENABLE_MAX30210x2)
		[SH_TEMP_SENSOR] = &max30210x2,
#elif defined(ENABLE_MAX30210)
		[SH_TEMP_SENSOR] = &max30210,
#endif

};

algo_t *algo_instances[SH_NUM_ALGOS] = {

};

static queue_t mxm_sh_report_queue;
static queue_t input_queue;
static sensor_data_t *sensor_data[SH_NUM_SENSORS + SH_NUM_ALGOS];

//sh_comm variable
volatile status_reg_t mxm_sh_status_reg;

sensor_t *mxm_sh_get_sensor_instance(int sid)
{
	if (sid >= 0 && sid < SH_NUM_SENSORS)
		return  ss_sensors[sid];

	return NULL;
}


void mxm_sh_sync_active_sensors(void)
{
	int i;
	for (i = 0; i < SH_NUM_SENSORS; i++) {
		sensor_t *sensor = ss_sensors[i];
		if (sensor != NULL && sensor->run_mode.sensor_enabled) {
			if (sensor->sync_data)
				sensor->sync_data();
		}
	}
}

int mxm_sh_get_num_sensors(void)
{
	int i;
	int num_avail = 0;

	for (i = 0; i < SH_NUM_SENSORS; i++) {
		if (ss_sensors[i] != NULL)
			num_avail++;
	}

	return num_avail;
}

int mxm_sh_init_all_sensors()
{
	int i;

	for (i = 0; i < SH_NUM_SENSORS; i++) {
		sensor_t *sensor = ss_sensors[i];
		if (sensor != NULL) {
			sensor->idx = i;
			sensor_data[i] = &sensor->report_buf;
			sensor->presense = false;
			sensor->initialized = false;
		}
	}

	return 0;
}

int mxm_sh_init_all_algorithms(void)
{
	int ret = 0;
	int i;

	for (i = 0; i < SH_NUM_ALGOS; i++) {
		algo_t *algo = algo_instances[i];
		if (algo != NULL) {
			algo->idx = i;
			sensor_data[i + SH_NUM_SENSORS] = &algo->report_buf;
		}
	}

	return ret;
}

int mxm_sh_init()
{
	int ret = 0;

	static uint32_t fifo_buffer[1024];
	static uint32_t input_fifo_buffer[40];


	ret = queue_init_by_name(&mxm_sh_report_queue,
			fifo_buffer,
			sizeof(uint8_t),
			sizeof(fifo_buffer),
			"sh_output_fifo");
	if (ret < 0) {
		pr_err("%s:%d failed.\n", __func__, __LINE__);
		return ret;
	}

	ret = queue_init_by_name(&input_queue,
			input_fifo_buffer,
			sizeof(uint8_t),
			sizeof(input_fifo_buffer),
			"sh_input_fifo");
	if (ret < 0) {
		pr_err("%s:%d failed.\n", __func__, __LINE__);
		return ret;
	}

	mxm_sh_init_all_sensors();
	mxm_sh_init_all_algorithms();

	return 0;
}


void mxm_sh_execute_once(void)
{

}
