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
#include "max86178.h"
#include "sh_utils.h"
#include "sh_comm_defs.h"
#include "mxm_assert.h"


#include "gpio.h"
#include "mxc_delay.h"

#ifdef SENSOR_KX123
#include "kx123.h" //for kx123_poll_data_buffer
#endif
/*
	- Find better function name for _max86178_ppg_enable

*/

///DEBUG------------------------------------
#ifdef ENABLE_ACCEL_DEBUG
extern int32_t accelTimeStamp;
extern int32_t ppgTimeStamp;
#endif
///----------------------------------------

extern sensor_t max86178;

#define CFG_PARAM_OPERATING_MODES		0x00
#define CFG_PARAM_OPERATING_MODES_LEN	2
#define CFG_PARAM_OPERATING_STATUS_LEN	3

extern sensor_t max86178;
static bool data_valid = 0;
//uint8_t sample_id_cnt = 0;
static uint32_t led_data[MAX86178_LED_NUM * NUM_SAMPLES_PER_CHANNEL];


//gpio_cfg_t max86178_intb_gpio = {PORT_5, PIN_4, 0, 1}; // - INT1 PIN
gpio_cfg_t max86178_intb_gpio = {PORT_0, PIN_12, GPIO_FUNC_IN , GPIO_PAD_PULL_UP  }; // - INT1 PIN

static void mxm_sh_register_sensor(sensor_t *ss);

int max86178_dump_all_regs(uint8_t *buf, int size);
int max86178_main(void);
int _max86178_ppg_enable(int state);
int max86178_pop_data(sensor_data_t **sensor_data);
int max86178_execute_once(void *);
int max86178_get_num_samples(void *data);

uint8_t max86178_report_buf[sizeof(uint8_t) * MAX86178_DATA_WORD_SIZE * MAX86178_LED_NUM * NUM_SAMPLES_PER_CHANNEL *2];

sensor_modes_t max86178_modes = {
		.num_modes = 2,
		.required_bytes = {
				MAX86178_DATA_WORD_SIZE * MAX86178_LED_NUM * NUM_SAMPLES_PER_CHANNEL, // 3*2*3,
				MAX86178_DATA_WORD_SIZE * MAX86178_LED_NUM * NUM_SAMPLES_PER_CHANNEL //3*2*9,
		},
	};

static void mxm_sh_register_sensor(sensor_t *ss)
{
	ss->initialized = 1;
}

//static void sleep_ms(int delay)
//{
//	TMR_Delay(MXC_TMR0, MSEC(delay), NULL);
//}

static void gpio_irq_handler_free(const gpio_cfg_t *gpio)
{
	//platform_assert(gpio);
	mxm_assert(gpio);

	GPIO_RegisterCallback(gpio, NULL, NULL);
	// Configure and enable interrupt
	GPIO_IntDisable(gpio);
}

static void register_gpio_irq_handler(void (*irq_handler)(void *), const gpio_cfg_t *gpio, void *cbdata)
{
	//platform_assert(gpio);
	mxm_assert(gpio);
	pr_info("\n\rregister_gpio_irq_handler!\n\r");

	GPIO_Config(gpio);
	// Register callback
	GPIO_RegisterCallback(gpio, irq_handler, cbdata);

	// Configure and enable interrupt
//#if defined(MAX32660)
	GPIO_IntConfig(gpio, GPIO_INT_EDGE, GPIO_INT_FALLING);
//#else
	//GPIO_IntConfig(gpio, GPIO_INT_FALLING_EDGE);
//#endif
	GPIO_IntEnable(gpio);
	NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(gpio->port));
}

static int max86178_get_sample_rate_wrapper(void *data)
{
	struct max86178_dev *sd = max86178_get_device_data();
	return max86178_get_sample_rate(sd);
}

static int max86178_set_sample_rate_wrapper(void *data, int sample_rate)
{
	struct max86178_dev *sd = max86178_get_device_data();
	return max86178_set_sample_rate(sd, sample_rate);
}

static int max86178_write_reg_wrapper(uint8_t *reg_addr, uint8_t *val)
{
	struct max86178_dev *sd = max86178_get_device_data();
	return max86178_write_reg(sd->comm, *reg_addr, *val);
}

static int max86178_read_reg_wrapper(uint8_t *buf, int len)
{
	struct max86178_dev *sd = max86178_get_device_data();
	return max86178_read_reg(sd->comm, buf, len);
}

int _max86178_ppg_enable(int state)
{
	int ret = 0;
	struct max86178_dev *sd = max86178_get_device_data();
	data_valid = 0;
	//sample_id_cnt = 0;
	memset(led_data, 0, sizeof(led_data));
	if (state) {
		if (state > 0 && state < max86178_modes.num_modes) {
			sensor_t *sensor = mxm_sh_get_sensor_instance(SH_MAX86178);
			sensor_data_t *ppg_report_data = &sensor->report_buf;

		///	ppg_report_data->size = max86178_modes.required_bytes[state]; //TODO move it below if not FW default
			{
				struct max86178_dev *sd = max86178_get_device_data();
				ppg_report_data->size = max86178_get_meas_num(sd) * MAX86178_DATA_WORD_SIZE * NUM_SAMPLES_PER_CHANNEL;
				pr_info("ppg report size = %d\n", ppg_report_data->size);
			}
#if 0
			/* FIX ME: Unify enable functions in max86178_ss_whrm_aec_scd_instance.c,
				max86178_ss_wspo2_instance.c and max86178_ss_whrm_instance.c modules */
			if (!sd->operating_cfg.firmware_default) {
				pr_debug("%s:%d runs user settings\n", __func__, __LINE__);

				return 0;
			}
			/* Algorithms initialize, bypass the following initialization to prevent double init */
			if (max86178_reset(sd) < 0) {
				pr_err("Max86171 reset failed. %s:%d\n", __func__, __LINE__);
				return -1;
			}

			ret = max86178_sensor_enable(sd, sd->led_ctrl.agc_is_enabled, state);
			mxm_sh_sync_sample_rates(SH_MAX86178, max86178_get_sample_rate(sd));
#endif
			ret = max86178_sensor_enable(sd, 0, 1);
		}
	} else {
		ret = max86178_sensor_enable(sd, 0, 0);
		/* No DAC calibartion for OS61 */
		sd->operating_cfg.dac_calib = 0;
		sd->operating_cfg.firmware_default = 1;
	}

	return ret;
}

int max86178_sync_data(void)
{
#if 0
	//Toggle mode off and on to clear FIFO
	int ret = max86178_write_reg(MAX86110_MODE_CONFIGURATION, 0);
	ret |= max86178_write_reg(MAX86110_MODE_CONFIGURATION, 0x03);
	return ret;
#endif
	return 0;
}


/* Test Mode */

static void self_test_int_handler(void* cbdata) {
	uint8_t *buf = cbdata;
	buf[0] &= ~FAILURE_INT;
	gpio_irq_handler_free(&max86178_intb_gpio);
}


int max86178_init_chip_test_mode(void)
{
	int ret;
	struct max86178_dev *max86178_driver = max86178_get_device_data();
	if(max86178_driver == NULL){
		pr_info("Failed to retrieve sensor struct\r\n");
		return -1;
	}

	// initialize the sensor with some default parameters
	ret = max86178_sensor_enable(max86178_driver, 0, 1);
	if(max86178_driver == NULL){
		pr_info("Failed max86178_reset\r\n");
		return -1;
	}
	return ret;
}

static void max86178_self_test(uint8_t *tx_buf) {
	int ret;

	tx_buf[0] = 0x00;
	// set default for max86178
	ret = max86178_init_chip_test_mode();
	if(ret != 0) {
		tx_buf[0] |= FAILURE_COMM;
		//printf("PPG comm failed\r\n");
	} else {
		ret = 0;
	}

	// set interrupt to failed by default
	tx_buf[0] |= FAILURE_INT;

	register_gpio_irq_handler(&self_test_int_handler, &max86178_intb_gpio, tx_buf);
	// wait interrupt
	mxc_delay(MXC_DELAY_MSEC(100));

	return;
}
/* end of test mode */

// that function is called with NULL pointer if called to
// execute irq handler from another interrupt
int max86178_execute_once(void *data)
{
	struct max86178_dev *sd = max86178_get_device_data();
	MXM_SH_ASSERT(sd);
	if(max86178_get_irq_state(NULL) > 0) {
		if (max86178_irq_handler(sd) > 0)
			max86178_irq_reset_ref_cnt();

		//printf("Queue sample count: %d\r\n", sd->queue.num_item);
	}
	return 0;

//	struct max86178_dev *sd = max86178_get_device_data();
//	MXM_SH_ASSERT(sd);
//
//	max86178_irq_handler(sd);
//
//	return 0;
}

int max86178_get_cfg_params(void *data, uint8_t param, uint8_t *tx_buf, int tx_buf_len) {
	struct max86178_dev *sd = max86178_get_device_data();

	switch(param) {
		case (CFG_PARAM_OPERATING_MODES):
			if (tx_buf_len < CFG_PARAM_OPERATING_STATUS_LEN)
				return -1;
			tx_buf[0] = sd->operating_cfg.val;
			tx_buf[1] = sd->ppg_cfg1.val;
			tx_buf[2] = sd->dac_calib_status;
			break;

		default:
			return -1;
	}

	return 0;
}

int max86178_set_cfg_params(void *data, uint8_t *params, int param_sz) {

	if (param_sz <= 0)
		return -1;

	struct max86178_dev *sd = max86178_get_device_data();

	switch(params[0]) {
		case (CFG_PARAM_OPERATING_MODES):
			if (param_sz != CFG_PARAM_OPERATING_MODES_LEN)
				return -1;

			sd->operating_cfg.val = params[1];
			sd->ppg_cfg1.val = params[2];

			pr_debug("Set max86178 config: %.2X %.2X\n", sd->operating_cfg.val, sd->ppg_cfg1.val);
			if (sd->operating_cfg.firmware_default){
				pr_debug("Firmware default settings are enabled\n");
			}
			else{
				pr_debug("Firmware default settings are disabled\n");
			}


			break;
		default:
			return -1;

	}

	return 0;
}

sensor_t max86178 = {
	.dev_data = NULL,
	.version = {
			.major = 1,
			.minor = 2,
			.revision = 3,
		},
/* TODO: Modify MAX86178 register list */
	.num_dump_regs = 0x7D,
	.reg_size = 1,
	.name = "max86178_ppg",

	/* Report */
	.report_buf = { .buf = &max86178_report_buf[0], .ts =0/*, .updated = false, */},
	.item_size = 3, /* 3 bytes */
	/* MAX86178 number of slots = 9, OS58 supports 4 slot  */
	.num_items = 9, /* */
	.mode_info = &max86178_modes,
	/* Functions */
	.dump_reg = &max86178_dump_all_regs,
	.init = &max86178_main,
	.read_reg = &max86178_read_reg_wrapper,
	.write_reg = &max86178_write_reg_wrapper,
	.enable = &_max86178_ppg_enable,
	.pop_data = &max86178_pop_data,
	.sync_data = &max86178_sync_data,
	.execute_once = &max86178_execute_once,
	.num_samples = &max86178_get_num_samples,
	.get_sample_rate = &max86178_get_sample_rate_wrapper,
	.set_sample_rate = &max86178_set_sample_rate_wrapper,

	.set_cfg = &max86178_set_cfg_params,
	.get_cfg = &max86178_get_cfg_params,

	.get_irq_state = max86178_get_irq_state,
	//self test
	.self_test = max86178_self_test,
};


int max86178_get_num_samples(void *data)
{
	return queue_len(max86178.queue);
}

static int max86178_check_samples_ready(struct max86178_dev *pdev, uint32_t *led_results_buf){
	//uint32_t ppg_data_ack = 0;
	fifo_data_t fifo_data;
	int ret, i;
	int ppg_count;

	ppg_count = queue_len(max86178.queue);
//	printf("Que len = %d\n", ppg_count);

	if (ppg_count < 2*max86178_get_meas_num(pdev)) // TWo PD always
		return -1;

	else if(ppg_count > 2*max86178_get_meas_num(pdev))
	{
		ppg_count = 2*max86178_get_meas_num(pdev);
	}

//	printf("MEAS num = %d\n", max86178_get_meas_num(pdev));

#if 1
	for(i = 0; i < ppg_count; ++i) {

		/* Read PPG samples of MEAS */
		ret = dequeue(max86178.queue, &fifo_data);
		if (ret < 0) {
			return -1;
		}

		led_results_buf[i] = fifo_data.raw;

		//ppg_data_ack |= (1 << fifo_data.type);

		///printf("i: %d   raw: %x   type: %x   data: %x\n", i, fifo_data.raw, fifo_data.type, fifo_data.val);
		///printf("t:%d\n", fifo_data.type);
	}
#else
		// Testing using a pattern
	int tag;
	float f=1;
	int j = 0;
	for(i = 0; i < ppg_count; ++i) {

		tag = (int)f;
		f+=0.5;
		led_results_buf[ i ] = j+1 + 256*(j) + 256*256*tag;
		j+=2;
		printf("i: %d   Buf %x \n", i, led_results_buf[ i ] );
	}
#endif

	return ppg_count;
}

/*
TODO: Add a routine to copy sensor data automatically
*/
/*max86178.sample_cnt*/
static uint32_t sample_cnt = 0;
int max86178_pop_data(sensor_data_t **sensor_data)
{
	int size;
	int idx = 0, i;
	uint8_t *report_buf = sensor_data[SH_MAX86178]->buf;
	struct max86178_dev *sd = max86178_get_device_data();

	size = max86178_check_samples_ready(sd, led_data);

	if(size <= 0){
		pr_info("max86178_check_samples_ready has failed\r\n");
		return -1;
	}

	sample_cnt++;
	sensor_data[SH_MAX86178]->ts = ++sample_cnt;

	for(i = 0; i < size; i++)
	{
		mxm_sh_add_u24_to_report(&idx, report_buf, led_data[i]);
		/* Clear for next run */
		led_data[i] = 0;
	}

	return 0;
}

int max86178_main(void)
{
	int ret = max86178_driver_init();
	struct max86178_dev *sd = max86178_get_device_data();
	if(sd == NULL || ret != 0){
		pr_err("max86178_main has failed ret:%d\r\n", ret);
		return -1;
	}

	max86178.dev_data = sd;
	max86178.queue = &sd->queue;
	mxm_sh_register_sensor(&max86178);
	return ret;
}

int max86178_dump_all_regs(uint8_t *buf, int size)
{
	struct max86178_dev *sd = max86178_get_device_data();
	int ret = 0;
	int i, j;
	int num_read = 0;
	uint8_t reg_addrs[18][2] = {   /*125 resigters */
			{0x00, 0x02},    //3
			{0x04, 0x0A},   //7
			{0x0C, 0x11},  //6
			{0x15, 0x1E},  //10
			{0x20, 0x26},  //7
			{0x28, 0x2E},  //7
			{0x30, 0x36},  //7
			{0x38, 0x3E},  //7
			{0x40, 0x46},  //7
			{0x48, 0x4E},  //7
			{0x50, 0x56},  //7
			{0x58, 0x5E},  //7
			{0x68, 0x6D},  //6
			{0x70, 0x71},  //2
			{0x78, 0x7A},  //3
			{0x7C, 0x7E},  //3
			{0xD0, 0xEA},  //27
			{0xFE, 0xFF}  //2
	};


	for (i = 0; i < 18; i++) {
		uint8_t start_addr = reg_addrs[i][0];
		uint8_t end_addr = reg_addrs[i][1];
		int len = end_addr - start_addr + 1;
		uint8_t tmp[2 * len];

		tmp[0] = start_addr;
		ret = max86178_read_reg(sd->comm, tmp, len);
		if (ret < 0)
			return ret;

		for (j = start_addr; j <= end_addr; j++) {
			buf[2 * num_read] = j;
			buf[2 * num_read + 1] = tmp[j - start_addr];
			if (num_read++ >= size)
				return 0;
		}

	}


	return ret;
}
