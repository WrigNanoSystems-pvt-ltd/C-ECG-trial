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

#include "regHandler.h"
#include "adxl367.h"

#include "sh_defs.h"
#include "sensorhub.h"
#include "sh_comm_defs.h"

#include "mxc_delay.h"
#include "gpio.h"
#include "board.h"
#include "math.h"

#define CFG_PARAM_OPERATING_MODES		0x00
#define CFG_CONVERTION_FACTOR_TO_MG		0x01
#define CFG_PARAM_OPERATING_MODES_LEN	0x04


#ifndef ADXL367Z_FIFO_SIZE
#define ADXL367Z_FIFO_SIZE 256
#endif

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

int8_t adxl367_report_buf[3*sizeof(int16_t)];

sensor_modes_t adxl367_modes = {
	.num_modes = 2,
	.required_bytes = {
		6,
		6,
	},
};

extern sensor_t adxl367_accel;

static uint32_t time_stamp;
static queue_t adxldataq;
static accel_report_t adxldataq_databuf[ADXL367Z_FIFO_SIZE];

static int adxl367_init();
static int adxl367_dump_reg(uint8_t * buf, int size);
static int adxl367_read_reg(uint8_t * read_adr, int len);
static int adxl367_write_reg(uint8_t * reg_addr, uint8_t * val);
static int adxl367_ss_enable(int en);
static int adxl367_sync_data(void);
static int adxl367_get_num_samples(void * data);
static int adxl367_get_sample_rate(void * data);
static int adxl367_set_sample_rate(void * data, int sample_rate);
static int adxl367_execute_once(void * dev_data);
static int adxl367_pop_data(sensor_data_t ** sensor_data);
static int adxl367_poll(void* data);

int adxl367_set_cfg_params(void * data, uint8_t *params, int param_sz);
int adxl367_get_cfg_params(void * data, uint8_t param, uint8_t * tx_buf, int tx_buf_len);
static int adxl367_get_irq_state(void * data);
void adxl367_self_test_custom(uint8_t * txbuf);

#define SH_adxl367 0

sensor_t adxl367_accel = {
	.dev_data = NULL,
	.idx = SH_adxl367,
	.reg_size = 1,
	.version = {
			.major = 1,
			.minor = 2,
			.revision = 3,
		},
	.sensors = NULL,
	.name = "accel",
	.num_dump_regs = 0x00,
	.queue = &adxldataq,

	/* Functions */
	.init = adxl367_init,
	.dump_reg = adxl367_dump_reg,
	.read_reg = adxl367_read_reg,
	.write_reg = adxl367_write_reg,
	.enable = adxl367_ss_enable,
	.sync_data = adxl367_sync_data,
	.num_samples = adxl367_get_num_samples,
	.get_sample_rate = adxl367_get_sample_rate,
	.set_sample_rate = adxl367_set_sample_rate,
	.execute_once = adxl367_execute_once,
	.pop_data = adxl367_pop_data,

	.set_cfg = adxl367_set_cfg_params,
	.get_cfg = adxl367_get_cfg_params,

	.get_irq_state = adxl367_get_irq_state,

	.self_test = adxl367_self_test_custom,

	.mode = 0,
	.run_mode = {
		.val = 0,
	},

	.report_buf = { .buf = (uint8_t *)&adxl367_report_buf[0], },
	.mode_info = &adxl367_modes,
};

static void mxm_sh_register_sensor(sensor_t *ss)
{
	ss->initialized = 1;
}

static adxl367_t *mAdxl;
static tRegHandler spiHandler;

static int adxl367_init(){
    int ret = 0;

    ret = regHandler_spi_init(&spiHandler);
    if(ret != REG_HANDLER_NO_ERROR)
    {
    	printf("Reg Handler failed!\r\n");
    	return -1;
    }

	ret = queue_init_by_name(&adxldataq, &adxldataq_databuf[0], sizeof(accel_report_t), sizeof(adxldataq_databuf), "adxl367");


    mAdxl = getAdxl367();
    //This inits mAdxl with default configuration. If some custom settings necessary, it should be done by methods before init
    ret = mAdxl->init(&spiHandler);

    if(ret != 0)
    {
    	return -1;
    }

    mxm_sh_register_sensor(&adxl367_accel);
    return 0;
}

static int adxl367_ss_enable(int en){
	adxl367_accel.report_buf.size = adxl367_modes.required_bytes[en];
	adxl367_accel.mode = en;
	if((en == 0) & (adxl367_accel.queue != NULL)) {
		queue_reset(adxl367_accel.queue);
	}

	return mAdxl->setMeasureEnable(en);
}

static int adxl367_read_reg(uint8_t * read_adr, int len){
	int index = 0;
	int ret = 0;

	if(len <= 0)
		return -1;

	ret = mAdxl->readRegister(read_adr[index], &read_adr[index], len);


	return (ret==REG_HANDLER_NO_ERROR) ? REG_HANDLER_NO_ERROR : -1;

}

static int adxl367_write_reg(uint8_t * reg_addr, uint8_t * val){
	int ret = mAdxl->writeRegister(*reg_addr, val, 1);

	return (ret==REG_HANDLER_NO_ERROR) ? 0 : -1;
}

static int adxl367_dump_reg(uint8_t * buf, int size){
	int arr_index = 0;
	int j, k;
	int result = 0;
	uint8_t reg_addrs[][2] = {
			{ADXL367_REG_X_DATA_H, ADXL367_REG_EX_ADC_DATA_H+1},
			{ADXL367_REG_X_8_BIT, ADXL367_REG_X_8_BIT+2}

	};
	uint8_t start_addr;
	uint8_t end_addr;
	uint8_t reg_buf;

	k = 0;
	for(arr_index = 0; arr_index < COUNT_OF(reg_addrs); arr_index++){
		start_addr = reg_addrs[arr_index][0];
		end_addr = reg_addrs[arr_index][1];

		for(j = start_addr; j <= end_addr; j++){
			reg_buf = j;
			result = adxl367_read_reg(&reg_buf, 1);
			if (result < 0)
				return -1;
			buf[2 * k] = j;
			buf[2 * k + 1] = reg_buf;
			k++;
		}
	}

	return 0;
}

static int adxl367_pop_data(sensor_data_t ** sensor_data){
	//sensor_data_t *accel_report_data = sensor_data[SH_adxl367];
	accel_report_t rd;
	int ret;
	int idx = 0;

	ret = dequeue(adxl367_accel.queue, &rd);
	if (ret != 0){
		pr_info ("Accel Deque Error:%d, len:%d\r\n",ret,queue_len(adxl367_accel.queue));
		return ret;
	}

	adxl367_report_buf[idx++] = ((int)rd.x >> 8) & 0xFF;
	adxl367_report_buf[idx++] = ((int)rd.x) & 0xFF;

	adxl367_report_buf[idx++] = ((int)rd.y >> 8) & 0xFF;
	adxl367_report_buf[idx++] = ((int)rd.y) & 0xFF;

	adxl367_report_buf[idx++] = ((int)rd.z >> 8) & 0xFF;
	adxl367_report_buf[idx++] = ((int)rd.z) & 0xFF;

	//pr_info("adxl367_ss.c ACC_X %d ACC_Y %d ACC_Z %d\n", rd.x, rd.y, rd.z);

	adxl367_accel.report_buf.ts = ++time_stamp;
	return 0;
}

static int adxl367_sync_data(void){
	int ret = 0;
	/*  Clear fifo content */

	if(ret > 0)
		mAdxl->setFifoMode(ADXL367_FIFO_MODE_STREAM);

	return ret ? 0 : -1;
}

static int adxl367_get_num_samples(void * data){
	return queue_len(adxl367_accel.queue);
}

static int adxl367_get_sample_rate(void * data){
	int ret;
	adxl367_odr_t odr;
	odr = mAdxl->getOdr();
	switch(odr){
		case ADXL367_ODR_12P5HZ:
			return 12;
		case ADXL367_ODR_25HZ:
			return 25;
		case ADXL367_ODR_50HZ:
			return 50;
		case ADXL367_ODR_100HZ:
			return 100;
		case ADXL367_ODR_200HZ:
			return 200;
		case ADXL367_ODR_400HZ:
			return 400;
		default:
			return -1;
	}


	return -1;
}

static int adxl367_set_sample_rate(void * data, int sample_rate){
	int ret = 0;
	adxl367_odr_t odr;

	switch(sample_rate){
		case (12):
			odr =ADXL367_ODR_12P5HZ;
			break;
		case (25):
			odr =ADXL367_ODR_25HZ;
			break;
		case (50):
			odr =ADXL367_ODR_50HZ;
			break;
		case (100):
			odr =ADXL367_ODR_100HZ;
			break;
		case (200):
			odr =ADXL367_ODR_200HZ;
			break;
		case (400):
			odr =ADXL367_ODR_400HZ;
			break;
		default:
			ret  = -1;
			break;
	}

	if(ret == 0){
		mAdxl->setOdr(odr);
	}

	return ret > 0 ? 0: -1;
}

static int adxl367_get_irq_state(void * data){
	return 0;
}



static void adxl367_int_handler_self_test(void * cbdata){
		uint8_t *buf = cbdata;

		buf[0] &= ~FAILURE_INT;

		GPIO_RegisterCallback(&lis2dh12_irq, NULL, NULL);
		GPIO_IntDisable(&lis2dh12_irq);
}

void adxl367_self_test_custom(uint8_t * txbuf){

	int ret;
	txbuf[0] = 0x00;

	txbuf[0] |= FAILURE_INT;

	//enable int1 pin on data ready
	uint8_t adxl_settings_registers[] = {0x2A};
	uint8_t adxl_settings_values[] = {0x01};


	//INT1 should trigger when FIFO has filled to watermark level
	GPIO_Config(&lis2dh12_irq);
	GPIO_RegisterCallback(&lis2dh12_irq, adxl367_int_handler_self_test, txbuf);
	GPIO_IntConfig(&lis2dh12_irq, GPIO_INT_EDGE, GPIO_INT_RISING );
	GPIO_IntEnable(&lis2dh12_irq);
	NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(lis2dh12_irq.port));

	adxl367_write_reg(adxl_settings_registers,adxl_settings_values);

	adxl367_execute_once(NULL);
	adxl367_execute_once(NULL);
	adxl367_execute_once(NULL);

	mxc_delay(MXC_DELAY_MSEC(100));

}

int adxl367_get_cfg_params(void * data, uint8_t param, uint8_t * tx_buf, int tx_buf_len){
	switch(param) {
		case (CFG_PARAM_OPERATING_MODES):
			if (tx_buf_len < CFG_PARAM_OPERATING_MODES_LEN)
				return -1;
			tx_buf[0] = CFG_PARAM_OPERATING_MODES;
			tx_buf[1] = mAdxl->getOpMode();
			break;
		case CFG_CONVERTION_FACTOR_TO_MG:
			((double*)data)[0] = adxl367_conversion_to_mg[mAdxl->getRange()][1];
		default:
			return -1;
	}

	return 0;
}

int adxl367_set_cfg_params(void * data, uint8_t *params, int param_sz){
	int ret = 0;
	if (param_sz <= 0)
		return -1;

	switch(params[0]) {
		case (CFG_PARAM_OPERATING_MODES):
			mAdxl->setOpMode(params[1]);
			break;
		default:
			return -1;

	}

	return ret;
}



static int adxl367_poll(void* data)
{
	int ret = 0;
	accel_report_t rd;
	uint8_t raw_data[6]  = {0};

	uint8_t xHigh, xLow, yHigh, yLow, zHigh, zLow = 0;

	enter_critical_section();

	adxl367_range_t range = mAdxl->getRange();
	ret = mAdxl->read14bit(raw_data, 6);
	if(ret){
		usb_printf("ADXL Poll Error \r\n");
		return -1;
	}

	xLow = ((raw_data[1]>>2) & 0x3F) | ((raw_data[0] << 6) & 0xC0);
	xHigh = (raw_data[0] >> 2);
	yLow = ((raw_data[3]>> 2) & 0x3F) | ((raw_data[2] << 6)& 0xC0);
	yHigh = (raw_data[2] >> 2);
	zLow = ((raw_data[5]>> 2) & 0x3F) | ((raw_data[4] << 6)& 0xC0);
	zHigh = (raw_data[4] >> 2);

	if(raw_data[0] & 0x80)
	{
		xHigh |= (0xFF<<6);
	}
	if(raw_data[2] & 0x80)
	{
		yHigh |= (0xFF<<6);
	}
	if(raw_data[4] & 0x80)
	{
		zHigh |= (0xFF<<6);
	}

	rd.x = ((int16_t)(xLow | (uint16_t)(xHigh << 8)));//*adxl367_conversion_to_mg[range][1];
	rd.y = ((int16_t)(yLow | (uint16_t)(yHigh << 8)));//*adxl367_conversion_to_mg[range][1];
	rd.z = ((int16_t)(zLow | (uint16_t)(zHigh << 8)));//*adxl367_conversion_to_mg[range][1];

	//printf("%d - %d - %d \n", rd.x, rd.y, rd.z);
	ret = enqueue(&adxldataq, &rd);
	if (ret != 0) {
		pr_info("Accel Enque Error:%d, Len:%d\r\n", ret, queue_len(&adxldataq));
		return -1;
	}

	//double total_g =sqrt( x*(double)x+ y* (double)y + z*(double)z);
	//usb_printf("acc x:%d\t y:%d\t z:%d \t t:%f\r\n", rd.x,rd.y,rd.z,total_g);
	//usb_printf("acc x:%d\t y:%d\t z:%d \t t:%f\r\n", x,y,z,total_g);
	//usb_printf("acc x:%d\t y:%d\t z:%d \t t:%f\r\n", raw_data8[0],raw_data8[1],raw_data8[2],total_g);

	exit_critical_section();

	return 0;
}


static int adxl367_execute_once(void * dev_data){
	adxl367_poll(NULL);

	return 0;
}
