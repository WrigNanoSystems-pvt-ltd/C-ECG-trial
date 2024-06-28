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


#include "max86178.h"
#include "max86178_platform.h"
#include "sensorhub.h"
#include "mxm_assert.h"
#include "adxl367.h"

#include "Peripherals.h"

#define ENABLE_MAX86178IRQ

extern pdata_t max86178_pdata;

// global queue buffer
char driver_fifo[sizeof(uint32_t) * MAX86178_DRIVER_FIFO_SZ];
char driver_fifo_algo[sizeof(uint32_t) * MAX86178_DRIVER_FIFO_SZ];
char device_info[sizeof(struct max86178_dev)];

queue_t queue_algo;

static max86178_fifo_read_cb fifo_read_cb;
extern sensor_t adxl367_accel;
extern uint8_t gUseAcc;
///DEBUG------------------------------------
#ifdef ENABLE_ACCEL_DEBUG
extern int32_t accelTimeStamp;
int32_t ppgTimeStamp = 0;
#endif
///------------------------------------------

#ifdef ENABLE_MAX86178IRQ
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
#endif

static int update_bits(uint8_t reg, uint8_t mask, uint8_t val)
{
    int ret_val = MAX86178_SUCCESS;
    uint8_t reg_val;

    if (val > mask) {
        ret_val = MAX86178_FAILURE;
    } else {
        // place val to masked position
        int i = 0;

        for (i = 0 ; i < sizeof(mask) * 8; i++)
            if (mask & 1 << i)
                break;

        val = val << i;
    }

    if (ret_val == MAX86178_SUCCESS) {
        ret_val = max86178_read_reg(0, &reg_val, 1);//(reg, &reg_val, 1);
        if (ret_val == MAX86178_SUCCESS) {
            uint8_t tmp = 0;

            tmp = reg_val & ~mask;
            tmp |= val & mask;
            ret_val = max86178_write_reg(0, reg, tmp);
        }
    }

    return ret_val;
}

static void *max86178_device_mem(void *dev_ptr)
{
	static void *dev_data_ptr;

	if (dev_ptr)
		dev_data_ptr = dev_ptr;
	return dev_data_ptr;
}

void *max86178_get_device_data(void)
{
	return max86178_device_mem(NULL);
}

static void *max86178_set_device_data(void *dev_ptr)
{
	return max86178_device_mem(dev_ptr);
}

int max86178_regulator_onoff(struct max86178_dev *sd, char enable)
{
	sd->regulator_state = enable;
	return 0;
}

int max86178_enter_test_mode(struct max86178_dev *sd)
{
	int ret = 0;
	return ret;
}

int max86178_exit_test_mode(struct max86178_dev *sd)
{
	int ret = 0;
	return ret;
}

int max86178_reset(struct max86178_dev *sd) // OK
{
	int ret = 0;

	if (!sd->regulator_state) {
		ret = max86178_regulator_onoff(sd, PWR_ON);
		if (ret < 0) {
			pr_err("Unable to turn on the regulator. %s:%d, ret: %d\n",
				__func__, __LINE__, ret);
		}
	}

	ret = max86178_write_reg(sd->comm, MAX86178_SYSTEM_CFG1_REG,
			MAX86178_SYSTEM_RESET_MASK);
	sd->die_temp.frac = 0;
	sd->die_temp.tint = 0;

	return ret;
}

int max86178_poweron(struct max86178_dev *sd){ // OK
	int ret = 0;
	uint8_t buf[1];

	buf[0] = MAX86178_SYSTEM_CFG1_REG;
	ret = max86178_read_reg(sd->comm, buf, 1);
	buf[0] &= ~MAX86178_SYSTEM_SHDN_MASK;
	ret = max86178_write_reg(sd->comm, MAX86178_SYSTEM_CFG1_REG,
									   buf[0]);
	if (sd->regulator_state) {
		ret |= max86178_regulator_onoff(sd, PWR_ON);
		if (ret < 0) {
			pr_err("Unable to turn off the regulator. %s:%d, ret: %d\n",
					__func__, __LINE__, ret);
		}
	}
	return ret;
}


int max86178_poweroff(struct max86178_dev *sd) // OK
{
	int ret = 0;
	uint8_t buf[1];

	buf[0] = MAX86178_SYSTEM_CFG1_REG;
	ret = max86178_read_reg(sd->comm, buf, 1);
	buf[0] |= MAX86178_SYSTEM_SHDN_MASK;
	ret = max86178_write_reg(sd->comm, MAX86178_SYSTEM_CFG1_REG,
									   buf[0]);
	if (sd->regulator_state) {
		ret |= max86178_regulator_onoff(sd, PWR_OFF);
		if (ret < 0) {
			pr_err("Unable to turn off the regulator. %s:%d, ret: %d\n",
					__func__, __LINE__, ret);
		}
	}
	return ret;
}


int max86178_enable_die_temp(struct max86178_dev *sd) //OK
{
	int ret = 0;

	return ret;
}

int max86178_init_fifo(struct max86178_dev *sd, uint8_t a_full_val) // OK
{
	int ret;

	ret = max86178_write_reg(sd->comm, MAX86178_FIFO_CFG1_REG,
			MAX86178_FIFO_A_FULL_MASK & a_full_val);

	ret |= max86178_write_reg(sd->comm, MAX86178_FIFO_CFG2_REG,
			MAX86178_FIFO_RO_MASK |
			MAX86178_A_FULL_ONCE |
			MAX86178_FIFO_STAT_CLR_MASK |
			MAX86178_FLUSH_FIFO_MASK);

	return ret;
}

int max86178_set_sample_rate(struct max86178_dev *sd, uint16_t rate) // OK , be sure internal oscillator is seleceted as frame clock
{
	int ret = -1;
	uint16_t	afe_clk, clk_div;
	uint8_t reg;

	/* Return if sample rate is zero */
	if (rate == 0)
		return ret;

	reg = MAX86178_PLL_CFG6_REG;
	ret = max86178_read_reg(sd->comm, &reg, 1);
	if (ret < 0) {
		pr_err("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	/* Identify selected AFE clock in Hz */
	if(reg & MAX86178_PLL_CLK_FREQ_SEL_MASK)
		afe_clk = 32768;
	else
		afe_clk = 32000;

	/* Calculate AFE clock divider */
	clk_div = afe_clk / rate;

	/* Set AFE clock division into the MAX86178 register */
	ret = max86178_write_reg(sd->comm, MAX86178_PPG_FRM_RATE_LSB_REG, (clk_div & MAX86178_FR_CLK_DIV_L_MASK));
	if (ret < 0) {
		pr_err("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	max86178_write_reg(sd->comm, MAX86178_PPG_FRM_RATE_MSB_REG, ((clk_div >> 8) & MAX86178_FR_CLK_DIV_H_MASK));
	if (ret < 0) {
		pr_err("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

int max86178_get_sample_rate(struct max86178_dev *sd) // OK be sure to set internal oscillator as ppg frame clock
{
	int ret = -1;
	uint8_t reg;
	uint16_t	afe_clk, clk_div, sample_rate;

	reg = MAX86178_PLL_CFG6_REG;
	ret = max86178_read_reg(sd->comm, &reg, 1);
	if (ret < 0) {
		pr_err("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	/* Identify selected AFE clock in Hz */
	if(reg & MAX86178_PLL_CLK_FREQ_SEL_MASK)
		afe_clk = 32768;
	else
		afe_clk = 32000;

	/* Read AFE clock division  */
	reg = MAX86178_PPG_FRM_RATE_MSB_REG;
	ret = max86178_read_reg(sd->comm, &reg, 1);
	if (ret < 0) {
		pr_err("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	/* MSB of clock division */
	clk_div = reg & MAX86178_FR_CLK_DIV_H_MASK;
	clk_div <<= 8;

	reg = MAX86178_PPG_FRM_RATE_LSB_REG;
	ret = max86178_read_reg(sd->comm, &reg, 1);
	if (ret < 0) {
		pr_err("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}
	/* LSB of clock division */
	clk_div |= reg;

	/* Calculate sample rate */
	sample_rate = afe_clk / clk_div;

	return sample_rate;
}

int max86178_get_meas_num(struct max86178_dev *sd) // OK
{
	int ret = -1;
	uint8_t reg, i;
	uint8_t meas_num = 0;

	reg = MAX86178_PPG_CFG1_REG;
	ret = max86178_read_reg(sd->comm, &reg, 1);
	if (ret < 0) {
		pr_err("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	for(i=0; i < 6; i++)
	{
		if(reg & 0x1)
			meas_num++;

		reg >>= 1;
	}


	return meas_num;
}


int max86178_read_die_temp(struct max86178_dev *sd) //OK
{

	return 0;
}

static inline int max86178_get_num_samples_in_fifo(struct max86178_dev *sd) //OK
{

    int ret_val = MAX86178_SUCCESS;
    uint8_t reg_val[2] = {0};

    reg_val[0] = MAX86178_OVF_CNT_REG;

    if (ret_val == MAX86178_SUCCESS) {
       // ret_val = read_reg(MAX86178_FIFO_CNT1_REG, reg_val, 2);
    	ret_val = max86178_read_reg(sd->comm, reg_val, 2);

        if (ret_val == MAX86178_SUCCESS) {
            if ((reg_val[0] & ~MAX86178_FIFO_DATA_COUNT_MSB_MASK) > 0) {
                // Data overflow
                ret_val = MAX86178_MAX_FIFO_DEPTH;
            } else {
                ret_val = (reg_val[0] & MAX86178_FIFO_DATA_COUNT_MSB_MASK) ? MAX86178_MAX_FIFO_DEPTH : reg_val[1];
            }
        }
    }

    return ret_val;

}

static inline int max86178_read_fifo_data(struct max86178_dev *sd, uint8_t *fifo_data, int num_samples) // OK
{
	uint16_t num_bytes = num_samples * MAX86178_DATA_WORD_SIZE;
	int ret = 0;

	fifo_data[0] = MAX86178_FIFO_DATA_REG;
	ret = max86178_read_reg(sd->comm, fifo_data, num_bytes);
	if (ret < 0) {
		pr_err("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	return ret;
}
//static uint8_t counter = 0;
int max86178_fifo_irq_handler(struct max86178_dev *sd) // OK check for ECG-ACC interaction is required.
{
	uint8_t fifo_buf[MAX86178_MAX_FIFO_DEPTH * MAX86178_DATA_WORD_SIZE];
	int ret;
	int num_samples;
	int ppg_discard = 0;
	fifo_data_t fifo_data;
	uint16_t idx;
	int i;
	static unsigned int ppg_data_number = 0;

	num_samples = max86178_get_num_samples_in_fifo(sd);

//	printf("num_sample %d count %d\n", num_samples, counter);
//	counter++;

	if (num_samples <= 0) {
		pr_debug("%s:%d failed. err: %d\n", __func__, __LINE__, num_samples);
		return 0;
	}
	//pr_debug("ppgSamp: %d\n",num_samples);
	ret = max86178_read_fifo_data(sd, fifo_buf, num_samples);
	if (ret < 0) {
		pr_debug("%s:%d failed. ret: %d\n", __func__, __LINE__, ret);
		return -1;
	}

	if(fifo_read_cb)
	{
		int ret = fifo_read_cb();
		if(ret == 1)
			ppg_discard = 0;
	}

	///DEBUG------------------------------------
#ifdef ENABLE_ACCEL_DEBUG
	ppgTimeStamp = (platform_get_time_ms());
#endif
	/// ------------------------------------

	for (i = 0; i < num_samples; i++) {
		idx = MAX86178_DATA_WORD_SIZE * i;
		fifo_data.raw = fifo_buf[idx + 0] << 16
				| fifo_buf[idx + 1] << 8
				| fifo_buf[idx + 2];

		if(fifo_data.type < DATA_TYPE_PF && ppg_discard){
			continue;
		}

		if (fifo_data.type < DATA_TYPE_PF) { // algo queue for PPG algo processing
			fifo_data_t algo_fifo_data = fifo_data;

			if(algo_fifo_data.val > MAX86178_MAX_PPG_VALUE)
				algo_fifo_data.val = 0;

			enqueue(&queue_algo, &algo_fifo_data);
			ppg_data_number++;
		}

		ret = enqueue(&sd->queue, &fifo_data);
		if (ret < 0) {
			pr_err("Enqueue data is failed. ret: %d, thrown: %d\n",
					ret, num_samples - i);
			return -1;
		}
#ifdef ENABLE_ACCEL_DEBUG
		{
			static int pe = 0;
			printf("p:%d\n",pe++);
		}
#endif
	}

		/*
			Please note that this method can only work with single sample reading per interrupt
			from OS58, otherwise it breaks Accel and OS58 sync.
		*/


	uint8_t ppg_enabled, ecg_enabled, ch_enabled, iq_enabled, acc_enabled;
	uint8_t acc_read_count = 0, ppg1 = 0 , ppg2 = 0;

	max86178_is_ppg_enabled(&ppg_enabled);
	max86178_is_ecg_enabled(&ecg_enabled);
    max86178_is_acc_enabled(&acc_enabled);
	max86178_is_iq_enabled(&iq_enabled);

	max86178_is_ppg1_enabled(&ppg1);
	max86178_is_ppg2_enabled(&ppg2);

	if(0 == ppg1 && 0 == ppg2)
		ppg_enabled = 0;

	if(ppg_enabled)
	{
		ppg1 = ppg1 + ppg2;
		max86178_get_num_of_channel(&ch_enabled);

        if(acc_enabled)
        {
            if(ppg1)
                acc_read_count = ppg_data_number / (ch_enabled * ppg1);
            else
                acc_read_count = 0;
            for(i = 0; i < acc_read_count; i++){
                //lis2dh12_poll_data_buffer(NULL);
                adxl367_accel.execute_once(NULL);
        		ppg_data_number -= (ch_enabled * ppg1);
            }
        }
	}

	return num_samples;
}

int max86178_set_led_current(struct max86178_dev *sd, uint8_t led_num, int led_mA_val) //OK
{
	return 0;
}


// Initial AFE configuration with no timing data enabled
struct regmap ppg_init_cfg[] = {
   //MYG fill with MEAS1-2 config & ECG default for Test purposes - check ECG default from MRD104
	{MAX86178_SYSTEM_CFG1_REG, MAX86178_SYSTEM_SHDN_MASK}, // CSB/I2C_SEL pin selects interface , disable all timing data
	/* PLL */
	///////////////////////////////////////////////////////////////////////////////{MAX86178_PIN_FUNC_CFG_REG, 0x1E},
	{MAX86178_PLL_CFG1_REG, 0x01},  // PLL EN MDIV = 256 PLL_CLK = 32768*256
	{MAX86178_PLL_CFG2_REG, 0xFF},  // PLL EN M =  MDIV+1 = 256 PLL_CLK = 32768*256
	//BiOZ Clock
	{MAX86178_PLL_CFG3_REG, 0x00},  // BioZ NDIV = 256, BIOZ KDIV = 1
    //ECG ADC Clock
	{MAX86178_PLL_CFG4_REG, 0x03}, // ECG_FDIV = 4
	{MAX86178_PLL_CFG5_REG, 0x40}, // ECG_NDIV = 64
    // 32768 internal osc as input to PLL
	{MAX86178_PLL_CFG6_REG, 0x20 }, // internal 32678Hz ref clock for PLL
    //PPG frame clock
	{MAX86178_PPG_FRM_RATE_MSB_REG, 0x05},
	{MAX86178_PPG_FRM_RATE_LSB_REG, 0x1B}, // PPG to 25 hz with ref_clk 32768 division of 1307 - 0x51B

	{MAX86178_PPG_CFG1_REG, 0x03}, // En chan1-2-3
    //MEAS1 settings:
	{MAX86178_MEAS1_SELECT_REG, 0x00 }, // Both to LED1_DRV
	{MAX86178_MEAS1_CFG1_REG  , 0x5A },
	{MAX86178_MEAS1_CFG2_REG  , 0x33 },
	{MAX86178_MEAS1_CFG3_REG  , 0x00 },
	{MAX86178_MEAS1_CFG4_REG  , 0x47 },
	{MAX86178_MEAS1_CFG5_REG  , 0x0E },
	{MAX86178_MEAS1_LED_A_REG , 0x1E },
	{MAX86178_MEAS1_LED_B_REG , 0x00 },
    //MEAS2 settings:
	{MAX86178_MEAS2_SELECT_REG, 0x09 }, // Both to LED2_DRV
	{MAX86178_MEAS2_CFG1_REG  , 0x5A },
	{MAX86178_MEAS2_CFG2_REG  , 0x33 },
	{MAX86178_MEAS2_CFG3_REG  , 0x00 },
	{MAX86178_MEAS2_CFG4_REG  , 0x47 },
	{MAX86178_MEAS2_CFG5_REG  , 0x0E },
	{MAX86178_MEAS2_LED_A_REG , 0x1F },
	{MAX86178_MEAS2_LED_B_REG , 0x00 },
    //ECG settings
	{MAX86178_ECG_CFG1_REG, 0x04 },  // DEC= 64 => ECG SR = 512 with 32768 ECG_ADC clock, ECG DISABLED IN THIS CONFIG !! TO ENABLE WRITE : 0x05
	{MAX86178_ECG_CFG2_REG, 0x81 },  //PGA gain is 1V/V , INA gain is 20V/V
	{MAX86178_ECG_CFG3_REG, 0x01 },  // Automatic INA recovery disabled , Enable the ECG MUX and assigned to ECGP = ECG_EL1 , ECGN= ECG_EL2, RLD = ECG_EL3
	{MAX86178_ECG_CFG4_REG, 0x3D },  // Automatic fast recovery normal mode , Fast recovery threshold set to 95% of full scale
	{MAX86178_CAL_CFG_1_REG, 0x00 },
	{MAX86178_CAL_CFG_2_REG, 0x00 },
	{MAX86178_CAL_CFG_3_REG, 0x00 },
	{MAX86178_LEAD_DETECT_CFG_1_REG, 0x00 },
	{MAX86178_LEAD_DETECT_CFG_2_REG, 0x00 },
	{MAX86178_LEAD_BIAS_CFG_1_REG, 0x04 },
	{MAX86178_RLD_CFG_1_REG, 0xCF },
	{MAX86178_RLD_CFG_2_REG, 0x40 },

    //BiOZ Respiration setting
	{MAX86178_BIOZ_CFG1_REG, 0xF4}, //ADC_OSR = 1024 , DAC_OSR = 256, ECG and BioZ bandgap bias enabled
	{MAX86178_BIOZ_CFG2_REG, 0x00 }, // Set the digital HPF to bypass, Set the digital LPF to bypass,
	{MAX86178_BIOZ_CFG3_REG, 0x28 },
	{MAX86178_BIOZ_CFG4_REG, 0x00 },
	{MAX86178_BIOZ_CFG5_REG, 0x00 },
	{MAX86178_BIOZ_CFG6_REG, 0x33 }, // Set the analog HPF to 1kHz
	{MAX86178_BIOZ_CFG7_REG, 0x8A },
	{MAX86178_BIOZ_CFG8_REG, 0x02 },
	{MAX86178_BIOZ_LOW_THRESH_REG, 0x00 },
	{MAX86178_BIOZ_HIGH_THRESH_REG, 0xFF },
	{MAX86178_BIOZ_MUX_CFG1_REG , 0x42}, // Enable the MUX
	{MAX86178_BIOZ_MUX_CFG2_REG , 0x02},
	{MAX86178_BIOZ_MUX_CFG3_REG , 0xA0},
	{MAX86178_BIOZ_MUX_CFG4_REG , 0x1B},
	{MAX86178_BIOZ_LEAD_DETCT_CFG1_REG, 0x00},
	{MAX86178_BIOZ_LOFF_THRESH_REG, 0x00},
	{MAX86178_BIOZ_LEAD_BIAS_CFG1_REG, 0x07},
	{MAX86178_RESPIRATION_CFG1_REG, 0x00},

};


struct regmap locationfinder_cfg[] = {
	    {MAX86178_PPG_CFG1_REG, 0x03}, // En chan1-2
	    {MAX86178_MEAS1_SELECT_REG, 0x09 }, // Both to LED2_DRV
	    {MAX86178_MEAS1_CFG1_REG  , 0x5A },  // 10 avearaging, tint=117
	    {MAX86178_MEAS1_CFG2_REG , 0x33 },
	    {MAX86178_MEAS1_LED_A_REG , 0x1E },
	    {MAX86178_MEAS1_LED_B_REG , 0x1E },
	    {MAX86178_MEAS2_SELECT_REG, 0x00 }, // Both to LED2_DRV
	    {MAX86178_MEAS2_CFG1_REG  , 0x5A },  // 10 avearaging, tint=117
	    {MAX86178_MEAS2_CFG2_REG , 0x33 },
	    {MAX86178_MEAS2_LED_A_REG , 0x1E },
	    {MAX86178_MEAS2_LED_B_REG , 0x1E },
};

// Configuration backup used by location finder
struct regmap shadow_cfg[] = {
   //MYG fill with MEAS1-2 config & ECG default for Test purposes - check ECG default from MRD104
	{MAX86178_SYSTEM_CFG1_REG, MAX86178_SYSTEM_SHDN_MASK}, // CSB/I2C_SEL pin selects interface , disable all timing data
	/* PLL */
	///////////////////////////////////////////////////////////////////////////////{MAX86178_PIN_FUNC_CFG_REG, 0x1E},
	{MAX86178_PLL_CFG1_REG, 0x01},  // PLL EN MDIV = 256 PLL_CLK = 32768*256
	{MAX86178_PLL_CFG2_REG, 0xFF},  // PLL EN M =  MDIV+1 = 256 PLL_CLK = 32768*256
	//BiOZ Clock
	{MAX86178_PLL_CFG3_REG, 0x00},  // BioZ NDIV = 256, BIOZ KDIV = 1
    //ECG ADC Clock
	{MAX86178_PLL_CFG4_REG, 0x03}, // ECG_FDIV = 4
	{MAX86178_PLL_CFG5_REG, 0x40}, // ECG_NDIV = 64
    // 32768 internal osc as input to PLL
	{MAX86178_PLL_CFG6_REG, 0x20 }, // internal 32678Hz ref clock for PLL
    //PPG frame clock
	{MAX86178_PPG_FRM_RATE_MSB_REG, 0x05},
	{MAX86178_PPG_FRM_RATE_LSB_REG, 0x1B}, // PPG to 25 hz with ref_clk 32768 division of 1307 - 0x51B

	{MAX86178_PPG_CFG1_REG, 0x03}, // En chan1-2
    //MEAS1 settings:
	{MAX86178_MEAS1_SELECT_REG, 0x00 }, // Both to LED1_DRV
	{MAX86178_MEAS1_CFG1_REG  , 0x5A },
	{MAX86178_MEAS1_CFG2_REG  , 0x33 },
	{MAX86178_MEAS1_CFG3_REG  , 0x00 },
	{MAX86178_MEAS1_CFG4_REG  , 0x47 },
	{MAX86178_MEAS1_CFG5_REG  , 0x0E },
	{MAX86178_MEAS1_LED_A_REG , 0x1E },
	{MAX86178_MEAS1_LED_B_REG , 0x00 },
    //MEAS2 settings:
	{MAX86178_MEAS2_SELECT_REG, 0x09 }, // Both to LED2_DRV
	{MAX86178_MEAS2_CFG1_REG  , 0x5A },
	{MAX86178_MEAS2_CFG2_REG  , 0x33 },
	{MAX86178_MEAS2_CFG3_REG  , 0x00 },
	{MAX86178_MEAS2_CFG4_REG  , 0x47 },
	{MAX86178_MEAS2_CFG5_REG  , 0x0E },
	{MAX86178_MEAS2_LED_A_REG , 0x1F },
	{MAX86178_MEAS2_LED_B_REG , 0x00 },
    //ECG settings
	{MAX86178_ECG_CFG1_REG, 0x04 },  // DEC= 64 => ECG SR = 512 with 32768 ECG_ADC clock, ECG DISABLED IN THIS CONFIG !! TO ENABLE WRITE : 0x05
	{MAX86178_ECG_CFG2_REG, 0x81 },  //PGA gain is 1V/V , INA gain is 20V/V
	{MAX86178_ECG_CFG3_REG, 0x01 },  // Automatic INA recovery disabled , Enable the ECG MUX and assigned to ECGP = ECG_EL1 , ECGN= ECG_EL2, RLD = ECG_EL3
	{MAX86178_ECG_CFG4_REG, 0x3D },  // Automatic fast recovery normal mode , Fast recovery threshold set to 95% of full scale
	{MAX86178_CAL_CFG_1_REG, 0x00 },
	{MAX86178_CAL_CFG_2_REG, 0x00 },
	{MAX86178_CAL_CFG_3_REG, 0x00 },
	{MAX86178_LEAD_DETECT_CFG_1_REG, 0x00 },
	{MAX86178_LEAD_DETECT_CFG_2_REG, 0x00 },
	{MAX86178_LEAD_BIAS_CFG_1_REG, 0x04 },
	{MAX86178_RLD_CFG_1_REG, 0xCF },
	{MAX86178_RLD_CFG_2_REG, 0x40 },

    //BiOZ Respiration setting
	{MAX86178_BIOZ_CFG1_REG, 0xF4}, //ADC_OSR = 1024 , DAC_OSR = 256, ECG and BioZ bandgap bias enabled
	{MAX86178_BIOZ_CFG2_REG, 0x00 }, // Set the digital HPF to bypass, Set the digital LPF to bypass,
	{MAX86178_BIOZ_CFG3_REG, 0x28 },
	{MAX86178_BIOZ_CFG4_REG, 0x00 },
	{MAX86178_BIOZ_CFG5_REG, 0x00 },
	{MAX86178_BIOZ_CFG6_REG, 0x33 }, // Set the analog HPF to 1kHz
	{MAX86178_BIOZ_CFG7_REG, 0x8A },
	{MAX86178_BIOZ_CFG8_REG, 0x02 },
	{MAX86178_BIOZ_LOW_THRESH_REG, 0x00 },
	{MAX86178_BIOZ_HIGH_THRESH_REG, 0xFF },
	{MAX86178_BIOZ_MUX_CFG1_REG , 0x42}, // Enable the MUX
	{MAX86178_BIOZ_MUX_CFG2_REG , 0x02},
	{MAX86178_BIOZ_MUX_CFG3_REG , 0xA0},
	{MAX86178_BIOZ_MUX_CFG4_REG , 0x1B},
	{MAX86178_BIOZ_LEAD_DETCT_CFG1_REG, 0x00},
	{MAX86178_BIOZ_LOFF_THRESH_REG, 0x00},
	{MAX86178_BIOZ_LEAD_BIAS_CFG1_REG, 0x07},
	{MAX86178_RESPIRATION_CFG1_REG, 0x00},

};

int max86178_sensor_enable(struct max86178_dev *sd, int agc_enable, int enable) // OK
{
	int ret = 0;
	uint8_t a_full;

	queue_reset(&sd->queue);
	if (enable) {

		ret = max86178_poweron(sd);
		if (ret < 0)
			goto fail;

		max86178_irq_clr_to_zero();

		/*INTx is enabled and gets cleared automatically after 244us  */
		//ret = max86178_write_reg(NULL, MAX86178_PIN_FUNC_CFG_REG, 0x19); // check whether needed! INT1 is the INT line in MAX88678 Wrist demo schematic
		ret = max86178_write_reg(NULL, MAX86178_SYSTEM_PIN_FUNC_REG, 0x1F); //
		if(ret < 0)
			goto fail;

		ret = max86178_write_reg(NULL, MAX86178_INT1_ENABLE1_REG,  MAX86178_A_FULL_EN_MASK | MAX86178_FRAME_RDY_EN_MASK);
		ret = max86178_write_reg(NULL, MAX86178_INT2_ENABLE1_REG,  MAX86178_PPG_FRAME_RDY_EN2_MASK); //MAX86178_A_FULL_EN_MASK | MAX86178_FRAME_RDY_EN_MASK);

		//Flush fifo
		uint8_t fifo_cfg2_value= MAX86178_FIFO_CFG2_REG;

		ret = max86178_read_reg(NULL, &fifo_cfg2_value,1);

		ret = max86178_write_reg(NULL, MAX86178_FIFO_CFG2_REG,  fifo_cfg2_value | MAX86178_FLUSH_FIFO_MASK); //MAX86178_A_FULL_EN_MASK | MAX86178_FRAME_RDY_EN_MASK);

		if(ret < 0)
			goto fail;
	} else {
		ret = max86178_poweroff(sd);
		if (ret < 0)
			goto fail;
	}
	return 0;

fail:
	pr_err("%s failed. ret: %d\n", __func__, ret);
	return ret;
}

int max86178_startup_init(struct max86178_dev *sd) //OK
{
	int ret = 0;
	uint8_t a_full;

	ret = max86178_reset(sd);
	if (ret < 0)
		goto fail;

	ret |= max86178_block_write(sd->comm, ppg_init_cfg, ARRAY_SIZE(ppg_init_cfg));

	a_full = MAX86178_FIFO_AFULL;

	ret |= max86178_write_reg(sd->comm, MAX86178_FIFO_CFG1_REG, a_full);

	max86178_irq_clr_to_zero();
	ret |= max86178_poweroff(sd);
	pr_info("%s done\n", __func__);
	return ret;
fail:
	pr_err("%s failed. ret: %d\n", __func__, ret);
	return ret;
}

int max86178_irq_handler(void *arg) // OK check for overlapping read of status reg2
{
	struct max86178_dev *sd = arg;
	int ret = 0;
	union int_status status;

	status.val[0] = MAX86178_STATUS1_REG;
	ret = max86178_read_reg(sd->comm, status.val, 2);
	//pr_debug("Status reg: %X %X\n", status_s.val[0], status_s.val[1]);
	if (ret < 0) {
		pr_err("Comm failed. err: %d. %s:%d\n",
			ret, __func__, __LINE__);
		return -1;
	}

	if (status.a_full || status.fifo_data_rdy || status.frame_rdy) {
		return max86178_fifo_irq_handler(sd);
	}

	if (status.thresh1_hilo)
			pr_debug("thresh1_hilo interrupt was triggered.\n");

	if (status.thresh2_hilo)
			pr_debug("thresh2_hilo interrupt was triggered.\n");

	if (status.pwr_rdy)
		pr_info("Pwr_rdy interrupt was triggered.\n");

	if (status.alc_ovf)
			pr_info("alc_ovf interrupt was triggered.\n");

	if (status.exp_ovf)
			pr_info("exp_ovf interrupt was triggered.\n");


	status.val[0] = MAX86178_STATUS2_REG;
	ret = max86178_read_reg(sd->comm, status.val, 2);
	//pr_debug("Status reg: %X %X\n", status_s.val[0], status_s.val[1]);
	if (ret < 0) {
		pr_err("Comm failed. err: %d. %s:%d\n",
			ret, __func__, __LINE__);
		return -1;
	}

	if (status.invalid_cfg)
		pr_debug("INVALID AFE CFG!\n");

	/*if (status.vdd_oor) {
		sd->vdd_oor_cnt++;
		pr_info("VDD Out of range cnt: %d\n", sd->vdd_oor_cnt);
	}*/

	return 0;
}

static const uint8_t max86178_part_info[][2] = { // check for MRD106 conf
	{MAX86178_PART_ID_VAL, 1},
};

int max86178_get_part_info(dev_comm_t *comm,
		uint8_t *part_id, uint8_t *rev_id, uint8_t *num_pd)  //OK check
{
	int i;
	int ret;
	uint8_t buf[2];

	buf[0] = MAX86178_REV_ID_REG;
	//buf[0] = MAX86178_PART_ID_REG;
	ret = max86178_read_reg(comm, buf, 2);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(max86178_part_info); i++) {
		if (buf[1] == max86178_part_info[i][0]) {
			*num_pd = max86178_part_info[i][1];
			*rev_id = buf[0];
			*part_id = buf[1];
			return 0;
		}
	}

	*part_id = 0x39; // MYG check to fake Wilson GUI for test pusposes
	// Unsupported part
	return -1;
}

void max86178_print_raw_data(void) //OK
{
	fifo_data_t fifo_data;
	int ret;
	static int cnt = 0;
	static int sample_cnt = 0;
	static uint32_t led_data[10];
	uint32_t ir, red, green;
	struct max86178_dev *max86178_driver = max86178_get_device_data();
	//bool dump_reg = 1;

	ret = max86178_sensor_enable(max86178_driver, 0, 1);
	if (ret < 0){
		pr_err("%s:%d - Err: %d\n", __func__, __LINE__, ret);
	}
	else{
		pr_info("Max86140 successful initialized\n");
	}


#if 0
	extern int max86178_comm_test(dev_comm_t *comm);

	if (dump_reg) {
			max86178_comm_test(max86178_driver->comm);
			dump_reg = 0;
	}
#endif

	while(1) {
		/* Polling */
		//max86178_fifo_irq_handler(max86178_get_device_data());
		ret = dequeue(&max86178_driver->queue, &fifo_data);
		if (ret < 0)
			continue;

#if 1
#if 1
		pr_info("\tT: %d\n", fifo_data.type);
#else
		pr_debug("\tType: -> Type: %d, val:%d, Raw:%X\n",
					fifo_data.type, fifo_data.val, fifo_data.raw);
#endif
#endif
		if (fifo_data.type > DATA_TYPE_MEAS6) {
			pr_debug("Wrong Data Type: -> Type: %d, val:%d, Raw:%X\n",
					fifo_data.type, fifo_data.val, fifo_data.raw);
			continue;
		}

		cnt++;
		led_data[fifo_data.type] = fifo_data.val;
		if (cnt < 6)
			continue;

		cnt = 0;
		sample_cnt++;
		green = led_data[DATA_TYPE_MEAS1];
		ir = led_data[DATA_TYPE_MEAS2];
		red = led_data[DATA_TYPE_MEAS3];

		pr_info("%d, %d, %d, %d\n\n\n", sample_cnt, green, ir, red);


		memset(led_data, 0, sizeof(led_data));
	}

	UNUSED(green);
	UNUSED(ir);
	UNUSED(red);

	return;
}

/* MAX86178 IRQ Functions */
static volatile uint16_t max86178_irq_cnt = 0;
int max86178_get_irq_state(void *data) //OK
{
	int ret;
	enter_critical_section();
	ret = max86178_irq_cnt;
	exit_critical_section();
	return ret;
}

void max86178_irq_clr(){
	uint8_t read_buf[3];

	if(max86178_get_irq_state(NULL) > 0){ //OK
		read_buf[0] = MAX86178_STATUS1_REG;
		/* Read three status registers to clear interrupts */
		max86178_read_reg(NULL, read_buf, 3);
		enter_critical_section();
		max86178_irq_cnt--;
		exit_critical_section();
	}
}

void max86178_irq_clr_to_zero(){ //OK
	uint8_t read_buf[3];
	read_buf[0] = MAX86178_STATUS1_REG;
	/* Read three status registers to clear interrupts */
	max86178_read_reg(NULL, read_buf, 3);
	enter_critical_section();
	max86178_irq_cnt = 0;
	exit_critical_section();
}

void max86178_irq_reset_ref_cnt() { //OK
	enter_critical_section();
	max86178_irq_cnt = 0;
	exit_critical_section();
}

#ifdef ENABLE_MAX86178IRQ
static void max86178_irq_handler_fast(void *data) {
	max86178_irq_cnt++;
}
#endif

/* Max8614X IRQ Functions */



int max86178_init(pdata_t *pdata) //OK
{
	int ret = 0;
	uint8_t part_id, rev_id, num_pd;
	struct max86178_dev *handle;
	void *queue_buf;
	void *queue_buf_algo;

	ret = max86178_get_part_info(pdata->comm, &part_id, &rev_id, &num_pd);
	if (ret < 0) {
		pr_info("MAX86178 is not detected. Part_id: 0x%X, Rev_Id: 0x%X\n",
				part_id, rev_id);
		goto fail;
	}
	pr_info("MAX86178 detected. Part_id: 0x%X, Rev_Id: 0x%X\n", part_id, rev_id);

	handle = (void *)device_info;

	if (handle == NULL) {
		pr_err("%s:%d - Out of memory\n", __func__, __LINE__);
		ret = -2;
		goto fail;
	}

	memset(handle, 0, sizeof(struct max86178_dev));
	handle->part_id = part_id;
	handle->rev_id = rev_id;
	handle->num_pd = num_pd;
	max86178_set_device_data(handle);
	handle->comm = pdata->comm;
	handle->dac_calib_status = 0x00;

	queue_buf = (void *)driver_fifo;
	if (queue_buf == NULL) {
		ret = -4;
		goto fail;
	}

	queue_buf_algo = (void *)driver_fifo_algo;
	if (NULL == queue_buf_algo){
		ret = -4;
		goto fail;
	}


	//----------------------------------------------------
	ret = queue_init(&handle->queue, queue_buf,
			sizeof(fifo_data_t), sizeof(uint32_t) * MAX86178_DRIVER_FIFO_SZ);
	if (ret < 0) {
		ret = -5;
		goto fail;
	}

	//----------------------------------------------------
	ret = queue_init(&queue_algo, queue_buf_algo,
			sizeof(fifo_data_t), sizeof(uint32_t) * MAX86178_DRIVER_FIFO_SZ);
	if (ret < 0) {
		ret = -5;
		goto fail;
	}


#ifdef ENABLE_MAX86178IRQ
	register_gpio_irq_handler(max86178_irq_handler_fast, pdata->gpio, handle);
	//register_gpio_irq_handler((void *)max86178_irq_handler, pdata->gpio, handle);
#endif
	/* OS64 does not need DAC calibration */
	handle->operating_cfg.dac_calib = 0;
	handle->operating_cfg.firmware_default = 1;
	ret = max86178_startup_init(handle);
	if (ret < 0) {
		ret = -3;
		goto fail;
	}

	return ret;
fail:
	pr_err("Init failed %s:%d\n", __func__, __LINE__, ret);
	return ret;
}

#if 0
static int max86178_remove(struct max86178_dev *sd)
{
	queue_destroy(&sd->queue);
	free(sd);
	return 0;
}
#endif

int max86178_dump_regs(dev_comm_t *port, uint8_t *buf,
		uint8_t start_addr, uint8_t end_addr) //OK check for result register val order
{
  int i;
  int32_t ret;
  uint8_t tmp[2];

  for (i = start_addr; i <= end_addr; i++) {
	tmp[0] = i;
    ret = max86178_read_reg(port, tmp, 1);
    if (ret != 0)
      return ret;
	buf[i - start_addr] = tmp[0];
  }

  return 0;
}

int max86178_get_num_of_channel(uint8_t * p_num_ch) //OK
{
	int ret = 0;
	uint8_t index = 0;
	uint8_t num_of_ch = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	uint8_t reg[1];

	reg[0] = MAX86178_PPG_CFG1_REG;
	ret = max86178_read_reg(sd->comm, reg, 1);


	if(0 == ret){
		for(index = 0; index < MAX86178_MEAS_CHMAX; index++){
			num_of_ch += ((reg[0] >> index) & 1);
		}
	}

	*p_num_ch = num_of_ch;

	return ret;
}

int max86178_is_ppg_enabled(uint8_t * p_ppg_enabled) //OK
{
	int ret = 0;
	uint8_t nm_ch = 0;

	ret = max86178_get_num_of_channel(&nm_ch);

	if(0 == ret){
		if(nm_ch > 0){
			*p_ppg_enabled = 1;
		}
		else{
			*p_ppg_enabled = 0;
		}
	}

	return  ret;
}

int max86178_is_ecg_enabled(uint8_t *p_ecg_enabled) //OK
{
	int ret = 0;

	uint8_t reg = MAX86178_EGC_CONFIG_1_REG;
	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_read_reg(sd->comm, &reg, 1);

	if(0 == ret){
		if((reg & MAX86178_ECG_ENABLE_MASK)){
			*p_ecg_enabled = 1;
		}
		else{
			*p_ecg_enabled = 0;
		}
	}

	return ret;
}

int max86178_is_iq_enabled(uint8_t * p_iq_enabled) // OK
{
	int ret = 0;

	uint8_t reg = MAX86178_BIOZ_CFG1_REG;
	struct max86178_dev *sd = max86178_get_device_data();


	ret = max86178_read_reg(sd->comm, &reg, 1);

	if(0 == ret){
		if((reg & MAX86178_BIOZ_ENABLE_MASK)){
			*p_iq_enabled = 1;
		}
		else{
			*p_iq_enabled = 0;
		}
	}

	return ret;
}

int max86178_is_acc_enabled(uint8_t * p_acc_enabled)
{
    *p_acc_enabled = gUseAcc;
    return 0;
}

int max86178_is_ecg_faster_than_ppg(uint8_t *p_is_true) // OK check for MRD106 conf for PLL/Clock selection - get form ME15 ECG config
{
	int ret = 0;

	uint8_t regs[2] = {0};

	uint32_t  ppgFrameRateDiv;
	uint32_t  ecgDecRateDiv;

	struct max86178_dev *sd = max86178_get_device_data();

	/*Reading PPG Frame Rate*/
	regs[0] = MAX86178_FR_CLK_DIV_MSB_REG;
	ret = max86178_read_reg(sd->comm, regs, 2);
	if(0 == ret){
		regs[0] &= 0x7F;
		ppgFrameRateDiv = ((uint32_t)regs[0] << 8) | ((uint32_t)regs[1]);

		/*ECG Frame Rate*/
		regs[0] = MAX86178_EGC_CONFIG_1_REG;
		ret = max86178_read_reg(sd->comm, regs, 1);
	}

	if(0 == ret){
		ecgDecRateDiv = (uint32_t)(regs[0] >> 1);
		ecgDecRateDiv &= 0x07;
		ecgDecRateDiv = (1<<ecgDecRateDiv)*16;

		if(ecgDecRateDiv < ppgFrameRateDiv){
			*p_is_true = 1;
		}
		else{
			*p_is_true = 0;
		}

	}

	return ret;
}

int max86178_is_bioz_faster_than_ppg(uint8_t *p_is_true)
{
	int ret = 0;

	uint8_t regs[2] = {0};

	uint32_t  ppgFrameRateDiv;
	uint32_t  biozDecRateDiv;

	struct max86178_dev *sd = max86178_get_device_data();

	/*Reading PPG Frame Rate*/
	regs[0] = MAX86178_FR_CLK_DIV_MSB_REG;
	ret = max86178_read_reg(sd->comm, regs, 2);
	if(0 == ret){
		regs[0] &= 0x7F;
		ppgFrameRateDiv = ((uint32_t)regs[0] << 8) | ((uint32_t)regs[1]);

		/*ECG Frame Rate*/
		regs[0] = MAX86178_EGC_CONFIG_1_REG;
		ret = max86178_read_reg(MAX86178_PLL_CFG3_REG, regs, 1);
	}

	if(0 == ret){

		uint32_t divNBIOZ = regs[0]>>6;
		divNBIOZ = 1<<((divNBIOZ<3?divNBIOZ:2)+8);
		max86178_read_reg(MAX86178_FR_CLK_32000_DIV_LSB_200HZ, regs, 1);
		biozDecRateDiv = (regs[0]>>3)&7;
		biozDecRateDiv = (1<<(biozDecRateDiv+3));

		if(biozDecRateDiv < ppgFrameRateDiv){
			*p_is_true = 1;
		}
		else{
			*p_is_true = 0;
		}

	}

	return ret;
}

int max86178_get_num_photo_diodes(uint8_t *p_num_diode) //OK
{
	int ret = 0;

	uint8_t reg = MAX86178_PPG_CFG2_REG;
	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_read_reg(sd->comm, &reg, 1);

	reg &= (MAX86178_PPG1_PWRDN_MASK | MAX86178_PPG2_PWRDN_MASK);
	reg >>= 2;

	if(0 == ret){
		if(3 == reg)
			*p_num_diode = 0;
		if(1 == reg || 2 == reg)
			*p_num_diode = 1;
		if(0 == reg)
			*p_num_diode = 2;
	}


	return ret;
}

int max86178_is_enabled(uint8_t * p_is_enabled) //OK
{
	int ret = 0;

	uint8_t is_iq_enabled = 0;
	uint8_t is_ecq_enabled = 0;
	uint8_t is_ppg_enabled = 0;

	ret = max86178_is_iq_enabled(&is_iq_enabled);

	if(0 == ret){
		ret = max86178_is_ecg_enabled(&is_ecq_enabled);
	}

	if(0 == ret){
		ret = max86178_is_ppg_enabled(&is_ppg_enabled);
	}

	if(0 == ret){
		*p_is_enabled = is_ppg_enabled | is_ecq_enabled | is_iq_enabled;
	}
	else{
		*p_is_enabled = 0;
	}

	return ret;
}

int max86178_set_frame_ready_int(uint8_t enable) // OK Check for MRD106 max86178 interrupt line
{

#define FIFO_INT_CTRL_REQ_MRD106 (MAX86178_INT2_ENABLE1_REG)

	int ret = 0;

	uint8_t reg = FIFO_INT_CTRL_REQ_MRD106;
	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_read_reg(sd->comm, &reg, 1);

	if(enable){
		reg |= 0x40;
	}
	else{
		reg &= 0xBF;
	}

	if(0 == ret){
		ret = max86178_write_reg(sd->comm, FIFO_INT_CTRL_REQ_MRD106, reg);
	}

	return ret;
}

int max86178_set_a_full_int(uint8_t enable) // OK Check for MRD106 max86178 interrupt line
{

#define FIFO_INT_CTRL_REQ_MRD106 (MAX86178_INT2_ENABLE1_REG)

	int ret = 0;

	uint8_t reg = FIFO_INT_CTRL_REQ_MRD106;
	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_read_reg(sd->comm, &reg, 1);

	if(enable){
		reg |= 0x80;
	}
	else{
		reg &= 0x7F;
	}

	if(0 == ret){
		ret = max86178_write_reg(sd->comm, FIFO_INT_CTRL_REQ_MRD106, reg);
	}

	return ret;
}

int max86178_set_fifo_a_full(uint8_t level) // OK
{
	int ret = 0;

	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_write_reg(sd->comm, MAX86178_FIFO_CFG1_REG, MAX86178_MAX_FIFO_DEPTH - level);

	return ret;

}


const uint8_t measch_leds_regs[MAX86178_MEAS_CHMAX][MAX86178_LED_DRIVER_MAX] =
{
		{0x36u, 0x37u}, {0x3Eu, 0x3Fu}, {0x46u, 0x47u},
		{0x4Eu, 0x4Fu}, {0x56u, 0x57u}, {0x5Eu, 0x5Fu},

}; //OK

//const uint8_t measx_led_rge_regs[MAX86178_MEAS_CHMAX] = {0x22, 0x2A, 0x32, 0x3A, 0x42, 0x4A, 0x52, 0x5A, 0x62};

const uint8_t measx_led_rge_regs[MAX86178_MEAS_CHMAX] = {MAX86178_MEAS1_CFG4_REG, MAX86178_MEAS2_CFG4_REG, MAX86178_MEAS3_CFG4_REG,
														 MAX86178_MEAS4_CFG4_REG, MAX86178_MEAS5_CFG4_REG,MAX86178_MEAS6_CFG4_REG  }; // OK


const float led_curr_steps[] = {0.125, 0.250, 0.375, 0.50};

int max86178_get_list_of_enabled_channels(uint8_t * p_channels) //OK
{
	int ret = 0;
	int index = 0;
	uint8_t reg = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	reg = MAX86178_PPG_CFG1_REG;
	ret = max86178_read_reg(sd->comm, &reg, 1);

	if(0 == ret){
		for(index = 0; index < MAX86178_MEAS_CHMAX; index++){
			p_channels[index] = ((reg >> index) & 1u);
		}
	}

	return ret;
}

int max86178_enable_channels(uint8_t * p_channels) // 0 for disable, 1 for enable
{
	int ret = 0;
	int index = 0;
	uint8_t reg = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	if(0 == ret){
		for(index = 0; index < MAX86178_MEAS_CHMAX; index++){
			reg |= (p_channels[index]& 1u) << index;
		}
	}
	ret = max86178_write_reg(sd->comm, MAX86178_PPG_CFG1_REG, reg);


	return ret;
}


int max86178_set_leds_current(max86178_meas_ch_t ch, max86178_led_type led_type, uint8_t led_current) // OK Check for MRD106 LED drive routing to LEDs
{
	int ret = 0;
	uint8_t reg_addr = 0;
	float curr_per_step = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	if(ch >= MAX86178_MEAS_CHMAX || led_type >= MAX86178_LED_DRIVER_MAX){
		ret = -1;
	}

	if(0 == ret){
		reg_addr = measch_leds_regs[ch][led_type];

		ret |=  max86178_get_led_current_steps(ch, &curr_per_step);

		led_current = (uint8_t)((float)led_current / curr_per_step);

		ret |= max86178_write_reg(sd->comm, reg_addr, led_current);
	}

	return ret;
}

int max86178_get_leds_current(max86178_meas_ch_t ch, max86178_led_type led_type, uint8_t * led_current) //OK Check for MRD106 LED drive routing to LEDs
{
	int ret = 0;
	uint8_t reg_addr = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	if(ch >= MAX86178_MEAS_CHMAX || led_type >= MAX86178_LED_DRIVER_MAX){
		ret = -1;
	}

	if(0 == ret){
		reg_addr = measch_leds_regs[ch][led_type];

		ret = max86178_read_reg(sd->comm, &reg_addr, 1);
		*led_current = reg_addr;
	}

	return ret;
}

int max86178_get_led_current_steps(max86178_meas_ch_t ch, float * curr_per_step) // OK
{
	int ret = 0;
	uint8_t reg_addr = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	if(ch >= MAX86178_MEAS_CHMAX || curr_per_step == NULL){
		ret = -1;
	}

	if(0 == ret){
		reg_addr = measx_led_rge_regs[ch];
		ret = max86178_read_reg(sd->comm, &reg_addr, 1);
	}

	if(0 == ret){
		reg_addr &= 0b00000011;
		//reg_addr >>= 0;
		*curr_per_step = led_curr_steps[reg_addr];
	}

	return ret;
}

int max86178_is_ppg1_enabled(uint8_t * p_is_enabled) //OK
{
	int ret = 0;
	uint8_t reg_addr = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	if(p_is_enabled == NULL){
		ret = -1;
	}

	if(0 == ret){
		reg_addr = MAX86178_PPG_CFG2_REG;
		ret = max86178_read_reg(sd->comm, &reg_addr, 1);
	}

	if(0 == ret){
		if(reg_addr & 0b00000100){
			*p_is_enabled = 0;
		}
		else{
			*p_is_enabled = 1;
		}
	}

	return ret;
}


int max86178_is_ppg2_enabled(uint8_t * p_is_enabled) //OK
{
	int ret = 0;
	uint8_t reg_addr = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	if(p_is_enabled == NULL){
		ret = -1;
	}

	if(0 == ret){
		reg_addr = MAX86178_PPG_CFG2_REG;
		ret = max86178_read_reg(sd->comm, &reg_addr, 1);
	}

	if(0 == ret){
		if(reg_addr & 0b00001000){
			*p_is_enabled = 0;
		}
		else{
			*p_is_enabled = 1;
		}
	}

	return ret;
}



int max86178_turn_led_on(max86178_leds_t led) // OK check for MRD106 configuration , LED here means channel firing assoicated LED
{
	int ret = 0;
	uint8_t reg_addr = MAX86178_PPG_CFG1_REG;
	const uint8_t led_regs[MAX86178_LED_MAX][3] = {
												{0x30, 0x36, 0x37}, // updated for MAX86178
												{0x38, 0x3E, 0x3F},
												{0x40, 0x46, 0x47}
											};

	struct max86178_dev *sd = max86178_get_device_data();

	if(led >= MAX86178_LED_MAX)
		ret = -1;

	if(0 == ret){

		max86178_poweron(sd);

		ret = max86178_read_reg(sd->comm, &reg_addr, 1);

		if(0 == ret){
			reg_addr |= 1 << led;

			ret = max86178_write_reg(sd->comm, MAX86178_PPG_CFG1_REG, reg_addr);
		}
	}

	if(0 == ret){
		/* Drives LED1 */
		ret = max86178_write_reg(sd->comm, led_regs[led][0], led); // drive check this
	}

	if(0 == ret){
		/* Full current for first LED Driver */
		ret =  max86178_write_reg(sd->comm, led_regs[led][1], 0xFF);
	}

	if(0 == ret){
		/* Turns of second LED driver */
		ret = max86178_write_reg(sd->comm, led_regs[led][2], 0x00);
	}


	return ret;
}

int max86178_turn_led_off(max86178_leds_t led) // OK check for Green it is not available in MRD106 cech which Channel drives which LEDs
{
	int ret = 0;
	uint8_t reg_addr = MAX86178_PPG_CFG1_REG;

	struct max86178_dev *sd = max86178_get_device_data();

	if(led >= MAX86178_LED_MAX)
		ret = -1;

	if(0 == ret){
		ret = max86178_read_reg(sd->comm, &reg_addr, 1);
		reg_addr &= ~(1 << led); // close channel
		ret = max86178_write_reg(sd->comm, 0x11, reg_addr);
	}

	return ret;
}

int max86178_disable_all_ch() //OK
{
	int ret = 0;

	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_write_reg(sd->comm, MAX86178_PPG_CFG1_REG, 0x00);

	return ret;
}

int max86178_clear_fifo() //OK
{
	int ret = 0;
	unsigned char byt = 0;
	struct max86178_dev *sd;

	/* Getting sensor descriptor */
	sd = max86178_get_device_data();

	if(NULL == sd)
		ret = -1;

	if(0 == ret)
	{
		byt = MAX86178_FIFO_CFG2_REG;
		ret = max86178_read_reg(sd->comm, &byt, 1);
	}

	if(0 == ret)
	{
		byt |= MAX86178_FLUSH_FIFO_MASK;
		ret = max86178_write_reg(sd->comm, MAX86178_FIFO_CFG2_REG, byt);
	}

	return ret;
}

int max86178_set_integration_time(uint8_t meas, uint8_t int_time) //OK
{
	int ret = 0;
	unsigned char byt = 0;
	struct max86178_dev *sd;

	const static uint8_t reg_addr[9] = {MAX86178_MEAS1_CFG1_REG, MAX86178_MEAS2_CFG1_REG, MAX86178_MEAS3_CFG1_REG, \
										MAX86178_MEAS4_CFG1_REG, MAX86178_MEAS5_CFG1_REG, MAX86178_MEAS6_CFG1_REG };

	/* Getting sensor descriptor */
	sd = max86178_get_device_data();

	if(meas >= 6)
		return -1;

	/* Reading the related register */
	if (0 == ret) {
		byt = reg_addr[meas];
		ret = max86178_read_reg(sd->comm, &byt, 1);
	}

	if(0 == ret) {
		/* Clearing average setting */
		byt = (byt & ~MAX86178_MEAS_TINT_MASK);

		/* Setting new DAC offset settings for the request PPGx */
		byt |= (int_time << MAX86178_MEAS_TINT_POS);

		/* Write this value to the related register address */
		ret = max86178_write_reg(sd->comm, reg_addr[meas], byt);

		uint8_t temp = reg_addr[meas];
		ret = max86178_read_reg(NULL, & temp, 1);
	}

    return ret;
}

int max86178_set_average_sample(uint8_t meas, unsigned int avg_sample) // OK
{
	int ret = 0;
	unsigned char byt = 0;
	int avg_index = 0;
	struct max86178_dev *sd;

	const static uint8_t reg_addr[6] = {MAX86178_MEAS1_CFG1_REG, MAX86178_MEAS2_CFG1_REG, MAX86178_MEAS3_CFG1_REG, \
										MAX86178_MEAS4_CFG1_REG, MAX86178_MEAS5_CFG1_REG, MAX86178_MEAS6_CFG1_REG };

	/* Getting sensor descriptor */
	sd = max86178_get_device_data();

	if(meas >= 6) {
		ret = -1;
	}

	/* Reading the related register */
	if (0 == ret) {
		byt = reg_addr[meas];
		ret = max86178_read_reg(sd->comm, &byt, 1);
	}

	if(0 == ret) {
		/* Clearing average setting */
		byt = (byt & ~MAX86178_MEAS_AVER_MASK);

		/* Converting algorithm average units to sensor average units */
		ret = -1;
		for(avg_index = 0; avg_index < 7; ++avg_index)
		{
			if( (1<<avg_index) == avg_sample)
			{
				ret = 0;
				break;
			}
		}
	}

	if(0 == ret)
	{
		/* Setting new average setting */
		byt |= (avg_index << MAX86178_MEAS_AVG_POS);

		/* Write this value to the related register address */
		ret = max86178_write_reg(sd->comm, reg_addr[meas], byt);


		uint8_t temp = reg_addr[meas];
		ret = max86178_read_reg(NULL, & temp, 1);

	}

	return ret;
}

int max86178_set_dac_offset(uint8_t meas, uint8_t pd_sel, uint8_t offset) //OK
{
	int ret = 0;
	unsigned char byt = 0;
	struct max86178_dev *sd;

	const static uint8_t reg_addr[6] = {MAX86178_MEAS1_CFG3_REG, MAX86178_MEAS2_CFG3_REG, MAX86178_MEAS3_CFG3_REG, \
										MAX86178_MEAS4_CFG3_REG, MAX86178_MEAS5_CFG3_REG, MAX86178_MEAS6_CFG3_REG } ;

	const static uint8_t dac_offset_mask[2] = {MAX86178_MEAS_PPG1_DAC_OFF_MASK, MAX86178_MEAS_PPG2_DAC_OFF_MASK};

	const static uint8_t dac_offset_pos[2] = {MAX86178_MEAS_PPG1_DAC_OFF_POS, MAX86178_MEAS_PPG2_DAC_OFF_POS};

	/* Getting sensor descriptor */
	sd = max86178_get_device_data();

	if(meas >= 6 || offset > 0x0F) {
		ret = -1;
	}

	/* Reading the related register */
	if (0 == ret) {
		byt = reg_addr[meas];
		ret = max86178_read_reg(sd->comm, &byt, 1);
	}

	if (0 == ret) {
		/* Clearing DAC offset settings for the request PPGx */
		byt = (byt & ~dac_offset_mask[pd_sel]);

		/* Setting new DAC offset settings for the request PPGx */
		byt |= (offset << dac_offset_pos[pd_sel]);

		/* Write this value to the related register address */
		ret = max86178_write_reg(sd->comm, reg_addr[meas], byt);
	}

	return ret;
}


int max86178_fill_header_data(uint8_t header[126]) // OK: MYG check header registers, update for BiOZ if required
{
	int ret = 0;
	struct max86178_dev *sd;
	/* Getting sensor descriptor */
	sd = max86178_get_device_data();

	header[0] = 0x10;
	ret |= max86178_read_reg(sd->comm, &header[0], 1);

	header[1] = 0x11;
	ret |= max86178_read_reg(sd->comm, &header[1], 1);

	header[2] = 0x12;
	ret |= max86178_read_reg(sd->comm, &header[2], 1);

	header[3] = 0x13;
	ret |= max86178_read_reg(sd->comm, &header[3], 1);

	header[4] = 0x14;
	ret |= max86178_read_reg(sd->comm, &header[4], 1);

	header[5] = 0x18;
	ret |= max86178_read_reg(sd->comm, &header[5], 1);

	header[6] = 0x19;
	ret |= max86178_read_reg(sd->comm, &header[6], 1);

	header[7] = 0x1A;
	ret |= max86178_read_reg(sd->comm, &header[7], 1);

	header[8] = 0x1C;
	ret |= max86178_read_reg(sd->comm, &header[8], 1);

	header[9] = 0x1D;
	ret |= max86178_read_reg(sd->comm, &header[9], 1);

	header[10] = 0x1E;
	ret |= max86178_read_reg(sd->comm, &header[10], 1);

	/* Meas1 */
	header[11] = 0x20;
	ret |= max86178_read_reg(sd->comm, &header[11], 7);

	header[20] = 0xFE;
	ret |= max86178_read_reg(sd->comm, &header[20], 1);

	header[21] = 0xFF;
	ret |= max86178_read_reg(sd->comm, &header[21], 1);

	/* Meas2 */
	header[36] = 0x28;
	ret |= max86178_read_reg(sd->comm, &header[36], 7);

	/* Meas3 */
	header[45] = 0x30;
	ret |= max86178_read_reg(sd->comm, &header[45], 7);

	/* Meas4 */
	header[54] = 0x38;
	ret |= max86178_read_reg(sd->comm, &header[54], 7);

	/* Meas5 */
	header[63] = 0x40;
	ret |= max86178_read_reg(sd->comm, &header[63], 7);

	/* Meas6 */
	header[72] = 0x48;
	ret |= max86178_read_reg(sd->comm, &header[72], 7);

	/* Meas7 */
	header[81] = 0x50;
	ret |= max86178_read_reg(sd->comm, &header[81], 7);

	/* Meas8 */
	header[90] = 0x58;
	ret |= max86178_read_reg(sd->comm, &header[90], 7);

	/* Meas9 */
	header[99] = 0x60;
	ret |= max86178_read_reg(sd->comm, &header[99], 7);

	header[108] = 0x90;
	ret |= max86178_read_reg(sd->comm, &header[108], 12);

	header[120] = 0x9E;
	ret |= max86178_read_reg(sd->comm, &header[120], 1);

	header[121] = 0xA8;
	ret |= max86178_read_reg(sd->comm, &header[121], 2);

	return ret;
}

void max86178_register_fifo_read_callback(max86178_fifo_read_cb func)
{
	fifo_read_cb = func;
}

void max86178_unregister_fifo_read_callback()
{
	fifo_read_cb = NULL;
}

int max86178_bioz_enable(max86178_bioz_meas_t meas)
{
    int ret_val = MAX86178_SUCCESS;

    if (meas >= MAX86178_BIOZ_MEAS_INVALID)
        ret_val = MAX86178_FAILURE;

    if (ret_val == MAX86178_SUCCESS)
        ret_val = update_bits(MAX86178_BIOZ_CFG1_REG, MAX86178_BIOZ_ENABLE_MASK, (uint8_t)meas);

    return ret_val;
}

int max86178_bioz_disable(void)
{
    int ret_val = MAX86178_SUCCESS;

    if (ret_val == MAX86178_SUCCESS)
        ret_val = max86178_bioz_enable(MAX86178_BIOZ_MEAS_DISABLE);

    return ret_val;
}

int max86178_respiration_onoff(uint8_t onoff)
{
    int ret_val = MAX86178_SUCCESS;

    if (ret_val == MAX86178_SUCCESS)
        ret_val = update_bits(MAX86178_RESPIRATION_CFG1_REG, MAX86178_RESP_ENABLE_MASK, (uint8_t)(onoff > 0 ? 1 : 0));

    return ret_val;
}

int max86178_locationfinder_enable()
{
	int ret = 0;
	ret = max86178_block_read(max86178_pdata.comm, shadow_cfg, ARRAY_SIZE(shadow_cfg));
	if(ret < 0)
	{
		pr_info("max86178 location finder enable fault\r\n");
		return ret;
	}
	ret = max86178_init(&max86178_pdata);
	if(ret < 0)
	{
		pr_info("max86178 location finder enable fault\r\n");
		return ret;
	}
	ret = max86178_block_write(max86178_pdata.comm, locationfinder_cfg, ARRAY_SIZE(locationfinder_cfg));
	if(ret < 0)
	{
		pr_info("max86178 location finder enable fault\r\n");
		return ret;
	}

	return ret;
}

int max86178_locationfinder_disable()
{
	int ret = 0;

	ret = max86178_block_write(max86178_pdata.comm, shadow_cfg, ARRAY_SIZE(shadow_cfg));;
	if(ret < 0)
	{
		pr_info("max86178 location finder disable fault\r\n");
		return ret;
	}
	return ret;
}
