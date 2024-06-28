// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Analog Devices, Inc.
 * Author: Cosmin Tanislav <cosmin.tanislav@analog.com>
 * Modified: Ali Mert Turker <mert.turker@analog.com>
 */


#include "adxl367.h"

#include "mxc_delay.h"
#include <stdbool.h>
#include <stddef.h>


static adxl367_t mAdxl367;
static struct adxl367_state mState;
static tRegHandler *adxlHandler;

static int get_right_most_set_bit(uint32_t num)
{
  if(num==0)
  {  return 0;
  }
  else
  {
    int pos = 0;
    for (int i = 0; i < 32; i++) {
        if (!(num & (1 << i)))
            pos++;
        else
            break;
    }
    return pos;
  }
}


static int field_prep(int mask, int value)
{
	int rmsbPosition = get_right_most_set_bit(mask);

	return ((mask >> rmsbPosition) & value) << rmsbPosition;
}

static int field_get(int mask, int regis)
{
	int rmsbPosition = get_right_most_set_bit(mask);

	return (mask & regis) >> rmsbPosition;
}

static int _adxl367_set_act_proc_mode(enum adxl367_act_proc_mode mode)
{
	int ret = REG_HANDLER_NO_ERROR;
	uint8_t value  = field_prep(ADXL367_ACT_LINKLOOP_MASK, mode);

	ret = updateReg(adxlHandler, ADXL367_REG_ACT_INACT_CTL, ADXL367_ACT_LINKLOOP_MASK, value);

	return ret;
}

static void _adxl367_scale_act_thresholds(enum adxl367_range old_range, enum adxl367_range new_range)
{
	mState.act_threshold = mState.act_threshold
			    * adxl367_range_scale_factor_tbl[old_range]
			    / adxl367_range_scale_factor_tbl[new_range];
	mState.inact_threshold = mState.inact_threshold
			      * adxl367_range_scale_factor_tbl[old_range]
			      / adxl367_range_scale_factor_tbl[new_range];
}

static int _adxl367_set_act_threshold(enum adxl367_activity_type act, unsigned int threshold)
{
	uint8_t reg = adxl367_threshold_h_reg_tbl[act];
	int ret = REG_HANDLER_NO_ERROR;

	if (threshold > ADXL367_THRESH_MAX)
		return -1;

	mState.act_threshold_buf[0] = field_prep(ADXL367_THRESH_H_MASK, field_get(ADXL367_THRESH_VAL_H_MASK, threshold));
	mState.act_threshold_buf[1] = field_prep(ADXL367_THRESH_L_MASK, field_get(ADXL367_THRESH_VAL_L_MASK, threshold));


	ret = writeReg(adxlHandler, reg, mState.act_threshold_buf,	sizeof(mState.act_threshold_buf));
	if (ret != REG_HANDLER_NO_ERROR)
		return ret;

	if (act == ADXL367_ACTIVITY)
		mState.act_threshold = threshold;
	else
		mState.inact_threshold = threshold;

	return 0;
}
static int _adxl367_set_range(enum adxl367_range range)
{
	int ret = REG_HANDLER_NO_ERROR;

	ret = updateReg(adxlHandler, ADXL367_REG_FILTER_CTL, ADXL367_FILTER_CTL_RANGE_MASK,  field_prep(ADXL367_FILTER_CTL_RANGE_MASK, range));
	if (ret)
		return ret;

	_adxl367_scale_act_thresholds(mState.range, range);

	/* Activity thresholds depend on range */
	ret = _adxl367_set_act_threshold(ADXL367_ACTIVITY, mState.act_threshold);
	if (ret)
		return ret;

	ret = _adxl367_set_act_threshold(ADXL367_INACTIVITY, mState.inact_threshold);
	if (ret)
		return ret;

	mState.range = range;

	return ret;
}


static void adxl367_scale_act_thresholds(struct adxl367_state *st, enum adxl367_range old_range, enum adxl367_range new_range)
{
	st->act_threshold = st->act_threshold
			    * adxl367_range_scale_factor_tbl[old_range]
			    / adxl367_range_scale_factor_tbl[new_range];
	st->inact_threshold = st->inact_threshold
			      * adxl367_range_scale_factor_tbl[old_range]
			      / adxl367_range_scale_factor_tbl[new_range];
}

static int adxl367_set_act_threshold(struct adxl367_state *st, enum adxl367_activity_type act, unsigned int threshold)
{
	uint8_t reg = adxl367_threshold_h_reg_tbl[act];
	int ret = REG_HANDLER_NO_ERROR;

	if (threshold > ADXL367_THRESH_MAX)
		return -1;

	st->act_threshold_buf[0] = field_prep(ADXL367_THRESH_H_MASK, field_get(ADXL367_THRESH_VAL_H_MASK, threshold));
	st->act_threshold_buf[1] = field_prep(ADXL367_THRESH_L_MASK, field_get(ADXL367_THRESH_VAL_L_MASK, threshold));


	ret = writeReg(adxlHandler, reg, st->act_threshold_buf,	sizeof(st->act_threshold_buf));
	if (ret != REG_HANDLER_NO_ERROR)
		return ret;

	if (act == ADXL367_ACTIVITY)
		st->act_threshold = threshold;
	else
		st->inact_threshold = threshold;

	return 0;
}

static int _adxl367_time_ms_to_samples(unsigned int ms)
{
	int freq_hz = adxl367_samp_freq_tbl[mState.odr][0];
	int freq_microhz = adxl367_samp_freq_tbl[mState.odr][1];
	/* Scale to decihertz to prevent precision loss in 12.5Hz case. */
	int freq_dhz = freq_hz * 10 + freq_microhz / 100000;

	return (ms * freq_dhz / 10000);
}

static int _adxl367_set_act_time_ms(unsigned int ms)
{
	uint8_t val = _adxl367_time_ms_to_samples(ms);
	int ret = REG_HANDLER_NO_ERROR;

	if (val > ADXL367_TIME_ACT_MAX)
		val = ADXL367_TIME_ACT_MAX;

	ret = writeReg(adxlHandler, ADXL367_REG_TIME_ACT, &val, 1);
	if (ret)
		return ret;

	mState.act_time_ms = ms;

	return 0;
}

static int _adxl367_set_inact_time_ms(unsigned int ms)
{
	unsigned int val = _adxl367_time_ms_to_samples(ms);
	int ret = REG_HANDLER_NO_ERROR;

	if (val > ADXL367_TIME_INACT_MAX)
		val = ADXL367_TIME_INACT_MAX;

	mState.inact_time_buf[0] = field_prep(ADXL367_TIME_INACT_H_MASK,
					   field_get(ADXL367_TIME_INACT_VAL_H_MASK,
						     val));
	mState.inact_time_buf[1] = field_prep(ADXL367_TIME_INACT_L_MASK,
					   field_get(ADXL367_TIME_INACT_VAL_L_MASK,
						     val));

	ret = writeReg(adxlHandler, ADXL367_REG_TIME_INACT_H,
				mState.inact_time_buf, sizeof(mState.inact_time_buf));
	if (ret)
		return ret;

	mState.inact_time_ms = ms;

	return 0;
}


static int _adxl367_set_odr(enum adxl367_odr odr)
{
	int ret = REG_HANDLER_NO_ERROR;

	ret = updateReg(adxlHandler, ADXL367_REG_FILTER_CTL, ADXL367_FILTER_CTL_ODR_MASK, field_prep(ADXL367_FILTER_CTL_ODR_MASK, odr));
	if (ret)
		return ret;

	/* Activity timers depend on ODR */
	ret = _adxl367_set_act_time_ms(mState.act_time_ms);
	if (ret)
		return ret;

	ret = _adxl367_set_inact_time_ms(mState.inact_time_ms);
	if (ret)
		return ret;



	return 0;
}

static int _adxl367_set_fifo_mode(enum adxl367_fifo_mode fifo_mode)
{
	return updateReg(adxlHandler, ADXL367_REG_FIFO_CTL, ADXL367_FIFO_CTL_MODE_MASK, field_prep(ADXL367_FIFO_CTL_MODE_MASK, fifo_mode));
}

static int _adxl367_set_fifo_format(enum adxl367_fifo_format fifo_format)
{
	return updateReg(adxlHandler, ADXL367_REG_FIFO_CTL, ADXL367_FIFO_CTL_FORMAT_MASK, field_prep(ADXL367_FIFO_CTL_FORMAT_MASK, fifo_format));
}

static int _adxl367_set_fifo_samples(unsigned int fifo_watermark, unsigned int fifo_set_size)
{
	unsigned int fifo_samples = fifo_watermark * fifo_set_size;
	uint8_t fifo_samples_h, fifo_samples_l = 0;
	int ret = REG_HANDLER_NO_ERROR;

	if (fifo_samples > ADXL367_FIFO_MAX_WATERMARK)
		fifo_samples = ADXL367_FIFO_MAX_WATERMARK;

	if (fifo_set_size == 0)
		return 0;

	fifo_samples /= fifo_set_size;

	fifo_samples_h = field_prep(ADXL367_SAMPLES_H_MASK, field_get(ADXL367_SAMPLES_VAL_H_MASK, fifo_samples));
	fifo_samples_l = field_prep(ADXL367_SAMPLES_L_MASK, field_get(ADXL367_SAMPLES_VAL_L_MASK, fifo_samples));

	ret = updateReg(adxlHandler, ADXL367_REG_FIFO_CTL, ADXL367_SAMPLES_H_MASK, fifo_samples_h);
	if (ret)
		return ret;

	return updateReg(adxlHandler, ADXL367_REG_FIFO_SAMPLES, ADXL367_SAMPLES_L_MASK, fifo_samples_l);
}

static int _adxl367_read8bit(uint8_t *buffer, uint8_t size)
{
	//It returns 3 byte data for each axis
	return readReg(adxlHandler, ADXL367_REG_X_8_BIT	, buffer, 3);
}

static int _adxl367_read14bit(uint8_t *buffer, uint8_t size)
{
	//It returns 10 byte data as it follows
	// 0...10
	// |xHigh|xLow|yHigh|yLow|zHigh|zLow|tempHigh|tempLow|exAdcHigh|exAdcLow|
	return readReg(adxlHandler, ADXL367_REG_X_DATA_H, buffer, size);
}

static int _adxl367_readFifo(uint8_t *buffer, uint16_t length)
{
	return readFifo(adxlHandler, buffer, length);
}

static int _adxl367_getFifoCnt()
{
	int retVal = REG_HANDLER_NO_ERROR;
	uint8_t val = 0;
	retVal = readReg(adxlHandler, ADXL367_REG_FIFO_ENTRIES, &val, 1);
	if(retVal != REG_HANDLER_NO_ERROR)
	{
		return retVal;
	}

	return val;
}

static tRegHandlerErr _adxl367_readRegister(uint8_t address, uint8_t *value, uint16_t len)
{
	return readReg(adxlHandler, address, value, len);
}

static tRegHandlerErr _adxl367_writeRegister(uint8_t address, uint8_t* value, uint16_t len)
{
	return writeReg(adxlHandler, address, value , len);
}


static void _setRange(enum adxl367_range range)
{
	mState.range = range;
}

static void _setOdr(enum adxl367_odr odr)
{
	mState.odr = odr;
}

static void _setOpMode(enum adxl367_op_mode opMode)
{
	mState.opMode = opMode;
}

static void _setFifoFormat(enum adxl367_fifo_format fifo_format)
{
	mState.fifoFormat = fifo_format;
}

static void _setFifoMode(enum adxl367_fifo_mode fifo_mode)
{
	mState.fifoMode = fifo_mode;
}

static void _setFifoSize(unsigned int fifo_set_size)
{
	mState.fifo_set_size = fifo_set_size;
}

static void _setFifoWatermark(unsigned int fifo_watermark)
{
	mState.fifo_watermark = fifo_watermark;
}

static void _setWakeUpMode(bool wakeUpMode)
{
	mState.wakeUpMode = wakeUpMode;
}

static void _setWakeUpRate(adxl367_wakeup_rate_t rate)
{
	mState.wakeUpRate = rate;
}

static adxl367_range_t _getRange()
{
	return mState.range;
}

static adxl367_odr_t _getOdr()
{
	return mState.odr;
}

static adxl367_op_mode_t _getOpMode()
{
	return mState.opMode;
}

static adxl367_fifo_format_t _getFifoFormat()
{
	return mState.fifoFormat;
}

static adxl367_fifo_mode_t _getFifoMode()
{
	return mState.fifoMode;
}

static unsigned int _getFifoWaterMark()
{
	return mState.fifo_watermark;
}

static unsigned int _getFifoSize()
{
	return mState.fifo_set_size;
}

static unsigned int _getWakeUpMode()
{
	return mState.wakeUpMode;
}

static adxl367_wakeup_rate_t _getWakeUpRate()
{
	return mState.wakeUpRate;
}

static bool _getMeasureEnable()
{
	return mState.measureEnable;
}

static int _adxl367_set_wakeup_mode(bool wakeUpMode)
{
	uint8_t value = wakeUpMode ? 1 : 0;
	tRegHandlerErr ret = REG_HANDLER_NO_ERROR;
	value = field_prep(ADXL367_POWER_CTL_WAKEUP_MASK, value);

	ret = updateReg(adxlHandler, ADXL367_REG_POWER_CTL, ADXL367_POWER_CTL_WAKEUP_MASK, value);
	if(ret != REG_HANDLER_NO_ERROR)
		return ret;

	return ret;
}

static int _adxl367_set_wakeup_rate(adxl367_wakeup_rate_t rate)
{
	tRegHandlerErr ret = REG_HANDLER_NO_ERROR;
	uint8_t value = field_prep(ADXL367_TIMER_WAKEUP_MASK, rate);

	ret = updateReg(adxlHandler, ADXL367_REG_TIMER_CTL, ADXL367_TIMER_WAKEUP_MASK, value);
	if(ret != REG_HANDLER_NO_ERROR)
		return ret;
	return ret;
}


static int _adxl367_set_measure_en(bool en)
{
	adxl367_op_mode_t op_mode = en ? ADXL367_OP_MEASURE : ADXL367_OP_STANDBY;
	tRegHandlerErr ret = REG_HANDLER_NO_ERROR;
	uint8_t value = field_prep(ADXL367_POWER_CTL_MODE_MASK, op_mode);

	ret = updateReg(adxlHandler, ADXL367_REG_POWER_CTL, ADXL367_POWER_CTL_MODE_MASK, value);
	if (ret != REG_HANDLER_NO_ERROR)
		return ret;
	/*
	 * Wait for acceleration output to settle after entering
	 * measure mode.
	 */
	if (en)
	{
		//msleep(100);
	}

	mState.measureEnable = en;

	return 0;
}

static int _adxl367_verify_devid()
{
	uint8_t val = 0;
	uint16_t length = 1;
	tRegHandlerErr ret = REG_HANDLER_NO_ERROR;

	ret = readReg(adxlHandler, ADXL367_REG_DEVID, &val, length);
	if(val != ADXL367_DEVID_AD)
	{
		printf("Invalid Device ID : %X\r\n", val);
		return -1;
	}

	return 0;
}

static int _adxl367_setup()
{
	int ret = REG_HANDLER_NO_ERROR;

	ret = _adxl367_set_act_proc_mode(ADXL367_LOOPED);
	if (ret)
		return ret;

	ret = _adxl367_set_range(mState.range);
	if(ret)
		return ret;

	ret = _adxl367_set_odr(mState.odr);
	if (ret)
		return ret;

	if(!mState.wakeUpMode)
	{
		if(mState.fifoMode != ADXL367_FIFO_MODE_DISABLED)
		{
			_adxl367_set_fifo_mode(mState.fifoMode);
			_adxl367_set_fifo_format(mState.fifoFormat);
			ret = _adxl367_set_fifo_samples(mState.fifo_watermark, mState.fifo_set_size);
			if (ret)
				return ret;
		}
	}
	else
	{
		ret = _adxl367_set_wakeup_mode(mState.wakeUpMode);
		if(ret)
			return ret;
		ret = _adxl367_set_wakeup_rate(mState.wakeUpRate);
		if(ret)
			return ret;
	}



	return _adxl367_set_measure_en(true);
}

static int _adxl367_reset()
{
	uint8_t resetVal = ADXL367_RESET_CODE;
	return writeReg(adxlHandler, ADXL367_REG_RESET,  &resetVal, 1);
}

static int _adxl367_init(tRegHandler *regHandler)
{
	int ret = REG_HANDLER_NO_ERROR;

	if(regHandler == NULL)
	{
		return -1;
	}

	adxlHandler = regHandler;

	ret = _adxl367_reset();
	if (ret)
		return ret;

	mxc_delay(5000);

	ret = _adxl367_verify_devid();
	if (ret)
		return ret;

	ret = _adxl367_setup();
	if (ret)
		return ret;

	return ret;

}


void _adxl367()
{
	mAdxl367.init = _adxl367_init;
	mAdxl367.reset = _adxl367_reset;

	mAdxl367.setRange = _setRange;
	mAdxl367.setOdr = _setOdr;
	mAdxl367.setOpMode = _setOpMode;
	mAdxl367.setFifoFormat = _setFifoFormat;
	mAdxl367.setFifoMode = _setFifoMode;
	mAdxl367.setFifoWatermark = _setFifoWatermark;
	mAdxl367.setFifoSize = _setFifoSize;
	mAdxl367.setWakeUpMode = _setWakeUpMode;
	mAdxl367.setWakeUpRate = _setWakeUpRate;
	mAdxl367.setMeasureEnable = _adxl367_set_measure_en;

	mAdxl367.getRange = _getRange;
	mAdxl367.getOdr = _getOdr;
	mAdxl367.getOpMode = _getOpMode;
	mAdxl367.getFifoFormat = _getFifoFormat;
	mAdxl367.getFifoMode = _getFifoMode;
	mAdxl367.getFifoWatermark = _getFifoWaterMark;
	mAdxl367.getFifoSize = _getFifoSize;
	mAdxl367.getWakeUpMode = _getWakeUpMode;
	mAdxl367.getWakeUpRate = _getWakeUpRate;
	mAdxl367.getMeasureEnable = _getMeasureEnable;
	mAdxl367.getFifoCnt = _adxl367_getFifoCnt;

	mAdxl367.read14bit = _adxl367_read14bit;
	mAdxl367.read8bit = _adxl367_read8bit;
	mAdxl367.readFifo = _adxl367_readFifo;

	mAdxl367.readRegister = _adxl367_readRegister;
	mAdxl367.writeRegister = _adxl367_writeRegister;

	mState.range = ADXL367_2G_RANGE;
	mState.odr = ADXL367_ODR_100HZ;
	mState.fifoMode = ADXL367_FIFO_MODE_DISABLED;
	mState.fifo_watermark = 0x00;
	mState.fifo_set_size = 0x80;
	mState.wakeUpMode = false;
	mState.wakeUpRate = ADXL367_WAKEUP_RATE_12;
	mState.measureEnable = false;
}


adxl367_t* getAdxl367()
{
	_adxl367();

	return &mAdxl367;
}

