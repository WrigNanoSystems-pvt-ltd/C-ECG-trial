/*
 * Copyright (C) 2021 Analog Devices, Inc.
 * Author: Ali Mert Turker <mert.turker@analog.com>
 */

#ifndef _ADXL367_H_
#define _ADXL367_H_
#include <stdint.h>
#include <stdbool.h>
#include "regHandler.h"
#include "adxl367_definitions.h"

typedef struct adxl367
{
	int (*init)(tRegHandler *regHandler);
	int (*reset)();

	void (*setRange)(adxl367_range_t);
	void (*setOdr)(adxl367_odr_t);
	void (*setOpMode)(adxl367_op_mode_t);
	void (*setFifoFormat)(adxl367_fifo_format_t);
	void (*setFifoMode)(adxl367_fifo_mode_t);
	void (*setFifoSize)(unsigned int fifo_set_size);
	void (*setFifoWatermark)(unsigned int fifo_watermark);
	void (*setWakeUpMode)(bool en);
	void (*setWakeUpRate)(adxl367_wakeup_rate_t);
	int (*setMeasureEnable)(bool en);

	adxl367_range_t (*getRange)();
	adxl367_odr_t (*getOdr)();
	adxl367_op_mode_t (*getOpMode)();
	adxl367_fifo_format_t (*getFifoFormat)();
	adxl367_fifo_mode_t (*getFifoMode)();
	unsigned int (*getFifoSize)();
	unsigned int (*getFifoWatermark)();
	bool (*getWakeUpMode)();
	adxl367_wakeup_rate_t (*getWakeUpRate)();
	bool (*getMeasureEnable)();
	int (*getFifoCnt)();

	int (*read8bit)(uint8_t *buffer, uint8_t size);
	int (*read14bit)(uint8_t *buffer, uint8_t size);
	int (*readFifo)(uint8_t *buffer, uint16_t size);

	tRegHandlerErr (*readRegister)(uint8_t address, uint8_t *value, uint16_t len);
	tRegHandlerErr (*writeRegister)(uint8_t address, uint8_t *value, uint16_t len);
}adxl367_t;

adxl367_t* getAdxl367();

#endif /* _ADXL367_H_ */
