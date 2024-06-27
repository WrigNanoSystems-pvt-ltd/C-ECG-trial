/*
 * regHandler_i2c.c
 *
 *  Created on: Apr 15, 2022
 *      Author: Mert.Turker
 */
#include "regHandler_i2c.h"
#include "i2c_api.h"
#include "mxc_device.h"
#include "nvic_table.h"
#include "mxc_pins.h"

#define DATA_LEN 10
#define I2C_SPEED 100000
#define ASEL

#ifdef ASEL
#define I2C_ADDR 0x1D
#elif
#define I2C_ADDR 0x53
#endif


/***** Globals *****/
static uint8_t rx_data[DATA_LEN];
static uint8_t tx_data[DATA_LEN];

static tRegHandlerErr i2c_readReg(tRegHandler *handler, uint8_t addr, uint8_t *buffer, uint16_t length)
{
	int retVal = 0;


    retVal = i2c_api_write(ADXL367_I2C_BUS, I2C_ADDR, &addr, 1, 0);
	if(retVal != 0)
	{
		printf("i2c_api_write_failed. \n");
		return REG_HANDLER_CONNECTION_ERROR;
	}

    retVal = i2c_api_read(ADXL367_I2C_BUS,I2C_ADDR, buffer, length, 0);
    if(retVal != 0)
    {
    	printf("i2c_api_read failed");
    	return REG_HANDLER_CONNECTION_ERROR;
    }

    return REG_HANDLER_NO_ERROR;
}
static tRegHandlerErr i2c_writeReg(tRegHandler *handler, uint8_t addr, uint8_t *buffer, uint16_t length)
{
	int retVal = 0;
	tx_data[0] = addr;
	memcpy(tx_data+1, buffer, length);

	retVal = i2c_api_write(ADXL367_I2C_BUS,I2C_ADDR, tx_data, length+1, 0);
	if(retVal != 0)
	{
		printf("i2c_api_write_failed. \n");
		return REG_HANDLER_CONNECTION_ERROR;
	}

	return REG_HANDLER_NO_ERROR;
}

static tRegHandlerErr i2c_readFifo(tRegHandler *handler, uint8_t *buffer, uint16_t length)
{
	int retVal = 0;
	tx_data[0] = 0x0D;

	retVal = i2c_api_write(ADXL367_I2C_BUS,I2C_ADDR, &tx_data, 1, 0);
	if(retVal != 0)
	{
		printf("i2c_api_write_failed. \n");
		return REG_HANDLER_CONNECTION_ERROR;
	}

    retVal = i2c_api_read(ADXL367_I2C_BUS,I2C_ADDR, buffer, length, 0);
    if(retVal != 0)
    {
    	printf("spi_api_read failed");
    	return REG_HANDLER_CONNECTION_ERROR;
    }

    return REG_HANDLER_NO_ERROR;
}

tRegHandlerErr regHandler_i2c_init(tRegHandler *handler)
{
	int retVal = 0;
	retVal = i2c_api_init(ADXL367_I2C_BUS);
	if(retVal != 0)
	{
	    	printf("i2c_api_init failed");
	    	return REG_HANDLER_CONNECTION_ERROR;
	}

	handler->ctx = handler;
	handler->readReg = i2c_readReg;
	handler->writeReg = i2c_writeReg;
	handler->readFifo = i2c_readFifo;

	return REG_HANDLER_NO_ERROR;
}
