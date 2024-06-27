/*
 * regHandler_spi.c
 *
 *  Created on: Apr 8, 2022
 *      Author: Mert.Turker
 */
#include "regHandler_spi.h"
#include "spi_api.h"
#include "mxc_device.h"
#include "nvic_table.h"
#include "mxc_pins.h"

#define DATA_LEN 10
#define SPI_SPEED 8000000

#if defined(FTHR) || defined(FTHR2)
#define SS_PORT MXC_GPIO0
#define SS_PIN PIN_20
#elif MRD106
#define SS_PORT MXC_GPIO0
#define SS_PIN PIN_22
#endif

static int spiIndex;



/***** Globals *****/
static uint8_t rx_data[DATA_LEN];
static uint8_t tx_data[DATA_LEN];

static tRegHandlerErr spi_readReg(tRegHandler *handler, uint8_t addr, uint8_t *buffer, uint16_t length)
{
	int retVal = 0;
    tx_data[0] = 0x0B;
    tx_data[1] = addr;
    retVal = spi_api_read(spiIndex, tx_data, buffer, 2, length);
    if(retVal != 0)
    {
    	printf("spi_api_read failed");
    	return REG_HANDLER_CONNECTION_ERROR;
    }

    return REG_HANDLER_NO_ERROR;
}
static tRegHandlerErr spi_writeReg(tRegHandler *handler, uint8_t addr, uint8_t *buffer, uint16_t length)
{
	int retVal = 0;
	tx_data[0] = 0x0A;
	tx_data[1] = addr;
	memcpy(tx_data+2, buffer, length);
	retVal = spi_api_write(spiIndex, tx_data, length+2);
	if(retVal != 0)
	{
		printf("spi_api_write failed");
		return REG_HANDLER_CONNECTION_ERROR;
	}

	return REG_HANDLER_NO_ERROR;
}

static tRegHandlerErr spi_readFifo(tRegHandler *handler, uint8_t *buffer, uint16_t length)
{
	int retVal = 0;
	tx_data[0] = 0x0D;
    retVal = spi_api_read(spiIndex, tx_data, buffer, 1, length);
    if(retVal != 0)
    {
    	printf("spi_api_read failed");
    	return REG_HANDLER_CONNECTION_ERROR;
    }

    return REG_HANDLER_NO_ERROR;
}

tRegHandlerErr regHandler_spi_init(tRegHandler *handler)
{
	int retVal = 0;
	retVal = spi_api_init(SPI_SPEED);
	if(retVal != 0)
	{
	    	printf("spi_api_init failed");
	    	return REG_HANDLER_CONNECTION_ERROR;
	}

	retVal = spi_api_open(&spiIndex,  MXC_GPIO_GET_IDX(SS_PORT), SS_PIN);
	if(retVal != 0)
	{
	 	printf("spi_open failed");
	 	return REG_HANDLER_CONNECTION_ERROR;
	}
	handler->ctx = handler;
	handler->readReg = spi_readReg;
	handler->writeReg = spi_writeReg;
	handler->readFifo = spi_readFifo;

	return REG_HANDLER_NO_ERROR;
}
