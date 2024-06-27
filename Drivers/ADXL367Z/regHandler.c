/*
 * regHandler.c
 *
 *  Created on: Apr 8, 2022
 *      Author: Mert.Turker
 */
#include "reghandler.h"
#include <stddef.h>

tRegHandlerErr readReg(tRegHandler *handler, uint8_t addr, uint8_t *buffer, uint16_t length)
{
	if(handler->readReg)
	{
		return handler->readReg(handler->ctx, addr, buffer, length);
	}

	return REG_HANDLER_SYSTEM_ERROR;
}
tRegHandlerErr writeReg(tRegHandler *handler, uint8_t addr, uint8_t *buffer, uint16_t length)
{
	if(handler->writeReg)
	{
		return handler->writeReg(handler->ctx, addr, buffer, length);
	}

	return REG_HANDLER_SYSTEM_ERROR;
}

tRegHandlerErr updateReg(tRegHandler *handler, uint8_t addr, uint8_t mask, uint8_t value)
{
	int ret = REG_HANDLER_NO_ERROR;
	uint8_t tmp, origRegVal = 0;
	uint16_t len = 0;

	if((handler->readReg == NULL) || (handler->writeReg == NULL))
	{
		return REG_HANDLER_SYSTEM_ERROR;
	}

	ret = handler->readReg(handler->ctx, addr, &origRegVal, len);
	if(ret != REG_HANDLER_NO_ERROR)
	{
		return ret;
	}

	tmp = origRegVal & (~mask);
	tmp |= value & mask;

	if(tmp != origRegVal)
	{
		ret = handler->writeReg(handler->ctx, addr, &tmp, 1);
		if(ret != REG_HANDLER_NO_ERROR)
		{
			return ret;
		}
	}

	return ret;
}

tRegHandlerErr readFifo(tRegHandler *handler, uint8_t *buffer, uint16_t length)
{
	if(handler->readFifo)
	{
		return handler->readFifo(handler->ctx, buffer, length);
	}

	return REG_HANDLER_SYSTEM_ERROR;
}
