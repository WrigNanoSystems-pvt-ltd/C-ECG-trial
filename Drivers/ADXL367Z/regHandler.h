/*
 * regHandler.h
 *
 *  Created on: Apr 8, 2022
 *      Author: Mert.Turker
 */

#ifndef DRIVERS_ADXL367Z_REG_HANDLER_H_
#define DRIVERS_ADXL367Z_REG_HANDLER_H_

#include <stdint.h>

typedef enum
{
	REG_HANDLER_NO_ERROR,
	REG_HANDLER_WRONG_PARAM,
	REG_HANDLER_CONNECTION_ERROR,
	REG_HANDLER_SYSTEM_ERROR

}tRegHandlerErr;


typedef tRegHandlerErr (*fPtrReadReg)(void *ctx, uint8_t addr, uint8_t *buffer, uint16_t length);
typedef tRegHandlerErr (*fPtrWriteReg)(void *ctx, uint8_t addr, uint8_t *buffer, uint16_t length);
typedef tRegHandlerErr (*fPtrReadFifo)(void *ctx, uint8_t *buffer, uint16_t length);

typedef struct{
	void *ctx;
	fPtrReadReg readReg;
	fPtrWriteReg writeReg;
	fPtrReadFifo readFifo;
}tRegHandler;


tRegHandlerErr readReg(tRegHandler *handler, uint8_t addr, uint8_t *buffer, uint16_t length);
tRegHandlerErr writeReg(tRegHandler *handler, uint8_t addr, uint8_t *buffer, uint16_t length);
tRegHandlerErr updateReg(tRegHandler *handler, uint8_t addr, uint8_t mask, uint8_t value);
tRegHandlerErr readFifo(tRegHandler *handler, uint8_t *buffer, uint16_t length);
#endif /* DRIVERS_ADXL367Z_REG_HANDLER_H_ */
