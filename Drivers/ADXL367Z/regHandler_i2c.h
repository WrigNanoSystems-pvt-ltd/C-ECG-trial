/*
 * regHandler_i2c.h
 *
 *  Created on: Apr 15, 2022
 *      Author: Mert.Turker
 */
#include "regHandler.h"

#if defined(FTHR) || defined(FTHR2)
#define ADXL367_I2C_BUS 1
#elif MRD106
#define ADXL367_I2C_BUS 1
#elif MRD104
#define ADXL367_I2C_BUS 1
#endif


#ifndef DRIVERS_ADXL367Z_REGHANDLER_I2C_H_
#define DRIVERS_ADXL367Z_REGHANDLER_I2C_H_

tRegHandlerErr regHandler_i2c_init(tRegHandler *handler);

#endif /* DRIVERS_ADXL367Z_REGHANDLER_I2C_H_ */
