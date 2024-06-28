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

#ifndef _PLATFORM_MAX86178_H
#define _PLATFORM_MAX86178_H

#include <stdint.h>
#include "device.h"

int max86178_driver_init(void);

int _platform_spi_init(dev_comm_t *port);
int _platform_spi_read_reg(dev_comm_t *port, uint8_t *buffer, int len);
int _platform_spi_write_reg(dev_comm_t *port, uint8_t reg_addr, uint8_t *buf);

int _platform_spi_burst_write(dev_comm_t *port, uint8_t reg_addr, uint8_t buf);
int _platform_spi_burst_read(dev_comm_t *port, uint8_t reg_addr, uint8_t buf);


int platform_spi_read_reg(dev_comm_t *port, uint8_t *buffer, int len);
int platform_spi_read_word(dev_comm_t *port, const uint8_t* reg_addr, uint8_t *buffer);
int platform_spi_write_reg(dev_comm_t *port, uint8_t reg_addr, uint8_t buf);

int max86178_spi_init(dev_comm_t *port);
int max86178_write_reg(dev_comm_t *port, uint8_t reg_addr, uint8_t reg_data);
int max86178_read_reg(dev_comm_t *port, uint8_t *buffer, int len);
int max86178_read_word_data(dev_comm_t *port, uint8_t reg_addr, uint8_t *buf);

int max86178_block_write(dev_comm_t *port, const struct regmap reg_block[], int size);
int max86178_block_read(dev_comm_t *comm, struct regmap reg_block[], int size);

int max86178_comm_test(dev_comm_t *port);
void max86178_init_board(void);

int spi_rw_test(void);
#endif//_PLATFORM_MAX86178_H
