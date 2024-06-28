/**
 * @file    mx66.c
 * @brief   Board layer Driver for the Micron MX66 Serial Multi-I/O Flash Memory.
 */
/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
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
 *
 * $Date: 2019-08-02 10:50:10 -0500 (Fri, 02 Aug 2019) $
 * $Revision: 45194 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include <string.h>
#include "mxc_config.h"
#include "mxc_errors.h"
#include "mxc_delay.h"
#include "mx66.h"
#include "spi.h"
#include "board.h"
#include "spixfc.h"


/**
 * @ingroup mx25
 * @{
 */

/* **** Definitions **** */

/* **** Globals **** */

/***** Globals *****/

/* **** Static Functions **** */

/* ************************************************************************* */
static int flash_busy()
{
    uint8_t buf;

    MX66_Read_SR(&buf);

    if(buf & MX66_WIP_MASK) {
        return E_BUSY;
    } else
        return E_NO_ERROR;
}

/* ************************************************************************* */
static int write_enable()
{
    uint8_t cmd = MX66_CMD_WRITE_EN;
    uint8_t buf;

    // Send the command
    if(MX66_Board_Write(&cmd, 1, 1, SPIXFC_WIDTH_1) != 1) {
        return E_COMM_ERR;
    }

    MX66_Read_SR(&buf);

    if(buf & MX66_WEL_MASK) {
        return E_NO_ERROR;
    }

    return E_BAD_STATE;
}

/* ************************************************************************* */
static inline int read_reg(uint8_t cmd, uint8_t* buf)
{
    // Send the command
    if(MX66_Board_Write(&cmd, 1, 0, SPIXFC_WIDTH_1) != 1) {
        return E_COMM_ERR;
    }
    // Read the data
    if(MX66_Board_Read(buf, 1, 1, SPIXFC_WIDTH_1) != 1) {
        return E_COMM_ERR;
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
static inline int write_reg(uint8_t* buf, unsigned len)
{
    if(write_enable() != 0){
        return E_BAD_STATE;
    }

    // Send the command and data
    if(MX66_Board_Write(buf, len, 1, SPIXFC_WIDTH_1) != (int)len) {
        return E_COMM_ERR;
    }

    return E_NO_ERROR;
}

/* **** Functions **** */

/* ************************************************************************* */

int MX66_Init(void)
{
	int ret = E_NO_ERROR;
	int cmd = MX66_CMD_EN_4B_MODE;
	ret = MX66_Board_Init();
	if(ret != E_NO_ERROR)
	{
		return E_COMM_ERR;
	}

    if(MX66_Board_Write(&cmd, 1, 1, SPIXFC_WIDTH_1) != 1) {
        return E_COMM_ERR;
    }

    return E_NO_ERROR;
}


/* ************************************************************************* */
int MX66_Reset(void)
{
    int busy_count = 0;

    // Send the Reset command
    uint8_t cmd;
    cmd = MX66_CMD_RST_EN;
    if(MX66_Board_Write(&cmd, 1, 1, SPIXFC_WIDTH_1) != 1) {
        return E_COMM_ERR;
    }

    cmd = MX66_CMD_RST_MEM;
    if(MX66_Board_Write(&cmd, 1, 1, SPIXFC_WIDTH_1) != 1) {
        return E_COMM_ERR;
    }

    while(flash_busy()) {
        busy_count++;
        if(busy_count > 10000) {
            return E_TIME_OUT;
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
uint32_t MX66_ID(void)
{
    uint8_t cmd = MX66_CMD_ID;
    uint8_t id[3];

    // Send the command
    if(MX66_Board_Write(&cmd, 1, 0, SPIXFC_WIDTH_1) != 1) {
        return 0;
    }

    // Read the data
    if(MX66_Board_Read(id, 3, 1, SPIXFC_WIDTH_1) != 3) {
        return 0;
    }

    return ((uint32_t)(id[2] | (id[1] << 8) | (id[0] << 16)));

}

/* ************************************************************************* */
int MX66_Quad(int enable)
{

    // Enable QSPI mode
    uint8_t pre_buf;
    uint8_t post_buf;
    int err;

    MX66_Read_SR(&pre_buf);

    if(enable) {
        pre_buf |= MX66_QE_MASK;
    } else {
        pre_buf &= ~MX66_QE_MASK;
    }

    if((err = write_enable()) != 0) {
        return E_BAD_STATE;
    }

    if(MX66_Write_SR(pre_buf) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    while(flash_busy()) {}

    if(MX66_Read_SR(&post_buf) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    if(enable) {
        if(!(post_buf & MX66_QE_MASK)) {
            return E_UNKNOWN;
        }
    } else {
        if(post_buf & MX66_QE_MASK) {
            return E_UNKNOWN;
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX66_Write_Protect(int enable)
{
    uint8_t pre_buf;
    uint8_t post_buf;

    MX66_Read_SR(&pre_buf);

    if(enable) {
        pre_buf |= MX66_WP_MASK;
    } else {
        pre_buf &= ~MX66_WP_MASK;
    }

    if(write_enable() != 0)
        return E_BAD_STATE;

    MX66_Write_SR(pre_buf);

    while(flash_busy()) {}

    MX66_Read_SR(&post_buf);

    if(enable) {
        if(!(post_buf & MX66_WP_MASK)) {
            return E_UNKNOWN;
        }
    } else {
        if(post_buf & MX66_WP_MASK) {
            return E_UNKNOWN;
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX66_Read(uint32_t address, uint8_t *rx_buf, uint32_t rx_len,
               spixfc_width_t width)
{
   uint8_t cmd[4];
   uint8_t command;

    if(flash_busy()) {
        return E_BUSY;
    }

    cmd[0] = (address >> 24) & 0xFF;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >>  8) & 0xFF;
    cmd[3] = address & 0xFF;

    // Send the command and dummy bits
    if(width == SPIXFC_WIDTH_1) {
        command = MX66_CMD_READ;

        if(MX66_Board_Write(&command,1,0, SPIXFC_WIDTH_1) != 1) {
            return E_COMM_ERR;
        }

		// Send the address
        if(MX66_Board_Write(&cmd[0],4,0, width) != 4) {
            return E_COMM_ERR;
        }

        // Send dummy bits
		MX66_Clock(MX66_Read_DUMMY, 0);

    } else if(width == SPIXFC_WIDTH_2) {
        command = MX66_CMD_DREAD;

        if(MX66_Board_Write(&command,1,0, SPIXFC_WIDTH_1)   != 1) {
            return E_COMM_ERR;
        }

		// Send the address
        if(MX66_Board_Write(&cmd[0],4,0, width)   != 4) {
            return E_COMM_ERR;
        }

        // Send dummy bits
		MX66_Clock(MX66_DREAD_DUMMY, 0);

    } else {
    	command = MX66_CMD_QREAD;

        if(MX66_Board_Write(&command,1,0, SPIXFC_WIDTH_1) != 1) {
            return E_COMM_ERR;
        }

		// Send the address
        if(MX66_Board_Write(&cmd[0],4,0, width) != 4) {
            return E_COMM_ERR;
        }

        // Send dummy bits
		MX66_Clock(MX66_QREAD_DUMMY, 0);
        //MX66_Clock(10, 0);
    }

    // Receive the data
    if(MX66_Board_Read(rx_buf, rx_len, 1, width) != (int)rx_len) {
        return E_COMM_ERR;
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX66_Program_Page(uint32_t address, const uint8_t *tx_buf, uint32_t tx_len,
                       spixfc_width_t width)
{

    int tx_cnt = 0;
    int timeout = 0;
    uint8_t cmd[4];
    uint8_t command;
	unsigned len;

    if(flash_busy()) {
      return E_BUSY;
    }

    while(tx_len > 0) {
        while(write_enable()) {
            timeout++;
            if(timeout > 100) {
                return E_TIME_OUT;
            }
        }

        cmd[0] = (address >> 24) & 0xFF;
        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >>  8) & 0xFF;
        cmd[3] = address & 0xFF;



        // Send the command and dummy bits
        if(width != SPIXFC_WIDTH_4) {
        	command = MX66_CMD_PPROG;

            if(MX66_Board_Write(&command,1,0, SPIXFC_WIDTH_1) != 1) {
                return E_COMM_ERR;
            }

	        // Send the address
			if(MX66_Board_Write(&cmd[0],4,0,SPIXFC_WIDTH_1) != 4) {
                return E_COMM_ERR;
            }
        } else {
        	command = MX66_CMD_QUAD_PROG;

		    if(MX66_Board_Write(&command,1,0,SPIXFC_WIDTH_1) != 1) {
                return E_COMM_ERR;
            }

            // Send the address
			if(MX66_Board_Write(&cmd[0],4,0,width) != 4) {
                return E_COMM_ERR;
            }
        }


        // Send the data
        if(tx_len >= 256) {
            len     = 256;
        } else {
            len     = tx_len;
        }

        if(MX66_Board_Write((&tx_buf[tx_cnt*256]), len, 1, width) != (int)len) {
            return E_COMM_ERR;
        }

        if(tx_len >= 256) {
            tx_len -= 256;
        } else {
            tx_len = 0;
        }
        address += 256;
        tx_cnt++;

        timeout = 0;
        while(flash_busy()) {
            timeout++;
            if(timeout > 10000) {
                return E_TIME_OUT;
            }
        }
    }
    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX66_Bulk_Erase(void)
{
    uint8_t cmd;
    int timeout = 0;

    if(flash_busy()) {
        return E_BUSY;
    }

    if(write_enable()){
        return E_BAD_STATE;
    }

    cmd = MX66_CMD_BULK_ERASE;

    // Send the command
    if(MX66_Board_Write(&cmd, 1, 1, SPIXFC_WIDTH_1) != 1) {
        return E_COMM_ERR;
    }

    while(flash_busy()) {
    	mxc_delay(MXC_DELAY_SEC(1));
        timeout++;
        if(timeout > 300) {
            return E_TIME_OUT;
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX66_Erase(uint32_t address, MX66_Erase_t size)
{
    uint8_t cmd[4];
    uint8_t command;
    uint8_t delay_ms = 0;
    int timeout = 0;

    if(flash_busy()) {
        return E_BUSY;
    }

    if(write_enable()){
        return E_BAD_STATE;
    }


    switch(size) {
        case MX66_Erase_4K:
        default:
        	command = MX66_CMD_4K_ERASE;
        	delay_ms = 25;
            break;
        case MX66_Erase_32K:
        	command = MX66_CMD_32K_ERASE;
        	delay_ms = 150;
            break;
        case MX66_Erase_64K:
        	command = MX66_CMD_64K_ERASE;
        	delay_ms = 220;
            break;
    }

    cmd[0] = (address >> 24) & 0xFF;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >>  8) & 0xFF;
    cmd[3] = address & 0xFF;

    // Send the command and the address
    if(MX66_Board_Write(&command, 1, 0, SPIXFC_WIDTH_1) != 1) {
        return E_COMM_ERR;
    }

    // Send the command and the address
    if(MX66_Board_Write(&cmd[0], 4, 1, SPIXFC_WIDTH_1) != 4) {
        return E_COMM_ERR;
    }

    while(flash_busy()) {
        timeout++;
        mxc_delay(MXC_DELAY_MSEC(delay_ms));
        if(timeout > 20) {
            return E_TIME_OUT;
        }
    }
    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX66_Read_SR(uint8_t* buf)
{
    uint8_t cmd = MX66_CMD_READ_SR;

    return read_reg(cmd, buf);
}

/* ************************************************************************* */
int MX66_Write_SR(uint8_t value)
{
    uint8_t cmd[2] = {MX66_CMD_WRITE_SR, value};

    return write_reg(cmd, 2);
}

/* ************************************************************************* */
int MX66_Read_CFG(uint8_t* buf)
{
    uint8_t cmd = MX66_CMD_READ_CFG;

    return read_reg(cmd, buf);
}

int MX66_Read_Sec(uint8_t* buf)
{
    uint8_t cmd = 0x2B;

    return read_reg(cmd, buf);
}

int MX66_Read_Lock_Reg(uint16_t * buf)
{
	uint8_t cmd = 0x2D;
	uint8_t data[2];

    // Send the command
    if(MX66_Board_Write(&cmd, 1, 0, SPIXFC_WIDTH_1) != 1) {
        return E_COMM_ERR;
    }
    // Read the data
    if(MX66_Board_Read(data, 2, 1, SPIXFC_WIDTH_1) != 2) {
        return E_COMM_ERR;
    }

    *buf = ((uint16_t)data[1] << 8) | ((uint16_t)data[0]);

    return 0;
}


/**@} end of ingroup mx25 */
