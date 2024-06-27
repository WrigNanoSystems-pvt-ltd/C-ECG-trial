/**
 * @file  mx66.h
 * @brief BSP driver to communicate via SPI/QPI with an MX66 Serial Flash Memory.
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
 * $Date: 2018-11-08 15:08:58 -0600 (Thu, 08 Nov 2018) $
 * $Revision: 39056 $
 *
 **************************************************************************** */

/* Define to prevent redundant inclusion */
#ifndef _MX66_H_
#define _MX66_H_

/* **** Includes **** */
#include "mxc_config.h"
#include "mxc_sys.h"
#include "spi.h"
#include "spixfc.h"
#include "mxc_errors.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup bsp
 * @defgroup mx25_driver MX66 SPI Multi-I/O Flash Memory Driver
 * @{
 */
/* **** Definitions **** */
#define MX66_Read_DUMMY             8       /**< Dummy byte sent on a standard read command per the MX66 datasheet.         */
#define MX66_DREAD_DUMMY            4       /**< Dummy data sent on a fast-read (Dual) read command per the MX66 datasheet. */
#define MX66_QREAD_DUMMY            6       /**< Dummy data sent on a fast-read (Quad) read command per the MX66 datasheet. */

#define MX66_WIP_MASK               0x01        /**< Status Register                */
#define MX66_WEL_MASK               0x02        /**< Write Enable Latch mask        */
#define MX66_QE_MASK                0x40        /**< Quad-SPI enable mask           */
#define MX66_WP_MASK                0x80        /**< Write protect enable mask      */

/**
 * @ingroup mx25_driver
 * @defgroup MX66_Commands MX66 SPI Command Definitions
 * @{
 */
#define MX66_CMD_RST_EN             0x66        /**< Reset Enable                   */
#define MX66_CMD_RST_MEM            0x99        /**< Reset Memory                   */
#define MX66_CMD_ID                 0x9F        /**< ID                             */
#define MX66_CMD_WRITE_EN           0x06        /**< Write Enable                   */
#define MX66_CMD_WRITE_DIS          0x04        /**< Write Disable                  */
#define MX66_CMD_EN_4B_MODE         0xB7        /**< Enable 4 - Byte Address Mode   */
#define MX66_CMD_EX_4B_MODE         0xE9        /**< Disable 4 - Byte Address Mode   */

#define MX66_CMD_READ               0x0C        /**< Read                           */
#define MX66_CMD_DREAD              0xBC        /**< Dual SPI Read                  */
#define MX66_CMD_QREAD              0xEC        /**< Quad SPI Read                  */
#define MX66_CMD_HPM                0xA3        /**< Hardware Protection Mode       */

#define MX66_CMD_READ_SR            0x05        /**< Read Status Register           */
#define MX66_CMD_WRITE_SR           0x01        /**< Write Status Register          */
#define MX66_CMD_READ_CFG			0x15		/**< Read Configuration Register    */

#define MX66_CMD_PPROG              0x12    /**< Page Program            ****           */
#define MX66_CMD_QUAD_PROG          0X3E    /**< Quad (4 x I/O) Page Program        */

#define MX66_CMD_4K_ERASE           0x21        /**< Page Erase                     */
#define MX66_CMD_32K_ERASE          0x5C        /**< Sector Type 2 (32KB) Erase     */
#define MX66_CMD_64K_ERASE          0xDC        /**< Sector Type 3 (64KB) Erase     */
#define MX66_CMD_BULK_ERASE         0xC7        /**< Bulk Erase                     */
/**@} end of group mx25_commands */
/**
 * Enumeration type to select the size for an Erase command.
 */
typedef enum {
    MX66_Erase_4K,      /**< 4KB Sector Erase  */
    MX66_Erase_32K,     /**< 32KB Block Erase */
    MX66_Erase_64K,     /**< 64KB Block Erase */
}
MX66_Erase_t;

/* *** Globals **** */

/* **** Function Prototypes **** */

/**
 * @brief      Initialize SPI configuration and reset n25q
 * @return     #E_SUCCESS      No error in function.
 * @return     @ref MXC_Error_Codes "Error Code"  Non-zero error code if unsuccessful.
 */
int MX66_Init(void);

/**
 * @brief      Reset the MX66 flash memory.
 * @return     #E_SUCCESS     No error in function.
 * @return     @ref MXC_Error_Codes "Error Code"  Non-zero error code if unsuccessful.
 */
int MX66_Reset(void);

/**
 * @brief      Read manufacturer ID.
 * @return     ID of the device, or 0 if an error occurred
 */
uint32_t MX66_ID(void);

/**
 * @brief      Enable/Disable the Quad Enable(QE) bit in the status register.
 * @param         enable          @arg @b 1 enables Quad Mode. @arg @b 0 disables Quad Mode.
 * @return     #E_SUCCESS         No error in function.
 * @return     @ref MXC_Error_Codes "Error Code"  Non-zero error code if unsuccessful.
 */
int MX66_Quad(int enable);

/**
 * @brief Read data out by using 4-wire SPI mode.
 * @param          address         Start address to read from
 * @param          rx_buf          Pointer to the buffer of receiving data
 * @param          rx_len          Size of the data to read
 * @param          width           spi_width_t for how many data lines to use
 * @return         #E_SUCCESS      No error in function.
 * @return         @ref MXC_Error_Codes "Error Code"  Non-zero error code if unsuccessful.
 */
int MX66_Read(uint32_t address, uint8_t *rx_buf, uint32_t rx_len, spixfc_width_t width);

/**
 * @brief enable write protection
 * @param         enable   Write protect state
 * @return 	      #E_SUCCESS      No error in function.
 * @return        @ref MXC_Error_Codes "Error Code"  Non-zero error code if unsuccessful.
 */
int MX66_Write_Protect(int enable);

/**
 * @brief Program the memory to @p tx_buf and length @p tx_len, applies to both SPI and QPI modes.
 * @details
 *        - SPI mode: All operations are in 4-wire SPI mode.
 *        - QPI mode: All operations are in quad SPI mode.
 * @param         address         Start address to program.
 * @param         tx_buf          Pointer to the buffer of data to write.
 * @param         tx_len          Size of the data to write.
 * @param         width           spi_width_t for how many data lines to use.
 * @return        #E_SUCCESS      No error in function.
 * @return        @ref MXC_Error_Codes "Error Code"  Non-zero error code if unsuccessful.
 */
int MX66_Program_Page(uint32_t address, const uint8_t *tx_buf, uint32_t tx_len, spixfc_width_t width);

/**
 * @brief      Bulk erase the MX66 flash memory.
 * @warning    Bulk erase typically takes between 100 to 150 seconds.
 * @return     #E_SUCCESS      No error in function.
 * @return     @ref MXC_Error_Codes "Error Code"  Non-zero error code if unsuccessful.
 */
int MX66_Bulk_Erase(void);

/**
 * @brief      Erase memory segments
 * @param         address         Start address to begin erasing.
 * @param         size            Size to erase, see #MX66_Erase_t.
 * @return        #E_SUCCESS      No error in function.
 * @return        @ref MXC_Error_Codes "Error Code"  Non-zero error code if unsuccessful.
 */
int MX66_Erase(uint32_t address, MX66_Erase_t size);

/**
 * @brief      Read status register.
 * @param      buf   Pointer to store the value of the status register.
 */
int MX66_Read_SR(uint8_t* buf);

/**
 * @brief      Write status register
 * @param      value  Value to write to the status register.
 */
int MX66_Write_SR(uint8_t value);

/**
 * @brief      Read configuration register.
 * @param      uf   Pointer to store the value of the configuration register.
 */
int MX66_Read_CFG(uint8_t* buf);

int MX66_Read_Sec(uint8_t* buf);

int MX66_Read_Lock_Reg(uint16_t * buf);

/**@} end of group mx25_driver */
#ifdef __cplusplus
}
#endif

#endif /* _MX66_H_ */
