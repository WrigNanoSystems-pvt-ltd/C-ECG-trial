/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * Description: Communications Device Class ACM (Serial Port) over USB
 * $Id: descriptors.h 31172 2017-10-05 19:05:57Z zach.metzinger $
 *
 *******************************************************************************
 */
 
/**
 * @file    mscmem.h
 * @brief   Memory routines used by the USB Mass Storage Class example.  
 *          See the msc_mem_t structure in msc.h for function details.
 */
 
#ifndef __MSC_MEM_H__
#define __MSC_MEM_H__

#include <stdint.h>

#define ERASE_MEMORY_ON_INIT        1   /* Configuration option to clear the memory (to 0s) on initialization. */
                                        /* Use 1 to clear or 0 to leave untouched. */

#define MX66_SECTOR_SIZE            4096        /* Number of bytes in one sector of the MX66 */
#define MX66_SECTOR_SIZE_SHIFT      12          /* The shift value used to convert between addresses and block numbers */
#define MX66_NUM_SECTORS            65536        /* Total number of sectors in the MX66 */
#define MX66_BLOCK_SIZE				65536

#define LBA_PER_SECTOR              (MX66_SECTOR_SIZE >> LBA_SIZE_SHIFT)
#define INVALID_SECTOR              MX66_NUM_SECTORS    /* Use a sector number past the end of memory to indicate invalid */

int mscmem_init(void);
int mscmem_start(void);
int mscmem_stop(void);
uint32_t mscmem_size(void);
int mscmem_read(uint32_t lba, uint8_t* buffer);
int mscmem_write(uint32_t lba, uint8_t* buffer);
int mscmem_ready(void);
int mscmem_write_dirty_sector();

#endif  /* __MSC_MEM_H__ */
