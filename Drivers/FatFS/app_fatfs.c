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

#include <stdint.h>
#include <string.h>

#include "mxc_errors.h"

#include "app_fatfs.h"

#include "ff.h"

static FATFS FatFs;
static FRESULT res;
static FIL fil;
static BYTE Buff[512];
static char file_name_with_type[20];

static int f_year, f_month, f_mday, f_hour, f_min, f_sec;

static int app_fatfs_delete_all_files();

int app_fatfs_init()
{
	/* Register work area */
	res = f_mount(&FatFs, "", 1);
	if(FR_OK != res|| FatFs.fs_type != FS_FAT32)
	{
		res = f_mkfs("", FM_FAT32, 0, Buff, sizeof Buff);
	}

	return res;
}

int app_fatfs_open_file(char * file_name, uint8_t file_name_len, int year, int month, int mday,int hour, int min, int sec)
{
	FRESULT res;

	app_fatfs_delete_all_files();

	memcpy(file_name_with_type, file_name, file_name_len);
	file_name_with_type[file_name_len++] = '.';
	file_name_with_type[file_name_len++] = 'b';
	file_name_with_type[file_name_len++] = 'i';
	file_name_with_type[file_name_len++] = 'n';

	f_year = year;
	f_month = month;
	f_mday= mday;
	f_hour = hour;
	f_min = min;
	f_sec = sec;

	res = f_open(&fil, file_name_with_type, FA_CREATE_ALWAYS | FA_WRITE);

	return res;
}

int app_fatfs_close_file()
{
	FRESULT res = FR_OK;
	FILINFO fno;

	res |= f_sync(&fil);
	res |= f_close(&fil);

	fno.fdate = (WORD)(((f_year - 1980) * 512U) | f_month * 32U | f_mday);
	fno.ftime = (WORD)(f_hour * 2048U | f_min * 32U | f_sec / 2U);
	res = f_utime(file_name_with_type, &fno);

	return res;
}

int app_fatfs_write_file(const uint8_t const * p_data, uint32_t len)
{
	FRESULT res = 0;
	uint32_t ind = 0;

	for(ind = 0; ind < len; ++ind)
	{
		res |= f_putc(p_data[ind], &fil);
	}


	return res;
}


static int app_fatfs_delete_all_files()
{
	FRESULT res;
	FILINFO fno;
	DIR dir;

	res = f_opendir(&dir, "");
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno);                   /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
			f_unlink(fno.fname);
		}

	}
	f_closedir(&dir);
	return res;
}
