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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>
#include "version.h"
#include "sh_utils.h"
#include "sh_comm_defs.h"

#include "Peripherals.h"

void print_array(uint8_t *buf, int len)
{
    int i;
    pr_info("Data: ");
    for(i = 0; i < len; ++i) {
    	pr_info("%.2X ", buf[i]);
    }

    pr_info("\n");
    return;
}

void mxm_sh_add_u8_to_report(int *idx, uint8_t *buf, uint8_t data)
{
	buf[(*idx)++] = data;
}

void mxm_sh_add_u16_to_report(int *idx, uint8_t *buf, uint16_t data)
{
	buf[(*idx)++] = (data >> 8) & 0xFF;
	buf[(*idx)++] = data & 0xFF;
}

void mxm_sh_add_u24_to_report(int *idx, uint8_t *buf, uint32_t data)
{
	buf[(*idx)++] = (data >> 16) & 0xFF;
	buf[(*idx)++] = (data >> 8) & 0xFF;
	buf[(*idx)++] = data & 0xFF;
}

void mxm_sh_add_u32_to_report(int *idx, uint8_t *buf, uint32_t data)
{
	buf[(*idx)++] = (data >> 24) & 0xFF;
	buf[(*idx)++] = (data >> 16) & 0xFF;
	buf[(*idx)++] = (data >> 8) & 0xFF;
	buf[(*idx)++] = data & 0xFF;
}

int32_t mxm_sh_combine_u8_to_int32(int *idx, uint8_t *buf)
{
    int32_t data;
    data   = ((int32_t)buf[(*idx)++]) << 24;
    data  |= ((int32_t)buf[(*idx)++]) << 16;
    data  |= ((int32_t)buf[(*idx)++]) << 8;
    data  |= (int32_t)buf[(*idx)++];
    return data;
}

void mxm_sh_u24_to_str(uint8_t *buf, char *strout, int size)
{
	uint32_t val = 0;
	val = (buf[0] << 16) | (buf[1] << 8) | buf[2];
	snprintf(strout, size, " %d", val);
}

void mxm_sh_print_report(uint8_t *report)
{
	char debug_str[200];
	char str[15];
	int num_var = 4;
	int i = 0;
	int offset = 0;
	int len = 0;

	/* Print first 4 variables as u24 */
	while (i < num_var) {
		mxm_sh_u24_to_str(report + 3 * i, str, sizeof(str));
		offset = snprintf(&debug_str[len], sizeof(debug_str) - len, "%s,", str);
		len += offset;
		i++;
	}
	debug_str[len] = 0;
	pr_info("%s\n", debug_str);
}

void mxm_sh_print_n_report(uint8_t *report, uint32_t num_report, uint32_t report_len)
{
	int report_idx = 0;

	while(report_idx < num_report) {
		mxm_sh_print_report(report + report_idx * report_len);
		report_idx++;
	}
}
