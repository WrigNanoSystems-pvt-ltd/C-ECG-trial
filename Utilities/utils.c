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

#include "utils.h"
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "rtc.h"

/*
 * @brief Parse DeviceStudio set_sample_rate cmd
 * @details format is "set_rate <type> <value>"
 *
 * @return 0 on success, -1 on failure
 */
int parse_set_rate_cmd(const char* str, const char* dev_type, uint32_t* val)
{
	const char* num_start = str + strlen("set_sample_rate") + strlen(dev_type) + 2;
	unsigned int rate;

	int num_found = sscanf(num_start, "%x", &rate);
	if (num_found == 1) {
		*val = rate;
		return 0;
	} else {
		return -1;
	}
}

/*
 * @brief Parse DeviceStudio get_reg command
 * @details format is "get_reg <type> <addr>"
 *
 * @return 0 on success, -1 on failure
 */
int parse_get_reg_cmd(const char* str, const char* dev_type, uint8_t* addr)
{
	const char* num_start = str + strlen("get_reg") + strlen(dev_type) + 2;
	unsigned int addr32;

	int num_found = sscanf(num_start, "%x", &addr32);
	if (num_found == 1) {
		*addr = (uint8_t)addr32;
		return 0;
	} else {
		return -1;
	}
}

/*
 * @brief Parse DeviceStudio set_reg command
 * @details format is "set_reg <type> <addr> <val>"
 *
 * @return 0 on success, -1 on failure
 */
int parse_set_reg_cmd(const char* str, const char* dev_type, uint8_t* addr, uint32_t* val)
{
	const char* num_start = str + strlen("set_reg") + strlen(dev_type) + 2;
	unsigned int addr32, val32;

	int num_found = sscanf(num_start, "%x %x", &addr32, &val32);
	if (num_found == 2) {
		*addr = (uint8_t)addr32;
		*val = val32;
		return 0;
	} else {
		return -1;
	}
}

int parse_cmd_data(const char* str, const char* cmd, uint32_t *vals, int vals_sz, bool hex)
{
	const char* sptr = str + strlen(cmd);
	int found = 0;
	int ssfound;

	while (found < vals_sz) {
		while (*sptr != ' ' && *sptr != '\0') { sptr++; }
		if (*sptr == '\0')
			break;
		sptr++;

		if (hex)
			ssfound = sscanf(sptr, "%x", vals + found);
		else
			ssfound = sscanf(sptr, "%d", vals + found);
		if (ssfound != 1)
			break;
		found++;
	}

	return found;
}

bool starts_with(const char* str1, const char* str2)
{
	if (str1 == NULL || str2 == NULL)
		return false;

	while (*str1 && *str2) {
		if (*str1 != *str2)
			return false;
		str1++;
		str2++;
	}

	if (*str2)
		return false;

	return true;
}

uint64_t utils_get_time_ms(void)
{
	uint32_t sec;
	uint32_t subsec;
	uint64_t ms;

	int ret = 1;

	while(ret){
		ret= RTC_GetTime(&sec,&subsec);
	}
	subsec = (uint32_t)((double) subsec / 4.096f);


    ms = ((uint64_t)sec*1000)+(uint64_t)subsec;


    return ms;
}

