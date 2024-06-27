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

#include "app_flash.h"

#include <string.h>

#include "max32665.h"
#include "flc.h"
#include "icc.h"
#include "mxc_errors.h"



#define APP_FLASH_IDENTIFIER0			(0xCBu)
#define APP_FLASH_IDENTIFIER1			(0xBCu)

#define FLASH_INSTANCE_START_ADDR		(0x10080000)
#define FLASH_INSTANCE_PAGE_START_ADDR	(0x100FE000)
#define FLASH_INSTANCE_PAGE_SIZE		(0x2000)

#define APP_FLASH_HEADER_SIZE			(4)

typedef struct{
	uint8_t identifier0;
	uint8_t identifier1;
	uint8_t index;
	uint8_t len;
} app_flash_header_t;

static app_flash_header_t g_header = {APP_FLASH_IDENTIFIER0, APP_FLASH_IDENTIFIER1, 0u, 0u};

static uint8_t g_page[FLASH_INSTANCE_PAGE_SIZE];

int app_flash_init()
{
	int ret = 0;

	memset(g_page, 0, FLASH_INSTANCE_PAGE_SIZE);

	return ret;
}

int app_flash_write_data(app_flash_data_index_t flash_index, uint8_t const * const p_data, uint8_t len)
{
	int ret = 0;
	int index = 0;
	volatile uint8_t * p_page_addr = (uint8_t *)FLASH_INSTANCE_PAGE_START_ADDR;

	/* Read before write to prevent data loss */
	for(index = 0; index < FLASH_INSTANCE_PAGE_SIZE; index++){
		g_page[index] = *p_page_addr;
		p_page_addr++;
	}

	FLC_PageErase(FLASH_INSTANCE_PAGE_START_ADDR);

	g_header.index = flash_index;
	g_header.len = len;

	memcpy(&g_page[0], (uint8_t *)&g_header, APP_FLASH_HEADER_SIZE);
	memcpy(&g_page[APP_FLASH_HEADER_SIZE], p_data, len);

	while(MXC_FLC0->cn & MXC_F_FLC_CN_PEND);

	// Unlock flash
	MXC_FLC1->cn &= ~MXC_F_FLC_CN_UNLOCK;
	MXC_FLC1->cn |= MXC_S_FLC_CN_UNLOCK_UNLOCKED;

	// Write in 32-bit units until we are 128-bit aligned
	MXC_FLC1->cn &= 0xF7FFFFFF;
	MXC_FLC1->addr = FLASH_INSTANCE_PAGE_START_ADDR;


	ICC_Disable();

	FLC_Write(FLASH_INSTANCE_PAGE_START_ADDR, FLASH_INSTANCE_PAGE_SIZE, (uint32_t *)&g_page[0]);

	FLC_LockInfoBlock(FLASH_INSTANCE_PAGE_START_ADDR);

	ICC_Enable();

	return ret;
}

int app_flash_read_data(app_flash_data_index_t flash_index, uint8_t * p_data, uint8_t len)
{
	int index = 0;
	int is_found = 0;
	volatile uint8_t * p_page_addr = (uint8_t *)FLASH_INSTANCE_PAGE_START_ADDR;

	/* Read before write to prevent data loss */
	for(index = 0; index < FLASH_INSTANCE_PAGE_SIZE; index++){
		g_page[index] = *p_page_addr;
		p_page_addr++;
	}

	/* This is the header we are looking for*/
	g_header.index = index;
	g_header.len = len;


	for(index = 0; index < FLASH_INSTANCE_PAGE_SIZE - 1; index++){
		/* Valid header found */
		if(g_header.identifier0 == g_page[index] && g_header.identifier1 == g_page[index+1]){
			/* Found the data */
			if(g_page[index+2] == flash_index && g_page[index+3] == len){
				memcpy(p_data, &g_page[index + 4], len);
				is_found = 1;
			}

		}
	}

	return is_found == 1 ? 0 : E_INVALID;
}



