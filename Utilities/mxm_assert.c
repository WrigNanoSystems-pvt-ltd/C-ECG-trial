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

#include <string.h>
#include <stdint.h>
#include "mxm_assert.h"
#include "usb_api.h"

#define MAX_FILE_NAME_LEN	64
#define MAX_MESSAGE_LEN		64

typedef struct{
	char filename[MAX_FILE_NAME_LEN];
	unsigned int line;
	char message[MAX_MESSAGE_LEN];
} error_message_t;

static error_message_t g_err_msg;

void app_error_handle_msg(char * filename, unsigned int filename_len, unsigned int line, char * msg, unsigned int msg_len)
{
	uint8_t * p_msg = (uint8_t *)&g_err_msg;
	memset(&g_err_msg, 0, sizeof(g_err_msg));

	if(filename_len > MAX_FILE_NAME_LEN)
		filename_len = MAX_FILE_NAME_LEN;

	if(msg_len > MAX_MESSAGE_LEN)
		msg_len = MAX_MESSAGE_LEN;

	memcpy(g_err_msg.filename, filename, filename_len);
	g_err_msg.line = line;
	memcpy(g_err_msg.message, msg, msg_len);

	usb_cdc_write(p_msg, sizeof(g_err_msg));

	while(1);
}

void app_error_handle(char * filename, unsigned int filename_len, unsigned int line)
{
	uint8_t * p_msg = (uint8_t *)&g_err_msg;

	memset(&g_err_msg, 0, sizeof(g_err_msg));

	if(filename_len > MAX_FILE_NAME_LEN)
		filename_len = MAX_FILE_NAME_LEN;

	memcpy(g_err_msg.filename, filename, filename_len);
	g_err_msg.line = line;

	usb_cdc_write(p_msg, sizeof(g_err_msg));

	while(1);
}
