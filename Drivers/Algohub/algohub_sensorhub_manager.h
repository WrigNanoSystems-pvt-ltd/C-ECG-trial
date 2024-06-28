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

#ifndef DRIVERS_ALGOHUB_ALGOHUB_SENSORHUB_MANAGER_H_
#define DRIVERS_ALGOHUB_ALGOHUB_SENSORHUB_MANAGER_H_


#define ALGOHUB_DEFAULT_WHRM0_CH_SLOT 	(0u) //Measurement 1
#define ALGOHUB_DEFAULT_WHRM1_CH_SLOT 	(0u) //Measurement 1
#define ALGOHUB_DEFAULT_IR_CH_SLOT 		(0u) //Measurement 1
#define ALGOHUB_DEFAULT_RED_CH_SLOT 	(1u) //Measurement 2

#define ALGOHUB_DEFAULT_WHRM0_CH_PD 	(0u)
#define ALGOHUB_DEFAULT_WHRM1_CH_PD 	(1u)
#define ALGOHUB_DEFAULT_IR_CH_PD 		(0u)
#define ALGOHUB_DEFAULT_RED_CH_PD 		(0u)

typedef enum{
	AH_SH_SPI_OWNER_HOST = 0,
	AH_SH_SPI_OWNER_SENSORUHB,

	AH_SH_SPI_OWNER_MAX
} ah_sh_spi_ownwers_t;

int ah_sh_init();

int ah_sh_switch_to_algohub_mode();

int ah_sh_switch_to_sensorhub_mode(int algoMode);

int ah_sh_read_software_version(uint8_t version[3]);

int ah_sh_get_spi_ownership(ah_sh_spi_ownwers_t owner);

ah_sh_spi_ownwers_t ah_sh_spi_ownership_status(void);


#endif /* DRIVERS_ALGOHUB_ALGOHUB_SENSORHUB_MANAGER_H_ */
