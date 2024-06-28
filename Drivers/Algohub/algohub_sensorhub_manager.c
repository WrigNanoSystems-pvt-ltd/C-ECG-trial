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

#include "mxc_delay.h"
#include "mxc_errors.h"

#include "Peripherals.h"

#include "algohub_api.h"
#include "algohub_config_api.h"

#include "sensorhub_api.h"
#include "sensorhub_config_api.h"

#include "spi_api.h"
#include "sensorhub.h"
#include "sh_comm.h"

#include "algohub_sensorhub_manager.h"

static uint8_t ah_sh_soft_ver[3];

static uint8_t regs_to_store[][2] = {{0x11, 0}, {0x12, 0}, {0x20, 0}, {0x21, 0}, {0x22, 0}, {0x23, 0}, \
									{0x24, 0}, {0x25, 0}, {0x26, 0}, {0x28, 0}, {0x29, 0}, {0x2A, 0}, \
									{0x2B, 0}, {0x2C, 0}, {0x2D, 0}, {0x2E, 0}, {0x30, 0}, {0x31, 0}, \
									{0x32, 0}, {0x33, 0}, {0x34, 0}, {0x35, 0}, {0x36, 0}, {0x38, 0}, \
									{0x39, 0}, {0x3A, 0}, {0x3B, 0}, {0x3C, 0}, {0x3D, 0}, {0x3E, 0}, \
									{0x40, 0}, {0x41, 0}, {0x42, 0}, {0x43, 0}, {0x44, 0}, {0x45, 0}, \
									{0x46, 0}, {0x48, 0}, {0x49, 0}, {0x4A, 0}, {0x4B, 0}, {0x4C, 0}, \
									{0x4D, 0}, {0x4E, 0}, {0x50, 0}, {0x51, 0}, {0x52, 0}, {0x53, 0}, \
									{0x54, 0}, {0x55, 0}, {0x56, 0}, {0x59, 0}, {0x5A, 0}, {0x5B, 0}, \
									{0x5C, 0}, {0x5D, 0}, {0x5E, 0}, {0x60, 0}, {0x61, 0}, {0x62, 0}, \
									{0x63, 0}, {0x64, 0}, {0x65, 0}, {0x66, 0}, {0x86, 0}, {0x90, 0}, \
									{0x91, 0}, {0x92, 0}};

/* sensorhub clears optical sensor registers and writes its initial settings.
 * Because of that, before switching to sensorhub mode, optical sensor registers must be restored.
 * When algohub/raw mode is selected back, optical sensor registers will be written. */
static int ah_sh_store_optical_sensor_settings();
static int ah_sh_apply_optical_sensor_settings();

/* By assuming sensorhub will have sensors at initial */
static ah_sh_spi_ownwers_t spi_line_owner = AH_SH_SPI_OWNER_SENSORUHB;

int ah_sh_init()
{
	int ret = 0;

	sh_init_hwcomm_interface();

	mxc_delay(MXC_DELAY_SEC(1));

	ret = sh_set_data_type(SS_DATATYPE_BOTH, false);

	if(0 == ret){
		ret = sh_set_fifo_thresh(1);
	}

	if(0 == ret){
		uint8_t ah_sh_ver_len;
		ret = sh_get_ss_fw_version(ah_sh_soft_ver, &ah_sh_ver_len);
	}


	if(0 == ret){
		uint8_t ah_sh_ver_len;
		ret = sh_set_report_period(25);
	}

	if(0 == ret){
		ret = ah_set_cfg_wearablesuite_whrmledpdconfig(
				((ALGOHUB_DEFAULT_WHRM0_CH_SLOT << 4) | ALGOHUB_DEFAULT_WHRM0_CH_PD) << 8
				| ((ALGOHUB_DEFAULT_WHRM1_CH_SLOT << 4) | ALGOHUB_DEFAULT_WHRM1_CH_PD));
	}

	if(0 == ret){
		ret = ah_set_cfg_wearablesuite_spo2ledpdconfig(
				((ALGOHUB_DEFAULT_IR_CH_SLOT << 4) | ALGOHUB_DEFAULT_IR_CH_PD) << 8
				| ((ALGOHUB_DEFAULT_RED_CH_SLOT << 4) | ALGOHUB_DEFAULT_RED_CH_PD));
	}

	return ret;
}

int ah_sh_read_software_version(uint8_t version[3])
{
	memcpy(version, ah_sh_soft_ver, sizeof(ah_sh_soft_ver));
	return 0;
}

int ah_sh_switch_to_algohub_mode()
{
	int ret = 0;

    if(0 == ret){
    	ret = sh_set_data_type(SS_DATATYPE_BOTH, false);
    	printf("sh_set_data_type %d \n", ret);
    }

	 if(0 == ret){
		 ret = algohub_enable();
		 printf("algohub_enable ret %d \n", ret);
	 }

	 return ret;
}

int ah_sh_switch_to_sensorhub_mode(int algoMode)
{
	int ret = 0;

	if(0 == ret){
		ret = sh_set_data_type(SS_DATATYPE_BOTH, true);
		pr_info("sh_set_data_type %d \n", ret);
	}

	if(0 == ret){
		ret = sensorhub_reset_sensor_configuration();
		pr_info("sensorhub_reset_sensor_configuration %d \n", ret);
	}

	if(0 == ret){
		ret = sensorhub_enable_sensors();
		pr_info("sensorhub_enable_sensors %d \n", ret);
	}

	if(0 == ret){
		if(0 == algoMode)
			ret = sensorhub_enable_algo(SENSORHUB_MODE_BASIC);
		else
			ret = sensorhub_enable_algo(SENSORHUB_MODE_EXTENDED);
		pr_info("sensorhub_enable_algo %d \n", ret);
	}

	return ret;
}


int ah_sh_get_spi_ownership(ah_sh_spi_ownwers_t owner)
{
	int ret = 0;

	if (owner == spi_line_owner) {
		return 0; // no need to continue
	}

	if (ret == 0) {

		sensor_t * sensor_os64;
		sensor_t * sensor_acc;

		spi_line_owner = owner;

		sensor_os64 = mxm_sh_get_sensor_instance(SH_MAX86178);
		sensor_acc = mxm_sh_get_sensor_instance(SH_ACC_SENSOR);

		if (owner == AH_SH_SPI_OWNER_HOST) {
			if( (sensor_os64 == NULL) /*|| (sensor_acc == NULL)*/ ) {
				return -1;
			}

			// release line
			//ret |= sh_spi_release();

			// init sensors
			if(0 == ret) {
				sensor_os64->init();
				sensor_acc->init();

				//ret |= ah_sh_apply_optical_sensor_settings();
			}
		}
		else { // to SensorHub

			ah_sh_store_optical_sensor_settings();

			sensor_os64->initialized = 0;
			sensor_acc->initialized = 0;

			ret |= ah_sh_store_optical_sensor_settings();

			ret = spi_api_disable();

			if(0 == ret){
				ret |= sh_spi_use();
			}
		}
	}


	return ret;
}

ah_sh_spi_ownwers_t ah_sh_spi_ownership_status(void)
{
	return spi_line_owner;
}


static int ah_sh_store_optical_sensor_settings()
{
	int ret = 0;
	sensor_t * sensor_os64;
	sensor_os64 = mxm_sh_get_sensor_instance(SH_MAX86178);

	for(int ind = 0; ind < ARRAY_SIZE(regs_to_store); ind++)
	{
		uint8_t reg_addr = regs_to_store[ind][0];
		ret |= sensor_os64->read_reg(&reg_addr, 1);
		regs_to_store[ind][1] = reg_addr;
	}

	return ret;
}

static int ah_sh_apply_optical_sensor_settings()
{
	int ret = 0;
	sensor_t * sensor_os64;
	sensor_os64 = mxm_sh_get_sensor_instance(SH_MAX86178);

	for(int ind = 0; ind < ARRAY_SIZE(regs_to_store); ind++)
	{
		ret |= sensor_os64->write_reg(&regs_to_store[ind][0], &regs_to_store[ind][1]);
	}

	return ret;
}
