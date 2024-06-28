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



#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "Peripherals.h"

#include "sh_comm.h"

#include "gpio.h"
#include "i2c_ah_sh_api.h"
#include "Peripherals.h"


#define SS_I2C_8BIT_SLAVE_ADDR      0xAA
#define SENSORHUB_I2C_ADRESS        SS_I2C_8BIT_SLAVE_ADDR

#define ENABLED   ((int)(1))
#define DISABLED  ((int)(0))

#define SS_WAIT_BETWEEN_TRIES_MS    (2)
#define SS_CMD_WAIT_PULLTRANS_MS    (5)
#define SS_FEEDFIFO_CMD_SLEEP_MS	(30)


#define SS_DEFAULT_RETRIES       ((int) (4))
#define SS_ZERO_DELAY               0
#define SS_ZERO_BYTES               0

#define PIN_INPUT					0
#define PIN_OUTPUT					1



#define BOOTLOADER_MAX_PAGE_SIZE 8192

/* BOOTLOADER HOST */
#define EBL_CMD_TRIGGER_MODE	0
#define EBL_GPIO_TRIGGER_MODE	1

const int sh_wakeup_wait_duration_us = 1250;

#ifdef FTHR
#define SH_RST_PORT PORT_0
#define SH_RST_PIN PIN_21
#define SH_MFIO_PORT PORT_0
#define SH_MFIO_PIN PIN_9
#elif FTHR2
#define SH_RST_PORT PORT_0
#define SH_RST_PIN PIN_21
#define SH_MFIO_PORT PORT_1
#define SH_MFIO_PIN PIN_13
#elif MRD104
#define SH_RST_PORT PORT_0
#define SH_RST_PIN PIN_30
#define SH_MFIO_PORT PORT_0
#define SH_MFIO_PIN PIN_31
#elif MRD106
#define SH_RST_PORT PORT_0
#define SH_RST_PIN PIN_30
#define SH_MFIO_PORT PORT_0
#define SH_MFIO_PIN PIN_31
#endif

/*
 * desc:
 *   platfrom specific function to init sensor comm interface and get data format.
 *
 * */
void sh_init_hubinterface(void){

	sh_init_hwcomm_interface();
    return;
}

/*
 * SSI API funcions
 * NOTE: Generic functions for any platform.
 *       exceptions: below needs needs modification according to platform and HAL drivers
 *       1. Hard reset function
 *       2. Enable/disable mfio event interrput
 *       3. mfio pin interrupt routine
 *
 * **/

/*global buffer for sensor i2c commands+data*/
static uint8_t sh_write_buf[512];

/* Mode to control sesnor hub resets. ie via GPIO based hard reset or Command based soft reset*/
static uint8_t ebl_mode = EBL_GPIO_TRIGGER_MODE;

static uint64_t actual_time = 0;
static uint64_t latest_read_request_time = 0;
static uint64_t latest_write_request_time = 0;
static uint64_t read_request_delay = 0;
static uint64_t write_request_delay = 0;
static uint8_t async_read_required = 0;
static uint8_t async_write_required = 0;

/* desc  :
 *         Func to init master i2c hardware comm interface with sennor hub
 *                 init mfio interrupt pin and attach irq to pin
 *                 init reset pin
 * params:
 *         N/A
 */
void sh_init_hwcomm_interface(){
	int ret = 0;

	ret = i2c_init();
	if(0 != ret)
		pr_info("ERR: I2C_INIT %d \n", ret);


#if defined(SYSTEM_USES_RST_PIN)
	reset_pin.port = SH_RST_PORT;
	reset_pin.mask = SH_RST_PIN;
	reset_pin.func = GPIO_FUNC_IN;
	reset_pin.pad = GPIO_PAD_PULL_UP;
	GPIO_Config(&reset_pin);
#endif

	mfio_pin.port = SH_MFIO_PORT;
	mfio_pin.mask = SH_MFIO_PIN;
	mfio_pin.func = GPIO_FUNC_IN;
	mfio_pin.pad = GPIO_PAD_NONE;
	GPIO_Config(&mfio_pin);

    return;
}


#if defined(SYSTEM_USES_RST_PIN)
/*
 * desc:
 *    function to reset sensor hub and put to application mode after reset  interface and get data format.
 *
 * params:
 *
 *    __I wakeupMode : 0x00 : application mode
 *                     0x08 : bootloader mode
 * */
int sh_hard_reset(int wakeupMode)
{
   reset_pin.port = SH_RST_PORT;
   reset_pin.mask = SH_RST_PIN;
   reset_pin.func = GPIO_FUNC_OUT;
   reset_pin.pad = GPIO_PAD_NONE;
   GPIO_Config(&reset_pin);

   mfio_pin.port = SH_MFIO_PORT;
   mfio_pin.mask = SH_MFIO_PIN;
   mfio_pin.func = GPIO_FUNC_OUT;
   mfio_pin.pad = GPIO_PAD_NONE;
   GPIO_Config(&mfio_pin);

   GPIO_OutClr(&reset_pin);

   WAIT_MS(SS_RESET_TIME);

   if( (wakeupMode & 0xFF) == 0 ) {

	   GPIO_OutSet(&mfio_pin);
	   GPIO_OutSet(&reset_pin);

	   WAIT_MS(SS_STARTUP_TO_MAIN_APP_TIME);
   }else {
	   GPIO_OutClr(&mfio_pin);
	   GPIO_OutSet(&reset_pin);

	   WAIT_MS(SS_STARTUP_TO_BTLDR_TIME);
   }

   	mfio_pin.port = SH_MFIO_PORT;
   	mfio_pin.mask = SH_MFIO_PIN;
   	mfio_pin.func = GPIO_FUNC_IN;
   	mfio_pin.pad = GPIO_PAD_PULL_UP;
	GPIO_Config(&mfio_pin);

	reset_pin.port = SH_RST_PORT;
   	reset_pin.mask = SH_RST_PIN;
   	reset_pin.func = GPIO_FUNC_IN;
   	reset_pin.pad = GPIO_PAD_NONE ;
	GPIO_Config(&reset_pin);

	return 0;
}
#endif

#define COMPILER_INLINED
static void LPM_pull_mfio_to_low_and_keep(int waitDurationInUs)
{
   	mfio_pin.port = SH_MFIO_PORT;
   	mfio_pin.mask = SH_MFIO_PIN;
   	mfio_pin.func = GPIO_FUNC_OUT;
   	mfio_pin.pad = GPIO_PAD_PULL_UP; //GPIO_PAD_NONE;
	GPIO_Config(&mfio_pin);

	GPIO_OutClr(&mfio_pin);

	wait_us(waitDurationInUs);

}
#define COMPILER_INLINED
static void LPM_pull_mfio_to_high ( void )
{
   	mfio_pin.port = SH_MFIO_PORT;
   	mfio_pin.mask = SH_MFIO_PIN;
   	mfio_pin.func = GPIO_FUNC_OUT;
   	mfio_pin.pad = GPIO_PAD_PULL_UP ; //GPIO_PAD_NONE;
	GPIO_Config(&mfio_pin);

	GPIO_OutSet(&mfio_pin);
}

static void LPM_set_mfio_as_input ( void )
{
   	mfio_pin.port = SH_MFIO_PORT;
   	mfio_pin.mask = SH_MFIO_PIN;
   	mfio_pin.func = GPIO_FUNC_IN;
   	mfio_pin.pad = GPIO_PAD_NONE;
	GPIO_Config(&mfio_pin);
}

int sh_set_ebl_mode(const uint8_t mode)
{
	int status;
	if (mode == EBL_CMD_TRIGGER_MODE || mode == EBL_GPIO_TRIGGER_MODE) {
		ebl_mode = mode;
		status =  SS_SUCCESS;
	} else
		status = SS_ERR_INPUT_VALUE;

	return status;
}

const int sh_get_ebl_mode(void)
{
   return ebl_mode;
}

int sh_reset_to_bootloader(void){

	int status;
	uint8_t hubMode;
#if defined(SYSTEM_USES_RST_PIN)
     if(ebl_mode == EBL_GPIO_TRIGGER_MODE)
    	 sh_hard_reset(0x08);
     if(ebl_mode == EBL_CMD_TRIGGER_MODE)
#endif
     {
    	 status = sh_set_sensorhub_operating_mode(0x08);
     }
     status = sh_get_sensorhub_operating_mode(&hubMode);
     if( status != 0x00 /*SS_SUCCESS*/ || hubMode != 0x08 ){
    	 status = -1;
     }

     return status;

}

int in_bootldr_mode(void)
{
	uint8_t cmd_bytes[] = { 0x02, 0x00 };
	uint8_t rxbuf[2]    = { 0 };

	sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
			0, 0,
			&rxbuf[0], sizeof(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

	if ((rxbuf[0] != SS_SUCCESS) && (rxbuf[0] != SS_BTLDR_SUCCESS))
		return -1;

	return (rxbuf[1] & SS_MASK_MODE_BOOTLDR);
}

int exit_from_bootloader(void)
{
	int status;
	uint8_t cmd_bytes[] = { 0x01, 0x00 };
	uint8_t data[]      = { 0x00 };

	status = sh_write_cmd_with_data( &cmd_bytes[0], sizeof(cmd_bytes),
										 &data[0], 1 /*sizeof(data)*/,
										 10*SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}

int stay_in_bootloader()
{

	uint8_t cmd_bytes[] = { 0x01, 0x00 };
	uint8_t data[]      = { SS_MASK_MODE_BOOTLDR };

	int status = sh_write_cmd_with_data(
			&cmd_bytes[0], sizeof(cmd_bytes),
			&data[0], sizeof(data), SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}

#if defined(SYSTEM_USES_MFIO_PIN)
static void cfg_mfio(int dir)
{
	if (dir == PIN_INPUT) {

	   	mfio_pin.port = SH_MFIO_PORT;
	   	mfio_pin.mask = SH_MFIO_PIN;
	   	mfio_pin.func = GPIO_FUNC_IN;
	   	mfio_pin.pad = GPIO_PAD_PULL_UP;
		GPIO_Config(&mfio_pin);
	} else {

	   	mfio_pin.port = SH_MFIO_PORT;
	   	mfio_pin.mask = SH_MFIO_PIN;
	   	mfio_pin.func = GPIO_FUNC_OUT;
	   	mfio_pin.pad = GPIO_PAD_NONE;
		GPIO_Config(&mfio_pin);
	}
}
#endif


#if defined(SYSTEM_USES_RST_PIN)
int sh_debug_reset_to_bootloader(void)
{

	int status = -1;

	if (ebl_mode == EBL_GPIO_TRIGGER_MODE) {

		reset_pin.port = SH_RST_PORT;
	   	reset_pin.mask = SH_RST_PIN;
	   	reset_pin.func = GPIO_FUNC_OUT;
	   	reset_pin.pad = GPIO_PAD_NONE;
		GPIO_Config(&reset_pin);
#if defined(SYSTEM_USES_MFIO_PIN)
		cfg_mfio(PIN_OUTPUT);
#endif
		GPIO_OutClr(&reset_pin);
		WAIT_MS(SS_RESET_TIME);
#if defined(SYSTEM_USES_MFIO_PIN)
		GPIO_OutClr(&mfio_pin);
#endif
		GPIO_OutSet(&reset_pin);

		WAIT_MS(SS_STARTUP_TO_BTLDR_TIME);
#if defined(SYSTEM_USES_MFIO_PIN)
		cfg_mfio(PIN_INPUT);
#endif
		reset_pin.port = SH_RST_PORT;
	   	reset_pin.mask = SH_RST_PIN;
	   	reset_pin.func = GPIO_FUNC_IN;
	   	reset_pin.pad = GPIO_PAD_NONE;
		GPIO_Config(&reset_pin);

		stay_in_bootloader();

		if (in_bootldr_mode() > 0)
			status = SS_SUCCESS;
	} else {
		stay_in_bootloader();

		status = SS_SUCCESS;
	}

    return status;
}


int sh_reset_to_main_app(void)
{
	int status = -1;

	if (ebl_mode == EBL_GPIO_TRIGGER_MODE) {

		reset_pin.port = SH_RST_PORT;
	   	reset_pin.mask = SH_RST_PIN;
	   	reset_pin.func = GPIO_FUNC_OUT;
	   	reset_pin.pad = GPIO_PAD_NONE;
		GPIO_Config(&reset_pin);

#if defined(SYSTEM_USES_MFIO_PIN)
		cfg_mfio(PIN_OUTPUT);

		GPIO_OutClr(&mfio_pin);
#endif

		WAIT_MS(SS_RESET_TIME);

		GPIO_OutClr(&reset_pin);

		WAIT_MS(SS_RESET_TIME);
#if defined(SYSTEM_USES_MFIO_PIN)
		GPIO_OutSet(&mfio_pin);
#endif

		WAIT_MS(SS_RESET_TIME);
		GPIO_OutSet(&reset_pin);
		WAIT_MS((2*SS_STARTUP_TO_MAIN_APP_TIME));

#if defined(SYSTEM_USES_MFIO_PIN)
		cfg_mfio(PIN_INPUT);
#endif
		reset_pin.port = SH_RST_PORT;
	   	reset_pin.mask = SH_RST_PIN;
	   	reset_pin.func = GPIO_FUNC_IN;
	   	reset_pin.pad = GPIO_PAD_NONE;
		GPIO_Config(&reset_pin);

		// Verify we exited bootloader mode
		if (in_bootldr_mode() == 0)
			status = SS_SUCCESS;
		else
			status = SS_ERR_UNKNOWN;
	} else {
		status = exit_from_bootloader();
		if (status == SS_BTLDR_SUCCESS)
			status = SS_SUCCESS;
	}

	return status;
}

#endif



/*
 *
 *   SENSOR HUB COMMUNICATION INTERFACE ( Defined in MAX32664 User Guide ) API FUNCTIONS
 *
 *
 * */

int sh_self_test(int idx, uint8_t *result, int sleep_ms){

	uint8_t cmd_bytes[] = { 0x70, (uint8_t)idx };
    uint8_t rxbuf[2];
    result[0] = 0xFF;

    int status = sh_read_cmd(&cmd_bytes[0],sizeof(cmd_bytes) ,
                             0, 0,
						     &rxbuf[0], sizeof(rxbuf),
						     sleep_ms  );

	if (status != SS_SUCCESS)
		return SS_ERR_TRY_AGAIN;

    result[0] = rxbuf[1];
	return status;
}

const char* sh_get_hub_fw_version(void)
{
    uint8_t cmd_bytes[2];
    uint8_t rxbuf[4];

    static char fw_version[32] = "SENSORHUB";

	int bootldr = sh_checkif_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_FWVERSION;
	} else {

		return &fw_version[0];
	}

    int status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
             	 	 	 	  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  SS_DEFAULT_CMD_SLEEP_MS );

    if (status == SS_SUCCESS) {
        snprintf(fw_version, sizeof(fw_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);
	}

    return &fw_version[0];
}


const char* sh_get_hub_algo_version(void)
{
    uint8_t cmd_bytes[3];
    uint8_t rxbuf[4];

    static char algo_version[64] = "SENSORHUBALGORITHMS";

	int bootldr = sh_checkif_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
		cmd_bytes[2] = 0;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_ALGOVER;
		cmd_bytes[2] = SS_CMDIDX_AVAILSENSORS;
	} else {

		return &algo_version[0];
	}

    int status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
                              0, 0,
                              &rxbuf[0], sizeof(rxbuf),
						      SS_DEFAULT_CMD_SLEEP_MS   );

    if (status == SS_SUCCESS) {
        snprintf(algo_version, sizeof(algo_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);

    }

    return &algo_version[0];
}

int sh_send_raw(uint8_t *rawdata, int rawdata_sz)
{
	return sh_write_cmd(&rawdata[0], rawdata_sz, 5 * SS_ENABLE_SENSOR_SLEEP_MS);
}

int sh_get_log_len(int *log_len)
{

	uint8_t cmd_bytes[] = { 0x90, 0x01 };
	uint8_t rxbuf[2]    = {0};
    int logLen = 0;

	int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
								   0, 0,
								   &rxbuf[0], sizeof(rxbuf),
								   SS_DEFAULT_CMD_SLEEP_MS   );

	if (status == SS_SUCCESS) {
		logLen = (rxbuf[1] << 8) | rxbuf[0];
	}
	*log_len = logLen;

	return status;
}

int sh_read_ss_log(int num_bytes, uint8_t *log_buf, int log_buf_sz)
{
	int bytes_to_read = num_bytes + 1; //+1 for status byte

	uint8_t cmd_bytes[] = { 0x90, 0x00 };
    int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
						     0, 0,
							 log_buf, bytes_to_read,
							 SS_CMD_WAIT_PULLTRANS_MS  );

	return status;
}


int sh_write_cmd( uint8_t *tx_buf,
		          int tx_len,
				  int sleep_ms)
{
	int retries = SS_DEFAULT_RETRIES;

	LPM_pull_mfio_to_low_and_keep(250);
	int ret = i2c_write(SS_I2C_8BIT_SLAVE_ADDR, (char*)tx_buf, tx_len, false);
	LPM_pull_mfio_to_high();
	LPM_set_mfio_as_input();

	while (ret != 0 && retries-- > 0) {
		WAIT_MS(1);
		LPM_pull_mfio_to_low_and_keep(250);
		ret = i2c_write(SS_I2C_8BIT_SLAVE_ADDR, (char*)tx_buf, tx_len, false);
    	LPM_pull_mfio_to_high();
    	LPM_set_mfio_as_input();

	}
    if (ret != 0)
       return SS_ERR_UNAVAILABLE;

    if(sleep_ms< 0){
    	latest_write_request_time = actual_time;
    	write_request_delay = sleep_ms *1000;
    	async_write_required++;
    	return SS_ASYNC_CALL_REQUIRED;
    }

    WAIT_MS(sleep_ms);

    char status_byte;
    LPM_pull_mfio_to_low_and_keep(250);
    ret = i2c_read(SS_I2C_8BIT_SLAVE_ADDR, &status_byte, 1, false);
	LPM_pull_mfio_to_high();
	LPM_set_mfio_as_input();

	bool try_again = (status_byte == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again)
			&& retries-- > 0) {

		WAIT_MS(sleep_ms);

	    LPM_pull_mfio_to_low_and_keep(250);
	    ret = i2c_read(SS_I2C_8BIT_SLAVE_ADDR, &status_byte, 1, false);
    	LPM_pull_mfio_to_high();
    	LPM_set_mfio_as_input();

    	try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}

    if (ret != 0 || try_again) {
    	return SS_ERR_UNAVAILABLE;
    }

	return (int) (SS_STATUS)status_byte;
}

int sh_write_async_resp_cmd( uint8_t *tx_buf,
		          int tx_len,
				  int sleep_ms){
	int retries = SS_DEFAULT_RETRIES;
	int ret =0;

	if(actual_time - latest_write_request_time < write_request_delay){
		return SS_ASYNC_CALL_REQUIRED;
	}

	async_write_required--;

	char status_byte;
	LPM_pull_mfio_to_low_and_keep(250);
	ret = i2c_read(SS_I2C_8BIT_SLAVE_ADDR, &status_byte, 1, false);
	LPM_pull_mfio_to_high();
	LPM_set_mfio_as_input();

	bool try_again = (status_byte == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again)
			&& retries-- > 0) {

		WAIT_MS(sleep_ms);

		LPM_pull_mfio_to_low_and_keep(250);
		ret = i2c_read(SS_I2C_8BIT_SLAVE_ADDR, &status_byte, 1, false);
		LPM_pull_mfio_to_high();
		LPM_set_mfio_as_input();

		try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}

	if (ret != 0 || try_again) {
		return SS_ERR_UNAVAILABLE;
	}

	return (int) (SS_STATUS)status_byte;

}

int sh_write_cmd_with_data(uint8_t *cmd_bytes,
		                   int cmd_bytes_len,
                           uint8_t *data,
						   int data_len,
                           int cmd_delay_ms)
{
    memcpy(sh_write_buf, cmd_bytes, cmd_bytes_len);
    memcpy(sh_write_buf + cmd_bytes_len, data, data_len);
    int status = sh_write_cmd(sh_write_buf,cmd_bytes_len + data_len, cmd_delay_ms);
    return status;
}


int sh_read_cmd( uint8_t *cmd_bytes,
		         int cmd_bytes_len,
	             uint8_t *data,
				 int data_len,
	             uint8_t *rxbuf,
				 int rxbuf_sz,
                 int sleep_ms )
{

	int retries = SS_DEFAULT_RETRIES;

    LPM_pull_mfio_to_low_and_keep(250);
    int ret = i2c_write(SS_I2C_8BIT_SLAVE_ADDR, (char *)cmd_bytes, cmd_bytes_len, (data_len != 0));
	LPM_pull_mfio_to_high();
	LPM_set_mfio_as_input();

    if (data_len != 0) {
        LPM_pull_mfio_to_low_and_keep(250);
        ret |= i2c_write(SS_I2C_8BIT_SLAVE_ADDR, (char *)data, data_len, false);
    	LPM_pull_mfio_to_high();
    	LPM_set_mfio_as_input();
    }
	while (ret != 0 && retries-- > 0) {
		WAIT_MS(1);

		LPM_pull_mfio_to_low_and_keep(250);
		ret = i2c_write(SS_I2C_8BIT_SLAVE_ADDR, (char*)cmd_bytes, cmd_bytes_len, (data_len != 0));
    	LPM_pull_mfio_to_high();
    	LPM_set_mfio_as_input();

    	if (data_len != 0) {

    		LPM_pull_mfio_to_low_and_keep(250);
    		ret |= i2c_write(SS_I2C_8BIT_SLAVE_ADDR, (char*)data, data_len, false);
        	LPM_pull_mfio_to_high();
        	LPM_set_mfio_as_input();

    	}
	}
    if (ret != 0)
    	return SS_ERR_UNAVAILABLE;

    if(sleep_ms < 0){
    	latest_read_request_time = actual_time;
    	read_request_delay = sleep_ms*-1;
    	async_read_required++;
    	return SS_ASYNC_CALL_REQUIRED;
    }

    WAIT_MS(sleep_ms);

	LPM_pull_mfio_to_low_and_keep(250);
	ret = i2c_read(SS_I2C_8BIT_SLAVE_ADDR, (char*)rxbuf, rxbuf_sz, false);
	LPM_pull_mfio_to_high();
	LPM_set_mfio_as_input();

	bool try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0) {
		WAIT_MS(1);

		LPM_pull_mfio_to_low_and_keep(250);
		ret = i2c_read(SS_I2C_8BIT_SLAVE_ADDR, (char*)rxbuf, rxbuf_sz, false);
		LPM_pull_mfio_to_high();
		LPM_set_mfio_as_input();

    	try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	}
    if (ret != 0 || try_again)
        return SS_ERR_UNAVAILABLE;

    return (int) ((SS_STATUS)rxbuf[0]);
}

int sh_read_async_resp_cmd( uint8_t *cmd_bytes,
		         int cmd_bytes_len,
	             uint8_t *data,
				 int data_len,
	             uint8_t *rxbuf,
				 int rxbuf_sz,
                 int sleep_ms )
{

	int ret;
	int retries = SS_DEFAULT_RETRIES;

	if(actual_time - latest_read_request_time < read_request_delay){
		return SS_ASYNC_CALL_REQUIRED;
	}

	async_read_required--;

	LPM_pull_mfio_to_low_and_keep(250);
	ret = i2c_read(SS_I2C_8BIT_SLAVE_ADDR, (char*)rxbuf, rxbuf_sz, false);
	LPM_pull_mfio_to_high();
	LPM_set_mfio_as_input();

	bool try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0) {
		WAIT_MS(1);

		LPM_pull_mfio_to_low_and_keep(250);
		ret = i2c_read(SS_I2C_8BIT_SLAVE_ADDR, (char*)rxbuf, rxbuf_sz, false);
		LPM_pull_mfio_to_high();
		LPM_set_mfio_as_input();

		try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	}
	if (ret != 0 || try_again)
		return SS_ERR_UNAVAILABLE;

	return (int) ((SS_STATUS)rxbuf[0]);

}


int sh_get_sensorhub_status(uint8_t *hubStatus){

	uint8_t ByteSeq[] = {0x00,0x00};
	uint8_t rxbuf[2]  = { 0 };

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
			                    0, 0,
			                    &rxbuf[0], sizeof(rxbuf),
								SS_DEFAULT_CMD_SLEEP_MS);

	*hubStatus = rxbuf[1];
	return status;
}


int sh_get_sensorhub_operating_mode(uint8_t *hubMode){

	uint8_t ByteSeq[] = {0x02,0x00};
	uint8_t rxbuf[2]  = { 0 };

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
			                    0, 0,
			                    &rxbuf[0], sizeof(rxbuf),
								SS_DEFAULT_CMD_SLEEP_MS);

	*hubMode = rxbuf[1];
	return status;
}


int sh_set_sensorhub_operating_mode(uint8_t hubMode){

	uint8_t ByteSeq[] =  {0x01,0x00,hubMode};
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_DEFAULT_CMD_SLEEP_MS);
    return status;

}


int sh_set_data_type(int data_type_, bool sc_en_)
{

	uint8_t cmd_bytes[] = { 0x10, 0x00 };
	uint8_t data_bytes[] = { (uint8_t)((sc_en_ ? SS_MASK_OUTPUTMODE_SC_EN : 0) |
							((data_type_ << SS_SHIFT_OUTPUTMODE_DATATYPE) & SS_MASK_OUTPUTMODE_DATATYPE)) };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								&data_bytes[0], sizeof(data_bytes),
								SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}


int sh_get_data_type(int *data_type_, bool *sc_en_){

	uint8_t ByteSeq[] = {0x11,0x00};
	uint8_t rxbuf[2]  = {0};

	int status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  SS_DEFAULT_CMD_SLEEP_MS);
	if (status == 0x00 /*SS_SUCCESS*/) {
		*data_type_ =
			(rxbuf[1] & SS_MASK_OUTPUTMODE_DATATYPE) >> SS_SHIFT_OUTPUTMODE_DATATYPE;
		*sc_en_ =
			(bool)((rxbuf[1] & SS_MASK_OUTPUTMODE_SC_EN) >> SS_SHIFT_OUTPUTMODE_SC_EN);

	}

	return status;

}


int sh_set_fifo_thresh( int threshold ){

	uint8_t cmd_bytes[]  = { 0x10 , 0x01 };
	uint8_t data_bytes[] = { (uint8_t)threshold };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								&data_bytes[0], sizeof(data_bytes),
								SS_DEFAULT_CMD_SLEEP_MS
	                            );
	return status;

}


int sh_get_fifo_thresh(int *thresh){

	uint8_t ByteSeq[] = {0x11,0x01};
	uint8_t rxbuf[2]  = {0};

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							 0, 0,
							 &rxbuf[0], sizeof(rxbuf),
							 SS_DEFAULT_CMD_SLEEP_MS);

	*thresh = (int) rxbuf[1];

	return status;

}


int sh_ss_comm_check(void){


	uint8_t ByteSeq[] = {0xFF, 0x00};
	uint8_t rxbuf[2];

	int status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  SS_DEFAULT_CMD_SLEEP_MS );

	int tries = 4;
	while (status == SS_ERR_TRY_AGAIN && tries--) {

		WAIT_MS(1);
		status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
									  0, 0,
									  &rxbuf[0], sizeof(rxbuf),
									  SS_DEFAULT_CMD_SLEEP_MS );

	}

	return status;
}


int sh_num_avail_samples(int *numSamples) {

	 uint8_t ByteSeq[] = {0x12,0x00};
	 uint8_t rxbuf[2]  = {0};

	 int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  SS_DEFAULT_CMD_SLEEP_MS);

	 if(status == 0x00)
		 *numSamples = (int) rxbuf[1];
	 else
		 *numSamples = 77;

	 return status;
}

int sh_num_avail_samples_alternative(int *numSamples) {

	 uint8_t ByteSeq[] = {0xC5};
	 uint8_t rxbuf[2]  = {0};

	 int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  1);

	 *numSamples = (int) rxbuf[1];

	 return status;
}


int sh_read_fifo_data( int numSamples,
		               int sampleSize,
		               uint8_t* databuf,
					   int databufSz) {

	int bytes_to_read = numSamples * sampleSize + 1; //+1 for status byte

	uint8_t ByteSeq[] = {0x12,0x01};

	if (databufSz < bytes_to_read) {
		return E_BAD_PARAM;
	}

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							 0, 0,
							 databuf, bytes_to_read,
							 8);

	return status;
}


int sh_set_reg(int idx, uint8_t addr, uint32_t val, int regSz){

	uint8_t ByteSeq[] = { 0x40 , ((uint8_t)idx) , addr};
	uint8_t data_bytes[4];

	for (int i = 0; i < regSz; i++) {
		data_bytes[i] = (val >> (8 * (regSz - 1)) & 0xFF);
	}
	int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
							             &data_bytes[0], (uint8_t) regSz,
										 SS_DEFAULT_CMD_SLEEP_MS);

    return status;
}


int sh_get_reg(int idx, uint8_t addr, uint32_t *val){


	uint32_t i32tmp;
	int status = 0;

    if(status == 0x00 /* SS_SUCCESS */) {
    	int reg_width = 1;
    	uint8_t ByteSeq2[] = { 0x41, ((uint8_t)idx) , addr} ;
    	uint8_t rxbuf2[5]  = {0};

    	status = sh_read_cmd(&ByteSeq2[0], sizeof(ByteSeq2),
    						0, 0,
    						&rxbuf2[0], reg_width + 1,
							SS_DEFAULT_CMD_SLEEP_MS);

    	if (status == 0x00  /* SS_SUCCESS */) {
    		i32tmp = 0;
    		for (int i = 0; i < reg_width; i++) {
    			i32tmp = (i32tmp << 8) | rxbuf2[i + 1];
    		}
            *val = i32tmp;
    	}
     }

    return status;
}



int sh_sensor_enable_( int idx , int mode, uint8_t ext_mode ){

	uint8_t ByteSeq[] = { 0x44, (uint8_t)idx, (uint8_t)mode, ext_mode };

	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), 25 * SS_ENABLE_SENSOR_SLEEP_MS);
    return status;

}


int sh_sensor_disable( int idx ){

	uint8_t ByteSeq[] = {0x44, ((uint8_t) idx), 0x00};

	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), 25 * SS_ENABLE_SENSOR_SLEEP_MS);
	return status;

}


int sh_get_input_fifo_size(int *fifo_size)
{

	uint8_t ByteSeq[] = {0x13,0x01};
	uint8_t rxbuf[3]; /* status + fifo size */

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  rxbuf, sizeof(rxbuf), 2*SS_DEFAULT_CMD_SLEEP_MS);

	*fifo_size = rxbuf[1] << 8 | rxbuf[2];
	return status;
}


int sh_feed_to_input_fifo(uint8_t *tx_buf, int tx_buf_sz, int *nb_written)
{
	int status;

	uint8_t rxbuf[3];
	tx_buf[0] = 0x14;
	tx_buf[1] = 0x00;

	if(!async_read_required){
		status= sh_read_cmd(tx_buf, tx_buf_sz,
								0, 0,
								rxbuf, sizeof(rxbuf), -15);

	}else{
		status= sh_read_async_resp_cmd(tx_buf, tx_buf_sz,
										0, 0,
										rxbuf, sizeof(rxbuf), -15);
		*nb_written = rxbuf[1] * 256 + rxbuf[2];
	}

	return status;
}


int sh_get_num_bytes_in_input_fifo(int *fifo_size)
{

    uint8_t ByteSeq[] = {0x13,0x04};
	uint8_t rxbuf[3]; /* status + fifo size */

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							 0, 0,
							 rxbuf, sizeof(rxbuf),
							 2*SS_DEFAULT_CMD_SLEEP_MS);

	*fifo_size = rxbuf[1] << 8 | rxbuf[2];

	return status;

}


/*
 * ALGARITIM RELATED FUNCTIONS
 *
 *
 *
 *
 *
 * */


int sh_enable_algo_(int idx, int mode)
{
    uint8_t cmd_bytes[] = { 0x52, (uint8_t)idx, (uint8_t)mode };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes), 0, 0, 25 * SS_ENABLE_SENSOR_SLEEP_MS);

	return status;
}

int sh_disable_algo(int idx){

	uint8_t ByteSeq[] = { 0x52, ((uint8_t) idx) , 0x00};

	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), 25 * SS_ENABLE_SENSOR_SLEEP_MS );

    return status;

}


int sh_set_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz){

	uint8_t ByteSeq[] = { 0x50 , ((uint8_t) algo_idx) , ((uint8_t) cfg_idx) };

	int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                             cfg, cfg_sz,
										 SS_DEFAULT_CMD_SLEEP_MS);

	return status;

}


int sh_get_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz){

	uint8_t ByteSeq[] = { 0x51 , ((uint8_t) algo_idx) , ((uint8_t) cfg_idx) };

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
						     0, 0,
							 cfg, cfg_sz,
							 SS_DEFAULT_CMD_SLEEP_MS);
	return status;

}

int sh_get_algo_version(uint8_t algoVersion[3]){

	int status = -1;
	uint8_t cmd_bytes[3];
    uint8_t rxbuf[4];

	cmd_bytes[0] = 0xFF; //SS_FAM_R_IDENTITY;
	cmd_bytes[1] = 0x07; //SH_CMDIDX_ALGO_VER;
	cmd_bytes[2] = 0x08; //SH_ALGOIDX_WHRM_SPO2_SUITE_OS6X

	status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
	             	 	 	 	 	 	0, 0,
									    &rxbuf[0], sizeof(rxbuf) ,
										SS_DEFAULT_CMD_SLEEP_MS );

	if (status == 0x00 /*SS_SUCCESS*/) {
		algoVersion[0] = rxbuf[1];
		algoVersion[1] = rxbuf[2];
		algoVersion[2] = rxbuf[3];
	}
	else{
		memset(algoVersion, 0, 3);
	}

	return status;
}

int sh_get_sens_cfg(int sens_idx, int cfg_idx, uint8_t *cfg, int cfg_sz){

	uint8_t ByteSeq[] = { 0x47 , ((uint8_t) sens_idx) , ((uint8_t) cfg_idx) };

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
						     0, 0,
							 cfg, cfg_sz,
							 SS_DEFAULT_CMD_SLEEP_MS);
	return status;

}


int sh_set_sens_cfg(int sens_idx, int cfg_idx, uint8_t *cfg, int cfg_sz){

	uint8_t ByteSeq[] = { 0x46 , ((uint8_t) sens_idx) , ((uint8_t) cfg_idx) };

	int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                             cfg, cfg_sz,
										 SS_DEFAULT_CMD_SLEEP_MS);

	return status;

}


/*
 * BOOTLOADER RELATED FUNCTIONS
 *
 *
 * */

static int bl_comm_delay_factor = 1;



int sh_set_bootloader_delayfactor(const int factor ) {

	int status = -1;
	if( factor >= 1  && factor < 51){
	    bl_comm_delay_factor = factor;
	    status = 0x00;
	}

	return status;

}

const int sh_get_bootloader_delayfactor(void){

     return bl_comm_delay_factor;
}

int sh_exit_from_bootloader(void)
{
#if defined(SYSTEM_USES_RST_PIN)
	return sh_reset_to_main_app();
#else
	return sh_set_sensorhub_operating_mode(0x00);
#endif
}

int sh_put_in_bootloader(void)
{
	return sh_set_sensorhub_operating_mode( 0x08);
}

int sh_get_usn(unsigned char* usn)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_BOOTLOADER, SS_CMDIDX_READUSN };
	uint8_t rxbuf[USN_SIZE+1] = {0};
	int status = SS_SUCCESS;

	status = sh_read_cmd(cmd_bytes, sizeof(cmd_bytes),
						0, 0,
						rxbuf, sizeof(rxbuf),
						SS_DEFAULT_CMD_SLEEP_MS);

	if (status == SS_BTLDR_SUCCESS) {
		memcpy(usn, &rxbuf[1], USN_SIZE);
		status = SS_SUCCESS;
	}

	return status;
}

int sh_checkif_bootldr_mode(void)
{
	uint8_t hubMode;
	int status = sh_get_sensorhub_operating_mode(&hubMode);

	if ((status != SS_SUCCESS) && (status != SS_BTLDR_SUCCESS)) {
		return -1;
	}

	return (hubMode & SS_MASK_MODE_BOOTLDR);
}

int sh_get_bootloader_pagesz(int *pagesz){


	uint8_t ByteSeq[]= { 0x81, 0x01 };
    uint8_t rxbuf[3];
    int sz = 0;

    int status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
                          0, 0,
                          &rxbuf[0], sizeof(rxbuf),
						  SS_DEFAULT_CMD_SLEEP_MS);
    if (status == SS_BTLDR_SUCCESS) {
           //rxbuf holds page size in big-endian format
            sz = (256*(int)rxbuf[1]) + rxbuf[2];
            if(sz > BOOTLOADER_MAX_PAGE_SIZE ) {
                   sz = -2;
            }
    }

    *pagesz = sz;

    return status;

}

int sh_set_bootloader_numberofpages(const int pageCount){

    uint8_t ByteSeq[] = { 0x80, 0x02 };

    uint8_t data_bytes[] = { (uint8_t)((pageCount >> 8) & 0xFF), (uint8_t)(pageCount & 0xFF) };

    int status = sh_write_cmd_with_data(&ByteSeq[0], sizeof(ByteSeq),
								        &data_bytes[0], sizeof(data_bytes),
										bl_comm_delay_factor * SS_DEFAULT_CMD_SLEEP_MS );

    return status;

}

int sh_set_bootloader_iv(uint8_t * iv_bytes, int aes_nonce_sz){

	 uint8_t ByteSeq[] = { 0x80, 0x00 };

	 int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                              &iv_bytes[0], aes_nonce_sz /*sizeof(iv_bytes)*/,
										  bl_comm_delay_factor * SS_DEFAULT_CMD_SLEEP_MS
										  );

     return status;

}


int sh_set_bootloader_auth(uint8_t * auth_bytes, int aes_auth_sz){

	 uint8_t ByteSeq[] = { 0x80, 0x01 };

	 int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                              &auth_bytes[0], aes_auth_sz /*sizeof(auth_bytes)*/,
										  bl_comm_delay_factor * SS_DEFAULT_CMD_SLEEP_MS
										  );

     return status;

}


int sh_set_bootloader_erase(void){

	uint8_t ByteSeq[] = { 0x80, 0x03 };

    int status = sh_write_cmd_with_data(&ByteSeq[0], sizeof(ByteSeq),
                                        0, 0,
										bl_comm_delay_factor * SS_BOOTLOADER_ERASE_DELAY);

    return status;

}


int sh_bootloader_flashpage(uint8_t *flashDataPreceedByCmdBytes , const int page_size){

	static const int flash_cmdbytes_len   = 2;
	static const int check_bytes_len      = 16;
	static const int page_write_time_ms   = 200;

    int status = -1;

    if( (*flashDataPreceedByCmdBytes == 0x80) &&  ( *(flashDataPreceedByCmdBytes+1) == 0x04 ) )
    {

		/* We do not use sh_write_cmd_with_data function because internal buffers of the function
		   is limited to 512 bytes which does not support if flashing page size is bigger */
		status = sh_write_cmd(flashDataPreceedByCmdBytes, page_size + check_bytes_len + flash_cmdbytes_len, bl_comm_delay_factor * page_write_time_ms);

    }
	return status;

}


int sh_get_ss_fw_version(uint8_t *fwDesciptor  , uint8_t *descSize)
{

	int status = -1;
	uint8_t cmd_bytes[2];
    uint8_t rxbuf[4];

	int bootldr = in_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = 0x81 ; //SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = 0x00 ; //SS_CMDIDX_BOOTFWVERSION;
	} else if (bootldr == 0) {
		cmd_bytes[0] = 0xFF; //SS_FAM_R_IDENTITY;
		cmd_bytes[1] = 0x03; //SS_CMDIDX_FWVERSION;
	} else {
		return -1;
	}

    status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
             	 	 	 	 	 	0, 0,
								    &rxbuf[0], sizeof(rxbuf) ,
									SS_DEFAULT_CMD_SLEEP_MS );

    if (status == 0x00 /*SS_SUCCESS*/) {
    	*fwDesciptor       = rxbuf[1];
    	*(fwDesciptor + 1) = rxbuf[2];
    	*(fwDesciptor + 2) = rxbuf[3];
    	*descSize = 3;
    }else{
    	*descSize = 0;
    }

    return status;

}


int sh_set_report_period(uint8_t period)
{

	uint8_t cmd_bytes[]  = { SS_FAM_W_COMMCHAN, SS_CMDIDX_REPORTPERIOD };
	uint8_t data_bytes[] = { (uint8_t)period };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								              &data_bytes[0], sizeof(data_bytes), SS_DEFAULT_CMD_SLEEP_MS );
	return status;
}

int sh_get_dhparams(uint8_t *dhParams  , uint8_t dhParamsSize)
{
	int status = -1;
	uint8_t cmd_bytes[2] = {SH_FAM_R_DHPARAMS, 0x00};

    status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
             	 	 	 	 	 	0, 0,
									dhParams, dhParamsSize ,
									SS_DEFAULT_CMD_SLEEP_MS );
    return status;
}

int sh_set_dhlocalpublic(uint8_t *publicKey , int public_sz)
{
	uint8_t cmd_bytes[] = { SH_FAM_W_DHPUBLICKEY, (uint8_t) 0x00 };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								  publicKey, public_sz, SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}

int sh_get_dhremotepublic(uint8_t *response, int response_sz)
{
	uint8_t cmd_bytes[] = { SH_FAM_R_DHPUBLICKEY , (uint8_t) 0x00 };

	int status = sh_read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
											0, 0,
											response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}

int sh_get_authentication(uint8_t *response, int response_sz)
{
	uint8_t cmd_bytes[] = { SH_FAM_R_SHADIGEST , (uint8_t) 0x00 };

	int status = sh_read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
								0, 0,
								response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}

int sh_spi_release()
{
	uint8_t ByteSeq[] =  {SS_FAM_W_SPI_SELECT, SS_CMDIDX_SPI_RELASE};
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_DEFAULT_CMD_SLEEP_MS);
    return status;
}


int sh_spi_use()
{
	uint8_t ByteSeq[] =  {SS_FAM_W_SPI_SELECT, SS_CMDIDX_SPI_USE};
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_DEFAULT_CMD_SLEEP_MS);
    return status;
}


int sh_spi_status(uint8_t * spi_status)
{
	uint8_t ByteSeq[] =  {SS_FAM_R_SPI_SELECT};

	uint8_t rxbuf[2]  = { 0 };

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
			                    0, 0,
			                    &rxbuf[0], sizeof(rxbuf),
								SS_DEFAULT_CMD_SLEEP_MS);

	*spi_status = rxbuf[1];
	return status;
}

int sh_comm_sync(uint64_t time){
	actual_time = time;

	if(async_read_required > 0 || async_write_required > 0){
		return SS_ASYNC_CALL_REQUIRED;
	}else{
		return SS_SUCCESS;
	}

}

