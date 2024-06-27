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

#ifndef SOURCE_SHCOMM_H_
#define SOURCE_SHCOMM_H_

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif


// Sensor/Algo indicies
#define SH_SENSORIDX_ACCEL	    0x04
#define SH_SENSORIDX_MAX86178	0x06
#define SH_SENSORIDX_ALGOHUB	0x07

#define SS_ALGOIDX_WHRM_WSPO2_SUITE_OS6X	 0x08
#define SH_NUM_CURRENT_ALGOS	8

#define PADDING_BYTE            (0xEE)
#define DATA_BYTE               (0xED)


#define SS_I2C_8BIT_SLAVE_ADDR 0xAA
#define SS_DEFAULT_CMD_SLEEP_MS 5
#define SS_DUMP_REG_SLEEP_MS 100
#define SS_ENABLE_SENSOR_SLEEP_MS 20
#define SS_BOOTLOADER_ERASE_DELAY	1000

#define SH_INPUT_DATA_DIRECT_SENSOR	0x00
#define SH_INPUT_DATA_FROM_HOST		0x01

#define SS_FAM_R_STATUS		0x00
	#define SS_CMDIDX_STATUS	0x00
		#define SS_SHIFT_STATUS_ERR				0
		#define SS_MASK_STATUS_ERR				(0x07 << SS_SHIFT_STATUS_ERR)
		#define SS_SHIFT_STATUS_DATA_RDY		3
		#define SS_MASK_STATUS_DATA_RDY			(1 << SS_SHIFT_STATUS_DATA_RDY)
		#define SS_SHIFT_STATUS_FIFO_OUT_OVR	4
		#define SS_MASK_STATUS_FIFO_OUT_OVR		(1 << SS_SHIFT_STATUS_FIFO_OUT_OVR)
		#define SS_SHIFT_STATUS_FIFO_IN_OVR		5
		#define SS_MASK_STATUS_FIFO_IN_OVR		(1 << SS_SHIFT_STATUS_FIFO_IN_OVR)

		#define SS_SHIFT_STATUS_LOG_OVR			6
		#define SS_MASK_STATUS_LOG_OVR			(1 << SS_SHIFT_STATUS_LOG_OVR)

		#define SS_SHIFT_STATUS_LOG_RDY			7
		#define SS_MASK_STATUS_LOG_RDY			(1 << SS_SHIFT_STATUS_LOG_RDY)



#define SS_FAM_W_MODE	0x01
#define SS_FAM_R_MODE	0x02
	#define SS_CMDIDX_MODE	0x00
		#define SS_SHIFT_MODE_SHDN		0
		#define SS_MASK_MODE_SHDN		(1 << SS_SHIFT_MODE_SHDN)
		#define SS_SHIFT_MODE_RESET		1
		#define SS_MASK_MODE_RESET		(1 << SS_SHIFT_MODE_RESET)
		#define SS_SHIFT_MODE_FIFORESET	2
		#define SS_MASK_MODE_FIFORESET	(1 << SS_SHIFT_MODE_FIFORESET)
		#define SS_SHIFT_MODE_BOOTLDR	3
		#define SS_MASK_MODE_BOOTLDR	(1 << SS_SHIFT_MODE_BOOTLDR)

/*MYG*/
#define SH_MODE_REQUEST_RET_BYTES        (2)
#define SH_MODE_REQUEST_DELAY            (2)
#define SH_STATUS_REQUEST_RET_BYTES      (2)
#define SH_STATUS_REQUEST_DELAY          (2)



#define SS_I2C_READ		0x03

#define SS_FAM_W_COMMCHAN	0x10
#define SS_FAM_R_COMMCHAN	0x11
	#define SS_CMDIDX_OUTPUTMODE	0x00
		#define SS_SHIFT_OUTPUTMODE_DATATYPE	0
		#define SS_MASK_OUTPUTMODE_DATATYPE		(0x03 << SS_SHIFT_OUTPUTMODE_DATATYPE)
			#define SS_DATATYPE_PAUSE				0
			#define SS_DATATYPE_RAW					1
			#define SS_DATATYPE_ALGO				2
			#define SS_DATATYPE_BOTH				3
		#define SS_SHIFT_OUTPUTMODE_SC_EN		2
		#define SS_MASK_OUTPUTMODE_SC_EN		(1 << SS_SHIFT_OUTPUTMODE_SC_EN)
	#define SS_CMDIDX_FIFOAFULL		0x01
    #define SS_CMDIDX_REPORTPERIOD	0x02

#define SS_FAM_R_OUTPUTFIFO	0x12
	#define SS_CMDIDX_OUT_NUMSAMPLES	0x00
	#define SS_CMDIDX_READFIFO		    0x01

#define SS_FAM_R_INPUTFIFO						0x13
	#define SS_CMDIDX_SAMPLE_SIZE				0x00
	#define SS_CMDIDX_INPUT_FIFO_SIZE			0x01
	#define SS_CMDIDX_SENSOR_FIFO_SIZE			0x02
	#define SS_CMDIDX_NUM_SAMPLES_SENSOR_FIFO	0x03
	#define SS_CMDIDX_NUM_SAMPLES_INPUT_FIFO	0x04

#define SS_FAM_W_INPUTFIFO	0x14
	#define SS_CMDIDN_WRITEFIFO		0x00
	#define SS_CMDIDX_WRITE_FIFO    0x00

#define SS_FAM_W_WRITEREG		0x40
#define SS_FAM_R_READREG		0x41
#define SS_FAM_R_REGATTRIBS		0x42
#define SS_FAM_R_DUMPREG		0x43

#define SS_FAM_W_SENSORMODE	0x44
#define SS_FAM_R_SENSORMODE	0x45


//TODO: Fill in known configuration parameters
#define SS_FAM_W_ALGOCONFIG	0x50
#define SS_FAM_R_ALGOCONFIG	0x51
// config for WHRM+WSPO2 ALGO SUITE
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2_CAL		                0x00
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_PERIOD          0x01
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_THRESHOLD       0x02
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_AFE_TIMEOUT       		0x03
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_TIMEOUT       			0x04
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_HR       			0x05
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_HEIGHT       			0x06
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_WEIGHT       			0x07
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_AGE       			0x08
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_GENDER       			0x09
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_ALGO_MODE        	        0x0A
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_AEC_ENABLE           	    0x0B
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_SCD_ENABLE           	    0x0C
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT_PERIOD     0x0D
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MOTION_MAG_THRESHOLD         0x0E
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_PD_CURRENT               0x0F
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_INIT_PD_CURRENT              0x10
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT            0x11
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_AUTO_PD_CURRENT_ENABLE       0x12
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_INTEGRATION_TIME         0x13
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_SAMPLING_AVERAGE         0x14
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_INTEGRATION_TIME         0x15
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_SAMPLING_AVERAGE         0x16
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_WHRMLEDPDCONFIGURATION       0x17
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2LEDPDCONFIGURATION       0x18
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_DRIVER_LED_SELECT			0x19
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_D_OFT_OPTION				0x1C
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_LED_CURR_OPTION			0x1D
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_LED_CURR_OPTION			0x1E
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_LED_CURR_STEP_OPTION		0x1F
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_MASTER_CH_SEL_OPTION			0x20
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_FULL_SCALE_PD_CURR_OPTION	0x21
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_AFE_TYPE_OPTION				0x22
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_T_INT_OPTION			0x1A
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_F_SMP_OPTION			0x1B
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_D_OFF_PPG1_OPTION	0x23
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_D_OFF_PPG2_OPTION	0x24
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_LED_CURR_OPTION		0x25
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_RESET_ALGO_CONFIG			0x26
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_GET_AFE_REQUEST_OPTION		0x27
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_CLEAR_AFE_REQUEST_OPTION		0x28


#define SS_FAM_W_ALGOMODE	0x52
#define SS_FAM_R_ALGOMODE	0x53

#define SS_FAM_W_SPI_SELECT	0x54
	#define SS_CMDIDX_SPI_RELASE	0x00
	#define SS_CMDIDX_SPI_USE		0x01
#define SS_FAM_R_SPI_SELECT 0x55

#define SS_FAM_W_EXTERNSENSORMODE	0x60
#define SS_FAM_R_EXTERNSENSORMODE	0x61

#define SS_FAM_R_SELFTEST    0x70

#define SS_FAM_W_BOOTLOADER	0x80
	#define SS_CMDIDX_SETIV			0x00
	#define SS_CMDIDX_SETAUTH		0x01
	#define SS_CMDIDX_SETNUMPAGES	0x02
	#define SS_CMDIDX_ERASE			0x03
	#define SS_CMDIDX_SENDPAGE		0x04
	#define SS_CMDIDX_ERASE_PAGE	0x05
#define SS_FAM_R_BOOTLOADER	0x81
	#define SS_CMDIDX_BOOTFWVERSION	0x00
	#define SS_CMDIDX_PAGESIZE		0x01
	#define SS_CMDIDX_READUSN 		0x02

#define USN_SIZE	24

#define SS_FAM_W_BOOTLOADER_CFG	0x82
#define SS_FAM_R_BOOTLOADER_CFG	0x83
	#define SS_CMDIDX_BL_SAVE		0x00
	#define SS_CMDIDX_BL_ENTRY		0x01
		#define SS_BL_CFG_ENTER_BL_MODE		0x00
		#define SS_BL_CFG_EBL_PIN			0x01
		#define SS_BL_CFG_EBL_POL			0x02
	#define SS_CMDIDX_BL_EXIT		0x02
		#define SS_BL_CFG_EXIT_BL_MODE		0x00
		#define SS_BL_CFG_TIMEOUT			0x01

/* Enable logging/debugging */
#define SS_FAM_R_LOG				0x90
	#define SS_CMDIDX_R_LOG_DATA	0x00
	#define SS_CMDIDX_R_LOG_LEN		0x01

	#define SS_CMDIDX_R_LOG_LEVEL	0x02
		#define SS_LOG_DISABLE		0x00
		#define SS_LOG_CRITICAL		0x01
		#define SS_LOG_ERROR		0x02
		#define SS_LOG_INFO			0x04
		#define SS_LOG_DEBUG		0x08

#define SS_FAM_W_LOG_CFG			0x91
	#define SS_CMDIDX_LOG_GET_LEVEL	0x00
	#define SS_CMDIDX_LOG_SET_LEVEL	0x01
/*
 *  Test Comamnd
 */
#define SS_FAM_W_RUN_TEST					0xF0
	#define SS_CMDIDX_TEST_SENSORHUB		0x00
	#define SS_CMDIDX_TEST_OPTICAL_SENSOR	0x01
	#define SS_CMDIDX_TEST_ACCEL_SENSOR		0x02
	// To manage a sync test cases
	#define SS_CMDIDX_TEST_START			0x10
	#define SS_CMDIDX_TEST_GET_RESULT		0x20

#define SS_FAM_R_IDENTITY			0xFF
	#define SS_CMDIDX_PLATTYPE		0x00
	#define SS_CMDIDX_PARTID		0x01
	#define SS_CMDIDX_REVID			0x02
	#define SS_CMDIDX_FWVERSION		0x03
	#define SS_CMDIDX_AVAILSENSORS	0x04
	#define SS_CMDIDX_DRIVERVER		0x05
	#define SS_CMDIDX_AVAILALGOS	0x06
	#define SS_CMDIDX_ALGOVER		0x07

#define SS_RESET_TIME	10
#define SS_STARTUP_TO_BTLDR_TIME	20
#define SS_STARTUP_TO_MAIN_APP_TIME	1000

#define SS_MAX_SUPPORTED_SENSOR_NUM	0xFE
#define SS_MAX_SUPPORTED_ALGO_NUM	0xFE

#define SS_APPPLICATION_MODE   0x00
#define SS_BOOTLOADER_MODE     0x08

#define CONFIG_ENABLE   (0x01)
#define CONFIG_DISABLE  (0x00)

#define SH_FAM_R_SHADIGEST          0xB2
#define SH_FAM_W_SHANONCE           0xB1
#define SH_FAM_R_DHPARAMS           0xB3
#define SH_FAM_W_DHPUBLICKEY        0xB4
#define SH_FAM_R_DHPUBLICKEY        0xB5

typedef enum {
	SS_SUCCESS             =0x00,
	SS_ERR_COMMAND         =0x01,
	SS_ERR_UNAVAILABLE     =0x02,
	SS_ERR_DATA_FORMAT     =0x03,
	SS_ERR_INPUT_VALUE     =0x04,
	SS_ERR_BTLDR_GENERAL   =0x80,
	SS_ERR_BTLDR_CHECKSUM  =0x81,
	SS_BTLDR_SUCCESS       =0xAA,
	SS_BTLDR_PARTIAL_ACK   =0xAB,
	SS_ASYNC_CALL_REQUIRED =0xB0,
	SS_ERR_TRY_AGAIN       =0xFE,
	SS_ERR_UNKNOWN         =0xFF,

} SS_STATUS;


#define SYSTEM_USES_MFIO_PIN
#define SYSTEM_USES_RST_PIN

/**
* @brief	Func to write to algohub/sensorhub via sending generic command byte sequences
*
* @param[in]	tx_buf   - command byte sequence
* @param[in]	tx_len   - command byte sequence length in bytes
* @param[in]	sleep_ms - time to wait for sensor hub to report status
*
* @return 1 byte status: 0x00 (SS_SUCCESS) on success
*/
int sh_write_cmd( uint8_t *tx_buf,
		          int tx_len,
				  int sleep_ms );


/**
* @brief	Func to write to algohub/sensorhub via sending generic command byte sequences and data bytes
*
* @param[in]	cmd_bytes      - command byte sequence
* @param[in]	cmd_bytes_len  - command byte sequence length in bytes
* @param[in]    data           - data byte array to be sent following cmd bytes
* @param[in]    data_len       - data array size in bytes
* @param[in]    cmd_delay_ms   - time to wait for sensor hub to report status
*
* @return 1 byte status: 0x00 (SS_SUCCESS) on success
*/
int sh_write_cmd_with_data(uint8_t *cmd_bytes,
		                   int cmd_bytes_len,
                           uint8_t *data,
						   int data_len,
                           int cmd_delay_ms);


/**
* @brief	Func to read from algohub/sensorhub via sending generic command byte sequences
*
* @param[in]	cmd_bytes      - command byte sequence
* @param[in]	cmd_bytes_len  - command byte sequence length in bytes
* @param[in]    data           - data byte array to be sent following cmd bytes
* @param[in]    data_len       - data array size in bytes
* @param[out]   rxbuf          - byte buffer to store incoming data (including status byte)
* @param[in]    rxbuf_sz       - incoming data buffer size in bytes ( to prevent overflow)
* @param[in]    cmd_delay_ms   - time to wait for sensor hub to report status
*
* @return 1 byte status: 0x00 (SS_SUCCESS) on success
*/
int sh_read_cmd( uint8_t *cmd_bytes,
		         int cmd_bytes_len,
	             uint8_t *data,
				 int data_len,
	             uint8_t *rxbuf,
				 int rxbuf_sz,
                 int sleep_ms );


/**
* @brief	func to read algohub/sensorhub status
* @param[out]	hubStatus   - pointer to output byte sesnor hub status will be written
* @details	 ensor hub status byte:   [2:0] ->  0 : no Err ,              1: comm failure with sensor
 *                                    [3]   ->  0 : FIFO below threshold; 1: FIFO filled to threshold or above.
 *                                    [4]   ->  0 : No FIFO overflow;     1: Sensor Hub Output FIFO overflowed, data lost.
 *                                    [5]   ->  0 : No FIFO overflow;     1: Sensor Hub Input FIFO overflowed, data lost.
 *                                    [6]   ->  0 : Sensor Hub ready;     1: Sensor Hub is busy processing.
 *                                    [6]   ->  reserved.
*
* @return 1 byte status: 0x00 (SS_SUCCESS) on success
*/
int sh_get_sensorhub_status(uint8_t *hubStatus);


/**
* @brief	func to read algohub/sensorhub operating mode
*
* @param[in]	hubMode   - pointer to output byte mode will be written
* @details      0x00: application operating mode
*               0x08: bootloader operating mode
*
* @return 1 byte status: 0x00 (SS_SUCCESS) on success
*/
int sh_get_sensorhub_operating_mode(uint8_t *hubMode);


/**
* @brief	func to set algohub/sensorhub operating mode
*
* @param[out]	hubMode   - pointer to output byte mode will be written
* @details      0x00: application operating mode
*               0x02: soft reset
*               0x08: bootloader operating mode
*
* @return 1 byte status: 0x00 (SS_SUCCESS) on success
*/
int sh_set_sensorhub_operating_mode(uint8_t hubMode);


/**
* @brief	func to set algohub/sensorhub data output mode
*
* @param[in]	data_type : 1 byte output format
* @details      outpur format 0x00 : no data
 *                            0x01 : sensor data  SS_DATATYPE_RAW
 *                            0x02 : algo data    SS_DATATYPE_ALGO
 *                            0x03 : algo+sensor  SS_DATATYPE_BOTH
*
* @return 1 byte status: 0x00 (SS_SUCCESS) on success
*/
int sh_set_data_type(int data_type, bool sc_en);


/**
* @brief	func to get algohub/sensorhub data output mode
*
* @param[out]	data_type   - pointer to  byte, output format will be written to.
*
* @param[out]    sc_en     -  pointer to  boolean, sample count enable/disable status format will be written to.
*                            If true, SmartSensor is prepending data with 1 byte sample count.
*
* @details      output format 0x00 : only algorithm data
 *                            0x01 : only raw sensor data
 *                            0x02 : algo + raw sensor data
 *                            0x03 : no data
*
* @return 1 byte status: 0x00 (SS_SUCCESS) on success
*/
int sh_get_data_type(int *data_type, bool *sc_en);


/**
 * @brief	func to set the number of samples for the SmartSensor to collect
 *			before issuing an mfio event reporting interrupt
 *
 * @param[in]	thresh - Number of samples (1-255) to collect before interrupt
 *
 * @return 1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_fifo_thresh( int threshold );


/**
 * @brief	func to get the number of samples the SmartSensor will collect
 *			before issuing an mfio event reporting interrupt
 *
 * @param[out]	thresh - Number of samples (1-255) collected before interrupt
 *
 * @return 1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_fifo_thresh(int *thresh);


/**
 * @brief	func to check that the SmartSensor is connected
 *
 * @return 1 byte connection status 0x00: on connection
 */
int sh_ss_comm_check(void);


/**
* @brief	func to get the number of available samples in SmartSensor output FIFO
*
* @param[out]	numSamples -  number of data struct samples (1-255)
*
* @return 1 byte status: 0x00 (SS_SUCCESS) on success
*/
int sh_num_avail_samples(int *numSamples);


/**
* @brief	func to pull samples from SmartSensor output FIFO
*
* @param[in]	numSamples  - number of data struct samples to be pulled
* @param[in]    sampleSize  - size of cumulative data sample struct (based on enabled sesnors+algorithms) in bytes
* @param[out]   databuf     - buffer samples be written
* @param[in]    databufSize - size of provided buffer size samples to be written
*
* @return 1 byte status: 0x00 (SS_SUCCESS) on success
*/
int sh_read_fifo_data( int numSamples, int sampleSize, uint8_t* databuf, int databufSz);


/**
 * @brief	func to set register of a device
 *
 * @param[in] idx   - Index of device to read
 * @param[in] addr  - Register address
 * @param[in] val   - Register value
 * @param[in] regSz - Size of sensor device register in bytes
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_reg(int idx, uint8_t addr, uint32_t val, int regSz);


/**
 * @brief	func to read register from a device
 *
 * @param[in]  idx - Index of device to read
 * @param[in]  addr - Register address
 * @param[out] val - Register value
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_reg(int idx, uint8_t addr, uint32_t *val);

/**
 * @brief	func to enable a sensor device onboard SmartSensor
 *
 * @param[in] idx             - index of sensor device( i.e max86178) to enable
 * @param[in] mode            - sensor mode defined in associated Sensor Hub API document
 * @param[in] ext_mode        - enable extermal data input to Sensot Hub, ie accelerometer data for WHRM+WSPo2
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_sensor_enable_( int idx , int mode, uint8_t ext_mode );


/**
 * @brief	func to disable a device on the SmartSensor
 *
 * @param[in] idx - Index of device
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_sensor_disable( int idx );


/**
 * @brief	func to get the total number of samples the input FIFO can hold
 *
 * @param[in] fifo_size - integer input FIFO capacity will be written to.
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_input_fifo_size(int *fifo_size);


/**
 * @brief	func to send ass external sensor data (accelerometer) to sensor hub's input FIFO
 *
 * @param[in]  tx_buf     - host sample data to be send to sensor hub input FIFO
 * @param[in]  tx_buf_sz  - number of bytes of tx_buf
 * @param[out] nb_written - number of samples succesfully written to sensor hub's input FIFO
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_feed_to_input_fifo(uint8_t *tx_buf, int tx_buf_sz, int *nb_written);


/**
 * @brief	func to get the total number of bytes in the sensor hub's input FIFO
 *
 * @param[in]  fifo_size - total number of sample bytes available in input FIFO
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_num_bytes_in_input_fifo(int *fifo_size);


/**
 * @brief	func to enable an algorithm on  SmartSensor
 *
 * @param[in] idx            - index of algorithm to enable
 * @param[in] mode           - algorithm mode defined in associated Sensor Hub API document
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_enable_algo_(int idx, int mode);



/**
 * @brief	func to disable an algorithm on the SmartSensor
 *
 * @param[in] idx - index of algorithm to disable
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_disable_algo(int idx);


/**
 * @brief	func to set the value of an algorithm configuration parameter
 *
 * @param[in] algo_idx   - index of algorithm
 * @param[in] cfg_idx    - index of configuration parameter
 * @param[in] cfg Array  - byte array of configuration
 * @param[in] cfg_sz     - size of cfg array
 *
 * @return 1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz);


/**
 * @brief	func to get the value of an algorithm configuration parameter
 *
 * @param[in] algo_idx  - index of algorithm
 * @param[in] cfg_idx   - index of configuration parameter
 * @param[out] cfg      - array of configuration bytes to be filled in
 * @param[in] cfg_sz    - number of configuration parameter bytes to be read
 *
 * @return 1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz);

/**
 * @brief	func to get the version of the algorithm
 *
 * @param[out] algoVersion - version of the algorithm
 *
 * @return 1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_algo_version(uint8_t algoVersion[3]);

/**
 * @brief	func to get the value of an sensor configuration parameter
 *
 * @param[in] sens_idx  - index of sensor
 * @param[in] cfg_idx   - index of configuration parameter
 * @param[out] cfg      - array of configuration bytes to be filled in
 * @param[in] cfg_sz    - number of configuration parameter bytes to be read
 *
 * @return 1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_sens_cfg(int sens_idx, int cfg_idx, uint8_t *cfg, int cfg_sz);

/**
 * @brief	func to set the value of an sensor configuration parameter
 *
 * @param[in] sens_idx   - index of sensor
 * @param[in] cfg_idx    - index of configuration parameter
 * @param[in] cfg Array  - byte array of configuration
 * @param[in] cfg_sz     - size of cfg array
 *
 * @return 1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_sens_cfg(int sens_idx, int cfg_idx, uint8_t *cfg, int cfg_sz);


/**
 * @brief		run the self test commands
 * param[in]	idx - the id of the sensor for the self test
 * param[in]	result - self-test response
 * param[in]	sleep_ms - duration of wait for read command
 *
 * @return		1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_self_test(int idx, uint8_t *result, int sleep_ms);


/**
 * @brief		transition from bootloder mode to application mode
 *
 * @return		1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_exit_from_bootloader(void);


/**
 * @brief		transition from application mode to bootloader mode
 *
 * @return		1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_put_in_bootloader(void);

/**
 * @brief	Check if SmartSensor is in bootloader mode
 *
 * @return	1 byte mode info : 1 if in bootloader mode, 0 if in main app, -1 if comm error
 */
int sh_checkif_bootldr_mode(void);

/**
* @brief	Get a string representing the SmartSensor firmware version
* @details	If in bootloader mode, returns bootloader version
*
* @return   Pointer to firmware version string
*/
const char* sh_get_hub_fw_version(void);

/**
* @brief	Get a string representing the SmartSensor algo version
* @details	If in bootloader mode, returns bootloader version
*
* @return   Pointer to algo version string
*/
const char* sh_get_hub_algo_version(void);


/**
 * @brief		send raw string to I2C
 * @param[in]	rawdata - Raw data string, after slave address
 * @param[out]	rawdata_sz - Raw data size
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int  sh_send_raw(uint8_t *rawdata, int rawdata_sz);

/**
 * @brief		get length of hub debug log data available
 * @param[out]	log_len - length of hub log data available
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_log_len(int *log_len);


/**
 * @brief		read hub debug log data available
 * @details	    first call sh_get_log_len() to get available log data in bytes then
 *              call this function with parameter num_bytes with a value smaller then available log data in bytes
 *
 * @param[in]	num_bytes  - number of log data bytes to be read
 * @param[in]	log_buf_sz - byte size of buffer log data will be dumped to
 * @param[out]	log_buf    - byte buffer log data will be dumped to
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_read_ss_log(int num_bytes, uint8_t *log_buf, int log_buf_sz);


/**
 * @brief		read sensor hub firmaware version
 *
 * @param[out]	fwDesciptor - byte array fw version will be written to
 * @param[out]	fwDescSz    - array size of firmware descriptor in bytes
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 *
 **/
int sh_get_ss_fw_version(uint8_t *fwDesciptor  , uint8_t *fwDescSz);


/**
 * @brief		Makes a request for Algohub/Sensorhub to release SPI lines
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 *
 **/
int sh_spi_release();

/**
 * @brief		Makes a request for Algohub/Sensorhub to use SPI lines
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 *
 **/
int sh_spi_use();

/**
 * @brief		Returns the status of SPI lines it uses SPI lines or not
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 *
 **/
int sh_spi_status(uint8_t * spi_status);

/**
 * @brief		Set the reporting period
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 *
 **/
int sh_set_report_period(uint8_t period);


/**
 * @brief		Get the dhparams
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 *
 **/
int sh_get_dhparams(uint8_t *dhParams  , uint8_t dhParamsSize);

/**
 * @brief		Set the dhlocalpublic
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 *
 **/
int sh_set_dhlocalpublic(uint8_t *publicKey , int public_sz);

/**
 * @brief		Get the dhlocalpublic
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 *
 **/
int sh_get_dhremotepublic(uint8_t *response, int response_sz);

/**
 * @brief		Get the authentication
 *
 * @return      1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 *
 **/
int sh_get_authentication(uint8_t *response, int response_sz);

/* ***************************************************************************************** *
 *																							 *
 *			BOOTLOADER ADDITIONS                     									     *
 *                                                    										 *
 * ***************************************************************************************** */
int sh_get_bootloader_pagesz(int *pagesz);
int sh_set_bootloader_numberofpages(const int pageCount);
int sh_set_bootloader_iv(uint8_t *ivbytes, int aes_nonce_sz);
int sh_set_bootloader_auth(uint8_t *auth_bytes, int aes_auth_sz);
int sh_set_bootloader_erase(void);
int sh_bootloader_flashpage(uint8_t *flashDataPreceedByCmdBytes , const int page_size);
int sh_set_bootloader_delayfactor(const int factor );
const int sh_get_bootloader_delayfactor(void);
int sh_set_ebl_mode(const uint8_t mode);
const int sh_get_ebl_mode(void);
int sh_reset_to_bootloader(void);
int sh_reset_to_main_app(void);
int sh_debug_reset_to_bootloader(void);
int exit_from_bootloader(void);
int sh_get_usn(unsigned char* usn);
int sh_comm_sync(uint64_t time);

void sh_init_hwcomm_interface();
int sh_hard_reset(int wakeupMode);


#ifdef __cplusplus
}
#endif




#endif /* _SENSOR_HUB_H */





