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

#ifndef ALGOHUB_ALGOHUB_API_H_
#define ALGOHUB_ALGOHUB_API_H_

#define ALGOHUB_INPUT_FRAME_SIZE 4
#define ALGOHUB_INPUT_BATCH_SIZE 1

typedef enum{
	ALGOHUB_PPG_GREEN0 = 0u,
	ALGOHUB_PPG_GREEN1,
	ALGOHUB_PPG_IR,
	ALGOHUB_PPG_RED,

	ALGOHUB_PPG_MAX
} algohub_ppg_singal_type_t;

typedef struct __attribute__((packed)){
	struct {
		uint32_t green1;
		uint32_t green2;
		uint32_t ir;
		uint32_t red;
	} ppg_data_in;

	struct {
		int16_t x;
		int16_t y;
		int16_t z;
	} acc_data_in;

} algohub_feed_data_t;

typedef struct __attribute__((packed)){
	uint32_t test_led;

	struct{
		uint8_t isAfeRequestExist:1;
		uint8_t algoMode;
	} algo;

	uint16_t hr;
	uint8_t hrConfidence;
	uint16_t rr;
	uint8_t rrConfidence;
	uint8_t activityClass;
	uint16_t r;
	uint8_t spo2Confidence;
	uint16_t spo2;
	uint8_t spo2PercentComplete;
	uint8_t spo2LowSignalQualityFlag;
	uint8_t spo2MotionFlag;
	uint8_t spo2LowPiFlag;
	uint8_t spo2UnreliableRFlag;
	uint8_t spo2State;
	uint8_t scdSontactState;
	uint8_t algoRet;
} algohub_report_t;

typedef struct __attribute__((packed)){

	struct{
		uint16_t is_led_curr_update_requested:1;
		uint16_t curr:15;
	} led_curr_adjustment;

	struct{
		uint8_t is_int_time_update_requested:1;
		uint8_t integration_time:7;
	} int_time_adjustment;

	struct{
		uint8_t is_avg_smp_update_requested:1;
		uint8_t avg_smp:7;
	} avg_smp_adjustment;

	struct{
		uint8_t is_dac_offset_update_requested:1;
		uint8_t dac_offset:7;
	} dac_offset_adjustment;

} algohub_afe_reqs_t;

/**
 * @brief	function to initialize algohub mode
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_init();

/**
 * @brief	function to enable algohub mode
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_enable();

/**
 * @brief	function to disable algohub mode
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_disable();

/**
 * @brief	function to feed measurement results to algohub
 *
 * @param[in]  p_data - PPG and accelerometer signal values which will be sent to algohub
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_feed_data(const algohub_feed_data_t * const p_data);



int algohub_feed_batch_data(const algohub_feed_data_t * const p_data, uint8_t number_of_inputs);


/**
 * @brief	function to read output FIFO of algohub
 *
 * @param[out]  p_result   			- will keep the results of algohub calculations
 * @param[in]  numberResultToBeRead - how many samples will be read
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_read_outputfifo(const algohub_report_t * p_result, const int numberResultToBeRead);

/**
 * @brief	function to reset algohub configuration. Algohub will return its default settings.
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_reset_configs();

/**
 * @brief	function to notify algohub that AFE request has been applied.
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_notify_afe_request_applied();

/**
 * @brief	function to read software version of algohub
 *
 * @param[out]  version   - Version of the algohub
 * 							version[0]: Major version number
 * 							version[1]: Minor version number
 * 							version[2]: Patch version number
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_read_software_version(uint8_t version[3]);

/**
 * @brief	function to get AFE requests
 *
 * @param[out]  afe_reqs   - AFE requests for two WHRM and two SpO2 channels
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_get_aferequest(uint8_t afe_reqs[20]);

/**
 * @brief	function to parse Algohub request. This function is only valid for default ledPDConfig
 *
 *
 * @param[in]  afe_reqs   - AFE requests for two WHRM and two SpO2 channels
 * @param[out]  afe		  -
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
void algohub_parse_aferequest(uint8_t afe_reqs[20], algohub_afe_reqs_t * afe);

/**
 * @brief	function to get ppg signal measurement channel number. This function is only valid for default ledPDConfig
 *
 *
 * @param[in]  ppg_signal   -	PPG signal type
 *
 * @return	Measurement channel number that PPG signal is assigned to
 */
int algohub_ppg_signal_meas_ch_no(algohub_ppg_singal_type_t ppg_signal);

/**
 * @brief	function to get ppg signal pd number. This function is only valid for default ledPDConfig
 *
 *
 * @param[in]  ppg_signal   -	PPG signal type
 *
 * @return	PD number that PPG signal is assigned to
 */
int algohub_ppg_signal_pd_no(algohub_ppg_singal_type_t ppg_signal);

/**
 * @brief	function to make request to the algohub/sensorhub to release SPI lines
 * 			If MAX86178 and LIS2DS12 SPI lines are shared between host processor and algohub/sensorhub,
 * 			this function must be called before making any access to these sensors.
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_send_spi_release_request();

/**
 * @brief	function to notify algohub/sensorhub that SPI lines has been released
 * 			If MAX86178 and LIS2DS12 SPI lines are shared between host processor and you want to transfer
 * 			the ownership of SPI lines to the algohub/sensorhub, this function must be called.
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int algohub_notify_spi_released();



int algohub_sync(uint64_t time);

/**
 * @brief TODO
 * 
 * 
*/
int algohub_process(uint64_t time);

#endif /* ALGOHUB_ALGOHUB_API_H_ */
