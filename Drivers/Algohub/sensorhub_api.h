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

#ifndef ALGOHUB_SENSORHUB_API_H_
#define ALGOHUB_SENSORHUB_API_H_

typedef enum{
	SENSORHUB_MODE_BASIC    = 1,
	SENSORHUB_MODE_EXTENDED = 2,
} sensorhub_report_mode_t;

typedef enum{
	SH_PPG_SIGNAL_GREEN0 = 0,
	SH_PPG_SIGNAL_GREEN1,
	SH_PPG_SIGNAL_IR,
	SH_PPG_SIGNAL_RED,

	SH_PPG_SIGNAL_MAX
} sensorhub_ppg_signal_t;

typedef struct __attribute__((packed)){
	uint32_t led1; // LED1, PD1
	uint32_t led2;
	uint32_t led3; // LED3, PD1
	uint32_t led4; // LED1, PD2
	uint32_t led5;
	uint32_t led6; // LED3, PD2
} max86178_data;

typedef struct __attribute__((packed)){
	int16_t x;
	int16_t y;
	int16_t z;
} accel_data;

typedef struct __attribute__((packed)){
	uint8_t current_operating_mode; // mode 1 & 2

	// WHRM data
	uint16_t hr;         	// mode 1 & 2
	uint8_t hr_conf;     	// mode 1 & 2
	uint16_t rr;         	// mode 1 & 2
	uint8_t rr_conf;		// mode 1 & 2
	uint8_t activity_class; // mode 1 & 2

	// WSPO2 data
	uint16_t r;						// mode 1 & 2
	uint8_t spo2_conf;		// mode 1 & 2
	uint16_t spo2;			// mode 1 & 2
	uint8_t percentComplete;		// mode 1 & 2
	uint8_t lowSignalQualityFlag;	// mode 1 & 2
	uint8_t motionFlag;				// mode 1 & 2
	uint8_t lowPiFlag;				// mode 1 & 2
	uint8_t unreliableRFlag;		// mode 1 & 2
	uint8_t spo2State;   			// mode 1 & 2
	uint8_t scd_contact_state;

	//Extended Report (mode2)
	uint32_t walk_steps;	// mode 2
	uint32_t run_steps;		// mode 2
	uint32_t kcal;			// mode 2
	uint32_t totalActEnergy;// mode 2
	uint8_t hrm_afe_state;  // mode 2
	uint8_t is_high_motion;	// mode 2

	struct{
		struct{
			uint16_t is_led_curr_update_requested : 1;
			uint16_t requested_led_curr : 15;
		} led_curr;

		struct{
			uint8_t is_int_time_update_requested : 1;
			uint8_t requested_int_time : 7;
		} int_time;

		struct{
			uint8_t is_averaging_update_requested : 1;
			uint8_t requested_avg : 7;
		} sample_averaging;

		struct{
			uint8_t is_dac_offset_update_requested : 1;
			uint8_t dac_offset : 7;
		} dac_offset;
	} afe_requests[SH_PPG_SIGNAL_MAX];

} whrm_wspo2_suite_sensorhub_data;

typedef struct __attribute__((packed)){
	max86178_data ppg_data;
	accel_data acc_data;
	whrm_wspo2_suite_sensorhub_data algo_data;
} sensorhub_output;

/**
 * @brief	function to initialize sensorhub mode
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sensorhub_interface_init();

/**
 * @brief	function to enable sensors in sensorhub mode
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sensorhub_enable_sensors();

/**
 * @brief	function to disable sensors in sensorhub mode
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sensorhub_disable_sensor();

/**
 * @brief	function to enable algorithm in sensorhub mode
 *
 * @param[in]  mode   - sensorhub reporting mode
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sensorhub_enable_algo(sensorhub_report_mode_t mode);

/**
 * @brief	function to disable algorithm in sensorhub mode
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sensorhub_disable_algo();

/**
 * @brief	function to get number of waiting samples in the output FIFO of sensorhub
 *
 * @param[out]  p_number_of_sample   - will keep the number of sample waiting to be read in the output FIFO of sensorhub
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sensorhub_get_output_sample_number(int * p_number_of_sample);

/**
 * @brief	function to read one samples from the output FIFO of sensorhub
 *
 * @param[out]  p_result   - will keep an output sample of sensorhub
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sensorhub_get_result(sensorhub_output *  p_result);

/**
 * @brief	function to read algorithm version in the sensorhub
 *
 * @param[out]  version   - Version of the algorithm
 * 							version[0]: Major version number
 * 							version[1]: Minor version number
 * 							version[2]: Patch version number
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sensorhub_get_version(uint8_t algoVersion[3]);

int sensorhub_reset_sensor_configuration();


#endif /* ALGOHUB_SENSORHUB_API_H_ */
