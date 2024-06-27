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

#ifndef ALGOHUB_SENSORHUB_CONFIG_API_H_
#define ALGOHUB_SENSORHUB_CONFIG_API_H_

/**
 * @brief	function to set Algorithm operation mode for Sensorhub
 *
 * @param[in]  algoMode   - Algorithm mode
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_algomode( const uint8_t algoMode );

/**
 * @brief	function to get Algorithm operation mode for Sensorhub
 *
 * @param[out]  algoMode   - Algorithm mode.
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_algomode( uint8_t *algoMode );

/**
 * @brief	function to enable/disable AFE controller mode for Sensorhub
 *
 * @param[in]  isAfecEnable	-	AFE enable flag.
 * 								0 : Disable AFE
 * 								1: Enable AFE
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_afeenable( const uint8_t isAecEnable );

/**
 * @brief	function to get AFE controller state for Sensorhub
 *
 * @param[out]  isAfecEnable -	Keeps the information that shows AFE is enabled or not.
 * 								0 : AFE is disabled
 * 								1 : AFE is enabled
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_afeenable( uint8_t *isAecEnable );

/**
 * @brief	function to enable/disable SCD controller mode for Sensorhub
 *
 * @param[in]  isScdcEnable	-	SDC enable flag.
 * 								0 : Disable SCD
 * 								1: Enable SCD
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_scdenable( const uint8_t isScdcEnable );

/**
 * @brief	function to get SCD controller state for Sensorhub
 *
 * @param[out]  isScdEnable -	Keeps the information that shows SCD is enabled or not.
 * 								0 : SCD is disabled
 * 								1 : SCD is enabled
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_scdenable( uint8_t *isScdEnable );

/**
 * @brief	function to set Target PD Current for Sensorhub
 *
 * @param[in]  targetPdCurr_x1000	-	as a multiple of full ADC range  - for HRM measurement channel
 * 										If full ADC range is 32uA and you want to set target pd current to 12uA.
 * 										targetPdCurr_x1000 must be equal to (12.0/32.0)*1000 = 375
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_targetpdcurrent(const uint16_t targetPdCurr_x10);

/**
 * @brief	function to get Target PD Current for Sensorhub
 *
 * @param[out]  targetPdCurr_x1000	-	as a multiple of full ADC range  - for HRM measurement channel
 * 										If full ADC range is 32uA and targetPdCurr_x1000 equal to 375.
 * 										Target PD current equal to ((375 / 1000) * 32) = 12
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_targetpdcurrent( uint16_t *targetPdCurr_x10);

/**
 * @brief	function to set Minimum PD Current for Sensorhub
 *
 * @param[in]  minPdCurr_x1000		-	as a multiple of full ADC range  - for HRM measurement channel
 * 										If full ADC range is 32uA and you want to set minimum pd current to 12uA.
 * 										minPdCurr_x1000 must be equal to (12.0/32.0)*1000 = 375
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_minpdcurrent(const uint16_t minPdCurr_x10);

/**
 * @brief	function to get Minimum PD Current for Sensorhub
 *
 * @param[out]  minPdCurr_x1000	-	as a multiple of full ADC range  - for HRM measurement channel
 * 										If full ADC range is 32uA and minimum equal to 375.
 * 										Target PD current equal to ((375 / 1000) * 32) = 12
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_minpdcurrent( uint16_t *minPdCurr_x10);

/**
 * @brief	function to set Initial PD Current for Sensorhub
 *
 * @param[in]  initPdCurr_x1000		-	as a multiple of full ADC range  - for HRM measurement channel
 * 										If full ADC range is 32uA and you want to set initial pd current to 12uA.
 * 										minPdCurr_x1000 must be equal to (12.0/32.0)*1000 = 375
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_initialpdcurrent(const uint16_t initPdCurr_x10);

/**
 * @brief	function to get Initial PD Current for Sensorhub
 *
 * @param[out]  initPdCurr_x1000	-	as a multiple of full ADC range  - for HRM measurement channel
 * 										If full ADC range is 32uA and initial equal to 375.
 * 										Target PD current equal to ((375 / 1000) * 32) = 12
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_initialpdcurrent( uint16_t *initPdCurr_x10);

/**
 * @brief	function to enable/disable Auto PD Current controller mode for Sensorhub
 *
 * @param[in]  isAutoPdCurrEnable	-	Auto PD Current enable flag.
 * 										0 : Disable Auto PD Current controller
 * 										1 : Enable Auto PD Current controller
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_autopdcurrentenable( const uint8_t isAutoPdCurrEnable );

/**
 * @brief	function to get Auto PD Current controller state for Sensorhub
 *
 * @param[out]  isAutoPdCurrEnable	 -	Keeps the information that shows auto current controller is enabled or not.
 * 										0 : Auto current controller is disabled
 * 										1 : Auto current controller is enabled
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_autopdcurrentenable( uint8_t *isAutoPdCurrEnable );

/**
 * @brief	function to set SpO2 Calibration Coefficients for Sensorhub
 *
 * @param[in]  val					-	val[0] represents SpO2 Calibration Coefficient A
 *										val[1] represents SpO2 Calibration Coefficient B
 *										val[2] represents SpO2 Calibration Coefficient C
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_spo2cal( const uint32_t val[3] );

/**
 * @brief	function to get SpO2 Calibration Coefficients for Sensorhub
 *
 * @param[out]  val					-	val[0] represents SpO2 Calibration Coefficient A
 *										val[1] represents SpO2 Calibration Coefficient B
 *										val[2] represents SpO2 Calibration Coefficient C
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_spo2cal( int32_t val[3]);

/**
 * @brief	function to set Motion Threshold for Sensorhub
 *
 * @param[in]  motionThreah_x1000	-	16-bit unsigned, 0.001g
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_motionthreshold( const uint16_t motionThresh);

/**
 * @brief	function to get Motion Threshold for Sensorhub
 *
 * @param[out]  motionThreah_x1000	-	16-bit unsigned, 0.001g
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_motionthreshold( uint16_t *motionThresh);

/**
 * @brief	function to set Target PD Period for Sensorhub
 *
 * @param[in]  targPdPeriod			-	Adjusted target PD current period in seconds
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_targetpdperiod( const uint16_t targPdPeriod);

/**
 * @brief	function to get Target PD Period for Sensorhub
 *
 * @param[out]  targPdPeriod		-	Adjusted target PD current period in seconds
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_targetpdperiod( uint16_t *targPdPeriod);

/**
 * @brief	function to set SpO2 Motion Period for Sensorhub
 *
 * @param[in]  spo2MotnPeriod			-
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_spo2motionperiod( const uint16_t spo2MotnPeriod);

/**
 * @brief	function to get SpO2 Motion Period for Sensorhub
 *
 * @param[out]  spo2MotnPeriod			-
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_spo2motionperiod( uint16_t *spo2MotnPeriod);

/**
 * @brief	function to set SpO2 Motion Threshold for Sensorhub
 *
 * @param[in]  spo2MotnThresh			-	Equal to 105 x milli-g threshold value
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_spo2motionthreshold( const uint32_t spo2MotnThresh );

/**
 * @brief	function to get SpO2 Motion Threshold for Sensorhub
 *
 * @param[out]  spo2MotnThresh			-	Equal to 105 x milli-g threshold value
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_spo2motionthreshold( uint32_t *spo2MotnThresh );

/**
 * @brief	function to set SpO2 AFE Controller Timeout for Sensorhub
 *
 * @param[in]  spo2AfeCtrlTimeout			-	SpO2 AGC Timeout (sec)
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_spo2afecontrltimeout( const uint8_t spo2AfeCtrlTimeout );

/**
 * @brief	function to get SpO2 AFE Controller Timeout for Sensorhub
 *
 * @param[out]  spo2AfeCtrlTimeout			-	SpO2 AGC Timeout (sec)
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_spo2afecontrltimeout( uint8_t *spo2AfeCtrlTimeout );

/**
 * @brief	function to set SpO2 Timeout for Sensorhub
 *
 * @param[in]  spo2AlgoTimeout			-	Timeout duration for SpO2 measurement in seconds
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_spo2timeout( const uint8_t spo2AlgoTimeout );

/**
 * @brief	function to get SpO2 Timeout for Sensorhub
 *
 * @param[out]  spo2AlgoTimeout			-	Timeout duration for SpO2 measurement in seconds
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_spo2timeout( uint8_t *spo2AlgoTimeout );

/**
 * @brief	function to set Initial Heart Rate for Sensorhub
 *
 * @param[in]  initialHr			-	Initial HR  algorithm value
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_initialhr( const uint8_t initialHr );

/**
 * @brief	function to get Initial Heart Rate for Sensorhub
 *
 * @param[out]  initialHr			-	Initial HR  algorithm value
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_initialhr( uint8_t *initialHr );

/**
 * @brief	function to set Initial Person Height for Sensorhub
 *
 * @param[in]  personHeight			-	Initial height in unit of centimeter
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_personheight( const uint16_t personHeight);

/**
 * @brief	function to get Initial Person Height for Sensorhub
 *
 * @param[out]  personHeight			-	Initial height in unit of centimeter
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_personheight( uint16_t *personHeight);

/**
 * @brief	function to set Initial Person Weight for Sensorhub
 *
 * @param[in]  personWeight			-	Initial Weight in unit of kilogram
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_personweight( const uint16_t personWeight);

/**
 * @brief	function to get Initial Person Weight for Sensorhub
 *
 * @param[out]  personWeight			-	Initial Weight in unit of kilogram
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_personweight( uint16_t *personWeight);

/**
 * @brief	function to set Initial Person Age for Sensorhub
 *
 * @param[in]  personAge			-	Initial Age
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_personage( const uint8_t personAge );

/**
 * @brief	function to get Initial Person Age for Sensorhub
 *
 * @param[out]  personAge			-	Initial Age
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_personage( uint8_t *personAge );

/**
 * @brief	function to set Initial Person Gender for Sensorhub
 *
 * @param[in]  personAge			-	Initial Gender
 *									-	0 : Male
 *									-	1 : Female
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_persongender( const uint8_t personGender );

/**
 * @brief	function to get Initial Person Gender for Sensorhub
 *
 * @param[out]  personAge			-	Initial Gender
 *									-	0 : Male
 *									-	1 : Female
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_persongender( uint8_t *personGender );

/**
 * @brief	function to set minimum integration time for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  minIntegrationTimeOpt	-	Minimum AFE integration time
 * 											0x00: 14.8us
 *											0x01: 29.4us
 *											0x02: 58.7us
 *											0x03: 117.3us
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_mintintoption( const uint8_t measChan, const uint8_t minIntegrationTimeOpt );

/**
 * @brief	function to get minimum integration time for Sensorhub
 *
 * @param[out]  minIntegrationTimeOpt[9]-	Minimum AFE integratio time  for all measurement channels
 * 											0x00: 14.8us
 *											0x01: 29.4us
 *											0x02: 58.7us
 *											0x03: 117.3us
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_mintintoption( uint8_t minIntegrationTimeOpt[9] );

/**
 * @brief	function to set maximum integration time for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  maxIntegrationTimeOpt	-	Maximum AFE integration time
 * 											0x00: 14.8us
 *											0x01: 29.4us
 *											0x02: 58.7us
 *											0x03: 117.3us
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_maxtintoption( const uint8_t measChan, const uint8_t maxIntegrationTimeOpt );

/**
 * @brief	function to get maximum integration time for Sensorhub
 *
 * @param[out]  maxIntegrationTimeOpt[9]-	Maximum AFE Integration time  for all measurement channels
 * 											0x00: 14.8us
 *											0x01: 29.4us
 *											0x02: 58.7us
 *											0x03: 117.3us
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_maxtintoption( uint8_t maxIntegrationTimeOpt[9] );

/**
 * @brief	function to set minimum sampling rate for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  minSampRateAveragingOpt	-	Minimum AFE sampling rate
 *											0x00: 25sps, avg = 1
 *											0x01: 50sps, avg = 2
 *											0x02: 100sps, avg = 4
 *											0x03: 200sps, avg = 8
 *											0x04: 400sps, avg = 16
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_minfsmpoption( const uint8_t measChan, const uint8_t minSampRateAveragingOpt );

/**
 * @brief	function to set minimum sampling rate for Sensorhub
 *
 * @param[out]  minSampRateAveragingOpt	-	Minimum AFE sampling rate for all measurement channels
 *											0x00: 25sps, avg = 1
 *											0x01: 50sps, avg = 2
 *											0x02: 100sps, avg = 4
 *											0x03: 200sps, avg = 8
 *											0x04: 400sps, avg = 16
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_minfsmpoption( uint8_t minSampRateAveragingOpt[9] );

/**
 * @brief	function to set maximum sampling rate for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  maxSampRateAveragingOpt	-	Maximum AFE sampling rate
 *											0x00: 25sps, avg = 1
 *											0x01: 50sps, avg = 2
 *											0x02: 100sps, avg = 4
 *											0x03: 200sps, avg = 8
 *											0x04: 400sps, avg = 16
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_maxfsmpoption( const uint8_t measChan, const uint8_t maxSampRateAveragingOpt );

/**
 * @brief	function to get maximum sampling rate for Sensorhub
 *
 * @param[out]  maxSampRateAveragingOpt	-	Maximum AFE sampling rate  for all measurement channels
 *											0x00: 25sps, avg = 1
 *											0x01: 50sps, avg = 2
 *											0x02: 100sps, avg = 4
 *											0x03: 200sps, avg = 8
 *											0x04: 400sps, avg = 16
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_maxfsmpoption( uint8_t maxSampRateAveragingOpt[9] );

/**
 * @brief	function to set maximum DAC offset for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  maxDacOffset				-	Maximum AFE DAC offset
 *											0x00: 0 uA
 *											0x01: 8 uA
 *											0x02: 16 uA
 *											0x03: 24 uA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_maxdacoffset( const uint8_t measChan, const uint8_t maxDacOffset );

/**
 * @brief	function to get maximum DAC offset for Sensorhub
 *
 * @param[out]  maxDacOffset			-	Maximum AFE DAC offset  for all measurement channels
 *											0x00: 0 uA
 *											0x01: 8 uA
 *											0x02: 16 uA
 *											0x03: 24 uA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_maxdacoffset( uint8_t maxDacOffsetOpt[9] );

/**
 * @brief	function to set minimum LED current for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  minLedCurr_x10			-	Minimum led current in units of 0.1mA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_minledcurr( const uint8_t measChan, const uint16_t minLedCurr_x10 );

/**
 * @brief	function to get minimum LED current for Sensorhub

 * @param[out]  minLedCurr_x10			-	Minimum led current in units of 0.1mA for all measurement channels
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_minledcurr( uint16_t minLedCurr_x10[9] );

/**
 * @brief	function to set maximum LED current for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  maxLedCurr_x10			-	Maximum led current in units of 0.1mA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_maxledcurr(const uint8_t measChan, const uint16_t maxLedCurr_x10);

/**
 * @brief	function to get maximum LED current for Sensorhub
 *
 * @param[out]  maxLedCurr_x10			-	Maximum led current in units of 0.1mA for all measurement channels
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_maxledcurr( uint16_t maxLedCurr_x10[9] );

/**
 * @brief	function to set maximum LED current step for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  minLedCurrStep_x10		-	Minimum led current step in units of 0.1mA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_minledcurrstep( const uint8_t measChan, const uint16_t minLedCurrStep_x10 );

/**
 * @brief	function to get maximum LED current step for Sensorhub
 *
 * @param[out]  minLedCurrStep_x10		-	Minimum led current step in units of 0.1mA for all measurement channels
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_minledcurrstep( uint16_t minLedCurrStep_x10[9] );

/**
 * @brief	function to set master channel selection for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  masterchsel				-	Master channel selection
 * 											0x00: Selects PD1 as a master
 * 											0x01: Selects PD2 as a master
 * 											0x02: Selects master channel automatically
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_masterchsel( const uint8_t measChan, const uint8_t masterchsel);

/**
 * @brief	function to get master channel selection for Sensorhub
 *
 * @param[out]  masterchsel				-	Master channel selection for all measurement channels
 * 											0x00: Selects PD1 as a master
 * 											0x01: Selects PD2 as a master
 * 											0x02: Selects master channel automatically
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_masterchsel( uint8_t masterchsel[9] );

/**
 * @brief	function to set full scale PD current for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  fullScalePdCurr			-	Full scale PD current
 * 											0x00: 4 uA
 * 											0x01: 8 uA
 * 											0x02: 16 uA
 *	 										0x03: 32 uA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_fulscalepdcurr( const uint8_t measChan, const uint8_t fullScalePdCurr );

/**
 * @brief	function to get full scale PD current for Sensorhub
 *
 * @param[in]  fullScalePdCurr			-	Full scale PD current for all measurement channels
 * 											0x00: 4 uA
 * 											0x01: 8 uA
 * 											0x02: 16 uA
 *	 										0x03: 32 uA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_fulscalepdcurr( uint8_t fullScalePdCurr[9] );

/**
 * @brief	function to set AFE type for Sensorhub
 *
 * @param[in]  afeType					-	AFE Type Selection
 * 											0x00: AFE that allows independent integration time and sampling average settings for different measurements, e.g., MAX86171.
 * 											0x01: AFE that uses shared integration time and sampling average settings for different measurements
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_afetype( const uint8_t afeType );

/**
 * @brief	function to set AFE type for Sensorhub
 *
 * @param[out]  afeType					-	AFE Type Selection
 * 											0x00: AFE that allows independent integration time and sampling average settings for different measurements, e.g., MAX86171.
 * 											0x01: AFE that uses shared integration time and sampling average settings for different measurements
 *
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_afetype( uint8_t * afeType );

/**
 * @brief	function to set initial integration time for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  initTintTime				-	Initial integration time
 *											0x00: 14.8us
 *											0x01: 29.4us
 *											0x02: 58.7us
 *											0x03: 117.3us
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_initintoption( const uint8_t measChan, const uint8_t initTintTime );

/**
 * @brief	function to get initial integration time for Sensorhub
 *
 * @param[out]  initTintTime				-	Initial integration time for all measurement channels
 *											0x00: 14.8us
 *											0x01: 29.4us
 *											0x02: 58.7us
 *											0x03: 117.3us
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_inittintoption( uint8_t initTintTime[9] );

/**
 * @brief	function to set initial sampling rate time for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  initSampRateAveragingOpt	-	Initial integration time
 * 											0x00: 25sps, avg = 1
 *											0x01: 50sps, avg = 2
 *											0x02: 100sps, avg = 4
 *											0x03: 200sps, avg = 8
 *											0x04: 400sps, avg = 16
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_initfsmpoption( const uint8_t measChan, const uint8_t initSampRateAveragingOpt );

/**
 * @brief	function to get initial sampling rate time for Sensorhub
 *
 * @param[out]  initSampRateAveragingOpt	-	Initial integration time for all measurement channels
 * 												0x00: 25sps, avg = 1
 *												0x01: 50sps, avg = 2
 *												0x02: 100sps, avg = 4
 *												0x03: 200sps, avg = 8
 *												0x04: 400sps, avg = 16
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_initfsmpoption( uint8_t initSampRateAveragingOpt[9] );

/**
 * @brief	function to set initial DAC Offset for PPG1 for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  initDacOffsetPPG1		-	Initial DAC Offset for PPG1
 * 											0x00: 0 uA
 *											0x01: 8 uA
 *											0x02: 16 uA
 *											0x03: 24 uA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_initdacoffppg1( const uint8_t measChan, const uint8_t initDacOffsetPPG1 );

/**
 * @brief	function to get initial DAC Offset for PPG1 for Sensorhub
 *
 * @param[out]  initDacOffsetPPG1		-	Initial DAC Offset for PPG1 for all measurement channels
 * 											0x00: 0 uA
 *											0x01: 8 uA
 *											0x02: 16 uA
 *											0x03: 24 uA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_initdacoffppg1(uint8_t initDacOffsetPPG1[9] );

/**
 * @brief	function to set initial DAC Offset for PPG2 for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  initDacOffsetPPG2		-	Initial DAC Offset for PPG1
 * 											0x00: 0 uA
 *											0x01: 8 uA
 *											0x02: 16 uA
 *											0x03: 24 uA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_initdacoffppg2( const uint8_t measChan, const uint8_t initDacOffsetPPG2 );

/**
 * @brief	function to get initial DAC Offset for PPG2 for Sensorhub
 *
 * @param[out]  initDacOffsetPPG2		-	Initial DAC Offset for PPG2 for all measurement channels
 * 											0x00: 0 uA
 *											0x01: 8 uA
 *											0x02: 16 uA
 *											0x03: 24 uA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_initdacoffppg2(uint8_t initDacOffsetPPG2[9] );

/**
 * @brief	function to set initial LED Current for Sensorhub
 *
 * @param[in]  measChan					-	Measurement Channel index between 0 and 8
 * @param[in]  initLedCurr				-	Initial LED Current in units of 0.1mA
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_initledcurr( const uint8_t measChan, const uint16_t initLedCurr );

/**
 * @brief	function to get initial LED Current for Sensorhub
 *
 * @param[out]  initLedCurr				-	Initial LED Current in units of 0.1mA for all measurement channel
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_initledcurr( uint16_t initLedCurr[9] );

/**
 * @brief	function to set whrm LED PD configuration for Sensorhub
 *
 * @param[in]  whrmledpdconfig			-	WHRM LED PD Configuration
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_whrmledpdconfig( const uint16_t whrmledpdconfig );

/**
 * @brief	function to get whrm LED PD configuration for Sensorhub
 *
 * @param[out]  whrmledpdconfig			-	WHRM LED PD Configuration
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_whrmledpdconfig( uint16_t *whrmledpdconfig);

/**
 * @brief	function to set SpO2 LED PD configuration for Sensorhub
 *
 * @param[in]  spo2ledpdconfig			-	SpO2 LED PD Configuration
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_spo2ledpdconfig( const uint16_t spo2ledpdconfig );

/**
 * @brief	function to get SpO2 LED PD configuration for Sensorhub
 *
 * @param[out]  spo2ledpdconfig			-	SpO2 LED PD Configuration
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_spo2ledpdconfig( uint16_t *spo2ledpdconfig);

/**
 * @brief	function to set LED Drirver Selection for Sensorhub
 *
 * @param[in]  measChan			-	Measurement Channel index between 0 and 8
 * @param[in]  driverLedSel		-	Driver LED Selection
 * 									0x00: LED Driver A will fire LED1
 * 									0x01: LED Driver A will fire LED1
 * 									0x02: LED Driver A will fire LED1
 * 									0x03: LED Driver A will fire LED1
 * 									0x04: LED Driver A will fire LED1
 * 									0x05: LED Driver A will fire LED1
 * 									0x06 -0xXF Do not use
 *
 * 									0x00: LED Driver B will fire LED1
 * 									0x10: LED Driver B will fire LED1
 * 									0x20: LED Driver B will fire LED1
 * 									0x30: LED Driver B will fire LED1
 * 									0x40: LED Driver B will fire LED1
 * 									0x50: LED Driver B will fire LED1
 * 									0x60 -0xXF Do not use
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_set_cfg_wearablesuite_meas_led_driversel(const uint8_t measChan, const uint8_t driverLedSel );

/**
 * @brief	function to set LED Drirver Selection for Sensorhub
 *
 * @param[in]  driverLedSel		-	Driver Selection for all measurement channels
 *
 * @return	1 byte status (SS_STATUS) : 0x00 (SS_SUCCESS) on success
 */
int sh_get_cfg_wearablesuite_meas_led_driversel( uint8_t driverLedSel[9] );

#endif /* ALGOHUB_SENSORHUB_CONFIG_API_H_ */
