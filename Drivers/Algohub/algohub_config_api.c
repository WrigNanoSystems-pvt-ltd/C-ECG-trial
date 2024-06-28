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
#include <string.h>
#include <stdint.h>

#include "algohub_config_api.h"
#include "sh_comm.h"


#define MIN_MACRO(a,b) ((a)<(b)?(a):(b))



int ah_set_cfg_wearablesuite_algomode( const uint8_t algoMode ){

	uint8_t Temp[1] = { algoMode };
	int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_ALGO_MODE, &Temp[0], 1);
	return status;
}

int ah_get_cfg_wearablesuite_algomode( uint8_t *algoMode ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_ALGO_MODE, &rxBuff[0], sizeof(rxBuff) );
	*algoMode =  rxBuff[1];
	return status;
}


int ah_set_cfg_wearablesuite_afeenable(const uint8_t isAfeEnable ){

	uint8_t Temp[1] = { isAfeEnable };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_AEC_ENABLE, &Temp[0], 1);
	return status;

}

int ah_get_cfg_wearablesuite_afeenable( uint8_t *isAfecEnable ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_AEC_ENABLE, &rxBuff[0], sizeof(rxBuff) );
	*isAfecEnable =  rxBuff[1];

	return status;
}

int ah_set_cfg_wearablesuite_scdenable( const uint8_t isScdcEnable ){

	uint8_t Temp[1] = { isScdcEnable };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_SCD_ENABLE, &Temp[0], 1);
	return status;

}

int ah_get_cfg_wearablesuite_scdenable( uint8_t *isScdEnable ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_SCD_ENABLE, &rxBuff[0], sizeof(rxBuff) );
	*isScdEnable =  rxBuff[1];

	return status;
}


int ah_set_cfg_wearablesuite_targetpdcurrent(const uint16_t targetPdCurr_x1000){


	uint8_t Temp[2] = { (uint8_t)((targetPdCurr_x1000 >> (1*8)) & 0xFF),  (uint8_t)((targetPdCurr_x1000 >> (0*8)) & 0xFF)};
	int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_targetpdcurrent( uint16_t *targetPdCurr_x1000){


	uint8_t rxBuff[2+1];  // first byte is status
    int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT, &rxBuff[0], sizeof(rxBuff));
    *targetPdCurr_x1000 = (rxBuff[1] << 8) +  rxBuff[2] ;

    return status;

}

int ah_set_cfg_wearablesuite_minpdcurrent(const uint16_t minPdCurr_x1000){


	uint8_t Temp[2] = { (uint8_t)((minPdCurr_x1000 >> (1*8)) & 0xFF),  (uint8_t)((minPdCurr_x1000 >> (0*8)) & 0xFF)};
	int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_PD_CURRENT, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_minpdcurrent( uint16_t *minPdCurr_x1000){


	uint8_t rxBuff[2+1];  // first byte is status
    int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_PD_CURRENT, &rxBuff[0], sizeof(rxBuff));
    *minPdCurr_x1000 = (rxBuff[1] << 8) +  rxBuff[2] ;

    return status;

}


int ah_set_cfg_wearablesuite_initialpdcurrent(const uint16_t initPdCurr_x1000){


	uint8_t Temp[2] = { (uint8_t)((initPdCurr_x1000 >> (1*8)) & 0xFF),  (uint8_t)((initPdCurr_x1000 >> (0*8)) & 0xFF)};
	int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_INIT_PD_CURRENT, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_initialpdcurrent( uint16_t *initPdCurr_x1000){


	uint8_t rxBuff[2+1];  // first byte is status
    int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_INIT_PD_CURRENT, &rxBuff[0], sizeof(rxBuff));
    *initPdCurr_x1000 = (rxBuff[1] << 8) +  rxBuff[2] ;

    return status;

}


int ah_set_cfg_wearablesuite_autopdcurrentenable( const uint8_t isAutoPdCurrEnable ){

	uint8_t Temp[1] = { isAutoPdCurrEnable };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_AUTO_PD_CURRENT_ENABLE, &Temp[0], 1);
	return status;

}

int ah_get_cfg_wearablesuite_autopdcurrentenable( uint8_t *isAutoPdCurrEnable ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_AUTO_PD_CURRENT_ENABLE, &rxBuff[0], sizeof(rxBuff) );
	*isAutoPdCurrEnable =  rxBuff[1];

	return status;
}

int ah_set_cfg_wearablesuite_spo2cal( const uint32_t val[3] ){


	uint8_t CalCoef[12] = { (uint8_t)((val[0] >> (3*8)) & 0xFF),  (uint8_t)((val[0] >> (2*8)) & 0xFF), (uint8_t)((val[0] >> (1*8)) & 0xFF), (uint8_t)((val[0] >> (0*8)) & 0xFF), // A
							(uint8_t)((val[1] >> (3*8)) & 0xFF),  (uint8_t)((val[1] >> (2*8)) & 0xFF), (uint8_t)((val[1] >> (1*8)) & 0xFF), (uint8_t)((val[1] >> (0*8)) & 0xFF), // B
							(uint8_t)((val[2] >> (3*8)) & 0xFF),  (uint8_t)((val[2] >> (2*8)) & 0xFF), (uint8_t)((val[2] >> (1*8)) & 0xFF), (uint8_t)((val[2] >> (0*8)) & 0xFF)  // C
						   };

	int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2_CAL, &CalCoef[0], 12);

	return status;
}

int ah_get_cfg_wearablesuite_spo2cal( int32_t val[3]){

	uint8_t rxBuff[12+1];  // first byte is status
    int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2_CAL, &rxBuff[0], sizeof(rxBuff));

    val[0] = (rxBuff[1] << 24) + (rxBuff[2] << 16) + (rxBuff[3] << 8) + (rxBuff[4] );
    val[1] = (rxBuff[5] << 24) + (rxBuff[6] << 16) + (rxBuff[7] << 8) + (rxBuff[8] );
    val[2] = (rxBuff[9] << 24) + (rxBuff[10] << 16) + (rxBuff[11] << 8) + (rxBuff[12] );

	return status;
}


/*  WARNING: GRANULARITY SHOULD BE 0.01 WILL TALK TO AFSHIN!!!!*/
int ah_set_cfg_wearablesuite_motionthreshold( const uint16_t motionThreah_x1000){

    uint8_t Temp[2] = { (uint8_t)((motionThreah_x1000 >> (1*8)) & 0xFF),  (uint8_t)((motionThreah_x1000 >> (0*8)) & 0xFF)};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_MOTION_MAG_THRESHOLD, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_motionthreshold( uint16_t *motionThreah_x1000){

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_MOTION_MAG_THRESHOLD, &rxBuff[0], sizeof(rxBuff));
	*motionThreah_x1000 = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}

int ah_set_cfg_wearablesuite_targetpdperiod( const uint16_t targPdPeriod){

    uint8_t Temp[2] = { (uint8_t)((targPdPeriod >> (1*8)) & 0xFF),  (uint8_t)((targPdPeriod >> (0*8)) & 0xFF)};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT_PERIOD, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_targetpdperiod( uint16_t *targPdPeriod){

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT_PERIOD, &rxBuff[0], sizeof(rxBuff));
	*targPdPeriod = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}


int ah_set_cfg_wearablesuite_spo2motionperiod( const uint16_t spo2MotnPeriod){

    uint8_t Temp[2] = { (uint8_t)((spo2MotnPeriod >> (1*8)) & 0xFF),  (uint8_t)((spo2MotnPeriod >> (0*8)) & 0xFF)};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_PERIOD, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_spo2motionperiod( uint16_t *spo2MotnPeriod){

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_PERIOD, &rxBuff[0], sizeof(rxBuff));
	*spo2MotnPeriod = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}

int ah_set_cfg_wearablesuite_spo2motionthreshold( const uint32_t spo2MotnThresh ){


	uint8_t Temp[4] = { (uint8_t)((spo2MotnThresh >> (3*8)) & 0xFF),  (uint8_t)((spo2MotnThresh >> (2*8)) & 0xFF),
			            (uint8_t)((spo2MotnThresh >> (1*8)) & 0xFF), (uint8_t)((spo2MotnThresh >> (0*8)) & 0xFF)   };

	int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_THRESHOLD, &Temp[0], 4);

	return status;

}

int ah_get_cfg_wearablesuite_spo2motionthreshold( uint32_t *spo2MotnThresh){

	uint8_t rxBuff[4+1];  // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_THRESHOLD, &rxBuff[0], sizeof(rxBuff));
    *spo2MotnThresh = (rxBuff[1] << 24) + (rxBuff[2] << 16) + (rxBuff[3] << 8) + (rxBuff[4]);

    return status;

}


int ah_set_cfg_wearablesuite_spo2afecontrltimeout( const uint8_t spo2AfeCtrlTimeout ){

	uint8_t Temp[1] = { spo2AfeCtrlTimeout };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_AFE_TIMEOUT, &Temp[0], 1);
	return status;

}

int ah_get_cfg_wearablesuite_spo2afecontrltimeout( uint8_t *spo2AfeCtrlTimeout ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_AFE_TIMEOUT, &rxBuff[0], sizeof(rxBuff) );
	*spo2AfeCtrlTimeout =  rxBuff[1];

	return status;
}


int ah_set_cfg_wearablesuite_spo2timeout( const uint8_t spo2AlgoTimeout ){

	uint8_t Temp[1] = { spo2AlgoTimeout };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_TIMEOUT, &Temp[0], 1);

	return status;

}

int ah_get_cfg_wearablesuite_spo2timeout( uint8_t *spo2AlgoTimeout ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_TIMEOUT, &rxBuff[0], sizeof(rxBuff) );
	*spo2AlgoTimeout =  rxBuff[1];

	return status;
}

int ah_set_cfg_wearablesuite_initialhr( const uint8_t initialHr ){

	uint8_t Temp[1] = { initialHr };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_HR, &Temp[0], 1);

	return status;

}

int ah_get_cfg_wearablesuite_initialhr( uint8_t *initialHr ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_HR, &rxBuff[0], sizeof(rxBuff) );
	*initialHr =  rxBuff[1];

	return status;
}


int ah_set_cfg_wearablesuite_personheight( const uint16_t personHeight){

    uint8_t Temp[2] = { (uint8_t)((personHeight >> (1*8)) & 0xFF),  (uint8_t)((personHeight >> (0*8)) & 0xFF)};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_HEIGHT, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_personheight( uint16_t *personHeight){

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_HEIGHT, &rxBuff[0], sizeof(rxBuff));
	*personHeight = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}


int ah_set_cfg_wearablesuite_personweight( const uint16_t personWeight){

    uint8_t Temp[2] = { (uint8_t)((personWeight >> (1*8)) & 0xFF),  (uint8_t)((personWeight >> (0*8)) & 0xFF)};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_WEIGHT, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_personweight( uint16_t *personWeight){

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_WEIGHT, &rxBuff[0], sizeof(rxBuff));
	*personWeight = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}

int ah_set_cfg_wearablesuite_personage( const uint8_t personAge ){

	uint8_t Temp[1] = { personAge };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_AGE, &Temp[0], 1);

	return status;

}

int ah_get_cfg_wearablesuite_personage( uint8_t *personAge ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_AGE, &rxBuff[0], sizeof(rxBuff) );
	*personAge =  rxBuff[1];

	return status;
}

int ah_set_cfg_wearablesuite_persongender( const uint8_t personGender ){

	uint8_t Temp[1] = { personGender };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_GENDER, &Temp[0], 1);

	return status;

}

int ah_get_cfg_wearablesuite_persongender( uint8_t *personGender ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_GENDER, &rxBuff[0], sizeof(rxBuff) );
	*personGender =  rxBuff[1];

	return status;
}


int ah_set_cfg_wearablesuite_mintintoption( const uint8_t measChan, const uint8_t minIntegrationTimeOpt ){

	uint8_t Temp[2] = { measChan, minIntegrationTimeOpt };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_INTEGRATION_TIME, &Temp[0], 2);

	return status;

}

int ah_get_cfg_wearablesuite_mintintoption( uint8_t minIntegrationTimeOpt[9] ){

	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_INTEGRATION_TIME, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; ++index)
		minIntegrationTimeOpt[index] =  rxBuff[1 + index];

	return status;
}

int ah_set_cfg_wearablesuite_maxtintoption( const uint8_t measChan, const uint8_t maxIntegrationTimeOpt ){

	uint8_t Temp[2] = { measChan, maxIntegrationTimeOpt };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_INTEGRATION_TIME, &Temp[0], 2);

	return status;

}

int ah_get_cfg_wearablesuite_maxtintoption( uint8_t maxIntegrationTimeOpt[9] ){

	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_INTEGRATION_TIME, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; ++index)
		maxIntegrationTimeOpt[index] =  rxBuff[1 + index];

	return status;
}


int ah_set_cfg_wearablesuite_minfsmpoption( const uint8_t measChan, const uint8_t minSampRateAveragingOpt ){

	uint8_t Temp[2] = { measChan, minSampRateAveragingOpt };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_SAMPLING_AVERAGE, &Temp[0], 2);

	return status;

}

int ah_get_cfg_wearablesuite_minfsmpoption( uint8_t minSampRateAveragingOpt[9] ){

	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_SAMPLING_AVERAGE, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; ++index)
		minSampRateAveragingOpt[index] =  rxBuff[1 + index];

	return status;
}


int ah_set_cfg_wearablesuite_maxfsmpoption( const uint8_t measChan, const uint8_t maxSampRateAveragingOpt ){

	uint8_t Temp[2] = { measChan, maxSampRateAveragingOpt };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_SAMPLING_AVERAGE, &Temp[0], 2);

	return status;

}

int ah_get_cfg_wearablesuite_maxfsmpoption( uint8_t maxSampRateAveragingOpt[9] ){

	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_SAMPLING_AVERAGE, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; ++index)
		maxSampRateAveragingOpt[index] =  rxBuff[1 + index];

	return status;
}

int ah_set_cfg_wearablesuite_maxdacoffset( const uint8_t measChan, const uint8_t maxDacOffset ){
	uint8_t Temp[2] = { measChan, maxDacOffset };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_D_OFT_OPTION, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_maxdacoffset( uint8_t maxDacOffsetOpt[9] ){
	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_D_OFT_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; ++index)
		maxDacOffsetOpt[index] =  rxBuff[1 + index];

	return status;
}

int ah_set_cfg_wearablesuite_minledcurr( const uint8_t measChan, const uint16_t minLedCurr_x10 ){
	uint8_t Temp[3] = { measChan, (uint8_t)(minLedCurr_x10 >> 8), (uint8_t)(minLedCurr_x10) };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_LED_CURR_OPTION, &Temp[0], 3);

	return status;
}

int ah_get_cfg_wearablesuite_minledcurr( uint16_t minLedCurr_x10[9] ){
	uint8_t rxBuff[1+18]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_LED_CURR_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; index++)
		minLedCurr_x10[index] =  (((uint16_t)rxBuff[(2 * index) + 1] << 8 ) | ((uint16_t)rxBuff[(2 * index) + 2]));


	return status;
}

int ah_set_cfg_wearablesuite_maxledcurr(const uint8_t measChan, const uint16_t maxLedCurr_x10)
{
	uint8_t Temp[3] = { measChan, (uint8_t)(maxLedCurr_x10 >> 8), (uint8_t)(maxLedCurr_x10) };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_LED_CURR_OPTION, &Temp[0], 3);

	return status;
}

int ah_get_cfg_wearablesuite_maxledcurr( uint16_t maxLedCurr_x10[9] ){
	uint8_t rxBuff[1+18]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_LED_CURR_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; index++)
		maxLedCurr_x10[index] =  (((uint16_t)rxBuff[(2 * index) + 1] << 8 ) | ((uint16_t)rxBuff[(2 * index) + 2]));


	return status;
}

int ah_set_cfg_wearablesuite_minledcurrstep( const uint8_t measChan, const uint16_t minLedCurrStep_x10 ){
	uint8_t Temp[3] = { measChan, (uint8_t)(minLedCurrStep_x10 >> 8), (uint8_t)(minLedCurrStep_x10) };
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_LED_CURR_STEP_OPTION, &Temp[0], 3);

	return status;
}

int ah_get_cfg_wearablesuite_minledcurrstep( uint16_t minLedCurrStep_x10[9] ){
	uint8_t rxBuff[1+18]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_LED_CURR_STEP_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; index++)
		minLedCurrStep_x10[index] =  (((uint16_t)rxBuff[(2 * index) + 1] << 8 ) | ((uint16_t)rxBuff[(2 * index) + 2]));


	return status;
}

int ah_set_cfg_wearablesuite_masterchsel( const uint8_t measChan, const uint8_t masterchsel){
	uint8_t Temp[2] = { measChan, masterchsel};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_MASTER_CH_SEL_OPTION, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_masterchsel( uint8_t masterchsel[9] ){
	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_MASTER_CH_SEL_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; index++)
		masterchsel[index] =  rxBuff[1 + index];


	return status;
}

int ah_set_cfg_wearablesuite_fulscalepdcurr( const uint8_t measChan, const uint8_t fullScalePdCurr ){
	uint8_t Temp[2] = { measChan, fullScalePdCurr};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_FULL_SCALE_PD_CURR_OPTION, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_fulscalepdcurr( uint8_t fullScalePdCurr[9] ){
	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_FULL_SCALE_PD_CURR_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; index++)
		fullScalePdCurr[index] =  rxBuff[1 + index];


	return status;
}

int ah_set_cfg_wearablesuite_afetype( const uint8_t afeType ){
	uint8_t Temp[1] = { afeType};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_AFE_TYPE_OPTION, &Temp[0], 1);

	return status;
}

int ah_get_cfg_wearablesuite_afetype( uint8_t * afeType ){
	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_AFE_TYPE_OPTION, &rxBuff[0], sizeof(rxBuff) );

	*afeType =  rxBuff[1];

	return status;
}

int ah_set_cfg_wearablesuite_initintoption( const uint8_t measChan, const uint8_t initTintTime ){
	uint8_t Temp[2] = { measChan, initTintTime};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_T_INT_OPTION, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_inittintoption( uint8_t initTintTime[9] ){
	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_T_INT_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; index++)
		initTintTime[index] =  rxBuff[1 + index];


	return status;
}

int ah_set_cfg_wearablesuite_initfsmpoption( const uint8_t measChan, const uint8_t initSampRateAveragingOpt ){
	uint8_t Temp[2] = { measChan, initSampRateAveragingOpt};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_F_SMP_OPTION, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_initfsmpoption( uint8_t initSampRateAveragingOpt[9] ){
	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_F_SMP_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; index++)
		initSampRateAveragingOpt[index] =  rxBuff[1 + index];


	return status;
}

int ah_set_cfg_wearablesuite_initdacoffppg1( const uint8_t measChan, const uint8_t initDacOffsetPPG1 ){
	uint8_t Temp[2] = { measChan, initDacOffsetPPG1};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_D_OFF_PPG1_OPTION, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_initdacoffppg1(uint8_t initDacOffsetPPG1[9] ){
	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_D_OFF_PPG1_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; index++)
		initDacOffsetPPG1[index] =  rxBuff[1 + index];


	return status;
}

int ah_set_cfg_wearablesuite_initdacoffppg2( const uint8_t measChan, const uint8_t initDacOffsetPPG2 ){
	uint8_t Temp[2] = { measChan, initDacOffsetPPG2};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_D_OFF_PPG2_OPTION, &Temp[0], 2);

	return status;
}

int ah_get_cfg_wearablesuite_initdacoffppg2(uint8_t initDacOffsetPPG2[9] ){
	uint8_t rxBuff[1+9]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_D_OFF_PPG2_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; index++)
		initDacOffsetPPG2[index] =  rxBuff[1 + index];


	return status;
}

int ah_set_cfg_wearablesuite_initledcurr( const uint8_t measChan, const uint16_t initLedCurr ){
	uint8_t Temp[3] = { measChan, initLedCurr >> 8, initLedCurr};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB,   SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_LED_CURR_OPTION, &Temp[0], 3);

	return status;
}

int ah_get_cfg_wearablesuite_initledcurr( uint16_t initLedCurr[9] ){
	uint8_t rxBuff[1+18]; // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB,  SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_LED_CURR_OPTION, &rxBuff[0], sizeof(rxBuff) );

	int index = 0;
	for(index = 0; index < 9; index++)
	{
		initLedCurr[index] =  (uint16_t)rxBuff[1 + (2 * index)] << 8 | (uint16_t)rxBuff[1 + ((2 * index) + 1)];
	}

	return status;
}

int ah_set_cfg_wearablesuite_whrmledpdconfig( const uint16_t whrmledpdconfig ){

	uint8_t Temp[2] = { (uint8_t)((whrmledpdconfig >> (1*8)) & 0xFF),  (uint8_t)((whrmledpdconfig >> (0*8)) & 0xFF)};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_WHRMLEDPDCONFIGURATION, &Temp[0], 2);
	return status;

}


int ah_get_cfg_wearablesuite_whrmledpdconfig( uint16_t *whrmledpdconfig) {

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_WHRMLEDPDCONFIGURATION, &rxBuff[0], sizeof(rxBuff));
	*whrmledpdconfig = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}


int ah_set_cfg_wearablesuite_spo2ledpdconfig( const uint16_t spo2ledpdconfig ){

	uint8_t Temp[2] = { (uint8_t)((spo2ledpdconfig >> (1*8)) & 0xFF),  (uint8_t)((spo2ledpdconfig >> (0*8)) & 0xFF)};
    int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2LEDPDCONFIGURATION, &Temp[0], 2);
	return status;

}


int ah_get_cfg_wearablesuite_spo2ledpdconfig( uint16_t *spo2ledpdconfig) {

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2LEDPDCONFIGURATION, &rxBuff[0], sizeof(rxBuff));
	*spo2ledpdconfig = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}

int ah_set_cfg_wearablesuite_reset_algo_config() {
	uint8_t Temp[1] = {1};
	int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_RESET_ALGO_CONFIG, &Temp[0], 1);

	return status;
}

int ah_get_cfg_wearablesuite_aferequest( uint8_t afe_reqs[20] ) {
	uint8_t rxBuff[1+20];  // first byte is status
    int status = sh_get_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_GET_AFE_REQUEST_OPTION, &rxBuff[0], sizeof(rxBuff));

    memcpy(afe_reqs, &rxBuff[1], 20);

    return status;
}

int ah_set_cfg_wearablesuite_clear_aferequest( uint8_t isApplied ) {
	uint8_t Temp[1] = {isApplied};
	int status = sh_set_sens_cfg(SH_SENSORIDX_ALGOHUB, SS_CFGIDX_WHRM_WSPO2_SUITE_CLEAR_AFE_REQUEST_OPTION, &Temp[0], 1);

	return status;
}


/* **********************************************************************************************
 * 																							   	*
 *   					   COMMAND INTERFACE RELATED METHODS								 	*
 *																								*
 * **********************************************************************************************/




