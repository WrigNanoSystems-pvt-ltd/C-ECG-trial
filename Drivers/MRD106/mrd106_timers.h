#include "tmr.h"
#ifndef _MRD106_TIMERS_H_
#define _MRD106_TIMERS_H_

#define TEMP_TIMER 			        MXC_TMR0
#define TEMP_TIMER_PS               TMR_PRES_1
#define TEMP_TIMER_IRQn 	        TMR0_IRQn

#define BLE_TIMER                   MXC_TMR1
#define BLE_TIMER_PS                TMR_PRES_2
#define BLE_TIMER_IRQn              TMR1_IRQn

#define LED_TIMER 			        MXC_TMR2
#define LED_TIMER_PS                TMR_PRES_2
#define LED_TIMER_IRQn	 	        TMR2_IRQn

//TMR3 is used for RTC

#define PERIODIC_PACK_TIMER         MXC_TMR4
#define PERIODIC_PACK_TIMER_PS      TMR_PRES_2
#define PERIODIC_PACK_TIMER_IRQn    TMR4_IRQn

//TMR5 is free
#endif