#ifndef _MRD106_HELPER_H_
#define _MRD106_HELPER_H_

#include <stdint.h>
#include "tmr.h"
#include "tmr_utils.h"

#include "app_led.h"
#include "usb_api.h"

//Macros

#define MS_TO_TICK(ms, prescaler)	((ms) * (PeripheralClock / (prescaler * 1000)))

//Timer


void mrd_timer_init(mxc_tmr_regs_t * tmr, uint32_t period_ms, uint32_t tmr_ps);
void mrd_timer_enable(mxc_tmr_regs_t * tmr, uint8_t en);
void mrd_timer_period_update(mxc_tmr_regs_t * tmr, uint32_t period_ms, uint32_t tmr_ps);
//Temperature Sensor Related Helpers
/*
 * temp_sensor_sampling_freq
 * 0 - 1 Hz
 * 1 - 2 Hz
 * 2 - 4 Hz
 * 3 - 8 Hz
 * 4 - 10 Hz
 * */
extern uint8_t temp_sensor_sampling_freq[5];

//Init Related Helpers
void mrd_ble_init();
void mrd_comm_init();
void mrd_peripheral_init();
void mrd_api_init();
void mrd_timers_init();
void mrd_isr_init();
void mrd_sensors_init();

//BLE Related Helpers

//AFE Related Helpers
extern uint64_t last_acc_execute_time;
extern uint64_t last_frame_time;
extern uint8_t is_afe_request_made;
int apply_afe_requests();
 
void mrd_sensors_enable(int en);
//USB Realated Helpers
void delay_us(unsigned int usec);
void usb_events(usb_dev_events_t evt);
#endif