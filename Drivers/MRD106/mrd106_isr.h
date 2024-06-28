#ifndef _MRD106_ISR_
#define _MRD106_ISR_

void temp_sensor_isr();
void ble_isr();
void led_isr();
void periodic_pack_isr();
void pmic_mpc_isr();
void afe_int_isr();

#endif
