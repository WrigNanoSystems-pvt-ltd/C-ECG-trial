#include "mrd106_isr.h"
#include "mrd106.h"
#include "mrd106_helper.h"

#include "main.h"
#include "gui.h"
#include "usb_api.h"

#include "tmr.h"
#include "max20356.h"
#include "max20356_registers.h"
#include "max20356_platform.h"
#include "board.h"

#include "utils.h"


void temp_sensor_isr()
{
    TMR_IntClear(TEMP_TIMER);

    app_main_evt_post(EVT_TEMP_SENSOR_SAMPLE);
}

void ble_isr()
{
    TMR_IntClear(BLE_TIMER);

    mrd_t *m_mrd = getMRD();
    m_mrd->setBleSentEvt(1);
}

void led_isr()
{
    TMR_IntClear(LED_TIMER);

    app_led_timer_handle();
}

void periodic_pack_isr()
{
    TMR_IntClear(PERIODIC_PACK_TIMER);
    mrd_t *m_mrd = getMRD();
    if(m_mrd->getMeasurementEnable() && !m_mrd->getFlashLogEnable())
    {
    	adapter_set_event(EVT_PERIODICDATAREADY);
    }
}

void pmic_mpc_isr()
{
	uint8_t value = 0;
    static volatile int isUsbPresent = 0;

	max20356_platform_read_register(MAX20356_DRIVER_PMIC,MAX20356_PMIC_REG_USBOKITRCFG, &value, 1);
	if ((value & MAX20356_USB_OK_INT_F) != 0) {
		max20356_platform_read_register(MAX20356_DRIVER_PMIC, MAX20356_PMIC_REG_STATUS1, &value, 1);
		if ((value & MAX20356_USB_OK) != 0) {
			isUsbPresent = 1;
			max20356_usb_power(1);
			usb_dev_init(delay_us, usb_events);
		} else {
			isUsbPresent = 0;
			max20356_usb_power(0);
			usb_dev_deinit();
		}
	}
	GPIO_IntClr(&pmic_interrupt_pin);
}

void afe_int_isr(){

	last_frame_time = utils_get_time_ms();

	GPIO_IntClr(&afe_interrupt_pin);

}
