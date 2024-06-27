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

#include "rtc.h"
#include "mxc_config.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_pins.h"

#include "mrd106.h"

#include "board.h"
#include "tmr.h"
#include "trng.h"
#include "tmr_utils.h"

#include "sensorhub.h"
#include "button.h"

#include "max20356.h"
#include "max17260.h"

#include "utils.h"

#include "app_flash.h"
#include "app_led_wrapper.h"
#include "app_led.h"

#include "algohub_api.h"
#include "algohub_config_api.h"
#include "sensorhub_api.h"
#include "algohub_sensorhub_manager.h"

#include "app_fatfs.h"

#include "main.h"
#include "Drivers/GUI/app_gui.h"
#include "Drivers/GUI/gui.h"
#include "Drivers/GUI/app_interface_process.h"
#include "Drivers/GUI/app_interface_ble_process.h"
#include "regHandler_spi.h"
#include "sh_defs.h"
#include "Drivers/MAX20356/max20356_registers.h"
#include "max20356_platform.h"
#include "max20356.h"
#include "windows_gui_packet_defs.h"
#include "button_gesture_handler.h"
#include "locationfinder.h"
#include "ble_api.h"
/**************************************************************************************************
  Macros
**************************************************************************************************/
#define HWREG(x) (*((volatile unsigned long *)(x))) // E1: This macro allows direct access to a hardware register at address x.
// E1: (volatile unsigned long *): This is a type cast. It tells the compiler to treat x as a pointer to a volatile unsigned long.
// E1: ANd then it is dereferencing it to access the value stored at the memory address 'x'.
// E1: “volatile” variables can change from outside the program and that’s why compilers aren’t supposed to optimize their access. 

/* Size of buffer for stdio functions */
#define PRINTF_BUF_SIZE 128 // E1: Defines the size of buffer for printf fxns

/**************************************************************************************************
  Local Variables E1: These variables manage global states, configurations, and buffers for sensors, queues, and event handling.
**************************************************************************************************/
static volatile unsigned int g_app_evt = 0;

// Sensor Related Variables
static sensor_t *acc;
static sensor_t *biosensor;
#if defined(ENABLE_MAX30208) || defined(ENABLE_MAX30210)
static sensor_t *temp;
#endif

/*! Buffer for stdio functions */
char printf_buffer[PRINTF_BUF_SIZE];

extern bool_t resetFlag;

static queue_t msgQueue;
static uint8_t msgQueueBuf[PACKETSIZE * NUM_EVTS];

static queue_t adapReportQueue;
static uint8_t adapReportQueueBuf[PACKETSIZE * 1024];

static mrd_t *m_mrd = NULL;

uint8_t power_but_evt = 0;
uint8_t loc_finder_enable = 0;

uint8_t ble_connected = 0;

unsigned int time = 0;

/* RTC Update  variables */
uint64_t rtc_time_update_value = 0;

/*SpO2 App settings*/
#define SPO2_MAX_DATA_COLLECTION_TIME_MS 1.5 * 60 * 1000
uint8_t spo2_meas_selected_pd = SPO2_APP_SETTING_USE_PD1;
uint8_t spo2_meas_selected_meas_int = SPO2_APP_SETTING_MEAS_CONTINIOUS;
uint8_t spo2_meas_selected_meas_int_values_min[SPO2_APP_SETTING_MEAS_INTERVAL_LIMIT] = {30 , 60 , 120};

int printf_enable = 0;

extern queue_t queue_algo;

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/



int spo2_state_machine(uint8_t restart)
{
	static uint64_t last_calculation_start_time = 0;
	static uint8_t ppg_active_meas_set = 0;
	static uint8_t ppg_active_meas_backup[8] = {0};
	static uint8_t spo2_sm_state = SPO2_SM_NOT_INITIALIZED;

	uint64_t current_time = utils_get_time_ms();
	uint64_t time_interval = spo2_meas_selected_meas_int_values_min[spo2_meas_selected_meas_int] * 60 * 1000; // minute to msc

	if (restart)
	{
		ppg_active_meas_set = 0;
	}

	if (restart || spo2_sm_state == SPO2_SM_NOT_INITIALIZED)
	{
		last_calculation_start_time = current_time;
		spo2_sm_state = SPO2_SM_ACTIVE;
		queue_reset(&queue_algo);

		if (ppg_active_meas_set)
		{
			max86178_enable_channels(ppg_active_meas_backup);
		}

		return 0;
	}

	if (spo2_sm_state == SPO2_SM_WAITING)
	{
		if (get_periodicpacket_status() && !gEcgEn && !gIqEn)
		{
			set_periodicpacket_status(0);
		}

		if (current_time - last_calculation_start_time > time_interval)
		{
			spo2_sm_state = SPO2_SM_NOT_INITIALIZED;
			gPpgEn = 1;
			max86178_enable_channels(ppg_active_meas_backup);
		}

		return 0;
	}

	if (spo2_sm_state == SPO2_SM_ACTIVE)
	{

		if (spo2_meas_selected_meas_int < SPO2_APP_SETTING_MEAS_INTERVAL_LIMIT && // If spo2 measurement is not continuous
			current_time - last_calculation_start_time > SPO2_MAX_DATA_COLLECTION_TIME_MS)
		{ // and if time spent in spo2 measurement reached to the limit

			spo2_sm_state = SPO2_SM_WAITING;
			max86178_get_list_of_enabled_channels(ppg_active_meas_backup);
			ppg_active_meas_set = 1;
			uint8_t ppg_all_disabled[8] = {0};
			gPpgEn = 0;
			max86178_enable_channels(ppg_all_disabled);

			return 0;
		}
		return 1;
	}

	return 0;
}

/*************************************************************************************************/
/*!
 *  \fn     main
 *
 *  \brief  Entry point for demo software.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/

int main(void)
{
	int retVal = 0;

#ifndef __IAR_SYSTEMS_ICC__
	setvbuf(stdout, printf_buffer, _IOLBF, PRINTF_BUF_SIZE);
#endif

	int index = 0;
	int ret = 0;
	uint8_t temp_report_buff[PACKETSIZE * NUM_PACKS_PER_CHUNK] = {0};

	m_mrd = getMRD();
	m_mrd->init();
	m_mrd->setStatusLedEnable(1);

	biosensor = m_mrd->getSensor(SH_MAX86178);
	acc = m_mrd->getSensor(SH_ACC_SENSOR);

#if defined(ENABLE_MAX30208) || defined(ENABLE_MAX30210)
	temp = m_mrd->getSensor(SH_TEMP_SENSOR);
	temp->init();
#endif

	/* Queue Init */
	ret = queue_init(&adapReportQueue, adapReportQueueBuf, PACKETSIZE, PACKETSIZE * 1024);
	pr_info("adapter report queue init = %d ", ret);

	ret = queue_init(&msgQueue, msgQueueBuf, PACKETSIZE, PACKETSIZE * NUM_EVTS);
	pr_info("msg queue init = %d ", ret);

	adapter_set_report_queue(&adapReportQueue);
	gui_set_msg_queue(&msgQueue);

	int ppg_len;

	max86178_register_fifo_read_callback(apply_afe_requests);

	uint64_t time_to_led = utils_get_time_ms();
	uint64_t sensorhub_evt_time = utils_get_time_ms();

	UNUSED(ret);
	uint64_t last_loop_time = 0;
	uint64_t current_time = 0;

	while (1)
	{
		current_time = utils_get_time_ms();
		if (current_time - last_loop_time > 15)
		{
			//usb_debug_printf("loop_time: %llu\r\n", current_time -last_loop_time);
		}
		last_loop_time = current_time;

		if (utils_get_time_ms() - time_to_led > 500)
		{
			time_to_led = utils_get_time_ms();
			uint8_t batt_per = 0;
			uint8_t charging = 0;
			max17260_get_battery_percent(&batt_per);
			max20356_is_usb_connected(&charging);

			m_mrd->setBatteryPercentage(batt_per);
			m_mrd->setChargingStatus(charging);

			check_LED_conditions();
		}

		if (g_app_evt)
		{
			if (g_app_evt & EVT_TEMP_SENSOR_SAMPLE)
			{
				g_app_evt &= ~EVT_TEMP_SENSOR_SAMPLE;
#if defined(ENABLE_MAX30208) || defined(ENABLE_MAX30210)
				if (m_mrd->getMeasurementEnable() && m_mrd->getTempSensorEnable())
				{
					temp->execute_once(NULL);
					if (temp->num_items)
					{
						uint64_t temp_report_time = utils_get_time_ms();
						format_data_temp(temp->report_buf.buf, temp_report_time);
					}
				}
#endif
			}

			if (g_app_evt & EVT_BLE_CONN_OPEN)
			{

				if (!m_mrd->getFlashLogEnable())
				{
					app_gui_connected();
				}
				ble_connected = 1;
				g_app_evt &= ~EVT_BLE_CONN_OPEN;
			}

			if (g_app_evt & EVT_BLE_CONN_CLOSE)
			{
				if (!m_mrd->getFlashLogEnable())
				{
					app_main_evt_post(EVT_MEASUREMENT_STOP);
					app_gui_reset();
					app_gui_disconnected();
				}

				gui_queue_reset();
				app_interface_process_reset();
				ble_connected = 0;
				if (!m_mrd->getMeasurementEnable())
				{
					g_app_evt &= ~EVT_BLE_CONN_CLOSE;
				}
			}

			button_gesture_handler_iterate();

			if (g_app_evt & EVT_BUTTON_SINGLE_PRESS)
			{
				if (!loc_finder_enable && !m_mrd->getMeasurementEnable() && !m_mrd->getFlashLogEnable())
				{
					app_main_evt_post(EVT_LOC_FIND_STARTED);
				}
				g_app_evt &= ~EVT_BUTTON_SINGLE_PRESS;
			}

			if (g_app_evt & EVT_BUTTON_DOUBLE_PRESS)
			{
				if (!loc_finder_enable && !m_mrd->getMeasurementEnable() && !m_mrd->getFlashLogEnable())
				{
					app_main_evt_post(EVT_FLASH_LOGGING_STARTED);
				}

				g_app_evt &= ~EVT_BUTTON_DOUBLE_PRESS;
			}

			if (g_app_evt & EVT_BUTTON_TRIPLE_PRESS)
			{

				static uint8_t led_on_off_state = 1;
				led_on_off_state = !led_on_off_state;
				m_mrd->setStatusLedEnable(led_on_off_state);
				check_LED_conditions();
				g_app_evt &= ~EVT_BUTTON_TRIPLE_PRESS;
			}

			if (g_app_evt & EVT_BUTTON_LONG_PRESS)
			{

				max20356_send_power_off_cmd();

				g_app_evt &= ~EVT_BUTTON_LONG_PRESS;
			}

			if (g_app_evt & EVT_FLASH_LOGGING_STARTED)
			{
				if (!ble_connected)
				{
					m_mrd->setFlashLogEnable(1);
					m_mrd->enableSensors(1);
				}
				g_app_evt &= ~EVT_FLASH_LOGGING_STARTED;
			}

			if (g_app_evt & EVT_FLASH_LOGGING_FINISHED)
			{
				m_mrd->setFlashLogEnable(0);

				uint8_t footer[18];
				memset(footer, 0, sizeof(footer));
				uint64_t wall = gui_get_flash_stop_wall_time();

				footer[0] = wall >> 24;
				footer[1] = wall >> 16;
				footer[2] = wall >> 8;
				footer[3] = wall;
				footer[4] = wall >> 40;
				footer[5] = wall >> 32;

				app_fatfs_write_file(footer, sizeof(footer));
				app_fatfs_close_file();
				max20356_send_power_off_cmd();

				g_app_evt &= ~EVT_FLASH_LOGGING_FINISHED;
			}

			if (g_app_evt & EVT_MEASUREMENT_START)
			{
				m_mrd->setMeasurementEnable(1);
				m_mrd->enableSensors(1);

				g_app_evt &= ~EVT_MEASUREMENT_START;
			}

			if (g_app_evt & EVT_MEASUREMENT_STOP)
			{
				m_mrd->setMeasurementEnable(0);
				m_mrd->enableSensors(0);
				adapter_clear_sensor_running();

				if (m_mrd->getFlashLogEnable())
				{
					app_main_evt_post(EVT_FLASH_LOGGING_FINISHED);
				}
				g_app_evt &= ~EVT_MEASUREMENT_STOP;
			}

			if (g_app_evt & EVT_USB_WRITE_CB)
			{
				unsigned char rxbuff[256] = {0};
				g_app_evt &= ~EVT_USB_WRITE_CB;

				usb_cdc_read(rxbuff, sizeof(rxbuff));
				//USB commands can be processed here
			}

			if (g_app_evt & EVT_BLE_WRITE_CB)
			{
				g_app_evt &= ~EVT_BLE_WRITE_CB;
			}

			if (g_app_evt & EVT_BLE_READ_CB)
			{
				g_app_evt &= ~EVT_BLE_READ_CB;
			}

			if (g_app_evt & EVT_TEMP_SENSOR_UPDATE)
			{
				g_app_evt &= ~EVT_TEMP_SENSOR_UPDATE;
#if defined(ENABLE_MAX30208) || defined(ENABLE_MAX30210)
				int temp_period = 1000 / temp_sensor_sampling_freq[m_mrd->getTempSensorFreq()];
				m_mrd->updateTimerPeriod(TEMP_TIMER, temp_period / 2, TEMP_TIMER_PS);
#endif
			}

			if (g_app_evt & EVT_RTC_UPDATE)
			{
				g_app_evt &= ~EVT_RTC_UPDATE;

				uint32_t sec_update = rtc_time_update_value / 1000;
				uint16_t subsec_update = (rtc_time_update_value - (sec_update * 1000)) * 0xFFFF;

				RTC_DisableRTCE(MXC_RTC);
				RTC_Init(MXC_RTC, sec_update, subsec_update, NULL);
				RTC_EnableRTCE(MXC_RTC);
			}

			static app_algo_state_t oldState;
			static uint8_t oldAccelState;
			if (g_app_evt & EVT_LOC_FIND_STARTED)
			{
				loc_finder_enable = 1;
				oldAccelState = gUseAcc;
				oldState = app_gui_get_ah_state();
				if (oldState == APP_ALGO_STATE_AH_ENABLED)
				{
					app_gui_set_ah_state(APP_ALGO_STATE_AH_SH_NOT_ENABLED);
				}
				max86178_locationfinder_enable();
				m_mrd->enableSensors(1);
				g_app_evt &= ~EVT_LOC_FIND_STARTED;
			}

			if (g_app_evt & EVT_LOC_FIND_STOPPED)
			{
				loc_finder_enable = 0;
				gUseAcc = 1;
				m_mrd->enableSensors(0);
				gUseAcc = oldAccelState;
				app_gui_set_ah_state(oldState);

				max86178_locationfinder_disable();
				g_app_evt &= ~EVT_LOC_FIND_STOPPED;
			}
		}

		DISPATCH_BLE(10);

		if (APP_INTERFACE_BLE_MRD104GUI == app_interface_ble_selected())
		{
			if (biosensor->initialized)
			{
				biosensor->execute_once(NULL);
			}

			if (m_mrd->getMeasurementEnable() && !gPpgEn && (gEcgEn || gIqEn) && gUseAcc && (current_time - last_acc_execute_time >= 40))
			{
				acc->execute_once(NULL);
				adapter_acc_execute_push(acc->queue);
				last_acc_execute_time = current_time;
			}

			if (APP_ALGO_STATE_AH_ENABLED == app_gui_get_ah_state() && spo2_state_machine(0))
			{
				algohub_process(current_time);
			}

			if (!m_mrd->getMeasurementEnable() || (m_mrd->getMeasurementEnable() && !spo2_state_machine(0)))
			{
				ble_sanity_handler();
			}

			ppg_len = queue_len(biosensor->queue);

			if (loc_finder_enable)
			{
				if (ppg_len >= 4)
				{
					locationfinder(biosensor->queue);
					DISPATCH_BLE(10);
					queue_reset(biosensor->queue);
				}
			}
			else
			{
				if (ppg_len)
				{
					for (index = 0; index < ppg_len; index++)
					{
						adapter_execute_push(biosensor->queue, acc->queue);
					}
				}
			}

			adapter_execute_pop();

			if (m_mrd->getFlashLogEnable())
			{
				int numPackReportQueue = queue_len(&adapReportQueue);
				int ind = 0;
				uint8_t buff[20];

				for (ind = 0; ind < numPackReportQueue; ind++)
				{
					dequeue(&adapReportQueue, buff);
					app_fatfs_write_file(buff, sizeof(buff));
				}
			}
		}

		/*
		  To minimize drop packets, we are checking how much times based since the last transmission.
		  According to the time diff, we are sending different number of BLE notification packets to solve that problem.
		  This is a temporary solution and it can removed in the future.
		  For now, it seems that it is working fine.
		*/
		if (m_mrd->getBleSentEvt())
		{
			if (APP_INTERFACE_BLE_MRD104GUI == app_interface_ble_selected())
			{
				int numPackReportQueue = 0;

				numPackReportQueue = queue_len(&adapReportQueue);

				if (m_mrd->getStopCmd() || (spo2_state_machine(0) == 0 && get_sensorstop_status()))
				{

					int remainingPack = NUM_PACKS_PER_CHUNK - (numPackReportQueue % NUM_PACKS_PER_CHUNK);

					if (remainingPack == NUM_PACKS_PER_CHUNK)
					{
						format_stop_packet();

						for (int ind = 0; ind < (remainingPack - 1); ind++)
							format_dont_care_packet();
					}
					else if (remainingPack == 1)
					{
						format_stop_packet();
					}
					else
					{
						format_stop_packet();
						remainingPack--;

						for (int ind = 0; ind < remainingPack; ind++)
							format_dont_care_packet();
					}

					m_mrd->setStopCmd(0);
				}

				while (numPackReportQueue >= NUM_PACKS_PER_CHUNK)
				{
					for (int index_ble = 0; index_ble < NUM_PACKS_PER_CHUNK; index_ble++)
					{
						dequeue(&adapReportQueue, &temp_report_buff[index_ble * 20]);
					}

					ble_send_notify(temp_report_buff, sizeof(temp_report_buff));
					memset(temp_report_buff, 0, sizeof(temp_report_buff));
					numPackReportQueue = queue_len(&adapReportQueue);
				}
			}

			m_mrd->setBleSentEvt(0);
		}

		if (resetFlag && wsfOsReadyToSleep() /*&& (Console_PrepForSleep() == E_NO_ERROR)*/)
		{
			/* Prevent interrupts from pre-empting this operation */
			__disable_irq();

			/* Power down the BLE hardware */
			BbDrvDisable();

			/* Reset and let bootloader run the new image */
			NVIC_SystemReset();
		}
	}
}

/*****************************************************************/
void HardFault_Handler(void)
{
	const uint32_t gcr = 0x40000000;
	const uint32_t gcr_rst0 = 0x00000004;

	const uint32_t gcr_reset = gcr + gcr_rst0;

	HWREG(gcr_reset) = 0x80000000;

	while (HWREG(gcr_reset) == 0x80000000)
		;

	pr_info("\nFaultISR: CFSR %08X, BFAR %08x\n", (unsigned int)SCB->CFSR, (unsigned int)SCB->BFAR);

	// Loop forever
	while (1);
}

void app_main_evt_post(unsigned int evt)
{
	g_app_evt |= evt; // takes Bitwise OR with the global event to effectively record the active events
}
