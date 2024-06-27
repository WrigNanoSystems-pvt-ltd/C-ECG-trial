#include "mrd106.h"
#include "mrd106_helper.h"
#include "mrd106_isr.h"
#include "mrd106_timers.h"

#include "main.h"
#include "board.h"
#include "utils.h"
#include "sh_defs.h"
//BLE Related Headers
#include "ble_api.h"

//Timers Related Headers
#include "rtc.h"
#include "nvic_table.h"
#include "tmr.h"
#include "tmr_utils.h"

//Algohub Related Headers
#include "algohub_api.h"
#include "algohub_config_api.h"
#include "algohub_sensorhub_manager.h"

//PMIC Related Headers
#include "max20356_platform.h"
#include "max20356_registers.h"

//GUI Related Headers
#include "gui.h"
#include "app_gui.h"

//AFE Related Headers
#include "max86178.h"

#include "utils.h"
#include "app_fatfs.h"
#include "app_ui.h"
#include "app_interface_process.h"
#include "mxc_delay.h"


//Temp Sensor Related Helpers
uint8_t temp_sensor_sampling_freq[5] = {1,2,4,8,10};

//Timer Related Helpers


//Helper
extern queue_t queue_algo;

static uint32_t s_find_prescaler_val(uint32_t tmr_pres)
{
	uint32_t val = 0;
	if(tmr_pres == TMR_PRES_1)
	{
		val = 1;
	}
	else
	{
		//Since CN is at 5:3 bits in timer control register
		//Logic can be undestood from MAX32665UG-pg347
		val = tmr_pres >> 2;
	}

	return val;
}


void mrd_timer_init(mxc_tmr_regs_t * tmr, uint32_t period_ms, uint32_t tmr_ps)
{
	// Declare variables
	tmr_cfg_t tmr_cfg;
	
	uint32_t ps = s_find_prescaler_val(tmr_ps);
	uint32_t period_ticks =  MS_TO_TICK(period_ms,ps);
	TMR_Shutdown(tmr);
	TMR_Disable(tmr);

	TMR_Init(tmr, tmr_ps, 0);

	tmr_cfg.mode = TMR_MODE_CONTINUOUS;
	tmr_cfg.cmp_cnt = period_ticks; 
	tmr_cfg.pol = 0;
	TMR_Config(tmr, &tmr_cfg);
}

void mrd_timer_enable(mxc_tmr_regs_t * tmr, uint8_t en)
{
	if(en)
		TMR_Enable(tmr);
	else
		TMR_Disable(tmr);
}

void mrd_timer_period_update(mxc_tmr_regs_t * tmr, uint32_t period_ms, uint32_t tmr_ps)
{
	mrd_timer_enable(tmr, 0);
	mrd_timer_init(tmr, period_ms, tmr_ps);
	mrd_timer_enable(tmr, 1);
}

//BLE Related Inits
void mrd_ble_init()
{
    /* Initialize Radio */
    WsfInit();

    /* Initialize attribute server database */
    ble_init();
    ble_service_callback(callback);
    ble_state_callback(ble_state_cb);
}

//COMM Related Inits
void mrd_comm_init()
{

}

//Timers Related Inits
void mrd_timers_init()
{
    //RTC Timer Init
    RTC_Init(MXC_RTC, 0, 0, NULL);
    RTC_EnableRTCE(MXC_RTC);
    
    // BLE Timer Init
    NVIC_SetVector(BLE_TIMER_IRQn, ble_isr);
    NVIC_EnableIRQ(BLE_TIMER_IRQn);
    mrd_timer_init(BLE_TIMER, 17, BLE_TIMER_PS);
    
     /* TEMP Polling Timer */
#if defined(ENABLE_MAX30208) || defined(ENABLE_MAX30210)
    NVIC_SetVector(TEMP_TIMER_IRQn, temp_sensor_isr);
    NVIC_EnableIRQ(TEMP_TIMER_IRQn);
    //TODO: Find out why we should divide with 2
    int temp_period = (1000/(temp_sensor_sampling_freq[0]))/2;
    mrd_timer_init(TEMP_TIMER, temp_period, TEMP_TIMER_PS);
#endif

    NVIC_SetVector(LED_TIMER_IRQn, led_isr);
    NVIC_EnableIRQ(LED_TIMER_IRQn);
    mrd_timer_init(LED_TIMER, 17, LED_TIMER_PS);

    NVIC_SetVector(PERIODIC_PACK_TIMER_IRQn, periodic_pack_isr);
    NVIC_EnableIRQ(PERIODIC_PACK_TIMER_IRQn);
    mrd_timer_init(PERIODIC_PACK_TIMER, 100, PERIODIC_PACK_TIMER_PS);
    
    
}

void mrd_peripheral_init()
{
    //Button driver init
    button_init();

    //Flash init
    app_flash_init();

    //FileSystem Init
    app_fatfs_init();

    //USB Init
    usb_dev_init(delay_us, usb_events);
}

void mrd_isr_init()
{
    //Setting AFE Interrupt Pin
    GPIO_IntConfig(&afe_interrupt_pin, GPIO_INT_EDGE, GPIO_INT_FALLING);
	GPIO_RegisterCallback(&afe_interrupt_pin, afe_int_isr, NULL);
	GPIO_IntEnable(&afe_interrupt_pin);

    //Setting pmic interrupt pin
    //TODO: Write explanation why this calue
    uint8_t value = 0;
    GPIO_IntConfig(&pmic_interrupt_pin, GPIO_INT_EDGE, GPIO_INT_RISING);
    GPIO_RegisterCallback(&pmic_interrupt_pin, pmic_mpc_isr, NULL);
    GPIO_IntEnable(&pmic_interrupt_pin);
    max20356_platform_read_register(MAX20356_DRIVER_PMIC,MAX20356_PMIC_REG_USBOKITRCFG, &value, 1);
}

//API Related Inits 
void mrd_api_init()
{
    
    /* Register a handler for Application events */
    AppUiActionRegister(SetAddress);

    app_interface_process_init();

    //Algohub Init
    int ret = 0;
    ret = ah_sh_init();
    pr_info("ah_sh_init ret %d \n", ret);

    ret = ah_sh_get_spi_ownership(AH_SH_SPI_OWNER_HOST);
    pr_info("ah_sh_get_spi_ownership ret %d \n", ret);

    app_gui_init();
}


void mrd_sensor_init(sensor_t **sensor, uint8_t sensorNo)
{
	*sensor = mxm_sh_get_sensor_instance(sensorNo);
}

//AFE Related Helpers
uint64_t last_acc_execute_time = 0;
uint64_t last_frame_time = 0;
uint8_t is_afe_request_made = 0;
static int num_of_applied_afe_requests = 0;

int apply_afe_requests()
{
	int is_afe_applied = 0;
	if(gPpgEn){
		last_acc_execute_time = utils_get_time_ms();
	}

	enum{
		UPDATING_SENSOR_SETTINGS,
		INFORMING_AFE
	};

	static uint8_t afe_update_state = UPDATING_SENSOR_SETTINGS;

	ble_sanity_handler();

	if( (utils_get_time_ms() - last_frame_time) > 10){
		return is_afe_applied;
	}

	int algo_queue_len = queue_len(&queue_algo);


	if(UPDATING_SENSOR_SETTINGS == afe_update_state)
	{
		int ret = 0;
		uint8_t afe_request[20];
		uint8_t index = 0;
		uint8_t meas_no = 0;
		uint8_t pd_no = 0;
		algohub_afe_reqs_t afe[ALGOHUB_PPG_MAX];

		if(is_afe_request_made  && 0 == (algo_queue_len%ALGOHUB_INPUT_FRAME_SIZE))
		{

			memset(afe, 0, sizeof(algohub_afe_reqs_t) * ALGOHUB_PPG_MAX);

			ret = ah_get_cfg_wearablesuite_aferequest(afe_request);
			pr_info("sh_get_cfg_wearablesuite_aferequest %d \n", ret);

			algohub_parse_aferequest(afe_request, afe);

			for(index = 0; index < ALGOHUB_PPG_MAX; index++)
			{
				meas_no = algohub_ppg_signal_meas_ch_no(index);
				pd_no = algohub_ppg_signal_pd_no(index);

				if(afe[index].led_curr_adjustment.is_led_curr_update_requested)
				{
					max86178_set_leds_current(meas_no, MAX86178_LED_DRIVER_A, afe[index].led_curr_adjustment.curr);
				}

				if(afe[index].int_time_adjustment.is_int_time_update_requested)
				{
					max86178_set_integration_time(meas_no, afe[index].int_time_adjustment.integration_time);
				}

				if(afe[index].avg_smp_adjustment.is_avg_smp_update_requested)
				{
					max86178_set_average_sample(meas_no, afe[index].avg_smp_adjustment.avg_smp);
				}

				if(afe[index].dac_offset_adjustment.is_dac_offset_update_requested)
				{
					max86178_set_dac_offset(meas_no, pd_no, afe[index].dac_offset_adjustment.dac_offset);
				}
			}

			is_afe_request_made--;
			afe_update_state = INFORMING_AFE;
			is_afe_applied = 1;
			num_of_applied_afe_requests++;
		}



		UNUSED(ret);

	}
	else
	{
		if(0 == algo_queue_len)
		{
			int ret = 0;
	    	ret = algohub_notify_afe_request_applied();
	    	pr_info("algohub_notify_afe_request_applied %d \n", ret);

	    	afe_update_state = UPDATING_SENSOR_SETTINGS;
	    	UNUSED(ret);
		}
	}


	return is_afe_applied;
}

void mrd_sensors_enable(int en)
{
	mrd_t *mrd = getMRD();
	sensor_t *ppg = mrd->getSensor(SH_MAX86178);
	sensor_t *acc =  mrd->getSensor(SH_ACC_SENSOR);

	/* Disable algohub when en  == 0 */
	if(0 == en){
		mrd->setStopCmd(1);
		if(APP_ALGO_STATE_AH_ENABLED == app_gui_get_ah_state())
		{
			algohub_disable();
			algohub_reset_configs();
		}

		app_gui_stop();
	}
	else{


		if(mrd->getFlashLogEnable())
		{

			char file_name[LOG_FILENAME_LEN];
		    uint8_t header[126];
		    struct flash_log_time start_time;
		    gui_get_flash_logfile_name(file_name);
		    gui_get_flash_log_start_time(&start_time);
		    gui_get_flash_log_header(header);
		    app_fatfs_open_file(file_name, sizeof(file_name), start_time.year, start_time.month, start_time.day, start_time.hour, start_time.min, start_time.sec);
		    app_fatfs_write_file(header, sizeof(header));

		}

		app_gui_start();

		if(APP_ALGO_STATE_AH_ENABLED == app_gui_get_ah_state())
		{
			ah_sh_switch_to_algohub_mode();
		}
	}

	mrd_timer_enable(BLE_TIMER, !mrd->getFlashLogEnable());
	mrd_timer_enable(TEMP_TIMER, mrd->getTempSensorEnable());

	/* Resetting queue in case of any remaining data due to raw mode */
	queue_reset(&queue_algo);

	queue_reset(ppg->queue);
	queue_reset(acc->queue);

	if(APP_ALGO_STATE_AH_ENABLED == app_gui_get_ah_state() || APP_ALGO_STATE_AH_SH_NOT_ENABLED == app_gui_get_ah_state())
	{
		spo2_state_machine(1);
		acc->enable(en);
		ppg->enable(en);
	}

	/*Reassign last_frame_time such that there will be no afe settings in next 30 ms */
	last_frame_time = utils_get_time_ms() + 30;

}


//USB Related Helpers
void delay_us(unsigned int usec)
{
    /* mxc_delay() takes unsigned long, so can't use it directly */
    mxc_delay(usec);
}

void usb_events(usb_dev_events_t evt){
	switch(evt){
	case USB_CDC_READ_READY:
		app_main_evt_post(EVT_USB_WRITE_CB);
		break;
    }
}


