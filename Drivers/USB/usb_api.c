/*******************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
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
 *
 *
 ******************************************************************************/

#include "usb.h"
#include "usb_api.h"

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>


#include "usb.h"
#include "usb_event.h"
#include "enumerate.h"
#include "cdc_acm.h"
#include "msc.h"
#include "descriptors.h"
#include "mscmem.h"
#include "mxc_config.h"
#include "mxc_sys.h"
#include "mxc_delay.h"

#include "mxc_errors.h"

#define EVENT_ENUM_COMP     MAXUSB_NUM_EVENTS
#define EVENT_REMOTE_WAKE   (EVENT_ENUM_COMP + 1)

#define BUFFER_SIZE  64

/***** Global Data *****/
volatile int configured;
volatile int suspended;
volatile unsigned int event_flags;
int remote_wake_en;

/***** Function Prototypes *****/
static int setconfig_callback(usb_setup_pkt *sud, void *cbdata);
static int setfeature_callback(usb_setup_pkt *sud, void *cbdata);
static int clrfeature_callback(usb_setup_pkt *sud, void *cbdata);
static int event_callback(maxusb_event_t evt, void *data);
static void usb_app_sleep(void);
static void usb_app_wakeup(void);
static int usb_read_callback(void);

/***** File Scope Variables *****/

static char is_init_complete = 0;

static event_handler fp_event_func;


/* Defaults, which will be changed by the configuration code */
static msc_cfg_t msc_cfg = {
  1,                    /* EP OUT */
  MXC_USBHS_MAX_PACKET, /* OUT max packet size */
  2,                    /* EP IN */
  MXC_USBHS_MAX_PACKET, /* IN max packet size */
};

static const msc_idstrings_t ids = {
    "MAXIM",            /* Vendor string.  Maximum of 8 bytes */
    "MSC Example",       /* Product string.  Maximum of 16 bytes */
    "1.0"              /* Version string.  Maximum of 4 bytes */
};

/* This EP assignment must match the Configuration Descriptor */
static acm_cfg_t acm_cfg = {
  2,                  /* EP OUT */
  MXC_USBHS_MAX_PACKET, /* OUT max packet size */
  3,                  /* EP IN */
  MXC_USBHS_MAX_PACKET, /* IN max packet size */
  4,                  /* EP Notify */
  MXC_USBHS_MAX_PACKET, /* Notify max packet size */
};

/* Functions to control "disk" memory. See msc.h for definitions. */
static const msc_mem_t mem = {
    mscmem_init,
    mscmem_start,
    mscmem_stop,
    mscmem_ready,
    mscmem_size,
    mscmem_read,
    mscmem_write,
};


/**
 * @brief      @brief      Callback function for startup that calls the system startup for USB.
 *
 * @return     0 if successful
 */
int usb_startup_cb()
{
    const sys_cfg_usbhs_t sys_usbhs_cfg = NULL;
    return SYS_USBHS_Init(&sys_usbhs_cfg);
}

/**
 * @brief      Callback function for shutdown that calls the system shutdown for USB.
 *
 * @return     Shutdown status
 */
int usb_shutdown_cb()
{
    return SYS_USBHS_Shutdown();
}

/******************************************************************************/
static int setconfig_callback(usb_setup_pkt *sud, void *cbdata)
{
	 /* Confirm the configuration value */
	if (sud->wValue == composite_config_descriptor.config_descriptor.bConfigurationValue) {
	//    	on++;
		configured = 1;
	    MXC_SETBIT(&event_flags, EVENT_ENUM_COMP);
	    if (usb_get_status() & MAXUSB_STATUS_HIGH_SPEED) {													///
	    	msc_cfg.out_ep = composite_config_descriptor_hs.endpoint_descriptor_1.bEndpointAddress & 0x7;
	    	msc_cfg.out_maxpacket = composite_config_descriptor_hs.endpoint_descriptor_1.wMaxPacketSize;
	    	msc_cfg.in_ep = composite_config_descriptor_hs.endpoint_descriptor_2.bEndpointAddress & 0x7;
	    	msc_cfg.in_maxpacket = composite_config_descriptor_hs.endpoint_descriptor_2.wMaxPacketSize;
	    } else {
	    	msc_cfg.out_ep = composite_config_descriptor.endpoint_descriptor_1.bEndpointAddress & 0x7;
			msc_cfg.out_maxpacket = composite_config_descriptor.endpoint_descriptor_1.wMaxPacketSize;
			msc_cfg.in_ep = composite_config_descriptor.endpoint_descriptor_2.bEndpointAddress & 0x7;
			msc_cfg.in_maxpacket = composite_config_descriptor.endpoint_descriptor_2.wMaxPacketSize;
	    }

	    acm_cfg.out_ep = composite_config_descriptor.endpoint_descriptor_4.bEndpointAddress & 0x7;
	    acm_cfg.out_maxpacket = composite_config_descriptor.endpoint_descriptor_4.wMaxPacketSize;
	    acm_cfg.in_ep = composite_config_descriptor.endpoint_descriptor_5.bEndpointAddress & 0x7;
	    acm_cfg.in_maxpacket = composite_config_descriptor.endpoint_descriptor_5.wMaxPacketSize;
	    acm_cfg.notify_ep = composite_config_descriptor.endpoint_descriptor_3.bEndpointAddress & 0x7;
	    acm_cfg.notify_maxpacket = composite_config_descriptor.endpoint_descriptor_3.wMaxPacketSize;

	    msc_configure(&msc_cfg);
	    return acm_configure(&acm_cfg);
	    /* Configure the device class */
	} else if (sud->wValue == 0) {
		configured = 0;
	    msc_deconfigure();
	    return acm_deconfigure();
	}

	return -1;
}
/******************************************************************************/
static int setfeature_callback(usb_setup_pkt *sud, void *cbdata)
{
    if(sud->wValue == FEAT_REMOTE_WAKE) {
        remote_wake_en = 1;
    } else {
        // Unknown callback
        return -1;
    }

    return 0;
}

/******************************************************************************/
static int clrfeature_callback(usb_setup_pkt *sud, void *cbdata)
{
    if (sud->wValue == FEAT_REMOTE_WAKE) {
        remote_wake_en = 0;
    } else {
        // Unknown callback
        return -1;
    }

    return 0;
}

/******************************************************************************/
static void usb_app_sleep(void)
{
    /* TODO: Place low-power code here */
    suspended = 1;
}

/******************************************************************************/
static void usb_app_wakeup(void)
{
    /* TODO: Place low-power code here */
    suspended = 0;
}

/******************************************************************************/
static int event_callback(maxusb_event_t evt, void *data)
{
	/* Set event flag */
	MXC_SETBIT(&event_flags, evt);

	switch (evt) {
		case MAXUSB_EVENT_NOVBUS:
			usb_event_disable(MAXUSB_EVENT_BRST);
	        usb_event_disable(MAXUSB_EVENT_SUSP);
	        usb_event_disable(MAXUSB_EVENT_DPACT);
	        usb_disconnect();
	        configured = 0;
	        enum_clearconfig();
	        msc_deconfigure();
	        acm_deconfigure();
	        usb_app_sleep();
	        break;
		case MAXUSB_EVENT_VBUS:
			usb_event_clear(MAXUSB_EVENT_BRST);
	        usb_event_enable(MAXUSB_EVENT_BRST, event_callback, NULL);
	    	usb_event_clear(MAXUSB_EVENT_BRSTDN);												///
	    	usb_event_enable(MAXUSB_EVENT_BRSTDN, event_callback, NULL);						///
	        usb_event_clear(MAXUSB_EVENT_SUSP);
	        usb_event_enable(MAXUSB_EVENT_SUSP, event_callback, NULL);
	        usb_connect();
	        usb_app_sleep();
	        break;
		case MAXUSB_EVENT_BRST:
			usb_app_wakeup();
	        enum_clearconfig();
	        msc_deconfigure();
	        acm_deconfigure();
	        configured = 0;
	        suspended = 0;
	        break;
		case MAXUSB_EVENT_BRSTDN:																///
			if (usb_get_status() & MAXUSB_STATUS_HIGH_SPEED) {
				enum_register_descriptor(ENUM_DESC_CONFIG, (uint8_t*)&composite_config_descriptor_hs, 0);
				enum_register_descriptor(ENUM_DESC_OTHER_SPEED, (uint8_t*)&composite_config_descriptor, 0);
			} else {
				enum_register_descriptor(ENUM_DESC_CONFIG, (uint8_t*)&composite_config_descriptor, 0);
				enum_register_descriptor(ENUM_DESC_OTHER_SPEED, (uint8_t*)&composite_config_descriptor_hs, 0);
			}
	    	break;
		case MAXUSB_EVENT_SUSP:
			usb_app_sleep();
	        break;
		case MAXUSB_EVENT_DPACT:
			usb_app_wakeup();
	        break;
		default:
			break;
	}

	return 0;
}

/******************************************************************************/
static int usb_read_callback(void)
{
    fp_event_func(USB_CDC_READ_READY);
    return 0;
}

/******************************************************************************/
void USB_IRQHandler(void)
{
    usb_event_handler();
}


/******************************************************************************/
int usb_dev_init(delay_usec delay_func, event_handler event_func)
{

	int ret = E_NO_ERROR;

	maxusb_cfg_options_t usb_opts;

	if(delay_func == NULL || event_func == NULL)
	{
		ret = E_BAD_PARAM;
	}

	if(E_NO_ERROR == ret)
	{
	    /* Initialize state */
	    configured = 0;
	    suspended = 0;
	    event_flags = 0;
	    remote_wake_en = 0;

	    /* Start out in full speed */
	    usb_opts.enable_hs = 0;							/* 0:Full Speed		1:High Speed */
	    usb_opts.delay_us = delay_func; 					/* Function which will be used for delays */
	    usb_opts.init_callback = usb_startup_cb;
	    usb_opts.shutdown_callback = usb_shutdown_cb;

	    fp_event_func = event_func;

	    /* Initialize the usb module */
	    ret = usb_init(&usb_opts);
	    ret |= enum_init();

	    /* Register enumeration data */
	    enum_register_descriptor(ENUM_DESC_DEVICE, (uint8_t*)&composite_device_descriptor, 0);
	    enum_register_descriptor(ENUM_DESC_CONFIG, (uint8_t*)&composite_config_descriptor, 0);

	    if (usb_opts.enable_hs) {
			/* Two additional descriptors needed for high-speed operation */
			enum_register_descriptor(ENUM_DESC_OTHER_SPEED, (uint8_t*)&composite_config_descriptor_hs, 0);
			enum_register_descriptor(ENUM_DESC_QUAL, (uint8_t*)&composite_device_qualifier_descriptor, 0);
	    }

	    enum_register_descriptor(ENUM_DESC_STRING, lang_id_desc, 0);
	    enum_register_descriptor(ENUM_DESC_STRING, mfg_id_desc, 1);
	    enum_register_descriptor(ENUM_DESC_STRING, prod_id_desc, 2);
	    enum_register_descriptor(ENUM_DESC_STRING, serial_id_desc, 3);
	    enum_register_descriptor(ENUM_DESC_STRING, cdcacm_func_desc, 4);
	    enum_register_descriptor(ENUM_DESC_STRING, msc_func_desc, 5);

	    /* Handle configuration */
	    enum_register_callback(ENUM_SETCONFIG, setconfig_callback, NULL);

	    /* Handle feature set/clear */
	    enum_register_callback(ENUM_SETFEATURE, setfeature_callback, NULL);
	    enum_register_callback(ENUM_CLRFEATURE, clrfeature_callback, NULL);

	    /* Initialize the MSC class driver */
	    ret |= msc_init(&composite_config_descriptor.msc_interface_descriptor, &ids, &mem);

	    /* Initialize the ACM class driver */
	    ret |= acm_init(&composite_config_descriptor.comm_interface_descriptor);

	    /* Register callbacks */
	    usb_event_enable(MAXUSB_EVENT_NOVBUS, event_callback, NULL);
	    usb_event_enable(MAXUSB_EVENT_VBUS, event_callback, NULL);
	    acm_register_callback(ACM_CB_READ_READY, usb_read_callback);

	    /* Start with USB in low power mode */
	    usb_app_sleep();
	    NVIC_EnableIRQ(USB_IRQn);
	}

	return ret;
}

int usb_dev_deinit()
{
	usb_disconnect();
	configured = 0;
	enum_clearconfig();
	acm_deconfigure();
	msc_deconfigure();
	usb_app_sleep();
    return E_NO_ERROR;
}

int usb_cdc_write(uint8_t * const buf, unsigned int len)
{
	int ret = E_NO_ERROR;
	uint8_t * p_buf = buf;

	if(buf == NULL || len == 0){
		ret = E_BAD_PARAM;
	}

	if(!is_init_complete){
		uint8_t rx;
		usb_cdc_read(&rx, 1);
	}

	if(E_NO_ERROR == ret)
	{
		if (acm_present()) {
			if (acm_write(p_buf, len) != len) {
				ret = E_COMM_ERR;
			}
		}
		else{
		ret = E_UNINITIALIZED;
		}
	}

	return ret;
}

int usb_cdc_read(uint8_t * buf, unsigned int len)
{
	int ret = E_NO_ERROR;
	unsigned int read_len;


	if(buf == NULL || len == 0){
		ret = E_BAD_PARAM;
	}

	if(E_NO_ERROR == ret)
	{
		if ((read_len = acm_canread()) > 0) {
			if (read_len > len) {
				read_len = len;
			}
			ret = read_len;
			// Read the data from USB
			if (acm_read(buf, read_len) != read_len) {
				ret =  E_COMM_ERR;
			}

			is_init_complete = 1;
		}
	}


	return ret;
}

void usb_printf(const char* format, ...)
{
	char buffer[256];
	int len;

	__gnuc_va_list args;
	va_start(args, format);
	len = vsnprintf(buffer, sizeof(buffer), format, args);
	if (len > 0) {
		usb_cdc_write((unsigned char *)buffer, len);
	}
	va_end(args);
}
