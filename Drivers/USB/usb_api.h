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

#ifndef DRIVERS_USB_USB_API_H_
#define DRIVERS_USB_USB_API_H_


typedef enum
{
	USB_CDC_READ_READY,				/* when new data is available from the host */
} usb_dev_events_t;

typedef void (*delay_usec)(unsigned int);
typedef void (*event_handler)(usb_dev_events_t);

/**
 *  \brief    Power up the USB and configure the necessary endpoints.
 *  \details  Power up the USB and configure the necessary endpoints.
 *  \param    delay_usec   function pointer to delay function in system
 *  \param    event_handler   function pointer to get events
 *  \return   Returns error codes that can be found in mxc_errors.h
 */
int usb_dev_init(delay_usec delay_func, event_handler event_func);

/**
 *  \brief    Deconfigure the MSD and ACM endpoints
 *  \details  Deconfigure the MSD and ACM endpoints but USB peripheral is still powered.
 *  \return   Returns error codes that can be found in mxc_errors.h
 */
int usb_dev_deinit();

/**
 *  \brief    Write the specified number of characters.
 *  \details  Write the specified number of characters. This function blocks until all characters
 *            have been transferred to and internal FIFO.
 *  \param    buf   buffer containing the characters to be sent
 *  \param    len   number of characters to write
 *  \return   The number of characters successfully written. If return value is negative, please
 *  		  refer to the mxc_errors.h
 *  \note     On some processors, the actually USB transaction is performed asynchronously, after
 *            this function returns. Successful return from this function does not guarantee
 *            successful reception of characters by the host.
 */
int usb_cdc_write(uint8_t * const buf, unsigned int len);

/**
 *  \brief    Read the specified number of characters.
 *  \details  Read the specified number of characters. This function blocks until the specified
 *            number of characters have been received.
 *  \param    buf   buffer to store the characters in
 *  \param    len   number of characters to read
 *  \return   Number of characters read, 0 if connection closes, -1 on error, or -2 if BREAK
 *            signal received. If return value is negative, please also refer to the mxc_errors.h
 */
int usb_cdc_read(uint8_t * buf, unsigned int len);

/*
 * Printf function which print data over usb cdc acm interface
 */
void usb_printf(const char* format, ...);

void run(delay_usec delay_func);

#endif /* DRIVERS_USB_USB_API_H_ */
