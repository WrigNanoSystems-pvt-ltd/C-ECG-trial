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

#ifndef UTILITIES_PERIPHERALS_H_
#define UTILITIES_PERIPHERALS_H_

#define SHOW_DEBUG_MSGS
#define SHOW_INFO_MSGS
#define SHOW_ERR_MSGS


#define __ASM            __asm                                      /*!< asm keyword for GNU Compiler          */
#define __INLINE         inline                                     /*!< inline keyword for GNU Compiler       */
#define __STATIC_INLINE  static inline

#include <stdio.h>
#include <stdint.h>

#include "core_cmFunc.h"
#include "device.h"

#include "tmr_utils.h"
#include "wsf_types.h"
#include "wsf_os.h"

#define WAIT_MS(ms) {												\
						TMR_TO_Start(MXC_TMR3, MSEC(ms), NULL);			\
						while(TMR_TO_Check(MXC_TMR3) != E_TIME_OUT)	\
							wsfOsDispatcher();						\
					}

#define wait_us(us) {												\
						TMR_TO_Start(MXC_TMR3, us, NULL);			\
						while(TMR_TO_Check(MXC_TMR3) != E_TIME_OUT)	\
							wsfOsDispatcher();						\
					}

#define UNUSED(x)	 	(void)(x)


/*! \def enter_critical_section()
    \brief Disables interrupt request

    Details.
*/
#define enter_critical_section()		/*__disable_irq()*/

/*! \def exit_critical_section()
    \brief Enables interrupt request

    Details.
*/
#define exit_critical_section()			/*__enable_irq()*/

/*! \def platform_disable_interrupt()
    \brief Disables interrupt request

    Details.
*/
#define platform_disable_interrupt()	/*__disable_irq()*/

/*! \def platform_enable_interrupt()
    \brief Enables interrupt request

    Details.
*/
#define platform_enable_interrupt()		/*__enable_irq()*/

/*! \def ARRAY_SIZE(array)
    \brief Return the array size

    Details.
*/
#define ARRAY_SIZE(array)			(sizeof(array)/sizeof(array[0]))

/*! \def pr_debug(fmt, args...)
    \brief Prints the debug messages to the defined output handle

    Details.
*/
#ifdef SHOW_DEBUG_MSGS
#define pr_debug(fmt, args...) 	printf("\r\n[" fmt, ##args);
#else
	#define pr_debug(fmt, args...) (void)(fmt)
#endif

/*! \def pr_info(fmt, args...)
    \brief Prints the info messages to the defined output handle

    Details.
*/
#ifdef SHOW_INFO_MSGS
#define pr_info(fmt, args...) printf("\r\n[" fmt, ##args);
#else
	#define pr_info(fmt, args...) (void)(fmt)
#endif


#ifdef SHOW_ERR_MSGS
#define pr_err(fmt, args...) printf("\r\n[" fmt, ##args);
#else
	#define pr_err(fmt, args...) (void)(fmt)
#endif

/* Types */
/*signed integer types*/
typedef	signed char  s8;/**< used for signed 8bit */
typedef	signed short int s16;/**< used for signed 16bit */
typedef	signed int s32;/**< used for signed 32bit */
typedef	signed long long int s64;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char u8;/**< used for unsigned 8bit */
typedef	unsigned short int u16;/**< used for unsigned 16bit */
typedef	unsigned int u32;/**< used for unsigned 32bit */
typedef	unsigned long long int u64;/**< used for unsigned 64bit */

#endif /* UTILITIES_PERIPHERALS_H_ */
