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

#ifndef _BUTTON_H_
#define _BUTTON_H_

/*******************************      INCLUDES    ****************************/


/*******************************      DEFINES     ****************************/
#define BUTTON_0		0
#define BUTTON_1		1


/******************************* Type Definitions ****************************/
/**
 * Type alias @c pb_callback for the push button callback.
 * @details The function is of type:
 * @code
 *  void pb_callback(void * pb)
 * @endcode
 * To receive notification of a push button event, define a callback
 * function and pass it as a pointer to the PB_RegisterCallback(unsigned int pb, pb_callback callback) function.
 * @param      pb    Pointer to the push button index that triggered the
 *                   callback.
 */
typedef void (*button_callback_t)(void *pb);



typedef enum{
	BUTTON_FALLING_EDGE_INT = 0,
	BUTTON_HIGH_LEVEL_INT,
	BUTTON_RISING_EDGE_INT,
	BUTTON_LOW_LEVEL_INT,
	BUTTON_BOTH_EDGE_INT,

	BUTTON_MAX_INT

} button_int_sel_t;



/******************************* Public Functions ****************************/
/**
 * @brief      Initialize all push buttons.
 * @return  \c #E_NO_ERROR  Push buttons initialized successfully.
 * @return     "Error Code" @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int button_init(void);

/**
 * @brief      Register or Unregister a callback handler for events on the @p pb push button.
 * @details
 * - Calling this function with a pointer to a function @p callback, configures the pushbutton @p pb and enables the
 * interrupt to handle the push button events.
 * - Calling this function with a <tt>NULL</tt> pointer will disable the interrupt and unregister the
 * callback function.
 * @p pb must be a value between 0 and \c num_pbs.
 *
 * @param      pb        push button index to receive event callbacks.
 * @param      callback  Callback function pointer of type @c pb_callback
 * @return     #E_NO_ERROR if configured and callback registered successfully.
 * @return     "Error Code" @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int button_register_cb(unsigned int pb, button_callback_t callback, button_int_sel_t type);

/**
 * @brief   Enable a callback interrupt.
 * @note    PB_RegisterCallback must be called prior to enabling the callback interrupt.
 * @param   pb          push button index value between 0 and \c num_pbs.
 */
void button_int_enable(unsigned int pb);

/**
 * @brief   Disable a callback interrupt.
 * @param   pb          push button index
 */
void button_int_disable(unsigned int pb);

/**
 * @brief   Clear a callback interrupt.
 * @param   pb          push button index value between 0 and \c num_pbs.
 */
void button_int_clear(unsigned int pb);

/**
 * @brief      Get the current state of the push button.
 * @param      pb     push button index value between 0 and \c num_pbs.
 * @return     TRUE   The button is pressed.
 * @return     FALSE  The button is not pressed.
 */
int button_get(unsigned int pb);

/**
 * @brief      Get the current callback pointer.
 * @param      pb     push button index value between 0 and \c num_pbs.
 * @return     Callback function.
 */
button_callback_t button_get_cb(unsigned int pb);


#endif /* _BUTTON_H_ */
