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


#include "i2c_ah_sh_api.h"

#include "Peripherals.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_config.h"
#include "i2c.h"


/***** Definitions *****/
#if  defined(FTHR) || defined(FTHR2)
#warning I2C ah sh api MAX3266FTHR Defined
#define I2C_MASTER	    		MXC_I2C0_BUS0
#elif MRD104
#warning I2C ah sh api MRD104 Board Defined
#define I2C_MASTER	    		MXC_I2C2_BUS0
#elif MRD106
#warning I2C ah sh api MRD106 Board Defined
#define I2C_MASTER	    		MXC_I2C2_BUS0
#endif

int i2c_init()
{
	int ret = 0;

    //Setup the I2CM
    ret = I2C_Shutdown(I2C_MASTER);
    pr_info("I2C_Shutdown %d \n", ret);

    if(0 == ret){
    	ret = I2C_Init(I2C_MASTER, I2C_FAST_MODE , NULL);
    	pr_info("I2C_Init %d \n", ret);
    }

    if(0 == ret)
    	NVIC_EnableIRQ(I2C2_IRQn);

    return ret;
}

int i2c_write(uint8_t addr, const char * p_data, int length, bool restart)
{
	int ret = 0;

	ret = I2C_MasterWrite(I2C_MASTER, addr, (uint8_t *)p_data, length, (int)restart);

	if(ret == length)
		return 0;
	else
		return -1;
}

int i2c_read(uint8_t addr, char * p_data, int length, bool restart)
{
	int ret = 0;

	ret =  I2C_MasterRead(I2C_MASTER, addr, (uint8_t *)p_data, length, (int)restart);

	if(ret == length)
		return 0;
	else
		return -1;
}

void i2c_close()
{
    I2C_Shutdown(I2C_MASTER);
}
