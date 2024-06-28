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

#include "max17260.h"
#include "max17260_platform.h"
#include "max17260_registers.h"
#include "mxc_errors.h"
#include "Peripherals.h"

#include <stdio.h>

/****************************************************************************/
/*  STATIC FUNCTION DECLERATIONS                                            */
/****************************************************************************/

#define MAX17260_STATUS_BIT_POR         UINT16_C(1 << 2)
#define MAX17260_FSTAT_BIT_DNR          UINT16_C(1 << 0)
#define MAX17260_MODEL_CFG_BIT_REFRESH  UINT16_C(1 << 15)

#define MAX17260_CMD_CLEAR        UINT16_C(0x0000)
#define MAX17260_CMD_SOFT_WAKEUP  UINT16_C(0x0090)

#define MAX17260_DESIGN_CAP  UINT16_C(380)
#define MAX17260_ICHG_TERM   UINT16_C(121)
#define MAX17260_VEMPTY      UINT16_C(0)

#define MAX17260_VCHG_4V2    UINT16_C(0x8000)
#define MAX17260_VCHG_4V4    UINT16_C(0x8400)

#define MAX17260_SOC_TO_BATTERY_PERCENT	(((float)1/256.0))

/****************************************************************************/
/*  STATIC FUNCTION DECLERATIONS                                            */
/****************************************************************************/

static void max17260_load_config(void);
static void max17260_defaults(void);
static void max17260_soft_wakeup(void);

/****************************************************************************/
/*  GLOBAL FUNCTION DEFINITIONS                                             */
/****************************************************************************/

int max17260_init(void)
{
    int      result = 0;
    uint16_t status;

    max17260_platform_read_register(MAX17260_REG_STATUS, &status);

    if (0 != (status & MAX17260_STATUS_BIT_POR))
    {
        max17260_load_config();
    }

    max17260_platform_write_register(MAX17260_REG_DESIGN_CAP, MAX17260_DESIGN_CAP);
    max17260_platform_write_register(MAX17260_REG_ICHG_TERM, MAX17260_ICHG_TERM);
    max17260_platform_write_register(MAX17260_REG_MODEL_CFG, MAX17260_MODEL_CFG_BIT_REFRESH);

    return result;
}

void max17260_display_values(void)
{
    uint16_t cap;
    uint16_t soc;
    uint16_t tte;

    max17260_platform_read_register(MAX17260_REG_REP_CAP, &cap);
    max17260_platform_read_register(MAX17260_REG_REP_SOC, &soc);
    max17260_platform_read_register(MAX17260_REG_REP_TTE, &tte);

    pr_info("Cap: %d \t Soc: %d\n", cap, soc);
}

int max17260_get_battery_percent(uint8_t *p_batt)
{
	int ret = 0;
	uint16_t soc;
	float batt_percent = 0;

	ret = max17260_platform_read_register(MAX17260_REG_REP_SOC, &soc);

	if(ret)
	{
		batt_percent = (float)soc * MAX17260_SOC_TO_BATTERY_PERCENT;

		*p_batt = (uint8_t)batt_percent;
	}

	return ret > 0 ? 0 : -1;
}

/****************************************************************************/
/*  STATIC FUNCTION DEFINITIONS                                             */
/****************************************************************************/

static void max17260_load_config(void)
{
    uint16_t status;
    uint16_t fstat;
    uint16_t hib_cfg;

    do
    {
        max17260_platform_read_register(MAX17260_REG_FSTAT, &fstat);
    } while (0 != (fstat & MAX17260_FSTAT_BIT_DNR));

    max17260_platform_read_register(MAX17260_REG_HIB_CFG, &hib_cfg);

    max17260_soft_wakeup();
    max17260_defaults();

    max17260_platform_write_register(MAX17260_REG_HIB_CFG, hib_cfg);
    max17260_platform_read_register(MAX17260_REG_STATUS, &status);

    do
    {
        status &= ~MAX17260_STATUS_BIT_POR;

        max17260_platform_write_register(MAX17260_REG_STATUS, status);
        max17260_platform_read_register(MAX17260_REG_STATUS, &status);
    } while (0 != (status & MAX17260_STATUS_BIT_POR));
}

static void max17260_defaults(void)
{
    max17260_platform_write_register(MAX17260_REG_DESIGN_CAP, MAX17260_DESIGN_CAP);
    max17260_platform_write_register(MAX17260_REG_ICHG_TERM, MAX17260_ICHG_TERM);
    max17260_platform_write_register(MAX17260_REG_VEMPTY, MAX17260_VEMPTY);

    if (1)
    {
        max17260_platform_write_register(MAX17260_REG_MODEL_CFG, MAX17260_VCHG_4V4);
    }
    else
    {
        max17260_platform_write_register(MAX17260_REG_MODEL_CFG, MAX17260_VCHG_4V2);
    }

    uint16_t model_cfg;

    do
    {
        max17260_platform_read_register(MAX17260_REG_MODEL_CFG, &model_cfg);
    } while (0 != (model_cfg & MAX17260_MODEL_CFG_BIT_REFRESH));
}

static void max17260_soft_wakeup(void)
{
    max17260_platform_write_register(MAX17260_REG_COMMAND, MAX17260_CMD_SOFT_WAKEUP);
    max17260_platform_write_register(MAX17260_REG_HIB_CFG, MAX17260_CMD_CLEAR);
    max17260_platform_write_register(MAX17260_REG_COMMAND, MAX17260_CMD_CLEAR);
}
