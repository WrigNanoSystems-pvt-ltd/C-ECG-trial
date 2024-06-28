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
#ifndef DRIVERS_MAX20356_REGISTERS_H
#define DRIVERS_MAX20356_REGISTERS_H

/****************************************************************************/
/*  PMIC DRIVER REGISTER TABLE                                              */
/****************************************************************************/

#define MAX20356_PMIC_REG_CHIP_ID           UINT8_C(0x00)
#define MAX20356_PMIC_REG_STATUS0           UINT8_C(0x01)
#define MAX20356_PMIC_REG_STATUS1           UINT8_C(0x02)
#define MAX20356_PMIC_REG_STATUS2           UINT8_C(0x03)
#define MAX20356_PMIC_REG_STATUS3           UINT8_C(0x04)
#define MAX20356_PMIC_REG_STATUS4           UINT8_C(0x05)

#define MAX20356_PMIC_REG_INT_1             UINT8_C(0x08)
#define MAX20356_PMIC_REG_INTMASK_1			UINT8_C(0x0E)

#define MAX20356_PMIC_REG_IVMON_CFG         UINT8_C(0x29)

#define MAX20356_PMIC_REG_BUCK_1_ENA        UINT8_C(0x30)
#define MAX20356_PMIC_REG_BUCK_1_ISET		UINT8_C(0x33)
#define MAX20356_PMIC_REG_BUCK_1_VSET       UINT8_C(0x34)

#define MAX20356_PMIC_REG_BUCK_2_ENA        UINT8_C(0x3C)
#define MAX20356_PMIC_REG_BUCK_2_VSET       UINT8_C(0x40)

#define MAX20356_PMIC_REG_BUCK_3_ENA        UINT8_C(0x48)
#define MAX20356_PMIC_REG_BUCK_3_VSET       UINT8_C(0x4C)

#define MAX20356_PMIC_REG_BBST_ENA          UINT8_C(0x54)
#define MAX20356_PMIC_REG_BBST_VSET         UINT8_C(0x56)

#define MAX20356_PMIC_REG_LDO1_ENA          UINT8_C(0x5A)
#define MAX20356_PMIC_REG_LDO1_VSET         UINT8_C(0x5C)

#define MAX20356_PMIC_REG_LDO2_ENA          UINT8_C(0x5E)
#define MAX20356_PMIC_REG_LDO2_CFG			UINT8_C(0x5F)
#define MAX20356_PMIC_REG_LDO2_VSET         UINT8_C(0x60)
#define MAX20356_PMIC_REG_LDO2_CTR			UINT8_C(0x61)

#define MAX20356_PMIC_REG_LDO3_ENA          UINT8_C(0x62)
#define MAX20356_PMIC_REG_LDO3_VSET         UINT8_C(0x64)

//#define MAX20356_PMIC_REG_LDO4_ENA          UINT8_C(0x5E)
//#define MAX20356_PMIC_REG_LDO4_VSET         UINT8_C(0x60)

#define MAX20356_PMIC_REG_LSW_1_ENA         UINT8_C(0x69)
#define MAX20356_PMIC_REG_LSW_2_ENA         UINT8_C(0x6C)
#define MAX20356_PMIC_REG_LSW_3_ENA         UINT8_C(0x6F)

#define MAX20356_PMIC_REG_MPC_0             UINT8_C(0x72)
#define MAX20356_PMIC_REG_MPC_1             UINT8_C(0x73)
#define MAX20356_PMIC_REG_MPC_2             UINT8_C(0x74)
#define MAX20356_PMIC_REG_MPC_3             UINT8_C(0x75)

#define MAX20356_PMIC_REG_BOOT_CFG			UINT8_C(0x81)
#define MAX20356_PMIC_REG_PWR_CMD			UINT8_C(0x83)

#define MAX20356_PMIC_REG_LOCK_MSK_1          UINT8_C(0x86)
#define MAX20356_PMIC_REG_LOCK_MSK_2          UINT8_C(0x87)
#define MAX20356_PMIC_REG_LOCK_MSK_3          UINT8_C(0x89)

#define MAX20356_PMIC_REG_LOCK_UNLOCK_1       UINT8_C(0x8A)
#define MAX20356_PMIC_REG_LOCK_UNLOCK_2       UINT8_C(0x8B)
#define MAX20356_PMIC_REG_LOCK_UNLOCK_3       UINT8_C(0x8C)

#define MAX20356_PMIC_REG_MPCITRSTS			UINT8_C(0x7A)
#define MAX20356_PMIC_REG_USBOKITRCFG 		UINT8_C(0x7E)

/****************************************************************************/
/*  ADC DRIVER REGISTER TABLE                                               */
/****************************************************************************/

#define MAX20356_ADC_REG_ADC_EN       UINT8_C(0x50)
#define MAX20356_ADC_REG_ADC_CFG      UINT8_C(0x51)
#define MAX20356_ADC_REG_ADC_DAT_AVG  UINT8_C(0x53)
#define MAX20356_ADC_REG_ADC_DAT_MIN  UINT8_C(0x54)
#define MAX20356_ADC_REG_ADC_DAT_MAX  UINT8_C(0x55)

#endif  /* DRIVERS_MAX20356_REGISTERS_H */
