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

#ifndef _MAX86176_REG_MAP_H_
#define _MAX86176_REG_MAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Peripherals.h"
#include "max86176_platform.h"
#include "queue.h"

/*	MAX86176 Registers	*/

/* Status registers */
#define MAX86176_STATUS1_REG				0x00

#define MAX86176_PWR_RDY_MASK				(0x01)
#define MAX86176_THRESH1_HILO_MASK			(0x01 << 1)
#define MAX86176_THRESH2_HILO_MASK			(0x01 << 2)
#define MAX86176_EXP_OVF_MASK				(0x01 << 3)
#define MAX86176_ALC_OVF_MASK				(0x01 << 4)
#define MAX86176_FIFO_DATA_RDY_MASK			(0x01 << 5)
#define MAX86176_FRAME_RDY_MASK				(0x01 << 6)
#define MAX86176_A_FULL_MASK				(0x01 << 7)

#define MAX86176_STATUS2_REG				0x01

#define MAX86176_INVALID_CFG_MASK			(0x01 << 2)
#define MAX86176_VDD_OOR_MASK				(0x01 << 6)
#define MAX86176_LED9_COMPB_MASK			(0x01 << 7)

#define MAX86176_STATUS3_REG				0x02

#define MAX86176_LED1_COMPB_MASK			(0x01)
#define MAX86176_LED2_COMPB_MASK			(0x01 << 1)
#define MAX86176_LED3_COMPB_MASK			(0x01 << 2)
#define MAX86176_LED4_COMPB_MASK			(0x01 << 3)
#define MAX86176_LED5_COMPB_MASK			(0x01 << 4)
#define MAX86176_LED6_COMPB_MASK			(0x01 << 5)
#define MAX86176_LED7_COMPB_MASK			(0x01 << 6)
#define MAX86176_LED8_COMPB_MASK			(0x01 << 7)

#define MAX86176_STATUS4_REG				0x03
#define MAX86176_STATUS5_REG				0x04
#define MAX86176_STATUS6_REG				0x05

/* FIFO registers */
#define MAX86176_FIFO_WR_PTR_REG			0x08
#define MAX86176_FIFO_WR_PTR_MASK			(0xFF)

#define MAX86176_FIFO_RD_PTR_REG			0x09
#define MAX86176_FIFO_RD_PTR_MASK			(0xFF)

#define MAX86176_OVF_CNT_REG				0x0A
#define MAX86176_OVF_CNT_MASK				(0x7F)
#define MAX86176_FIFO_DATA_COUNT_MSB_MASK	(0x01 << 7)

#define MAX86176_FIFO_DATA_CNT_REG			0x0B
#define MAX86176_FIFO_DATA_REG				0x0C

#define MAX86176_FIFO_CFG1_REG				0x0D
#define MAX86176_FIFO_A_FULL_MASK			(0xFF)

#define MAX86176_FIFO_CFG2_REG				0x0E
#define MAX86176_FIFO_RO_MASK				(0x01 << 1)
#define MAX86176_A_FULL_TYPE_MASK			(0x01 << 2)
#define MAX86176_FIFO_STAT_CLR_MASK			(0x01 << 3)
#define MAX86176_FLUSH_FIFO_MASK			(0x01 << 4)
#define MAX86176_FIFO_MARK_MASK				(0x01 << 5)

#define MAX86176_FIFO_RO_STOP				(0x00 << 1)
#define MAX86176_FIFO_RO_PUSH				(0x01 << 1)
#define MAX86176_A_FULL_RPT					(0x00 << 2)
#define MAX86176_A_FULL_ONCE				(0x01 << 2)
#define MAX86176_FIFO_STAT_RD_DATA_NOCLR	(0x00 << 3)
#define MAX86176_FIFO_STAT_RD_DATA_CLR		(0x01 << 3)

#define MAX86176_DATA_MASK					0x000FFFFF
#define MAX86176_TAG_MASK					0x00F00000

/* System Control registers */
#define MAX86176_SYSTEM_CFG1_REG			0x10
#define MAX86176_SYSTEM_RESET_MASK			(0x01)
#define MAX86176_SYSTEM_SHDN_MASK			(0x01 << 1)
#define MAX86176_PPG1_PWRDN_MASK			(0x01 << 2)
#define MAX86176_PPG2_PWRDN_MASK			(0x01 << 3)
#define MAX86176_SYNC_MODE_MASK				(0x03 << 4)
#define MAX86176_SW_FORCE_SYNC_MASK			(0x01 << 6)
#define MAX86176_TIMING_SYS_RESET_MASK		(0x01 << 7)

#define MAX86176_SYSTEM_CFG2_REG			0x11
#define MAX86176_MEAS1_EN_MASK				(0x01)
#define MAX86176_MEAS2_EN_MASK				(0x01 << 1)
#define MAX86176_MEAS3_EN_MASK				(0x01 << 2)
#define MAX86176_MEAS4_EN_MASK				(0x01 << 3)
#define MAX86176_MEAS5_EN_MASK				(0x01 << 4)
#define MAX86176_MEAS6_EN_MASK				(0x01 << 5)
#define MAX86176_MEAS7_EN_MASK				(0x01 << 6)
#define MAX86176_MEAS8_EN_MASK				(0x01 << 7)
#define MAX86176_MEAS_ALL_EN_MASK			(0xFF)

#define MAX86176_SYSTEM_CFG3_REG			0x12
#define MAX86176_MEAS1_CONFIG_SEL_MASK		(0x01)
#define MAX86176_COLLECT_RAW_DATA_MASK		(0x01 << 1)
#define MAX86176_PPG_TIMING_DATA_MASK		(0x01 << 2)
#define MAX86176_MASTER_TIMING_SYNC_MASK	(0x01 << 3)
#define MAX86176_ALC_DISABLE_MASK			(0x01 << 4)
#define MAX86176_EN_VDD_OOR_MASK			(0x01 << 6)
#define MAX86176_MEAS9_EN_MASK				(0x01 << 7)

#define MAX86176_SYSTEM_CFG4_REG			0x13
#define MAX86176_PROX_AUTO_MASK				(0x01)
#define MAX86176_PROX_DATA_EN_MASK			(0x01 << 1)
#define MAX86176_SAMP_SYNC_FREQ_MASK		(0x1F << 2)

#define MAX86176_PHOTO_DIODE_BIAS_REG		0x14
#define MAX86176_PD1_BIAS_MASK				(0x03)
#define MAX86176_PD2_BIAS_MASK				(0x03 << 2)
#define MAX86176_PD3_BIAS_MASK				(0x03 << 4)
#define MAX86176_PD4_BIAS_MASK				(0x03 << 6)

#define MAX86176_PIN_FUNC_CFG_REG			0x15
#define MAX86176_OUT_PIN_CFG_REG			0x16

#define MAX86176_PLL_CFG1_REG				0x18
#define MAX86176_PLL_CFG2_REG				0x19
#define MAX86176_PLL_CFG3_REG				0x1A

/* Frame Rate Clock registers */
#define MAX86176_FR_CLK_FREQ_REG			0x1C
#define MAX86176_FR_CLK_FINE_TUNE_MASK		(0x1F)
#define MAX86176_FR_CLK_SEL_MASK			(0x01 << 5)

#define MAX86176_FR_CLK_DIV_MSB_REG			0x1D
#define MAX86176_FR_CLK_DIV_H_MASK			(0x7F)

#define MAX86176_FR_CLK_32768_DIV_MSB_400HZ		0x00	// MSB=0x01, divider = d82 (0x52)
#define MAX86176_FR_CLK_32000_DIV_MSB_400HZ		0x00	// MSB=0x01, divider = d80 (0x50)

#define MAX86176_FR_CLK_32768_DIV_MSB_200HZ		0x00	// MSB=0x01, divider = d164 (0xA4)
#define MAX86176_FR_CLK_32000_DIV_MSB_200HZ		0x00	// MSB=0x01, divider = d160 (0xA0)

#define MAX86176_FR_CLK_32768_DIV_MSB_100HZ		0x01	// MSB=0x01, divider = d328 (0x148) 
#define MAX86176_FR_CLK_32000_DIV_MSB_100HZ		0x01	// MSB=0x01, divider = d320 (0x140)

#define MAX86176_FR_CLK_32768_DIV_MSB_50HZ		0x02	// MSB=0x05, divider = d655 (0x28F)
#define MAX86176_FR_CLK_32000_DIV_MSB_50HZ		0x02	// MSB=0x05, divider = d640 (0x280)

#define MAX86176_FR_CLK_32768_DIV_MSB_25HZ		0x05	// MSB=0x05, divider = d1311 (0x51F)  
#define MAX86176_FR_CLK_32000_DIV_MSB_25HZ		0x05	// MSB=0x05, divider = d1280 (0x500) 

#define MAX86176_FR_CLK_DIV_LSB_REG			0x1E
#define MAX86176_FR_CLK_DIV_L_MASK			(0xFF)

#define MAX86176_FR_CLK_32768_DIV_LSB_400HZ		0x52	// MSB=0x01, divider = d82 (0x52)
#define MAX86176_FR_CLK_32000_DIV_LSB_400HZ		0x50	// MSB=0x01, divider = d80 (0x50)

#define MAX86176_FR_CLK_32768_DIV_LSB_200HZ		0xA4	// MSB=0x01, divider = d164 (0xA4)
#define MAX86176_FR_CLK_32000_DIV_LSB_200HZ		0xA0	// MSB=0x01, divider = d160 (0xA0)

#define MAX86176_FR_CLK_32768_DIV_LSB_100HZ		0x48	// LSB=0x48, divider = d328 (0x148)  
#define MAX86176_FR_CLK_32000_DIV_LSB_100HZ		0x40	// LSB=0x40, divider = d320 (0x140) 

#define MAX86176_FR_CLK_32768_DIV_LSB_50HZ		0x8F	// MSB=0x05, divider = d655 (0x28F)
#define MAX86176_FR_CLK_32000_DIV_LSB_50HZ		0x80	// MSB=0x05, divider = d640 (0x280)

#define MAX86176_FR_CLK_32768_DIV_LSB_25HZ		0x1F	// LSB=0x1F, divider = d1311 (0x51F)
#define MAX86176_FR_CLK_32000_DIV_LSB_25HZ		0x00	// LSB=0x00, divider = d1280 (0x500) 

/* MEAS1 Setup registers */
#define MAX86176_MEAS1_SELECT_REG			0x20
#define MAX86176_MEAS1_CFG1_REG				0x21
#define MAX86176_MEAS1_CFG2_REG				0x22
#define MAX86176_MEAS1_CFG3_REG				0x23
#define MAX86176_MEAS1_CFG4_REG				0x24
#define MAX86176_MEAS1_LED_A_REG			0x25
#define MAX86176_MEAS1_LED_B_REG			0x26

/* MEAS2 Setup registers */
#define MAX86176_MEAS2_SELECT_REG			0x28
#define MAX86176_MEAS2_CFG1_REG				0x29
#define MAX86176_MEAS2_CFG2_REG				0x2A
#define MAX86176_MEAS2_CFG3_REG				0x2B
#define MAX86176_MEAS2_CFG4_REG				0x2C
#define MAX86176_MEAS2_LED_A_REG			0x2D
#define MAX86176_MEAS2_LED_B_REG			0x2E

/* MEAS3 Setup registers */
#define MAX86176_MEAS3_SELECT_REG			0x30
#define MAX86176_MEAS3_CFG1_REG				0x31
#define MAX86176_MEAS3_CFG2_REG				0x32
#define MAX86176_MEAS3_CFG3_REG				0x33
#define MAX86176_MEAS3_CFG4_REG				0x34
#define MAX86176_MEAS3_LED_A_REG			0x35
#define MAX86176_MEAS3_LED_B_REG			0x36

/* MEAS4 Setup registers */
#define MAX86176_MEAS4_SELECT_REG			0x38
#define MAX86176_MEAS4_CFG1_REG				0x39
#define MAX86176_MEAS4_CFG2_REG				0x3A
#define MAX86176_MEAS4_CFG3_REG				0x3B
#define MAX86176_MEAS4_CFG4_REG				0x3C
#define MAX86176_MEAS4_LED_A_REG			0x3D
#define MAX86176_MEAS4_LED_B_REG			0x3E

/* MEAS5 Setup registers */
#define MAX86176_MEAS5_SELECT_REG			0x40
#define MAX86176_MEAS5_CFG1_REG				0x41
#define MAX86176_MEAS5_CFG2_REG				0x42
#define MAX86176_MEAS5_CFG3_REG				0x43
#define MAX86176_MEAS5_CFG4_REG				0x44
#define MAX86176_MEAS5_LED_A_REG			0x45
#define MAX86176_MEAS5_LED_B_REG			0x46

/* MEAS6 Setup registers */
#define MAX86176_MEAS6_SELECT_REG			0x48
#define MAX86176_MEAS6_CFG1_REG				0x49
#define MAX86176_MEAS6_CFG2_REG				0x4A
#define MAX86176_MEAS6_CFG3_REG				0x4B
#define MAX86176_MEAS6_CFG4_REG				0x4C
#define MAX86176_MEAS6_LED_A_REG			0x4D
#define MAX86176_MEAS6_LED_B_REG			0x4E

/* MEAS7 Setup registers */
#define MAX86176_MEAS7_SELECT_REG			0x50
#define MAX86176_MEAS7_CFG1_REG				0x51
#define MAX86176_MEAS7_CFG2_REG				0x52
#define MAX86176_MEAS7_CFG3_REG				0x53
#define MAX86176_MEAS7_CFG4_REG				0x54
#define MAX86176_MEAS7_LED_A_REG			0x55
#define MAX86176_MEAS7_LED_B_REG			0x56

/* MEAS8 Setup registers */
#define MAX86176_MEAS8_SELECT_REG			0x58
#define MAX86176_MEAS8_CFG1_REG				0x59
#define MAX86176_MEAS8_CFG2_REG				0x5A
#define MAX86176_MEAS8_CFG3_REG				0x5B
#define MAX86176_MEAS8_CFG4_REG				0x5C
#define MAX86176_MEAS8_LED_A_REG			0x5D
#define MAX86176_MEAS8_LED_B_REG			0x5E

/* MEAS9 Setup registers */
#define MAX86176_MEAS9_SELECT_REG			0x60
#define MAX86176_MEAS9_CFG1_REG				0x61
#define MAX86176_MEAS9_CFG2_REG				0x62
#define MAX86176_MEAS9_CFG3_REG				0x63
#define MAX86176_MEAS9_CFG4_REG				0x64
#define MAX86176_MEAS9_LED_A_REG			0x65
#define MAX86176_MEAS9_LED_B_REG			0x66

/* MEASxSetup masks */
#define MAX86176_MEAS_DRV_PA_MASK			(0xFF)
//MEASx Selects
#define MAX86176_MEAS_DRVA_MASK				(0x07)
#define MAX86176_MEAS_DRVB_MASK				(0x07 << 3)
#define MAX86176_MEAS_AMB_MASK				(0x01 << 6)

//MEASx Config1
#define MAX86176_MEAS_AVER_MASK				(0x07)
#define MAX86176_MEAS_TINT_MASK				(0x03 << 3)

//MEASx Config2
#define MAX86176_MEAS_PPG1_ADC_RGE_MASK		(0x03)
#define MAX86176_MEAS_PPG2_ADC_RGE_MASK		(0x03 << 2)
#define MAX86176_MEAS_LED_RGE_MASK			(0x03 << 4)
#define MAX86176_MEAS_FILT_SEL_MASK			(0x01 << 6)
#define MAX86176_MEAS_SINC3_SEL_MASK		(0x01 << 7)

//MEASx Config3
#define MAX86176_MEAS_PPG1_DAC_OFF_MASK		(0x03)
#define MAX86176_MEAS_PPG2_DAC_OFF_MASK		(0x03 << 2)
#define MAX86176_MEAS_LED_SETLNG_MASK		(0x03 << 4)
#define MAX86176_MEAS_PD_SETLNG_MASK		(0x03 << 6)

//MEASx Config4
#define MAX86176_MEAS_PPG1_PDSEL_MASK		(0x03)
#define MAX86176_MEAS_PPG2_PDSEL_MASK		(0x03 << 2)
#define MAX86176_MEAS_PPG_GAIN_MASK			(0x03 << 4)
#define MAX86176_MEAS_BUFCHAN_MASK			(0x01 << 6)

/* LED Driver Values */
#define MAX86176_LED_DRV_A_PIN_1_MASK		(0x00)
#define MAX86176_LED_DRV_A_PIN_2_MASK		(0x01)
#define MAX86176_LED_DRV_A_PIN_3_MASK		(0x02)
#define MAX86176_LED_DRV_A_PIN_4_MASK		(0x03)
#define MAX86176_LED_DRV_A_PIN_5_MASK		(0x04)
#define MAX86176_LED_DRV_A_PIN_6_MASK		(0x05)

#define MAX86176_LED_DRV_B_PIN_1_MASK		(0x00 << 3)
#define MAX86176_LED_DRV_B_PIN_2_MASK		(0x01 << 3)
#define MAX86176_LED_DRV_B_PIN_3_MASK		(0x02 << 3)
#define MAX86176_LED_DRV_B_PIN_4_MASK		(0x03 << 3)
#define MAX86176_LED_DRV_B_PIN_5_MASK		(0x04 << 3)
#define MAX86176_LED_DRV_B_PIN_6_MASK		(0x05 << 3)

/* TINT  Values */
#define MAX86176_TINT_14_US_MASK			(0 << 3)
#define MAX86176_TINT_29_US_MASK			(1 << 3)
#define MAX86176_TINT_59_US_MASK			(2 << 3)
#define MAX86176_TINT_117_US_MASK			(3 << 3)

/* PPG PD Select values */
#define MAX86176_PPG1_PD1_MASK				(0x00)
#define MAX86176_PPG1_PD2_MASK				(0x01)
#define MAX86176_PPG1_PD3_MASK				(0x02)
#define MAX86176_PPG1_PD4_MASK				(0x03)
#define MAX86176_PPG2_PD1_MASK				(0x00 << 2)
#define MAX86176_PPG2_PD2_MASK				(0x01 << 2)
#define MAX86176_PPG2_PD3_MASK				(0x02 << 2)
#define MAX86176_PPG2_PD4_MASK				(0x03 << 2)

#define MAX86176_PPG_GAIN_1_MASK			(0x00 << 4)
#define MAX86176_PPG_GAIN_2_MASK			(0x01 << 4)
#define MAX86176_PPG_GAIN_4_MASK			(0x02 << 4)
	
#define MAX86176_PPG_BUFFCHAN_ADC1_MASK		(0x00 << 6)
#define MAX86176_PPG_BUFFCHAN_ADC2_MASK		(0x01 << 6)

/* ADC RGE Values */
#define MAX86176_PPG1_ADC_RGE_4uA_MASK		(0x00)
#define MAX86176_PPG1_ADC_RGE_8uA_MASK		(0x01)
#define MAX86176_PPG1_ADC_RGE_16uA_MASK		(0x02)
#define MAX86176_PPG1_ADC_RGE_32uA_MASK		(0x03)

#define MAX86176_PPG2_ADC_RGE_4uA_MASK		(0x00 << 2)
#define MAX86176_PPG2_ADC_RGE_8uA_MASK		(0x01 << 2)
#define MAX86176_PPG2_ADC_RGE_16uA_MASK		(0x02 << 2)
#define MAX86176_PPG2_ADC_RGE_32uA_MASK		(0x03 << 2)

/* LED RGE Values */
#define MAX86176_LED_RGE_32mA_MASK			(0x00 << 4)
#define MAX86176_LED_RGE_64mA_MASK			(0x01 << 4)
#define MAX86176_LED_RGE_96mA_MASK			(0x02 << 4)
#define MAX86176_LED_RGE_128mA_MASK			(0x03 << 4)

/* Average Values */
#define MAX86176_AVERAGE_1_MASK				(0)
#define MAX86176_AVERAGE_2_MASK				(1)
#define MAX86176_AVERAGE_4_MASK				(2)
#define MAX86176_AVERAGE_8_MASK				(3)
#define MAX86176_AVERAGE_16_MASK			(4)
#define MAX86176_AVERAGE_32_MASK			(5)
#define MAX86176_AVERAGE_64_MASK			(6)
#define MAX86176_AVERAGE_128_MASK			(7)

/* Threshold interrupt registers */
#define MAX86176_THR_MEAS_SEL_REG			0x70
#define MAX86176_THR1_MEAS_SEL_MASK			(0xF)
#define MAX86176_THR2_MEAS_SEL_MASK			(0xF << 4)

#define MAX86176_THR_HYST_REG				0x71
#define MAX86176_LEVEL_HYST_MASK			(0x7)
#define MAX86176_TIME_HYST_MASK				(0x3 << 3)
#define MAX86176_THRESH1_PPG_SEL_MASK		(0x1 << 6)
#define MAX86176_THRESH2_PPG_SEL_MASK		(0x1 << 7)

#define MAX86176_PPG_UPPER_THR1_REG			0x72
#define MAX86176_PPG_LOWER_THR1_REG			0x73
#define MAX86176_PPG_UPPER_THR2_REG			0x74
#define MAX86176_PPG_LOWER_THR2_REG			0x75


/* Picket Fence registers */
/*
#define MAX86176_PICKET_FENCE_SEL_REG		0x70
#define MAX86176_PPG1_PF_MEAS_SEL_MASK		(0xF)
#define MAX86176_PPG2_PF_MEAS_SEL_MASK		(0xF << 4)

#define MAX86176_PICKET_FENCE_CFG_REG		0x71
#define MAX86176_THRESH_SIGMA_MULT_MASK		(0x3 << 0)
#define MAX86176_IIR_INIT_VAL_MASK			(0x3 << 2)
#define MAX86176_IIR_TC_MASK				(0x3 << 4)
#define MAX86176_PF_ORDER_MASK				(0x1 << 6)*/

/* Interrupts enable registers */
#define MAX86176_INT1_ENABLE1_REG			0x80
#define MAX86176_INT1_ENABLE2_REG			0x81
#define MAX86176_INT1_ENABLE3_REG			0x82
#define MAX86176_INT1_ENABLE4_REG			0x83
#define MAX86176_INT1_ENABLE5_REG			0x84
#define MAX86176_INT1_ENABLE6_REG			0x85
#define MAX86176_INT2_ENABLE1_REG			0x86
#define MAX86176_INT2_ENABLE2_REG			0x87
#define MAX86176_INT2_ENABLE3_REG			0x88
#define MAX86176_INT2_ENABLE4_REG			0x89
#define MAX86176_INT2_ENABLE5_REG			0x8A
#define MAX86176_INT2_ENABLE6_REG			0x8B

#define MAX86176_LED_TX_EN_MASK				(0x01)
#define MAX86176_THRESH1_HILO_EN_MASK		(0x01 << 1)
#define MAX86176_THRESH2_HILO_EN_MASK		(0x01 << 2)
#define MAX86176_EXP_OVF_EN_MASK			(0x01 << 3)
#define MAX86176_ALC_OVF_EN_MASK			(0x01 << 4)
#define MAX86176_FIFO_DATA_RDY_EN_MASK		(0x01 << 5)
#define MAX86176_FRAME_RDY_EN_MASK			(0x01 << 6)
#define MAX86176_A_FULL_EN_MASK				(0x01 << 7)
#define MAX86176_INVALID_CFG_EN_MASK		(0x01 << 2)
#define MAX86176_VDD_OOR_EN_MASK			(0x01 << 6)
#define MAX86176_LED1_COMPB_EN_MASK			(0x01)
#define MAX86176_LED2_COMPB_EN_MASK			(0x01 << 1)
#define MAX86176_LED3_COMPB_EN_MASK			(0x01 << 2)
#define MAX86176_LED4_COMPB_EN_MASK			(0x01 << 3)
#define MAX86176_LED5_COMPB_EN_MASK			(0x01 << 4)
#define MAX86176_LED6_COMPB_EN_MASK			(0x01 << 5)
#define MAX86176_TIMING_SYS_RESET_EN_MASK	(0x01 << 7)

/* EGC Configuration registers */
#define MAX86176_EGC_CONFIG_1_REG			0x90
#define MAX86176_EGC_CONFIG_2_REG			0x91
#define MAX86176_EGC_CONFIG_3_REG			0x92

/* Lead Detect Confiuration registers */
#define MAX86176_LEAD_DETECT_CFG_1_REG		0x93
#define MAX86176_LEAD_DETECT_CFG_2_REG		0x94
#define MAX86176_AC_LEAD_DETECT_WAVE_REG	0x95
#define MAX86176_DC_LEAD_DETECT_DAC_REG		0x96
#define MAX86176_DC_LEAD_OFF_THR_REG		0x97
#define MAX86176_AC_LEAD_OFF_THR_1_REG		0x98
#define MAX86176_AC_LEAD_OFF_THR_2_REG		0x99
#define MAX86176_AC_LEAD_OFF_PGA_HPF_REG	0x9A
#define MAX86176_AC_LEAD_OFF_CAL_RES_REG	0x9B
#define MAX86176_LEAD_BIAS_CFG_1_REG		0x9E
#define MAX86176_CAL_CFG_1_REG				0xA0
#define MAX86176_CAL_CFG_2_REG				0xA1
#define MAX86176_CAL_CFG_3_REG				0xA2
#define MAX86176_RLD_CFG_1_REG				0xA8
#define MAX86176_RLD_CFG_2_REG				0xA9
#define MAX86176_UTIL_ADC_CFG_1_REG			0xE0
#define MAX86176_UTIL_ADC_DATA_HIGH_REG		0xE1
#define MAX86176_UTIL_ADC_DATA_LOW_REG		0xE2

#define MAX86176_MAX_PPG_VALUE				(524287)

/* Rev and Par Id registers */
#define MAX86176_REV_ID_REG					0xFE
#define MAX86176_PART_ID_REG				0xFF

#define MAX86176_DATA_WORD_SIZE		3
#define MAX86176_MAX_FIFO_DEPTH		256
#define MAX86176_DATA_TYPE_MASK		(0xF << 19)
#define MAX86176_FIFO_REM			(MAX86176_MAX_FIFO_DEPTH - (25*3))
#define MAX86176_DRIVER_FIFO_SZ		(MAX86176_MAX_FIFO_DEPTH * 8)

#define MAX86140_PART_ID_VAL	0x24 // Single PD with SPI serial Interface
#define MAX86141_PART_ID_VAL	0x25 // Dual PD with SPI serial Interface
#define MAX86142_PART_ID_VAL	0x26 // Single PD with I2C serial Interface
#define MAX86143_PART_ID_VAL	0x27 // Dual PD with I2C serial Interface
#define MAX86161_PART_ID_VAL	0x36 // green/IR/red (LED1/2/3) module; single PD with I2C serial interface
#define MAX86171_PART_ID_VAL	0x2C // MAX86171
#define MAX86176_PART_ID_VAL	0x39 // MAX86176

#define OS64L_I2C_ADDRESS 0x66//(0x64)
#define OS64H_I2C_ADDRESS 0x67//(0x65)


#define PWR_ON		true
#define PWR_OFF		false

/* Configuration */
#define NUM_SAMPLES_PER_CHANNEL 	2	// 2 PD

#define MAX86176_LED_NUM   			9 	//for OS61

//#define MAX86176_FIFO_AFULL   (MAX86176_MAX_FIFO_DEPTH - (MAX86176_LED_NUM * 25))     //DO NOT EXCEED 256 !!!!!
#define MAX86176_FIFO_AFULL	 (MAX86176_MAX_FIFO_DEPTH - 2)

#define THREE_MEASEREMENT_MASK  ((1 << DATA_TYPE_MEAS1) | \
						 	 	(1 << DATA_TYPE_MEAS2) | \
						 	 	(1 << DATA_TYPE_MEAS3))


#define ALL_MEASEREMENT_MASK   ((1 << DATA_TYPE_MEAS1) | \
						 	 	(1 << DATA_TYPE_MEAS2) | \
						 	 	(1 << DATA_TYPE_MEAS3) | \
								(1 << DATA_TYPE_MEAS4) | \
								(1 << DATA_TYPE_MEAS5) | \
								(1 << DATA_TYPE_MEAS6) | \
								(1 << DATA_TYPE_MEAS7) | \
								(1 << DATA_TYPE_MEAS8) | \
								(1 << DATA_TYPE_MEAS9))

#define MAX86176_MEAS_TINT_POS				(3)
#define MAX86176_MEAS_AVG_POS				(0)

#define MAX86176_MEAS_PPG1_DAC_OFF_POS		(0)
#define MAX86176_MEAS_PPG2_DAC_OFF_POS		(2)

/* end of Configuration */

typedef enum {
	MAX86176_PPG1 = 0,
	MAX86176_PPG2,
}max86176_ppg_num_t;

/* OS64's LEDS */
typedef enum {
	MAX86176_LED_GREEN = 0u,
	MAX86176_LED_RED,
	MAX86176_LED_IR,

	MAX86176_LED_MAX
} max86176_leds_t;

// DATA TYPES
typedef enum max86176_channels {
	DATA_TYPE_MEAS1 = 0x00,
	DATA_TYPE_MEAS2 = 0x01,
	DATA_TYPE_MEAS3 = 0x02,
	DATA_TYPE_MEAS4 = 0x03,
	DATA_TYPE_MEAS5 = 0x04,
	DATA_TYPE_MEAS6 = 0x05,
	DATA_TYPE_MEAS7 = 0x06,
	DATA_TYPE_MEAS8 = 0x07,
	DATA_TYPE_MEAS9 = 0x08,
	DATA_TYPE_DARK = 0x09,
	DATA_TYPE_ALC_OVF = 0x0A,
	DATA_TYPE_EXP_OVF = 0x0B,
	DATA_TYPE_PF = 0x0C,
	DATA_TYPE_INVALID = 0x0E,
	DATA_TYPE_RESERVED = 0x0F,
} max86176_channels_t;

typedef union {
	struct {
		uint32_t val:20;
		uint32_t type:4;
		uint32_t:8;
	};
	uint32_t raw;
} fifo_data_t;

typedef union {
	uint16_t val;
	struct {
		uint8_t tint;
		uint8_t frac:4;
		uint8_t:4;
	};
} die_temp_t;

/* Status registers */
union int_status {
	struct {
		struct {
			unsigned char pwr_rdy:1;
			unsigned char thresh1_hilo:1; 
			unsigned char thresh2_hilo:1; 
			unsigned char exp_ovf:1; 
			unsigned char alc_ovf:1; 
			unsigned char fifo_data_rdy:1; 
			unsigned char frame_rdy:1; 
			unsigned char a_full:1;
		};
		struct {
			unsigned char reserved_1:2;
			unsigned char invalid_cfg:1;
			unsigned char reserved_2:3; 
			unsigned char vdd_oor:1; 
			unsigned char timing_sys_reset:1; 
		};
		struct {
			unsigned char led1_compb:1;
			unsigned char led2_compb:1; 
			unsigned char led3_compb:1; 
			unsigned char led4_compb:1; 
			unsigned char led5_compb:1; 
			unsigned char led6_compb:1; 
			unsigned char reserved_3:2; 
		};
	};
	uint8_t val[2];
};

struct max86176_dev {
	dev_comm_t *comm;
	queue_t queue;
    int regulator_state;
    int curr_state;
    int int_gpio;
    int vdd_oor_cnt;
    int irq;
	die_temp_t die_temp;
	uint8_t part_id;
	uint8_t rev_id;
	uint8_t num_pd;
	union {
		struct {
			/*
				Choose what settings will be used; user settings/firmware settings.
				Deault value is 1, which is firmware default
			*/
			uint8_t firmware_default:1;
			/*
				If DAC Calibration procedure will be run before settings register.
				Default value is 1, which is enabled.
			*/
			uint8_t dac_calib:1;
			uint8_t rfu:6;
		};
		uint8_t val;
	} operating_cfg;

	union {
		struct {
			uint8_t ppg_tint:2;
			uint8_t ppg1_adc_rge:2;
			uint8_t ppg2_adc_rge:2;
			uint8_t add_offset:1;
			uint8_t alc_dis:1;
		};
		uint8_t val;
	} ppg_cfg1;

	uint8_t dac_calib_status;
};

typedef struct max86176_dev max86176_dev_t;

typedef int (*max86176_fifo_read_cb)(void);

int max86176_sensor_enable(struct max86176_dev *sd, int agc_enable, int enable);
void *max86176_get_device_data(void);
int max86176_init(pdata_t *pdata);
int max86176_dump_regs(dev_comm_t *port, uint8_t *buf,
		uint8_t start_addr, uint8_t end_addr);
int max86176_get_part_info(dev_comm_t *comm,
		uint8_t *part_id, uint8_t *rev_id, uint8_t *num_pd);

int max86176_set_sample_rate(struct max86176_dev *sd, uint16_t rate);
int max86176_get_sample_rate(struct max86176_dev *sd);
int max86176_fifo_irq_handler(struct max86176_dev *sd);
int max86176_irq_handler(void *arg);
int max86176_get_irq_state(void *data);
void max86176_irq_clr();
void max86176_irq_clr_to_zero();
void max86176_irq_reset_ref_cnt();
int max86176_run_dac_calibration(struct max86176_dev *sd, uint8_t ppg_cfg1);
int max86176_get_meas_num(struct max86176_dev *sd);

// sensor on off reset
int max86176_reset (struct max86176_dev *sd);
int max86176_poweron (struct max86176_dev *sd);
int max86176_poweroff (struct max86176_dev *sd);

int max86176_get_num_of_channel(uint8_t * p_num_ch);
int max86176_is_ppg_enabled(uint8_t * p_ppg_enabled);
int max86176_is_ecg_enabled(uint8_t *p_ecg_enabled);
int max86176_is_iq_enabled(uint8_t * p_iq_enabled);
int max86176_is_ecg_faster_than_ppg(uint8_t *p_is_true);
int max86176_get_num_photo_diodes(uint8_t *p_num_diode);
int max86176_is_enabled(uint8_t * p_is_enabled);
int max86176_set_frame_ready_int(uint8_t enable);
int max86176_set_a_full_int(uint8_t enable);
int max86176_set_fifo_a_full(uint8_t level);

void max86176_register_fifo_read_callback(max86176_fifo_read_cb func);
void max86176_unregister_fifo_read_callback();

typedef enum {
	MAX86176_MEAS_CH1 = 0,
	MAX86176_MEAS_CH2,
	MAX86176_MEAS_CH3,
	MAX86176_MEAS_CH4,
	MAX86176_MEAS_CH5,
	MAX86176_MEAS_CH6,
	MAX86176_MEAS_CH7,
	MAX86176_MEAS_CH8,
	MAX86176_MEAS_CH9,
	MAX86176_MEAS_CHMAX
} max86176_meas_ch_t;

typedef enum {
	MAX86176_LED_DRIVER_A = 0,
	MAX86176_LED_DRIVER_B,
	MAX86176_LED_DRIVER_MAX
} max86176_led_type;

int max86176_get_list_of_enabled_channels(uint8_t * p_channels);
int max86176_set_leds_current(max86176_meas_ch_t ch, max86176_led_type led_type, uint8_t led_current);
int max86176_get_leds_current(max86176_meas_ch_t ch, max86176_led_type led_type, uint8_t * led_current);
int max86176_get_led_current_steps(max86176_meas_ch_t ch, float * curr_per_step);
int max86176_is_ppg1_enabled(uint8_t * p_is_enabled);
int max86176_is_ppg2_enabled(uint8_t * p_is_enabled);

int max86176_turn_led_on(max86176_leds_t led);
int max86176_turn_led_off(max86176_leds_t led);
int max86176_disable_all_ch();
int max86176_clear_fifo();
int max86176_set_integration_time(uint8_t meas, uint8_t int_time);
int max86176_set_average_sample(uint8_t meas, unsigned int avg_sample);
int max86176_set_dac_offset(uint8_t meas, uint8_t pd_sel, uint8_t offset);

int max86176_fill_header_data(uint8_t header[126]);

// end of sensor on off reset
#ifdef __cplusplus
}
#endif
#endif//_MAX86176_REG_MAP_H_
