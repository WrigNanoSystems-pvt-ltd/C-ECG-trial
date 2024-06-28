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

#include "gui.h"

#include <stdint.h>
#include <string.h>

#include "app_gui.h"
#include "sensorhub.h"
#include "tmr.h"
#include "ble_api.h"
#include "nvic_table.h"
#include "utils.h"
#include "main.h"
#include "mrd106.h"

// USB PRINTF
#ifdef USB_DEBUG
#define usb_debug_printf(fmt, args...) usb_printf(fmt, ##args)
#else
#define usb_debug_printf(fmt, args...) (void)(fmt);
#endif

#define PACKETSIZE 20

static mrd_t *m_mrd = NULL;

static uint8_t gOS64NumCh = 1;
bool gPpgEn;
bool gEcgEn;
bool gIqEn;
uint8_t gUseAcc = 1;
static uint8_t gOS64NumPhotoDiodes;
static uint8_t gOptNumFramesToRead;
static uint8_t gFlashLogRunning;
static uint8_t gPacketCount;

static int32_t gSensorQueue[NUM_QUEUES][SENSORQUEUESIZE];	// queue that holds ECG or PPG data as parsed from gSensorFifoBuf[]
static uint8_t gSensorBleBuf[NUM_QUEUES][SENSORBLEBUFSIZE]; // buffer that holds the data from gSensorQueue[][] that will be sent over BLE
static uint8_t gAccBuf[SENSORBLEBUFSIZE];

static uint8_t gAccCount;
static int32_t gLeadsOffIQ[3];
static int32_t gUtilityAdc;
static uint8_t gDataFromCentral[PACKETSIZE];
static uint8_t gQueueWrPtr[NUM_QUEUES];
static uint8_t gQueueRdPtr[NUM_QUEUES];
static bool gAcLeadsOffConvRdy;
static bool gEcgSampleOccurred;
static int32_t gEcgSampleNum;
static uint8_t gNumItemsForBleRpt[NUM_QUEUES] = {1, 1, 1, 1}; // dummy values so that os64intHandler() does not get stuck checking for 0 samples
static uint8_t gSensorBleBufIx;
static bool gFlashlogStopOnBusyRequest;
static uint8_t dataToCentral[PACKETSIZE];
static uint8_t gEvtRequest[NUM_EVTS];
static int is_reporting_started = 0;
static uint8_t gSensorsRunning = 0;

static void format_data_ppg_pd1(void);
static void format_data_ppg_pd1_noAcc(void);
static void format_data_ppg_pd2(void);
static void format_data_ppg_pd2_noAcc(void);
static void format_data_bioz(void);
static void finish_send(uint8_t ix);

static void (*format_data_opt)(void);
static uint8_t gLogBuf[LOGBUFSIZE];
static uint8_t gNumPackets;

static void (*format_data_sensor[NUM_QUEUES])(void);

static queue_t *reportQueue = NULL;

static queue_t *msgQueue = NULL;

int gui_set_msg_queue(queue_t *queue)
{
	int ret = -1;
	if (queue != NULL)
	{
		msgQueue = queue;
		ret = 0;
	}
	else
	{
		reportQueue = NULL;
	}
	return ret;
}

int adapter_set_report_queue(queue_t *queue)
{
	int ret = -1;
	if (queue != NULL)
	{
		reportQueue = queue;
		ret = 0;
	}
	else
	{
		reportQueue = NULL;
	}
	return ret;
}

static uint32_t sensor_data_send(uint8_t *buf)
{
	int i32ret;
	if (reportQueue)
	{
		i32ret = enqueue(reportQueue, buf);
		if (i32ret != 0)
		{
			pr_info("BLE_Icarus_AddtoQueue has failed\r\n");
		}
	}
	return 0;
}

int adapter_read_frame(uint8_t buf[20])
{

	int ret = -1;
	int len = queue_len(reportQueue);
	if (len > 0)
	{
		ret = dequeue(reportQueue, &buf[0]);
	}
	return (ret == 0) ? 1 : 0;
}

static int32_t getQueueCount(void)
{
	return ((gQueueWrPtr[gSensorBleBufIx] - gQueueRdPtr[gSensorBleBufIx]) >= 0) ? (gQueueWrPtr[gSensorBleBufIx] - gQueueRdPtr[gSensorBleBufIx]) : (SENSORQUEUESIZE - (gQueueRdPtr[gSensorBleBufIx] - gQueueWrPtr[gSensorBleBufIx]));
}

static void setOptNumFramesToRead() // see API for calculations pertaining to how these valuse are chosen
{
	if (gUseAcc)
	{
		if (gOS64NumPhotoDiodes == 1)
		{
			format_data_opt = format_data_ppg_pd1;
			if (gOS64NumCh == 1)
			{
				gNumPackets = 1;
				gOptNumFramesToRead = 2;
			}
			else if (gOS64NumCh == 2)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 3;
			}
			else if (gOS64NumCh == 3)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 2;
			}
			else if (gOS64NumCh == 4)
			{
				gNumPackets = 1;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 5)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 6)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 7)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 8)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 9)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
		}
		else
		{
			format_data_opt = format_data_ppg_pd2;
			if (gOS64NumCh == 1)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 3;
			}
			else if (gOS64NumCh == 2)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 2;
			}
			else if (gOS64NumCh == 3)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 4)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 5)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 6)
			{
				gNumPackets = 3;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 7)
			{
				gNumPackets = 3;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 8)
			{
				gNumPackets = 3;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 9)
			{
				gNumPackets = 4;
				gOptNumFramesToRead = 1;
			}
		}
	}
	else
	{
		if (gOS64NumPhotoDiodes == 1)
		{
			format_data_opt = format_data_ppg_pd1_noAcc;
			if (gOS64NumCh == 1)
			{
				gNumPackets = 1;
				gOptNumFramesToRead = 6;
			}
			else if (gOS64NumCh == 2)
			{
				gNumPackets = 1;
				gOptNumFramesToRead = 3;
			}
			else if (gOS64NumCh == 3)
			{
				gNumPackets = 1;
				gOptNumFramesToRead = 2;
			}
			else if (gOS64NumCh == 4)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 3;
			}
			else if (gOS64NumCh == 5)
			{
				gNumPackets = 1;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 6)
			{
				gNumPackets = 1;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 7)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 8)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 9)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
		}
		else
		{
			format_data_opt = format_data_ppg_pd2_noAcc;
			if (gOS64NumCh == 1)
			{
				gNumPackets = 1;
				gOptNumFramesToRead = 3;
			}
			else if (gOS64NumCh == 2)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 3;
			}
			else if (gOS64NumCh == 3)
			{
				gNumPackets = 1;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 4)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 5)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 6)
			{
				gNumPackets = 2;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 7)
			{
				gNumPackets = 3;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 8)
			{
				gNumPackets = 3;
				gOptNumFramesToRead = 1;
			}
			else if (gOS64NumCh == 9)
			{
				gNumPackets = 3;
				gOptNumFramesToRead = 1;
			}
		}
	}
}

static void accel_readData(queue_t *accQueue, uint8_t *buf, uint8_t len)
{
	int ret = 0;
	static uint8_t tempaccelSampleXY[6] = {0};

	uint8_t accelSampleXYZ[6] = {0};
	ret = dequeue(accQueue, accelSampleXYZ);

	if (0 != ret)
	{
		/* use older acc data*/
		memcpy(accelSampleXYZ, tempaccelSampleXY, sizeof(tempaccelSampleXY));
	}
	else
	{
		memcpy(tempaccelSampleXY, accelSampleXYZ, sizeof(tempaccelSampleXY));
	}

	buf[0] = accelSampleXYZ[1];
	buf[1] = accelSampleXYZ[0];

	buf[2] = accelSampleXYZ[3];
	buf[3] = accelSampleXYZ[2];

	buf[4] = accelSampleXYZ[5];
	buf[5] = accelSampleXYZ[4];
}

static void format_data_ppg_pd1(void)
{
	// static int call = 0;
	DEBUG_PRINT("pd1 acc format call = %d \r\n", call++);

	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0, ox;

	// memset(buf, 0, sizeof(buf));
	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));
	buf[0] = gPacketCount++;
	buf[1] = NOTI_SNSR0;
	if (gOS64NumCh == 1)
	{
		for (ox = 0; ox < 6; ox++)
			buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][ox];
		for (ox = 0; ox < 12; ox++)
			buf[2 + 6 + ox] = gAccBuf[ox];
	}
	else if (gOS64NumCh == 2 || gOS64NumCh == 3 || gOS64NumCh == 6 || gOS64NumCh == 7 || gOS64NumCh == 8 || gOS64NumCh == 9)
	{
		for (ox = 0; ox < 18; ox++)
			buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][ox];
	}
	else if (gOS64NumCh == 4)
	{
		for (ox = 0; ox < 12; ox++)
			buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][ox];
		for (ox = 0; ox < 6; ox++)
			buf[2 + 12 + ox] = gAccBuf[ox];
	}
	else if (gOS64NumCh == 5)
	{
		for (ox = 0; ox < 15; ox++)
			buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][ox];
	}
	sensor_data_send(buf);
	if (gFlashLogRunning)
		memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	if (gNumPackets > 1)
	{
		// memset(buf, 0, sizeof(buf));
		buf[0] = gPacketCount++;
		buf[1] = NOTI_SNSR1;
		if (gOS64NumCh == 2)
		{
			for (ox = 0; ox < 18; ox++)
				buf[2 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 3)
		{
			for (ox = 0; ox < 12; ox++)
				buf[2 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 5 || gOS64NumCh == 6)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 7)
		{
			for (ox = 0; ox < 3; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
			for (ox = 0; ox < 6; ox++)
				buf[2 + 3 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 8)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
			for (ox = 0; ox < 6; ox++)
				buf[2 + 6 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 9)
		{
			for (ox = 0; ox < 9; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
			for (ox = 0; ox < 6; ox++)
				buf[2 + 9 + ox] = gAccBuf[ox];
		}
		sensor_data_send(buf);
		if (gFlashLogRunning)
			memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	}
	finish_send(ix);
}

static void format_data_ppg_pd1_noAcc(void)
{
	// static int call = 0;
	DEBUG_PRINT("pd1 NOacc format call = %d \r\n", call++);

	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0, ox;

	// memset(buf, 0, sizeof(buf));
	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));
	buf[0] = gPacketCount++;
	buf[1] = NOTI_SNSR0;
	if (gOS64NumCh == 5)
	{
		for (ox = 0; ox < 15; ox++)
			buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][ox];
	}
	else
	{
		for (ox = 0; ox < 18; ox++)
			buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][ox];
	}
	sensor_data_send(buf);
	if (gFlashLogRunning)
		memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	if (gNumPackets > 1)
	{
		// memset(buf, 0, sizeof(buf));
		buf[0] = gPacketCount++;
		buf[1] = NOTI_SNSR1;
		if (gOS64NumCh == 4)
		{
			for (ox = 0; ox < 18; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
		}
		else if (gOS64NumCh == 7)
		{
			for (ox = 0; ox < 3; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
		}
		else if (gOS64NumCh == 8)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
		}
		else if (gOS64NumCh == 9)
		{
			for (ox = 0; ox < 9; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
		}
		sensor_data_send(buf);
		if (gFlashLogRunning)
			memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	}
	finish_send(ix);
}

static void format_data_ppg_pd2(void)
{

	// static int call = 0;
	DEBUG_PRINT("pd2 acc format call = %d \r\n", call++);

	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0, ox;

	// memset(buf, 0, sizeof(buf));
	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));
	buf[0] = gPacketCount++;
	buf[1] = NOTI_SNSR0;
	{
		for (ox = 0; ox < 18; ox++)
			buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][ox];
	}
	DEBUG_PRINT("pd2 acc format sensordataSENT = %d \r\n", call);
	usb_debug_printf("tags: %d%d%d%d%d%d\r\n", (buf[2] >> 4 & 0x0F), (buf[5] >> 4 & 0x0F), (buf[8] >> 4 & 0x0F), (buf[11] >> 4 & 0x0F), (buf[14] >> 4 & 0x0F), (buf[17] >> 4 & 0x0F));
	sensor_data_send(buf);
	if (gFlashLogRunning)
		memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	if (gNumPackets > 1)
	{
		// memset(buf, 0, sizeof(buf));
		buf[0] = gPacketCount++;
		buf[1] = NOTI_SNSR1;
		if (gOS64NumCh == 1)
		{
			for (ox = 0; ox < 18; ox++)
				buf[2 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 2)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
			for (ox = 0; ox < 12; ox++)
				buf[2 + 6 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 3)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 4)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
			for (ox = 0; ox < 6; ox++)
				buf[2 + 6 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 5)
		{
			for (ox = 0; ox < 12; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
			for (ox = 0; ox < 6; ox++)
				buf[2 + 12 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 6 || gOS64NumCh == 7 || gOS64NumCh == 8 || gOS64NumCh == 9)
		{
			for (ox = 0; ox < 18; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
		}
		sensor_data_send(buf);
		if (gFlashLogRunning)
			memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	}
	if (gNumPackets > 2)
	{
		// memset(buf, 0, sizeof(buf));
		buf[0] = gPacketCount++;
		buf[1] = NOTI_SNSR2;
		if (gOS64NumCh == 6)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 7)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][36 + ox];
			for (ox = 0; ox < 6; ox++)
				buf[2 + 6 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 8)
		{
			for (ox = 0; ox < 12; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][36 + ox];
			for (ox = 0; ox < 6; ox++)
				buf[2 + 12 + ox] = gAccBuf[ox];
		}
		else if (gOS64NumCh == 9)
		{
			for (ox = 0; ox < 18; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][36 + ox];
		}
		sensor_data_send(buf);
		if (gFlashLogRunning)
			memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	}
	if (gNumPackets > 3)
	{
		// memset(buf, 0, sizeof(buf));
		buf[0] = gPacketCount++;
		buf[1] = NOTI_SNSR3;
		if (gOS64NumCh == 9)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gAccBuf[ox];
		}
		sensor_data_send(buf);
		if (gFlashLogRunning)
			memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	}
	finish_send(ix);
}

static void format_data_ppg_pd2_noAcc(void)
{
	// static int call = 0;
	DEBUG_PRINT("pd2 NOacc format call = %d \r\n", call++);

	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0, ox;

	// memset(buf, 0, sizeof(buf));
	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));
	buf[0] = gPacketCount++;
	buf[1] = NOTI_SNSR0;
	{
		for (ox = 0; ox < 18; ox++)
			buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][ox];
	}
	sensor_data_send(buf);
	if (gFlashLogRunning)
		memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	if (gNumPackets > 1)
	{
		// memset(buf, 0, sizeof(buf));
		buf[0] = gPacketCount++;
		buf[1] = NOTI_SNSR1;
		if (gOS64NumCh == 2 || gOS64NumCh == 6 || gOS64NumCh == 7 || gOS64NumCh == 8 || gOS64NumCh == 9)
		{
			for (ox = 0; ox < 18; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
		}
		else if (gOS64NumCh == 4)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
		}
		else if (gOS64NumCh == 5)
		{
			for (ox = 0; ox < 12; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][18 + ox];
		}
		sensor_data_send(buf);
		if (gFlashLogRunning)
			memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	}
	if (gNumPackets > 2)
	{
		// memset(buf, 0, sizeof(buf));
		buf[0] = gPacketCount++;
		buf[1] = NOTI_SNSR2;
		if (gOS64NumCh == 7)
		{
			for (ox = 0; ox < 6; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][36 + ox];
		}
		else if (gOS64NumCh == 8)
		{
			for (ox = 0; ox < 12; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][36 + ox];
		}
		else if (gOS64NumCh == 9)
		{
			for (ox = 0; ox < 18; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_OPT][36 + ox];
		}
		sensor_data_send(buf);
		if (gFlashLogRunning)
			memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	}
	finish_send(ix);
}

static void format_data_ecg(void)
{
	// static int call = 0;
	DEBUG_PRINT("ecg format call = %d \r\n", call++);

	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0, ox;

	// memset(buf, 0, sizeof(buf));
	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));
	buf[0] = gPacketCount++;
	buf[1] = NOTI_ECG;

	for (ox = 0; ox < 18; ox++)
		buf[2 + ox] = gSensorBleBuf[POS_QUEUE_ECG][ox];

	sensor_data_send(buf);
	if (gFlashLogRunning)
		memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	finish_send(ix);
}

static void format_data_acc(queue_t *accQueue)
{
	// static int call = 0;
	DEBUG_PRINT("acc format call = %d \r\n", call++);

	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0, ox;

	// memset(buf, 0, sizeof(buf));
	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));
	buf[0] = gPacketCount++;
	buf[1] = NOTI_ACC;

	for (ox = 0; ox < 3; ox++)
	{
		accel_readData(accQueue, buf + 2 + (ox * 6), ACCEL_NUM_BYTES_PER_SAMPLE);
	}

	sensor_data_send(buf);
	if (gFlashLogRunning)
		memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	finish_send(ix);
}

static void format_data_time(void)
{
	// static int call = 0;
	DEBUG_PRINT("time format call = %d \r\n", call++);

	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0, ox;

	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));
	for (uint8_t kx = 0; kx < 2; kx++)
	{
		if ((kx == 1) && !(!gUseAcc && (gOS64NumPhotoDiodes == 1)))
			break;
		// memset(buf, 0, sizeof(buf));
		buf[0] = gPacketCount++;
		buf[1] = NOTI_TIMESTAMP0 + kx;
		for (ox = 0; ox < 18; ox++)
			buf[2 + ox] = gSensorBleBuf[POS_QUEUE_TIME][ox]; // 0s will be in unused gSensorBleBuf[POS_QUEUE_TIME][ox] positions
		sensor_data_send(buf);
		if (gFlashLogRunning)
			memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	}
	finish_send(ix);
}

static void format_data_iq(void)
{
	// static int call = 0;
	DEBUG_PRINT("IQ format call = %d \r\n", call++);

	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0, ox;

	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));
	for (uint8_t kx = 0; kx < 2; kx++)
	{
		if ((kx == 1) && (gPpgEn || gEcgEn) && !gUseAcc)
			break;
		// memset(buf, 0, sizeof(buf));
		buf[0] = gPacketCount++;
		buf[1] = NOTI_IQ0 + kx;
		if (kx == 0)
		{
			for (ox = 0; ox < 18; ox++)
				buf[2 + ox] = gSensorBleBuf[POS_QUEUE_IQ][ox];
		}
		else
		{
			for (ox = 0; ox < 18; ox++)
				buf[2 + ox] = gAccBuf[ox];
		}
		sensor_data_send(buf);
		if (gFlashLogRunning)
			memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	}
	finish_send(ix);
}

static void format_data_bioz(void)
{
	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0, ox;

	memset(buf, 0, sizeof(buf));
	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));
	buf[0] = gPacketCount++;
	buf[1] = gSensorBleBufIx == POS_QUEUE_ECG ? NOTI_ECG : NOTI_IQ0;
	for (ox = 0; ox < 18; ox++)
		buf[2 + ox] = gSensorBleBuf[gSensorBleBufIx][ox];

	sensor_data_send(buf);
	if (gFlashLogRunning)
		memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	finish_send(ix);
}

void format_data_periodic(uint8_t *buf)
{

	uint8_t ix = 0;
	uint16_t temp_data;
	uint32_t rtc_time = 0;

	sensor_t *temp = mxm_sh_get_sensor_instance(SH_TEMP_SENSOR);

	memcpy(&temp_data, temp->report_buf.buf, sizeof(temp_data));

	rtc_time = (uint32_t)(utils_get_time_ms() / 1000); // ms to s

	buf[ix++] = gPacketCount++;
	buf[ix++] = NOTI_PERIODIC;
	buf[ix++] = (m_mrd->getChargingStatus() << 7) | (m_mrd->getBatteryPercentage() & 0x7F);
	buf[ix++] = /*gOS64DieTemperature[0]*/ 0;
	buf[ix++] = /*gOS64DieTemperature[1]*/ 0;
	buf[ix++] = (rtc_time >> 16) & 0xff;
	buf[ix++] = (rtc_time >> 8) & 0xff;
	buf[ix++] = rtc_time & 0xff;

	buf[ix++] = ((temp_data >> 8) & 0xFF);
	buf[ix++] = (temp_data & 0xFF);

	buf[ix++] = 0; // Reserved

	buf[ix++] = /*gOS64statusBuf[5]*/ 0;
	buf[ix++] = /*gLeadsOffIQ[0]&0xff*/ 0;
	buf[ix++] = /*(gLeadsOffIQ[1]>>8)&0xff*/ 0;
	buf[ix++] = /*gLeadsOffIQ[1]&0xff*/ 0;
	buf[ix++] = /*(gLeadsOffIQ[2]>>8)&0xff*/ 0;
	buf[ix++] = /*gLeadsOffIQ[2]&0xff*/ 0;
	buf[ix++] = /*(gUtilityAdc>>16)&0xff*/ 0;
	buf[ix++] = /*(gUtilityAdc>>8)&0xff*/ 0;
	buf[ix++] = /*gUtilityAdc&0xff*/ 0;
}

void format_data_algo(algo_packet_t *p_packet)
{
	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0;

	// memset(buf, 0, sizeof(buf));

	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));

	buf[ix++] = gPacketCount++;
	buf[ix++] = NOTI_ALGO;
	buf[ix++] = p_packet->algo_mode;   // Algo Mode
	buf[ix++] = p_packet->hr;		   // hr
	buf[ix++] = p_packet->hr_conf;	   // hr conf
	buf[ix++] = p_packet->rr_msb;	   // rr_msb
	buf[ix++] = p_packet->rr_lsb;	   // rr_lsb
	buf[ix++] = p_packet->rr_conf;	   // rr conf
	buf[ix++] = p_packet->spo2;		   // spo2
	buf[ix++] = p_packet->spo2_conf;   // spo2 conf
	buf[ix++] = p_packet->r_val_msb;   // r value msb
	buf[ix++] = p_packet->r_val_lsb;   // r value lsb
	buf[ix++] = p_packet->spo2_comp;   // spo2 complete
	buf[ix++] = p_packet->spo2_state;  // spo2 state
	buf[ix++] = p_packet->activity;	   // activity
	buf[ix++] = p_packet->scd_state;   // scd state
	buf[ix++] = p_packet->flag_region; // flag_region

	sensor_data_send(buf);
	if (gFlashLogRunning)
		memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	finish_send(ix);
}

void format_data_temp(uint8_t *report_buf, uint64_t rtc_time)
{
	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0;

	// memset(buf, 0, sizeof(buf));

	/*if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));*/

	buf[ix++] = gPacketCount++;
	buf[ix++] = NOTI_TEMP;
	buf[ix++] = report_buf[1];
	buf[ix++] = report_buf[0];
	buf[ix++] = report_buf[3];
	buf[ix++] = report_buf[2];
	buf[ix++] = (rtc_time >> 56) & 0xFF;
	buf[ix++] = (rtc_time >> 48) & 0xFF;
	buf[ix++] = (rtc_time >> 40) & 0xFF;
	buf[ix++] = (rtc_time >> 32) & 0xFF;
	buf[ix++] = (rtc_time >> 24) & 0xFF;
	buf[ix++] = (rtc_time >> 16) & 0xFF;
	buf[ix++] = (rtc_time >> 8) & 0xFF;
	buf[ix++] = rtc_time & 0xFF;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;

	sensor_data_send(buf);
	/*if (gFlashLogRunning)
		memcpy(gLogBuf+(ix++)*PACKETSIZE, buf, PACKETSIZE);*/
	finish_send(ix);
}

void format_data_loc_find(uint8_t *buffer, uint16_t size)
{
	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0;

	buf[ix++] = gPacketCount++;
	buf[ix++] = NOTI_LOC_FIND;
	for (int i = 0; i < size; i++)
	{
		buf[ix++] = buffer[i];
	}

	sensor_data_send(buf);
	finish_send(ix);
}

void format_stop_packet()
{
	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0;

	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));

	buf[ix++] = gPacketCount++;
	buf[ix++] = NOTI_STOP_PACK;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;

	sensor_data_send(buf);
	if (gFlashLogRunning)
		memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	finish_send(ix);
}

void format_dont_care_packet()
{
	uint8_t buf[PACKETSIZE];
	uint8_t ix = 0;

	if (gFlashLogRunning)
		memset(gLogBuf, 0, sizeof(gLogBuf));

	buf[ix++] = gPacketCount++;
	buf[ix++] = NOTI_DONT_CARE_PACK;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;
	buf[ix++] = 0;

	sensor_data_send(buf);
	if (gFlashLogRunning)
		memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
	finish_send(ix);
}

static void finish_send(uint8_t ix)
{

	static uint8_t stop_cmd = 0;
	uint8_t buf[PACKETSIZE];

	if (gSensorBleBufIx == POS_QUEUE_OPT /*PPG only, so that periodic data is last*/ ||
		(!gPpgEn && gSensorBleBufIx == POS_QUEUE_ECG) /*ECG only or ECG+IQ*/ ||
		(!gPpgEn && !gEcgEn) /*IQ only*/)
	{
		if (gEvtRequest[EVT_PERIODICDATAREADY])
		{
			gEvtRequest[EVT_PERIODICDATAREADY] = 0;
			// memset(buf, 0, PACKETSIZE);
			format_data_periodic(buf);
			sensor_data_send(buf);
			if (gFlashLogRunning)
				memcpy(gLogBuf + (ix++) * PACKETSIZE, buf, PACKETSIZE);
			if (gEvtRequest[EVT_SENSORSTOP])
			{
				gEvtRequest[EVT_SENSORSTOP] = 0;
				app_main_evt_post(EVT_MEASUREMENT_STOP);
				stop_cmd = 1;
			}
		}

		if (stop_cmd)
		{
			stop_cmd = 0;
			m_mrd->enableTimer(PERIODIC_PACK_TIMER, 0);
		}
	}
}

static uint8_t gFrameStartTag = 0;

int adapter_init(const os64wrapper_cfg_t *os64config,
				 int *optimum_fifo_threshold, bool *setFrameRdyIrq)
{
	m_mrd = getMRD();
	if (os64config == NULL || reportQueue == NULL)
		return -1;

	gPacketCount = 0;
	gAccCount = 0;
	gUtilityAdc = 0;
	// memset(gQueueWrPtr, 0, sizeof(gQueueWrPtr));
	// memset(gQueueRdPtr, 0, sizeof(gQueueRdPtr));
	// memset(gLeadsOffIQ, 0, sizeof(gLeadsOffIQ));
	gFlashlogStopOnBusyRequest = false;
	gAcLeadsOffConvRdy = false;
	gEcgSampleOccurred = false;
	gEcgSampleNum = 0;

	gPpgEn = os64config->ppgEn;
	gEcgEn = os64config->ecgEn;
	gIqEn = os64config->iqEn;
	gUseAcc = os64config->UseAcc;

	int optimum_fifo_threshold_;
	bool setFrameRdyIrq_;

	if (gIqEn)
	{
		gNumItemsForBleRpt[POS_QUEUE_IQ] = 6; //
		optimum_fifo_threshold_ = 1;		  // gNumItemsForBleRpt[POS_QUEUE_IQ]; // (gUseAcc ? 1 /*2 items (I and Q) per sample*/: gNumItemsForBleRpt[POS_QUEUE_IQ]);
		setFrameRdyIrq_ = false;
	}
	if (gEcgEn) // this must come after the gIqEn check so the the accel is sync'd to ECG if both ECG and IQ are enabled
	{
		gNumItemsForBleRpt[POS_QUEUE_IQ] = 6; // accel will be sync'd to ECG (but IQ uses 3 samples in either case, so it doesn't matter to re-set this value here)
		gNumItemsForBleRpt[POS_QUEUE_ECG] = gUseAcc ? 6 : 6;
		optimum_fifo_threshold_ = (gUseAcc ? 6 : gNumItemsForBleRpt[POS_QUEUE_ECG]);
		setFrameRdyIrq_ = false;
	}
	if (gPpgEn) // gPpgEn check must come last
	{
		gOS64NumPhotoDiodes = os64config->os64NumPhotoDiodes_enabled; // os64_getNumPhotoDiodes();
		gOS64NumCh = os64config->os64NumCh_enabled;					  // os64_getNumChannels();
		setOptNumFramesToRead();									  // gUseAcc, gOS64NumCh and gOS64NumPhotoDiodes must be set beforehand
		format_data_sensor[POS_QUEUE_OPT] = format_data_opt;
		gNumItemsForBleRpt[POS_QUEUE_OPT] = gOptNumFramesToRead * gOS64NumCh * gOS64NumPhotoDiodes;
		gNumItemsForBleRpt[POS_QUEUE_TIME] = gOptNumFramesToRead * 2; // when using PPG_TIME_DATA, each PPG frame has a timestamp and relative ECG sample number
		// os64_setFIFOAFull(gNumItemsForBleRpt[POS_QUEUE_OPT]);	// IQ data may be interspersed, so not all items will be PPG samples
		optimum_fifo_threshold_ = gNumItemsForBleRpt[POS_QUEUE_OPT];
		// os64_setDataIntFrameReady(gUseAcc?true:false);
		setFrameRdyIrq_ = gUseAcc ? true : false;
		gNumItemsForBleRpt[POS_QUEUE_ECG] = gNumItemsForBleRpt[POS_QUEUE_IQ] = 6; // accel will only be sync'd to PPG
		if (gEcgEn)
		{
			if (os64config->os64_isEcgFasterThanPpg)
			{
				// cannot use FRAME_RDY INT since there may be a large delta in ODRs (e.g., PPG @ 20Sps, ECG @ 1024Sps) and we'll overrun the BLE buffers
				optimum_fifo_threshold_ = gNumItemsForBleRpt[POS_QUEUE_ECG]; // set to gNumItemsForBleRpt[POS_QUEUE_ECG] samples since that is optimized for ECG data transfer. However, interspersed PPG/IQ samples will cause this to not always interrupt on 6 ECG samples.
				setFrameRdyIrq_ = false;									 // even though FRAME_RDY int is not used, the FRAME_RDY bit will assert if a PPG frame is ready. If true (and gUseAcc is true), read an accel sample. But it should be noted that the accel sample will really be sync'd with the last PPG item when the FIFO is read, not when the first PPG item is ready.
			}
			else
			{
				setFrameRdyIrq_ = true;
				// Not recommended to use FIFO_A_FULL int here since it does not guarantee a full frame due to the unknown ECG sample rate.
				// There will be only one PPG frame and 0 or more ECG samples depending on ECG's ODR relative to PPG's ODR on each interrupt
				// Check the number of items on each FRAME_RDY int and read them all
			}
		}
	}

	gFrameStartTag = os64config->os64ppgFrameFirstChanTag;

	format_data_sensor[POS_QUEUE_ECG] = format_data_ecg;
	format_data_sensor[POS_QUEUE_TIME] = format_data_time;
	format_data_sensor[POS_QUEUE_IQ] = format_data_bioz;

	queue_reset(reportQueue);

	*optimum_fifo_threshold = optimum_fifo_threshold_;
	*setFrameRdyIrq = setFrameRdyIrq_;

	pr_info(" adapter init configured \r\n");

	return 0;
}

static unsigned int ppg_counter = 0;
static unsigned int ecg_counter = 0;
static unsigned int bioz_counter = 0;

int adapter_execute_push(queue_t *tagQueue, queue_t *accQueue)
{

	// static bool frameAccelSampleHasBeenRead;
	static uint8_t pack_counter = 0;
	uint32_t val = 0;
	int ret;
	ret = dequeue(tagQueue, &val);
	if (ret != 0)
		return -1;

	// frameAccelSampleHasBeenRead = false;
	uint8_t val20 = (val >> 20) & 0x0f;
	uint8_t val19 = (val >> 19) & 0x1f;
	uint8_t val18 = (val >> 18) & 0x3f;

	if (val20 <= TAG_PPG_MAX)
	{

		bool cond = (gPpgEn && gEcgEn && !gEcgSampleOccurred);
		if (cond == false)
		{ // for PPG+ECG, push PPG to queue only if ECG has started to simplify associating the PPG_TIME_DATA + PPG sample with the proper ECG sample
			is_reporting_started = 1;

			gSensorQueue[POS_QUEUE_OPT][gQueueWrPtr[POS_QUEUE_OPT]++] = val;
			if (gQueueWrPtr[POS_QUEUE_OPT] >= SENSORQUEUESIZE)
				gQueueWrPtr[POS_QUEUE_OPT] = 0;

			/* Acc sampling logic changed due to OVF data */
			pack_counter++;
			if (pack_counter >= (gOS64NumCh * gOS64NumPhotoDiodes) && gUseAcc)
			{
				accel_readData(accQueue, gAccBuf + ACCEL_NUM_BYTES_PER_SAMPLE * gAccCount++, ACCEL_NUM_BYTES_PER_SAMPLE);
				pack_counter = 0;
			}
		}
		ppg_counter++;
		if (0 == ppg_counter % 150)
		{
			usb_debug_printf("ppg_c %d\r\n", ppg_counter);
		}
	}
	else if (val20 == TAG_ECG)
	{
		gEcgSampleOccurred = true;
		gEcgSampleNum++;
		gSensorQueue[POS_QUEUE_ECG][gQueueWrPtr[POS_QUEUE_ECG]++] = val;
		if (gQueueWrPtr[POS_QUEUE_ECG] >= SENSORQUEUESIZE)
			gQueueWrPtr[POS_QUEUE_ECG] = 0;
		ecg_counter++;
		if (0 == ecg_counter % 512)
		{
			usb_debug_printf("ecg_c %d\r\n", ecg_counter);
		}
	}
	else if (val20 == TAG_TIME)
	{
		gSensorQueue[POS_QUEUE_TIME][gQueueWrPtr[POS_QUEUE_TIME]++] = val;
		if (gQueueWrPtr[POS_QUEUE_TIME] >= SENSORQUEUESIZE)
			gQueueWrPtr[POS_QUEUE_TIME] = 0;
		gSensorQueue[POS_QUEUE_TIME][gQueueWrPtr[POS_QUEUE_TIME]++] = gEcgSampleNum;
		if (gQueueWrPtr[POS_QUEUE_TIME] >= SENSORQUEUESIZE)
			gQueueWrPtr[POS_QUEUE_TIME] = 0;
	}
	else if (val20 == TAG_I || val20 == TAG_Q)
	{
		gSensorQueue[POS_QUEUE_IQ][gQueueWrPtr[POS_QUEUE_IQ]++] = val;
		if (gQueueWrPtr[POS_QUEUE_IQ] >= SENSORQUEUESIZE)
			gQueueWrPtr[POS_QUEUE_IQ] = 0;
		bioz_counter++;
		if (0 == bioz_counter % 64)
		{
			usb_debug_printf("bioz_c %d\r\n", bioz_counter);
		}
	}
	else if (val19 == TAG_UTILITY)
	{
		gUtilityAdc = val;
	}

	return 0;
}

int adapter_execute_pop(void)
{

	int32_t queueCount;

	for (gSensorBleBufIx = 0; gSensorBleBufIx < NUM_QUEUES; gSensorBleBufIx++)
	{
		queueCount = getQueueCount();

		while (queueCount >= gNumItemsForBleRpt[gSensorBleBufIx])
		{

			if (gSensorBleBufIx == POS_QUEUE_TIME)
			{
				for (uint8_t ix = 0; ix < gNumItemsForBleRpt[gSensorBleBufIx] * 3 /*really 4B and 2B, which averages 3B*/; ix += 6)
				{
					gSensorBleBuf[gSensorBleBufIx][ix + 0] = (gSensorQueue[gSensorBleBufIx][gQueueRdPtr[gSensorBleBufIx]] >> 8) & 0xff; // timestamp is 10b, with 0s in MSb, so need to mask
					gSensorBleBuf[gSensorBleBufIx][ix + 1] = gSensorQueue[gSensorBleBufIx][gQueueRdPtr[gSensorBleBufIx]] & 0xff;
					gQueueRdPtr[gSensorBleBufIx]++;
					if (gQueueRdPtr[gSensorBleBufIx] >= SENSORQUEUESIZE)
						gQueueRdPtr[gSensorBleBufIx] = 0;
					gSensorBleBuf[gSensorBleBufIx][ix + 2] = (gSensorQueue[gSensorBleBufIx][gQueueRdPtr[gSensorBleBufIx]] >> 24) & 0xff;
					gSensorBleBuf[gSensorBleBufIx][ix + 3] = (gSensorQueue[gSensorBleBufIx][gQueueRdPtr[gSensorBleBufIx]] >> 16) & 0xff;
					gSensorBleBuf[gSensorBleBufIx][ix + 4] = (gSensorQueue[gSensorBleBufIx][gQueueRdPtr[gSensorBleBufIx]] >> 8) & 0xff;
					gSensorBleBuf[gSensorBleBufIx][ix + 5] = gSensorQueue[gSensorBleBufIx][gQueueRdPtr[gSensorBleBufIx]] & 0xff;
					gQueueRdPtr[gSensorBleBufIx]++;
					if (gQueueRdPtr[gSensorBleBufIx] >= SENSORQUEUESIZE)
						gQueueRdPtr[gSensorBleBufIx] = 0;
				}
			}
			else
			{

				for (uint8_t ix = 0; ix < gNumItemsForBleRpt[gSensorBleBufIx] * OS64_NUMBYTES_PER_ITEM; ix += OS64_NUMBYTES_PER_ITEM)
				{
					gSensorBleBuf[gSensorBleBufIx][ix] = (gSensorQueue[gSensorBleBufIx][gQueueRdPtr[gSensorBleBufIx]] >> 16) & 0xff;
					gSensorBleBuf[gSensorBleBufIx][ix + 1] = (gSensorQueue[gSensorBleBufIx][gQueueRdPtr[gSensorBleBufIx]] >> 8) & 0xff;
					gSensorBleBuf[gSensorBleBufIx][ix + 2] = gSensorQueue[gSensorBleBufIx][gQueueRdPtr[gSensorBleBufIx]] & 0xff;
					gQueueRdPtr[gSensorBleBufIx]++;
					if (gQueueRdPtr[gSensorBleBufIx] >= SENSORQUEUESIZE)
						gQueueRdPtr[gSensorBleBufIx] = 0;
				}
			}
			queueCount = getQueueCount();
			if (gUseAcc && ((gPpgEn && gSensorBleBufIx == POS_QUEUE_OPT)))
				gAccCount = 0;

			if (format_data_sensor[gSensorBleBufIx] != NULL)
			{
				format_data_sensor[gSensorBleBufIx]();
			}
		}
	}

	return 0;
}

int adapter_acc_execute_push(queue_t *accQueue)
{

	if (accQueue == NULL)
	{
		return -1;
	}

	if (accQueue->num_item >= 3)
	{
		format_data_acc(accQueue);
	}
}

// test functions
void adapter_check_queues(int *ecgItemCnt, int *ppgItemCnt, int *timeItemCnt, int *iqItemCnt)
{

	gSensorBleBufIx = POS_QUEUE_ECG;
	*ecgItemCnt = getQueueCount();
	gSensorBleBufIx = POS_QUEUE_OPT;
	*ppgItemCnt = getQueueCount();
	gSensorBleBufIx = POS_QUEUE_TIME;
	*timeItemCnt = getQueueCount();
	gSensorBleBufIx = POS_QUEUE_IQ;
	*iqItemCnt = getQueueCount();
}

int check_ble_packet_queue(void)
{

	return queue_len(reportQueue);
}

int adapter_is_reporting()
{
	return gSensorsRunning;
}

void adapter_set_sensor_running()
{
	gSensorsRunning = 1;
}

void adapter_clear_sensor_running()
{
	gSensorsRunning = 0;
}

void adapter_set_acc_state(int en)
{
	gUseAcc = en;
}

int adapter_get_acc_state()
{
	return gUseAcc;
}

void adapter_set_periodic_packets(int en)
{
	m_mrd->enableTimer(PERIODIC_PACK_TIMER, en);
}

void adapter_set_event(int evt)
{
	gEvtRequest[evt] = 1;
}

int is_adapter_reporting_started()
{
	return is_reporting_started;
}

int adapter_is_acc_enabled()
{
	return gUseAcc;
}

static int evtReqPresent()
{
	int pendingEvtReq = 0;
	for (int i = 0; i < NUM_EVTS; i++)
	{
		pendingEvtReq += gEvtRequest[i];
	}

	return pendingEvtReq;
}

int get_sensorstop_status()
{
	return gEvtRequest[EVT_SENSORSTOP];
}

int get_periodicpacket_status()
{
	return gEvtRequest[EVT_PERIODICDATAREADY];
}

void set_periodicpacket_status(uint8_t e)
{
	gEvtRequest[EVT_PERIODICDATAREADY] = e;
}

void ble_sanity_handler()
{
	int pendingMsgLen = queue_len(msgQueue);

	if (pendingMsgLen > 0)
	{

		if (evtReqPresent())
		{
			return;
		}
		uint8_t cPacket[PACKETSIZE];
		dequeue(msgQueue, cPacket);
		int ret = app_gui_exec(cPacket, dataToCentral);
		/* when ret equals to zero send response because some commands do not require response */
		if (0 == ret)
		{
			ble_send_data(dataToCentral, sizeof(dataToCentral));
		}
	}
}

void ble_data_handler(uint8_t const *const p_rxdata, uint16_t rx_len)
{

	memcpy(gDataFromCentral, p_rxdata, rx_len);
	enqueue(msgQueue, &gDataFromCentral);
}

void usb_data_handler(uint8_t const *const p_rxdata, uint16_t rx_len)
{
	int ret = 0;
	memcpy(gDataFromCentral, p_rxdata, rx_len);

	ret = app_gui_exec(gDataFromCentral, dataToCentral);
	/* when ret equals to zero send response because some commands do not require response */
	if (0 == ret)
		usb_cdc_write(dataToCentral, sizeof(dataToCentral));
}

int gui_queue_reset()
{
	if (msgQueue)
		queue_reset(msgQueue);
	memset(gEvtRequest, 0x00, NUM_EVTS);
}
