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

#ifndef _SH_DEFS_H_
#define _SH_DEFS_H_
#include <limits.h>
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef MXM_SH_ASSERT_ENABLE
#define MXM_SH_ASSERT(expr)                             \
if (!(expr))                                            \
{                                                       \
    mxm_sh_assert(#expr, __FILE__, __LINE__);           \
}

#define MXM_SH_ASSERT_FAIL() mxm_sh_assert("FAIL", __FILE__, __LINE__);
#else
#define MXM_SH_ASSERT(expr)
#define MXM_SH_ASSERT_FAIL()
#endif

#define USE_MODE_NUMBER_IN_DEPENDENT_LIST	SHRT_MAX

typedef union {
	struct {
		uint8_t err: 3;
		uint8_t data_ready: 1;
		uint8_t fifo_out_ovr_int:1;
		uint8_t fifo_in_ovr_int:1;
		//uint8_t log_ovr:1;
		uint8_t accel_underflow:1;
		union{
			uint8_t log_ready:1;
			uint8_t scd_detected:1;
		};
		//uint8_t log_ready:1;
	};
	uint8_t val;
} status_reg_t;

typedef union {
	struct {
		uint8_t shdn: 1;
		uint8_t reset:1;
		uint8_t fifo_reset:1;
		uint8_t bootldr:1;
		uint8_t : 4;
	};
	uint8_t val;
} sh_mode_t;

typedef struct {
	uint8_t major;
	uint8_t minor;
	uint8_t revision;
} version_t;

typedef union {
	struct {
		uint8_t sensor_enabled:1;
		uint8_t algo_enabled:1;
		uint8_t sample_count:1;
		uint8_t :5;
	};
	uint8_t val;
} report_mode_t;

typedef union {
	struct {
		uint8_t sensor_enabled:1;
		uint8_t algo_enabled:1;
		uint8_t sample_count:1;
		uint8_t :5;
	};
	uint8_t val;
} run_mode_t;

typedef struct {
	uint8_t num_modes;
	uint8_t required_bytes[];
} sensor_modes_t;

typedef struct {
	uint8_t *buf;
	uint32_t ts; /* Time stamp/sample counter of the most recent available sample in shared buffer */
	uint8_t size;
	bool reported;
} sensor_data_t;

typedef struct {
	void *dev_data;
	uint8_t idx; /* Smart Sensor uses, dont modify */
	uint8_t reg_size;
	version_t version;
	const char **sensors;
	const char *name;
	uint16_t num_dump_regs;
	queue_t *queue;
	int (*init)();
	int (*dump_reg)(uint8_t *buf, int size);
	int (*read_reg)(uint8_t *read_adr, int len);
	int (*write_reg)(uint8_t *reg_addr, uint8_t *val);
	int (*enable)(int mode);
	int (*sync_data)(void);
	int (*execute_once)(void *); /* Reads data from sensor */
	int (*pop_data)(sensor_data_t **sensor_data);
	int (*num_samples)(void *);
	int (*get_sample_rate)(void *data);
	int (*set_sample_rate)(void *data, int sample_rate);
	int (*get_irq_state)(void *data);

	int (*set_cfg)(void *data, uint8_t *params, int param_sz);
	int (*get_cfg)(void *data, uint8_t params, uint8_t *tx_buf, int tx_buf_sz);

	uint32_t last_ts; /* time stamp for Last copied data to sdata buffer */
	uint8_t mode;
	run_mode_t run_mode;
	/* Report */
	int item_size;
	int num_items;
	sensor_data_t report_buf;

	int err;

	/* Self test related members */
	void (*self_test)(uint8_t *);
	bool presense;	// FIXME TM COMMENT is this really presence (as in the sensor has been detected) or something to do with sensing (as in the sensor has sensed something)?
	bool initialized;
	bool ext_data_mode;
	sensor_modes_t *mode_info;
} sensor_t;

typedef struct {
	uint32_t ts;
	uint16_t sample_rate;
	const uint8_t sns_id;	// sensor id
	uint8_t optional;
	uint8_t mode;
} sns_subsc_info_t;

typedef struct _subscribed_sensors_t {
	uint8_t num_sensors;
	sns_subsc_info_t *sns_list;
} subscribed_sensors_t;

typedef struct {
	const char *name;
	uint8_t idx; /* do not modify */
	void *data;
	int sample_rate;
	int (*init)(void);
	int (*execute)(sensor_data_t **sensor_data); /* Runs algorithms with data in shared buffer */
	int (*enable)(int mode);
	int (*reset)(void);
	int (*get_version)(version_t *version);
	void (*end)(void);
	int (*get_sample_rate)(void *data);
	int (*set_sample_rate)(void *data, int sample_rate);
	int (*set_algo_cfg)(uint8_t *params, int param_sz);
	int (*get_algo_cfg)(uint8_t params, uint8_t *tx_buf, int tx_buf_sz);

	uint32_t last_ts;
	queue_t *queue;
	uint32_t sample_cnt;
	run_mode_t run_mode;
	sensor_data_t report_buf;
	uint8_t mode;
	subscribed_sensors_t subscribed_sensors;
} algo_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} accel_report_t;

typedef struct {
	float x;
	float y;
	float z;
} accel_data_t;

void mxm_sh_assert(const char *expr, const char *file, int line);
#ifdef __cplusplus
}
#endif
#endif /* _SH_DEFS_H_ */
