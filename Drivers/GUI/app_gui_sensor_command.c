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


#include <stdint.h>
#include <string.h>

#include "app_gui.h"
#include "windows_gui_packet_defs.h"
#include "Peripherals.h"

#include "sensorhub.h"

#include "algohub_sensorhub_manager.h"

#include "sh_comm.h"

#define NUM_OF_TEMP_SENSORS 2

struct sens_hal{
	int (*write)(uint8_t reg_addr, uint8_t reg_val);
	int (*read)(uint8_t * const reg_addr, uint8_t number_regs_to_read);
};

struct cmd_func_table{
	uint8_t group_id;
	uint8_t mode;
	int (*cmd)(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
};

enum{
	CMD_GROUP_ID = 0,
	CMD_MODE,
	CMD_REG_ADDR,
	CMD_NUMBER_REGS_TO_READ,
};

static int sens_int_init(void);
static int sens_int_connected(void);
static int sens_int_disconnected(void);
static int sens_int_reset(void);
static int sens_int_start(void);
static int sens_int_stop(void);
static int sens_int_exec(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);

static int sens_opt_readreg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int sens_opt_rmw(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int sens_opt_regblockread(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int sens_acc_readreg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int sens_acc_rmw(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int sens_acc_regblockread(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int sens_temp_readreg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int sens_temp_rmw(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);
static int sens_temp_regblockread(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE]);

static int sens_opt_ah_write(uint8_t reg_addr, uint8_t reg_val);
static int sens_opt_ah_read(uint8_t * const reg_addr, uint8_t number_regs_to_read);
static int sens_opt_sh_write(uint8_t reg_addr, uint8_t reg_val);
static int sens_opt_sh_read(uint8_t * const reg_addr, uint8_t number_regs_to_read);

static int sens_acc_ah_write(uint8_t reg_addr, uint8_t reg_val);
static int sens_acc_ah_read(uint8_t * const reg_addr, uint8_t number_regs_to_read);
static int sens_acc_sh_write(uint8_t reg_addr, uint8_t reg_val);
static int sens_acc_sh_read(uint8_t * const reg_addr, uint8_t number_regs_to_read);

static int sens_temp_write(uint8_t reg_addr, uint8_t reg_val);
static int sens_temp_read(uint8_t * const reg_addr, uint8_t number_regs_to_read);
static int sens_temp_skin_write(uint8_t reg_addr, uint8_t reg_val);
static int sens_temp_skin_read(uint8_t * const reg_addr, uint8_t number_regs_to_read);

static struct cmd_func_table cmds[] = {
		{.group_id = TGT_SENS_OPT,			.mode = READREG,			.cmd = sens_opt_readreg			},
		{.group_id = TGT_SENS_OPT,			.mode = RMW,				.cmd = sens_opt_rmw				},
		{.group_id = TGT_SENS_OPT,			.mode = REGBLOCKREAD,		.cmd = sens_opt_regblockread	},
		{.group_id = TGT_SENS_ACC,			.mode = READREG,			.cmd = sens_acc_readreg			},
		{.group_id = TGT_SENS_ACC,			.mode = RMW,				.cmd = sens_acc_rmw				},
		{.group_id = TGT_SENS_ACC,			.mode = REGBLOCKREAD,		.cmd = sens_acc_regblockread	},
		{.group_id = TGT_TEMP_AMB_CONF,		.mode = READREG,			.cmd = sens_temp_readreg		},
		{.group_id = TGT_TEMP_AMB_CONF,		.mode = RMW,				.cmd = sens_temp_rmw			},
		{.group_id = TGT_TEMP_AMB_CONF,		.mode = REGBLOCKREAD,		.cmd = sens_temp_regblockread	},
		{.group_id = TGT_TEMP_SKIN_CONF,	.mode = READREG,			.cmd = sens_temp_readreg		},
		{.group_id = TGT_TEMP_SKIN_CONF,	.mode = RMW,				.cmd = sens_temp_rmw			},
		{.group_id = TGT_TEMP_SKIN_CONF,	.mode = REGBLOCKREAD,		.cmd = sens_temp_regblockread	},
};

static struct sens_hal opt_ops[AH_SH_SPI_OWNER_MAX] =
{
		{.write = sens_opt_ah_write,		.read = sens_opt_ah_read},
		{.write = sens_opt_sh_write,		.read = sens_opt_sh_read},
};

static struct sens_hal acc_ops[AH_SH_SPI_OWNER_MAX] =
{
		{.write = sens_acc_ah_write,		.read = sens_acc_ah_read},
		{.write = sens_acc_sh_write,		.read = sens_acc_sh_read},
};

static struct sens_hal temp_ops[NUM_OF_TEMP_SENSORS] =
{
		{.write = sens_temp_write,		.read = sens_temp_read},
		{.write = sens_temp_skin_write,		.read = sens_temp_skin_read},
};

app_gui_cmd_interface_t sens_int =
{
		.init = sens_int_init,
		.connected = sens_int_connected,
		.disconnected = sens_int_disconnected,
		.reset = sens_int_reset,
		.start = sens_int_start,
		.stop = sens_int_stop,
		.exec = sens_int_exec,
};

static int sens_int_init(void)
{
	int ret = 0;


	return ret;
}

static int sens_int_connected(void)
{
	int ret = 0;

	return ret;
}

static int sens_int_disconnected(void)
{
	int ret = 0;


	return ret;
}

static int sens_int_reset(void)
{
	int ret = 0;

	return ret;
}

static int sens_int_start(void)
{
	int ret = 0;

	return ret;
}

static int sens_int_stop(void)
{
	int ret = 0;

	return ret;
}

static int sens_int_exec(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	int ind = 0;

	ret = -1;

	for(ind = 0; ind < ARRAY_SIZE(cmds); ind++)
	{
		if(command[CMD_GROUP_ID] == cmds[ind].group_id && command[CMD_MODE] == cmds[ind].mode)
		{
			ret = 0;
			break;
		}
	}

	if(0 == ret)
	{
		memcpy(response, command, COMMAND_PACKETSIZE);
		ret = cmds[ind].cmd(command, response);
	}

	return ret;
}

static int sens_opt_readreg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	uint8_t reg_addr = command[CMD_REG_ADDR];
	uint8_t num_regs_to_read  = command[CMD_NUMBER_REGS_TO_READ];
	ah_sh_spi_ownwers_t spi_stat = ah_sh_spi_ownership_status();

	ret = opt_ops[spi_stat].read(&reg_addr, num_regs_to_read);

	response[0] = MSG_REGVAL;
	response[1] = 0;
	response[2] = 0;
	response[3] = 0;
	response[4] = reg_addr;

	return ret;
}

static int sens_opt_rmw(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	uint8_t tx_buf[5];
	uint8_t start, stop, numBytes, ix;
	uint32_t writeval=0, readval=0, mask=0;
	ah_sh_spi_ownwers_t spi_stat = ah_sh_spi_ownership_status();

	tx_buf[0] = command[2];
	stop = command[3];
	start = command[4];
	numBytes = 1;

	for (ix=0; ix<(stop - start + 1); ix++)
		mask |= (1<<ix);
	mask <<= start;

	ret |= opt_ops[spi_stat].read(&tx_buf[0], numBytes);

	readval = tx_buf[0];
	readval &= ~mask;
	writeval = command[5];
	writeval <<= start;
	writeval &= mask;	// just in case
	writeval |= readval;
	for (ix=0; ix<numBytes; ix++)
		tx_buf[ix+1] = (writeval>>((numBytes-1-ix)*8))&0xff;	// MSB first

	tx_buf[0] = command[2];

	ret |= opt_ops[spi_stat].write(tx_buf[0], writeval);

	ret = 1;
	return ret;
}

static int sens_opt_regblockread(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	ah_sh_spi_ownwers_t spi_stat = ah_sh_spi_ownership_status();

	response[0] = MSG_REGBLOCKREAD;
	response[1] = command[CMD_REG_ADDR];

	ret = opt_ops[spi_stat].read(&response[1], command[CMD_NUMBER_REGS_TO_READ]);

	return ret;
}

static int sens_acc_readreg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	uint8_t reg_addr = command[CMD_REG_ADDR];
	uint8_t num_regs_to_read  = command[CMD_NUMBER_REGS_TO_READ];
	ah_sh_spi_ownwers_t spi_stat = ah_sh_spi_ownership_status();

	ret = acc_ops[spi_stat].read(&reg_addr, num_regs_to_read);

	response[0] = MSG_REGVAL;
	response[1] = 0;
	response[2] = 0;
	response[3] = 0;
	response[4] = reg_addr;

	return ret;
}

static int sens_acc_rmw(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	uint8_t tx_buf[5];
	uint8_t start, stop, numBytes, ix;
	uint32_t writeval=0, readval=0, mask=0;
	ah_sh_spi_ownwers_t spi_stat = ah_sh_spi_ownership_status();

	tx_buf[0] = command[2];
	stop = command[3];
	start = command[4];
	numBytes = 1;

	for (ix=0; ix<(stop - start + 1); ix++)
		mask |= (1<<ix);
	mask <<= start;

	ret |= acc_ops[spi_stat].read(&tx_buf[0], numBytes);

	readval = tx_buf[0];
	readval &= ~mask;
	writeval = command[5];
	writeval <<= start;
	writeval &= mask;	// just in case
	writeval |= readval;
	for (ix=0; ix<numBytes; ix++)
		tx_buf[ix+1] = (writeval>>((numBytes-1-ix)*8))&0xff;	// MSB first

	tx_buf[0] = command[2];

	ret |= acc_ops[spi_stat].write(tx_buf[0], writeval);

	ret = 1;
	return ret;
}

static int sens_acc_regblockread(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	ah_sh_spi_ownwers_t spi_stat = ah_sh_spi_ownership_status();

	response[0] = MSG_REGBLOCKREAD;
	response[1] = command[CMD_REG_ADDR];

	ret = acc_ops[spi_stat].read(&response[1], command[CMD_NUMBER_REGS_TO_READ]);

	return ret;
}

static int sens_temp_readreg(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	uint8_t reg_addr = command[CMD_REG_ADDR];
	uint8_t num_regs_to_read  = command[CMD_NUMBER_REGS_TO_READ];
	uint8_t selected_temp = command[CMD_GROUP_ID] == TGT_TEMP_AMB_CONF ? 0:1;

	ret = temp_ops[selected_temp].read(&reg_addr, num_regs_to_read);

	response[0] = MSG_REGVAL;
	response[1] = 0;
	response[2] = 0;
	response[3] = 0;
	response[4] = reg_addr;

	return ret;
}

static int sens_temp_rmw(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	uint8_t tx_buf[5];
	uint8_t start, stop, numBytes, ix;
	uint32_t writeval=0, readval=0, mask=0;
	uint8_t selected_temp = command[CMD_GROUP_ID] == TGT_TEMP_AMB_CONF ? 0:1;

	tx_buf[0] = command[2];
	stop = command[3];
	start = command[4];
	numBytes = 1;

	for (ix=0; ix<(stop - start + 1); ix++)
		mask |= (1<<ix);
	mask <<= start;

	ret |= temp_ops[selected_temp].read(&tx_buf[0], numBytes);

	readval = tx_buf[0];
	readval &= ~mask;
	writeval = command[5];
	writeval <<= start;
	writeval &= mask;	// just in case
	writeval |= readval;
	for (ix=0; ix<numBytes; ix++)
		tx_buf[ix+1] = (writeval>>((numBytes-1-ix)*8))&0xff;	// MSB first

	tx_buf[0] = command[2];

	ret |= temp_ops[selected_temp].write(tx_buf[0], writeval);

	ret = 1;
	return ret;
}

static int sens_temp_regblockread(uint8_t command[COMMAND_PACKETSIZE], uint8_t response[RESPONSE_PACKET_SIZE])
{
	int ret = 0;
	uint8_t selected_temp = command[CMD_GROUP_ID] == TGT_TEMP_AMB_CONF ? 0:1;

	response[0] = MSG_REGBLOCKREAD;
	response[1] = command[CMD_REG_ADDR];

	ret = temp_ops[selected_temp].read(&response[1], command[CMD_NUMBER_REGS_TO_READ]);

	return ret;
}

static int sens_opt_ah_write(uint8_t reg_addr, uint8_t reg_val)
{
	int ret = 0;
	sensor_t * opt = mxm_sh_get_sensor_instance(SH_MAX86178);
	ret = opt->write_reg(&reg_addr, &reg_val);

	return ret;
}

static int sens_opt_ah_read(uint8_t * const reg_addr, uint8_t number_regs_to_read)
{
	int ret = 0;
	sensor_t * opt = mxm_sh_get_sensor_instance(SH_MAX86178);
	ret = opt->read_reg((uint8_t *)reg_addr, number_regs_to_read);

	return ret;
}

static int sens_opt_sh_write(uint8_t reg_addr, uint8_t reg_val)
{
	int ret = 0;

	ret = sh_set_reg(SH_SENSORIDX_MAX86178, reg_addr, reg_val, 1);

	return ret;
}

static int sens_opt_sh_read(uint8_t * const reg_addr, uint8_t number_regs_to_read)
{
	int ret = 0;
	uint8_t reg_start_addr = reg_addr[0];

	for(int ind = 0; ind < number_regs_to_read; ind++)
	{
		uint32_t reg_val;
		ret |= sh_get_reg(SH_SENSORIDX_MAX86178, (reg_start_addr + ind), &reg_val);
		*(reg_addr + ind) = (uint8_t)reg_val;
	}

	return ret;
}

static int sens_acc_ah_write(uint8_t reg_addr, uint8_t reg_val)
{
	int ret = 0;
	sensor_t * acc = mxm_sh_get_sensor_instance(SH_ACC_SENSOR);
	ret = acc->write_reg(&reg_addr, &reg_val);

	return ret;
}
static int sens_acc_ah_read(uint8_t * const reg_addr, uint8_t number_regs_to_read)
{
	int ret = 0;
	sensor_t * acc = mxm_sh_get_sensor_instance(SH_ACC_SENSOR);
	ret = acc->read_reg((uint8_t *)reg_addr, number_regs_to_read);

	return ret;
}
static int sens_acc_sh_write(uint8_t reg_addr, uint8_t reg_val)
{
	int ret = 0;

	ret = sh_set_reg(SH_SENSORIDX_ACCEL, reg_addr, reg_val, 1);

	return ret;
}
static int sens_acc_sh_read(uint8_t * const reg_addr, uint8_t number_regs_to_read)
{
	int ret = 0;

	for(int ind = 0; ind < number_regs_to_read; ind++)
	{
		uint32_t reg_val;
		ret |= sh_get_reg(SH_SENSORIDX_ACCEL, reg_addr[ind], &reg_val);
		reg_addr[ind] = (uint8_t)reg_val;
	}

	return ret;
}

static int sens_temp_write(uint8_t reg_addr, uint8_t reg_val)
{
	int ret = 0;
	sensor_t * temp = mxm_sh_get_sensor_instance(SH_TEMP_SENSOR);

	uint8_t params[2] = {0, reg_val};

	ret = temp->set_cfg(&reg_addr, params,1);

	return ret;
}

static int sens_temp_read(uint8_t * const reg_addr, uint8_t number_regs_to_read)
{
	int ret = 0;
	sensor_t * temp = mxm_sh_get_sensor_instance(SH_TEMP_SENSOR);
	ret = temp->get_cfg((uint8_t *)reg_addr,0, reg_addr,1);
	return ret;

}

static int sens_temp_skin_write(uint8_t reg_addr, uint8_t reg_val)
{
	int ret = 0;
	sensor_t * temp = mxm_sh_get_sensor_instance(SH_TEMP_SENSOR);

	uint8_t params[2] = {1, reg_val};

	ret = temp->set_cfg(&reg_addr, params,1);

	return ret;
}

static int sens_temp_skin_read(uint8_t * const reg_addr, uint8_t number_regs_to_read)
{
	int ret = 0;
	sensor_t * temp = mxm_sh_get_sensor_instance(SH_TEMP_SENSOR);
	ret = temp->get_cfg((uint8_t *)reg_addr,1, reg_addr,1);
	return ret;

}
