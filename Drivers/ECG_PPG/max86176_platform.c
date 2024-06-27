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

#include "max86178_platform.h"
#include "max86178.h"
#include "board.h"
#include "mxc_delay.h"

#include "spi_api.h"

#include "Peripherals.h"

static pdata_t max86178_pdata;
extern int max86178_init(pdata_t *pdata);

// *****************************************************************************
#define SPI_READ_CMD        0x80
#define SPI_WRITE_CMD       0x00

#define SPI_SPEED_MAX86176      1000000  // Bit Rate


//extern gpio_cfg_t mfio_pin;
extern gpio_cfg_t max86178_intb_gpio;

#ifdef FTHR
static uint32_t spi_index = 2;
#define MAX86176_CS_PORT PORT_0
#define MAX86176_CS_PIN PIN_10
#else
static uint32_t spi_index = 0;
#define MAX86176_CS_PORT PORT_0
#define MAX86176_CS_PIN PIN_16
#endif

int _update_bits(dev_comm_t *comm, u8 reg_addr,
		u8 mask, u8 val)
{
	int ret;
	u8 tmp, orig = reg_addr;

	ret = max86178_read_reg(comm, &orig, 1);
	if (ret < 0)
		return ret;

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		ret = max86178_write_reg(comm, reg_addr, tmp);
	return ret;
}

int platform_spi_read_byte(dev_comm_t *comm,
		uint8_t *buf, int len)
{

	int ret;

	uint8_t tx_data[2] = {buf[0], SPI_READ_CMD};
	uint8_t rx_buf[len];


	ret = spi_api_read(spi_index, tx_data, rx_buf, sizeof(tx_data), sizeof(rx_buf));

	memcpy(buf, rx_buf, len);

	return ret;
}

int platform_spi_write_byte(dev_comm_t *comm,
		uint8_t reg_addr, uint8_t buf)
{
	int ret = E_SUCCESS;

	uint8_t tx_buf[3];

	tx_buf[0] = reg_addr;
	tx_buf[1] = SPI_WRITE_CMD;
	tx_buf[2] = buf;


	ret = spi_api_write(spi_index, tx_buf, sizeof(tx_buf));

	return ret;

}

#if defined(SENSOR_MAX86176_USES_I2C)
uint8_t os58_i2c_read_reg(uint8_t *buf, int buf_len) {
    uint8_t received_bytes;

	int ret;

	ret = I2C_MasterWrite(MXC_I2C0, OS64L_I2C_ADDRESS, buf, sizeof(uint8_t), 1);
	if (ret != sizeof(uint8_t)) // Register address length is 1 byte
	{
		printf("os58 read reg failed! (ret:%d)\r\n", ret);
		return ret == 0 ? -1 : ret;
	}

	//memset(buf, 0, buf_len); // I2C_MasterRead function overrites, not need to clear
	received_bytes = I2C_MasterRead(MXC_I2C0, OS64L_I2C_ADDRESS, buf, buf_len, 0);

	if (received_bytes <= 0) {
		printf("os58 read reg failed! (recieved %d bytes)\r\n", received_bytes);
	}

    return received_bytes;
}

bool os58_i2c_write_reg(uint8_t reg, uint8_t data)
{
	int ret;
	uint8_t i2c_buf[] = { reg, data };

	ret = I2C_MasterWrite(MXC_I2C0, OS64L_I2C_ADDRESS, i2c_buf, sizeof(i2c_buf), 0);

	if (ret != sizeof(i2c_buf))
		printf("os58 write reg failed!\r\n");

	return (ret == sizeof(i2c_buf));
}
#endif

/* Wrapper Functions */
int max86178_write_reg(dev_comm_t *comm, uint8_t reg_addr, uint8_t reg_data)
{
#if defined(SENSOR_MAX86176_USES_I2C)
	return os58_i2c_write_reg(reg_addr, reg_data);
#else
	return platform_spi_write_byte(comm, reg_addr, reg_data);
#endif
}

int max86178_read_reg(dev_comm_t *comm, uint8_t *buffer, int length)
{
#if defined(SENSOR_MAX86176_USES_I2C)
	return os58_i2c_read_reg(buffer, length);
#else
	return platform_spi_read_byte(comm, buffer, length);
#endif
}

int max86178_block_write(dev_comm_t *comm, const struct regmap reg_block[], int size)
{
	int i;
	int ret = 0;

	for (i = 0; i < size; i++) {
		ret = max86178_write_reg(comm, reg_block[i].addr, reg_block[i].val);
		if (ret < 0)
		{
			pr_info("max86178_write_reg returned %d\n", ret);
			return ret;
		}
	}

	return ret;
}

int max86178_comm_test(dev_comm_t *comm)
{
	uint8_t dump_buf[127];
	int i;
	max86178_dump_regs(comm, dump_buf, 0x00, 0x42);
	for (i = 0; i <= 0x42; i++)
		pr_debug("Reg[%.2X]: %.2X", i, dump_buf[i]);

	max86178_dump_regs(comm, dump_buf, 0xF0, 0xFF);
	for (i = 0xF0; i <= 0xFF; i++)
		pr_debug("Reg[%.2X]: %.2X", i, dump_buf[i - 0xF0]);


	return 0;
}

int read_part_id(void)
{
	int ret;
	uint8_t buf[2] = {0xFE, };

	ret = max86178_read_reg(NULL, &buf[0], 2);
	if (ret < 0)
		return ret;

	return 0;
}

int spi_rw_test(void)
{
	int ret;
	uint8_t reg_addr = 0x20;
	uint8_t reg_val = 0x21;
	//uint8_t read_reg_addr = reg_addr;
	uint8_t rx_buf[2];
	uint8_t read_byte;

	ret = max86178_write_reg(NULL, reg_addr, reg_val);
	pr_info("Max86140_SS_Write_Byte %d %d %d\n", ret, reg_addr, reg_val);
	mxc_delay(MXC_DELAY_MSEC(10));
	read_byte = reg_addr;
	ret = max86178_read_reg(NULL, &read_byte, sizeof(read_byte));
	pr_info("max86178_read_reg %d %d %d\n", ret, reg_addr, reg_val);
	mxc_delay(MXC_DELAY_MSEC(20));

	reg_addr = 0x23;
	reg_val = 0x3F;
	ret =  max86178_write_reg(NULL, reg_addr, reg_val);
	pr_info("Max86140_SS_Write_Byte %d %d %d\n", ret, reg_addr, reg_val);
	mxc_delay(MXC_DELAY_MSEC(10));
	read_byte = reg_addr;
	ret = max86178_read_reg(NULL, &read_byte, sizeof(read_byte));

	pr_info("max86178_read_reg %d %d %d\n", ret, reg_addr, reg_val);
	mxc_delay(MXC_DELAY_MSEC(20));

	reg_addr = 0x24;
	reg_val = 0x3F;
	ret = max86178_write_reg(NULL, reg_addr, reg_val);
	pr_info("max86178_read_reg %d %d %d\n", ret, reg_addr, reg_val);

	read_byte = reg_addr;
	ret = max86178_read_reg(NULL, &read_byte, sizeof(read_byte));

	mxc_delay(MXC_DELAY_MSEC(20));

	//read_reg_addr = 0x23;
	read_byte = reg_addr;
	ret = max86178_read_reg(NULL, &rx_buf[0], sizeof(rx_buf));
	mxc_delay(MXC_DELAY_MSEC(10));

	reg_addr = 0x23;
	reg_val = 0x00;
	ret = max86178_write_reg(NULL, reg_addr, reg_val);
	pr_info("Max86140_SS_Write_Byte %d %d %d\n", ret, reg_addr, reg_val);

	read_byte = reg_addr;
	ret = max86178_read_reg(NULL, &read_byte, sizeof(read_byte));

	pr_info("max86178_read_reg %d %d %d \n", ret, reg_addr, reg_val);
	mxc_delay(MXC_DELAY_MSEC(10));

	reg_addr = 0x24;
	reg_val = 0x00;
	ret = max86178_write_reg(NULL, reg_addr, reg_val);
	pr_info("Max86140_SS_Write_Byte %d %d %d \n", ret, reg_addr, reg_val);
	read_byte = reg_addr;
	ret = max86178_read_reg(NULL, &read_byte, sizeof(read_byte));
	pr_info("max86178_read_reg %d %d %d\n", ret, reg_addr, reg_val);
	mxc_delay(MXC_DELAY_MSEC(20));

	return ret;
}

int max86178_spi_test(void)
{
	int ret = 0;
	mxc_delay(MXC_DELAY_SEC(1));

	ret = spi_api_init(1000000);
	if(E_NO_ERROR == ret){
		pr_debug("SPI Initialized\n");
	}

	ret = spi_api_open(&spi_index, MAX86176_CS_PORT, MAX86176_CS_PIN);
	if(E_NO_ERROR == ret){
		pr_debug("SPI opened\n");
	}

	while (1) {
		read_part_id();
		spi_rw_test();
	}

	return ret;
}



int max86178_spi_init(dev_comm_t *comm)
{
	int ret = E_NO_ERROR;

	ret = spi_api_init(1000000);
	if(E_NO_ERROR == ret){
		pr_debug("SPI Initialized\n");
	}

	ret = spi_api_open(&spi_index, MAX86176_CS_PORT, MAX86176_CS_PIN);
	if(E_NO_ERROR == ret){
		pr_debug("SPI opened\n");
	}

//	I do not the reason of this, it is never used.
//  const gpio_cfg_t OS64_cs1 = { PORT_7, PIN_0, GPIO_FUNC_GPIO, GPIO_PAD_INPUT};
//
//  //Configure GPIO
//  if (GPIO_Config(&OS64_cs1) != E_NO_ERROR) {
//    pr_debug("Error setting OS64_cs1\n");
//  }


	return ret;
}


int max86178_driver_init(void)
{
	int ret = 0;
	//max86178_spi_test();
	memset(&max86178_pdata, 0, sizeof(max86178_pdata));
	//spi_config(&max86178_pdata.comm, SPI_BUS_ID, SPI_SS1);

	max86178_pdata.gpio = (gpio_cfg_t *)&max86178_intb_gpio;

	max86178_spi_init(max86178_pdata.comm);	// FIXME TM COMMENT this function call does nothing if "MAX32660" is defined.
	ret = max86178_init(&max86178_pdata);
	if (ret < 0) {
		pr_err("Initializing %s failed.\n", __func__);
		return ret;
	}

	pr_debug("%s:%d Done %d\n", __func__, __LINE__, ret);
	return 0;
}
