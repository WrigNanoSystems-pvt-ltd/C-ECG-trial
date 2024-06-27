 

ifeq "$(DRIVERS_DIR)" ""
$(error DRIVERS_DIR must be set)
endif

PROJ_CFLAGS+=-DENABLE_MAX86176IRQ
PROJ_CFLAGS+=-DENABLE_ADXL367
PROJ_CFLAGS+=-DTEMP_RAW_DATA


# Source files for this test (add path to VPATH below)
SRCS += BLE/ble_api.c
SRCS += BLE/ble_service.c

SRCS += USB/mscmem.c
SRCS += USB/usb_api.c

SRCS += SPI/spi_api.c
SRCS += I2C/i2c_api.c
SRCS += ADXL367Z/adxl367.c
SRCS += ADXL367Z/regHandler.c
SRCS += ADXL367Z/regHandler_spi.c
SRCS += ADXL367Z/regHandler_i2c.c
SRCS += ADXL367Z/adxl367_ss_instance.c
SRCS += Button/button.c
SRCS += Button/button_gesture_handler.c
SRCS += MRD106/mrd106.c
SRCS += MRD106/mrd106_isr.c
SRCS += MRD106/mrd106_helper.c
SRCS += max86178_platform.c
SRCS += max86178_ss_instance.c
SRCS += max86178.c

SRCS += GUI/gui.c
SRCS += GUI/app_gui_command.c
SRCS += GUI/app_gui_nim_command.c
SRCS += GUI/app_gui.c
SRCS += GUI/app_gui_sensor_command.c
SRCS += GUI/app_gui_nrf_command.c
SRCS += GUI/app_interface_process.c
SRCS += GUI/app_interface_ble_process.c
SRCS += GUI/MWA/app_mwa.c

SRCS += MAX30210/max30210_platform_i2c.c
SRCS += MAX30210/max30210_ss_instance.c
SRCS += MAX30210/max30210x2_ss_instance.c
SRCS += MAX30210/max30210.c

SRCS += MAX20356/max20356.c
SRCS += MAX20356/max20356_platform_i2c.c


SRCS += MAX17260/max17260.c
SRCS += MAX17260/max17260_platform_i2c.c

SRCS += LED/app_led.c
SRCS += LED/app_led_wrapper.c

SRCS += FLASH/app_flash.c

SRCS += Algohub/i2c_ah_sh_api.c
SRCS += Algohub/sh_comm.c
SRCS += Algohub/algohub_api.c
SRCS += Algohub/algohub_config_api.c
SRCS += Algohub/sensorhub_api.c
SRCS += Algohub/sensorhub_config_api.c
SRCS += Algohub/algohub_sensorhub_manager.c

SRCS += FatFS/source/diskio.c
SRCS += FatFS/source/ff.c
SRCS += FatFS/source/ffsystem.c
SRCS += FatFS/source/ffunicode.c
SRCS += FatFS/app_fatfs.c

SRCS += MX66/mx66.c

SRCS += LocationFinder/locationfinder.c

# Where to find Interface source files
VPATH += $(DRIVERS_DIR)/BLE

VPATH += $(DRIVERS_DIR)/USB

VPATH += $(DRIVERS_DIR)/SPI
VPATH += $(DRIVERS_DIR)/I2C
VPATH += $(DRIVERS_DIR)/Button

VPATH += $(DRIVERS_DIR)/ADXL367Z

VPATH += $(DRIVERS_DIR)/MAX86178

VPATH += $(DRIVERS_DIR)/GUI
VPATH += $(DRIVERS_DIR)/GUI/MWA

VPATH += $(DRIVERS_DIR)/MRD106
VPATH += $(DRIVERS_DIR)/MAX30208
VPATH += $(DRIVERS_DIR)/MAX30210
VPATH += $(DRIVERS_DIR)/MAX20356
VPATH += $(DRIVERS_DIR)/MAX17260


VPATH += $(DRIVERS_DIR)/Algohub


VPATH += $(DRIVERS_DIR)/LED

VPATH += $(DRIVERS_DIR)/FLASH

VPATH += $(DRIVERS_DIR)/FatFS
VPATH += $(DRIVERS_DIR)/FatFS/source
VPATH += $(DRIVERS_DIR)/MX66
VPATH += $(DRIVERS_DIR)/LocationFinder

# Where to find Interface header files
IPATH += $(DRIVERS_DIR)/BLE

IPATH += $(DRIVERS_DIR)/USB

IPATH += $(DRIVERS_DIR)/SPI
IPATH += $(DRIVERS_DIR)/I2C
IPATH += $(DRIVERS_DIR)/ADXL367Z
IPATH += $(DRIVERS_DIR)/Button

IPATH += $(DRIVERS_DIR)/MAX86178

IPATH += $(DRIVERS_DIR)/GUI
IPATH += $(DRIVERS_DIR)/GUI/MWA

IPATH += $(DRIVERS_DIR)/MRD106

IPATH += $(DRIVERS_DIR)/MAX20356
IPATH += $(DRIVERS_DIR)/MAX30210
IPATH += $(DRIVERS_DIR)/MAX17260


IPATH += $(DRIVERS_DIR)/Algohub


IPATH += $(DRIVERS_DIR)/LED

IPATH += $(DRIVERS_DIR)/FLASH

IPATH += $(DRIVERS_DIR)/FatFS
IPATH += $(DRIVERS_DIR)/FatFS/source
IPATH += $(DRIVERS_DIR)/MX66
IPATH += += $(DRIVERS_DIR)/LocationFinder

