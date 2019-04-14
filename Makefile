##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.0.0] date: [Sat Dec 29 17:39:41 NPT 2018] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = Complete_Play

USER_NAME = n-is
STM_CUBE_VERSION = STM32Cube_FW_F4_V1.23.0
######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = Build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/periphs/gpio.c \
Core/Src/periphs/adc.c \
Core/Src/periphs/dma.c \
Core/Src/periphs/i2c.c \
Core/Src/periphs/spi.c \
Core/Src/periphs/tim.c \
Core/Src/periphs/usart.c \
Core/Src/freertos.c \
\
Core/Src/utils/printf_config.c \
\
Core/Src/sys/stm32f4xx_it.c \
Core/Src/sys/stm32f4xx_hal_msp.c \
Core/Src/sys/stm32f4xx_hal_timebase_tim.c \
Core/Src/sys/system_stm32f4xx.c \
\
USB_DEVICE/App/usb_device.c \
USB_DEVICE/App/usbd_desc.c \
USB_DEVICE/App/usbd_cdc_if.c \
USB_DEVICE/Target/usbd_conf.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
\
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/list.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/timers.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
C:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

# ASM sources
ASM_SOURCES =  \
startup_stm32f407xx.s


##
# C++ sources
CXX_SOURCES = \
Core/Src/devs/wheel.cpp \
Core/Src/devs/mpu6050.cpp \
Core/Src/devs/hmc5883.cpp \
Core/Src/devs/freewheel.cpp \
Core/Src/devs/a4988.cpp \
Core/Src/devs/arduino.cpp \
Core/Src/devs/joystick.cpp \
\
Core/Src/parts/actuator.cpp \
Core/Src/parts/processor.cpp \
Core/Src/parts/state_sensor.cpp \
Core/Src/parts/devs_config.cpp \
Core/Src/parts/int_config.cpp \
\
Core/Src/parts/sensors/position_sensor.cpp \
Core/Src/parts/sensors/encoder.cpp \
Core/Src/parts/sensors/lidar.cpp \
\
Core/Src/parts/processor/robo_states.cpp \
Core/Src/parts/processor/game_field.cpp \
\
Core/Src/robot/robot.cpp \
\
Core/Src/utils/math/mat.cpp \
Core/Src/utils/math/interpolation.cpp \
Core/Src/utils/filter/exp_smooth.cpp \
Core/Src/utils/trajectory/min_jerk.cpp \
Core/Src/utils/trajectory/min_accel.cpp \
\
Core/Src/utils/logger.cpp \
\
Core/Src/sys/error.cpp \
\
Core/Src/robo_tasks.cpp \
Core/Src/main.cpp
##
#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
##
CXX = $(GCC_PATH)/$(PREFIX)g++
##
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
##
CXX = $(PREFIX)g++
##
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx


# AS includes
AS_INCLUDES =  \
-ICore\Inc

# C includes
C_INCLUDES =  \
-ICore/Inc \
-ICore/Inc/devs \
-ICore/Inc/periphs \
-ICore/Inc/robot \
-ICore/Inc/utils \
-ICore/Inc/utils\container \
-ICore/Inc/utils\control \
-ICore/Inc/utils\filter \
-ICore/Inc/utils\math \
-ICore/Inc/utils\trajectory \
-ICore/Inc/sys \
-ICore/Inc/parts \
-ICore/Inc/parts/actuators \
-ICore/Inc/parts/processor \
-ICore/Inc/parts/sensors \
\
-IUSB_DEVICE/App \
-IUSB_DEVICE/Target \
-IC:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Inc \
-IC:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IC:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/include \
-IC:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-IC:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IC:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IC:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IC:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
-IC:/Users/$(USER_NAME)/STM32Cube/Repository/${STM_CUBE_VERSION}/Drivers/CMSIS/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

##
CXXFLAGS = $(CFLAGS) -fno-exceptions
##

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407VGTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

##
# list of C++ objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))
##

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

##
$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

##

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
clean:
	del /s/q $(BUILD_DIR)

rebuild: clean all
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
