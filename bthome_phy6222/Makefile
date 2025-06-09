##############################################################################
PROJECT_NAME ?= phy6222_freertos
PROJECT_DEFS ?=
FREERTOS ?= 1
OFFLOAD_GPIO ?= 1
BTSTACK ?= 0
##############################################################################
COM_PORT = COM32
COM_SPEED = 25000
COM_STARTBAUD = 115200
UART_LOG_BPS = 115200
##############################################################################
# Source
SRC_PATH = ./source
SRC_PRJ = main.c osal_nuker.c
# pong.c display.c gfx.c fonts/FreeMono9pt7b.c fonts/FreeSans12pt7b.c

INCLUDES = -I$(SRC_PATH)

SRCS = $(addprefix $(SRC_PATH)/, $(SRC_PRJ))
##############################################################################
DEFINES = -D__GCC
DEFINES += $(PROJECT_DEFS)

DEFINES += -DARMCM0
DEFINES += -DPHY_MCU_TYPE=MCU_BUMBEE_M0
DEFINES += -DDEBUG_INFO=3
DEFINES += -DUSE_ROM_GPIO
##############################################################################
BIN_DIR = ./bin
OBJ_DIR = ./build
SDK_PATH = ./SDK
PYTHON = python3
GCC_PATH = 
CC = $(GCC_PATH)arm-none-eabi-gcc
OBJCOPY = $(GCC_PATH)arm-none-eabi-objcopy
OBJDUMP = $(GCC_PATH)arm-none-eabi-objdump
SIZE = $(GCC_PATH)arm-none-eabi-size
READELF = $(GCC_PATH)arm-none-eabi-readelf
##############################################################################
ARCH_FLAGS := -mcpu=cortex-m0 -mthumb -mthumb-interwork
OPT_CFLAGS ?= -Os
DEB_CFLAGS ?= -g3 -ggdb
##############################################################################
ASFLAGS	   := $(ARCH_FLAGS) $(OPT_CFLAGS) $(DEB_CFLAGS)
CFLAGS     := $(ARCH_FLAGS) $(OPT_CFLAGS) $(DEB_CFLAGS)
CFLAGS     += -W -Wall --std=gnu99
CFLAGS     += -fno-diagnostics-show-caret
CFLAGS     += -fdata-sections -ffunction-sections
CFLAGS     += -funsigned-char -funsigned-bitfields
#CFLAGS     += -fpack-struct
#CFLAGS     += -mno-unaligned-access
#CFLAGS     += -munaligned-access
CFLAGS     += -fms-extensions
CFLAGS     += -specs=nosys.specs
CFLAGS     += -Wl,--gc-sections


LDSCRIPT   ?= $(SDK_PATH)/misc/phy6222.ld

ADD_OPT =

LDFLAGS    := $(ARCH_FLAGS)
LDFLAGS    += --static -nostartfiles -nostdlib
LDFLAGS    += -Wl,--gc-sections
LDFLAGS    += -Wl,--script=$(LDSCRIPT)
# LDFLAGS    += -Wl,--no-warn-rwx-segments
ifeq ($(OFFLOAD_GPIO),1)
LDFLAGS    += -Wl,--just-symbols=bsp/symbols/bb_rom_sym_m0_gpio.gcc
endif
LDFLAGS    += -Wl,--just-symbols=$(SDK_PATH)/misc/bb_rom_sym_m0.gcc
LDFLAGS    += -Wl,-Map=$(OBJ_DIR)/$(PROJECT_NAME).map
LIBS       += -Wl,--start-group -lgcc -lnosys -Wl,--end-group

##############################################################################

INCLUDES += -Ibsp/ -Ibsp/CMSIS/include -Ibsp/CMSIS/device/phyplus/ -Ibsp/log

###########################################
# LIBs
SRCS += bsp/jump_table.c \
	bsp/log/log.c \
	bsp/driver/uart/uart.c \
	bsp/driver/gpio/gpio.c \
	bsp/driver/clock/clock.c \
	bsp/driver/pwrmgr/pwrmgr.c \
	bsp/driver/flash/flash.c \
	bsp/driver/spi/spi.c \
	bsp/ble/rf/patch.c \
	bsp/ble/controller/rf_phy_driver.c \

#	bsp/profiles/GATT/gattservapp.c
#	bsp/ble/host/l2cap_task.c
#	bsp/ble/host/sm_task.c
#	bsp/ble/host/gap_task.c
#	bsp/ble/host/gatt_task.c
#	bsp/ble/host/gatt_server.c
#	bsp/ble/host/att_server.c
#	bsp/ble/host/att_util.c
#	bsp/ble/host/sm_pairing.c
#	bsp/ble/host/gap_configmgr.c
#	bsp/ble/host/smp.c
#	bsp/ble/host/sm_mgr.c
#	bsp/ble/host/linkdb.c

# bsp/driver/spi/spi.c

ifeq ($(FREERTOS),1)
FREERTOS_PATH = ./freertos

SRCS_FREERTOS = portable/GCC/ARM_CM0/port.c
SRCS_FREERTOS += portable/GCC/ARM_CM0/portasm.c
SRCS_FREERTOS += portable/MemMang/heap_4.c

SRCS_FREERTOS += queue.c
SRCS_FREERTOS += list.c
SRCS_FREERTOS += tasks.c
SRCS_FREERTOS += timers.c
SRCS_FREERTOS += event_groups.c
SRCS_FREERTOS += croutine.c

INCLUDES += -I$(FREERTOS_PATH)
INCLUDES += -I$(FREERTOS_PATH)/include
INCLUDES += -I$(FREERTOS_PATH)/portable/GCC/ARM_CM0

SRCS += $(addprefix $(FREERTOS_PATH)/, $(SRCS_FREERTOS))
endif

ifeq ($(BTSTACK),1)
DEFINES += -DENABLE_BTSTACK
BTSTACK_PATH = ./btstack

SRCS_BTSTACK = 3rd-party/rijndael/rijndael.c
SRCS_BTSTACK += 3rd-party/micro-ecc/uECC.c

SRCS_BTSTACK += src/ad_parser.c \
	src/ble/att_db.c \
	src/ble/att_dispatch.c \
	src/ble/att_server.c \
	src/ble/gatt-service/ancs_client.c \
	src/ble/gatt-service/battery_service_client.c \
	src/ble/gatt-service/battery_service_server.c \
	src/ble/gatt-service/device_information_service_client.c \
	src/ble/gatt-service/device_information_service_server.c \
	src/ble/gatt-service/hids_device.c \
	src/ble/gatt_client.c \
	src/ble/le_device_db_memory.c \
	src/ble/le_device_db_tlv.c \
	src/ble/sm.c \
	src/btstack_audio.c \
	src/btstack_crypto.c \
	src/btstack_hid_parser.c \
	src/btstack_linked_list.c \
	src/btstack_memory.c \
	src/btstack_memory_pool.c \
	src/btstack_resample.c \
	src/btstack_ring_buffer.c \
	src/btstack_run_loop.c \
	src/btstack_tlv.c \
	src/btstack_tlv_none.c \
	src/btstack_util.c \
	src/hci.c \
	src/hci_cmd.c \
	src/hci_dump.c \
	src/hci_event_builder.c \
	src/hci_event.c \
	src/l2cap.c \
	src/l2cap_signaling.c \
	src/hci_transport_h4.c \
	platform/freertos/btstack_run_loop_freertos.c

INCLUDES += -I$(OBJ_DIR)/btstack/example

#SRCS += $(SRC_PATH)/gatt_battery_query.c
SRCS += $(SRC_PATH)/gap_le_advertisements.c
#SRCS += $(SRC_PATH)/gatt_counter.c
SRCS += $(SRC_PATH)/btstack_port.c

INCLUDES += -I$(BTSTACK_PATH)
INCLUDES += -I$(BTSTACK_PATH)/src/ble
INCLUDES += -I$(BTSTACK_PATH)/src/ble/gatt-service
INCLUDES += -I$(BTSTACK_PATH)/src
INCLUDES += -I$(BTSTACK_PATH)/3rd-party/micro-ecc
INCLUDES += -I$(BTSTACK_PATH)/3rd-party/rijndael
INCLUDES += -I$(BTSTACK_PATH)/platform/embedded
INCLUDES += -I$(BTSTACK_PATH)/platform/freertos
INCLUDES += -I$(BTSTACK_PATH)/example

SRCS += $(addprefix $(BTSTACK_PATH)/, $(SRCS_BTSTACK))

GATT_FILES = $(BTSTACK_PATH)/example/gatt_battery_query.gatt
#GATT_FILES = $(BTSTACK_PATH)/example/gatt_counter.gatt

GATT_OBJ = $(addprefix $(OBJ_DIR)/,$(GATT_FILES:.gatt=.h))
endif

##############################################################################

STARTUP_ASM = bsp/CMSIS/device/phyplus/phy6222_start.s
SRCS += bsp/CMSIS/device/phyplus/phy6222_cstart.c
SRCS += bsp/CMSIS/device/phyplus/phy6222_vectors.c

##############################################################################

CFLAGS  += $(DEFINES) $(INCLUDES)


SRC_O = $(SRCS:%.c=%.o) $(STARTUP_ASM:%.s=%.o)
OBJS = $(patsubst %, $(OBJ_DIR)/%, $(patsubst ./%, %, $(SRC_O)))

DEPENDENCY_LIST = $(OBJS:%o=%d)

##############################################################################
.PHONY: all directory clean size flash erase_and_flash

all: directory $(GATT_OBJ) $(SRC_O) $(OBJ_DIR)/$(PROJECT_NAME).elf $(OBJ_DIR)/$(PROJECT_NAME).hex $(BIN_OTA) $(OBJ_DIR)/$(PROJECT_NAME).asm size

$(OBJ_DIR)/%.h: %.gatt
	python3 ${BTSTACK_PATH}/tool/compile_gatt.py $< $@

%.elf %.map: $(SRC_O) $(LDSCRIPT) Makefile
	@echo LD: $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

%.hex: %.elf
	@echo OBJCOPY: $@
	@$(OBJCOPY) -O ihex $^ $@

%.bin: %.hex
	@echo Make: $@
	@$(PYTHON) ./phy62x2_ota.py $(ADD_OPT) $(OBJ_DIR)/$(PROJECT_NAME).hex

%.asm: %.elf
	@echo OBJDUMP: $@
	@$(OBJDUMP) -s -S $^ >$@ 

%.o : %.c
	@echo CC: $<
	@mkdir -p $(OBJ_DIR)/$(dir $@)
	@$(CC) $(CFLAGS) $(INCFLAGS) -c $< -o $(OBJ_DIR)/$@
	@$(CC) -MM $(CFLAGS) $(INCFLAGS) $< -MT $@ -MF $(OBJ_DIR)/$(patsubst %.o,%.d,$@)

%.o : %.s
	@echo CC: $<
	@mkdir -p $(OBJ_DIR)/$(dir $@)
	@$(CC) $(CFLAGS) $(INCFLAGS) -c $< -o $(OBJ_DIR)/$@
	@$(CC) -MM $(CFLAGS) $(INCFLAGS) $< -MT $@ -MF $(OBJ_DIR)/$(patsubst %.o,%.d,$@)

flash: $(OBJ_DIR)/$(PROJECT_NAME).hex
	@$(PYTHON) ./rdwr_phy62x2.py -p$(COM_PORT) -b $(COM_SPEED) --startbaud $(COM_STARTBAUD) -r wh $(OBJ_DIR)/$(PROJECT_NAME).hex

terminal:
	@$(PYTHON) ./miniterm.py $(COM_PORT) $(UART_LOG_BPS) --rts 1 --rtstoggle 100 --exit-char 3 --rtsexit 1

terminal_flash: flash size
	@$(PYTHON) ./miniterm.py $(COM_PORT) $(UART_LOG_BPS) --rts 1 --rtstoggle 100 --exit-char 3 --rtsexit 1

identify:
	@$(PYTHON) ./rdwr_phy62x2.py -p$(COM_PORT) -b $(COM_SPEED) i

erase_and_flash:
	@$(PYTHON) ./rdwr_phy62x2.py -p$(COM_PORT) -b $(COM_SPEED) -e -r wh $(OBJ_DIR)/$(PROJECT_NAME).hex

reset:
	@$(PYTHON) ./rdwr_phy62x2.py -p$(COM_PORT) -r i

directory:
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(OBJ_DIR)/btstack/example

size: $(OBJ_DIR)/$(PROJECT_NAME).elf
	@echo size:
	@$(SIZE) -t $^
	@$(READELF) -l $^
	@echo

clean:
	@echo clean
	@-rm -rf $(OBJ_DIR)

-include $(DEPENDENCY_LIST)

VPATH:=$(OBJ_DIR) $(SDK_PATH)
