TARGET := lcd_st7789vw
FLASH_SIZE := 131072U
RAM_SIZE := 32768U

################################################################################
# Directories
################################################################################
BUILD_DIR := build
OBJ_DIR := $(BUILD_DIR)/obj
BIN_DIR := $(BUILD_DIR)/bin

SRC_DIR := drivers

BSP_DIR := bsp
HAL_DIR := $(BSP_DIR)/firmware/GD32VF103_standard_peripheral
FIRMWARE_DIR := $(BSP_DIR)/firmware/RISCV
ENTRY_DIR := $(FIRMWARE_DIR)/env_Eclipse
DRIVER_DIR := $(FIRMWARE_DIR)/drivers
STUBS_DIR := $(FIRMWARE_DIR)/stubs

################################################################################
# Toolchain
################################################################################
COMPILER_DIR := $(BSP_DIR)/compiler/bin
PREFIX := $(COMPILER_DIR)/riscv64-unknown-elf-
CC := $(PREFIX)gcc
CXX := $(PREFIX)g++
SZ := $(PREFIX)size
OD := $(PREFIX)objdump
DBG := $(PREFIX)gdb

################################################################################
# Architecture
################################################################################
#march=ISA. rv32imac = RISC-V 32 bit IMAC extenions
#ami=Application Binary Interface. ilp32=32-bit CPU, no FPU
ARCH := -march=rv32imac -mabi=ilp32 -mcmodel=medlow
################################################################################
# Compiler flags and includes
################################################################################
PPFLAGS := \
	-DUSE_STDPERIPH_DRIVER \
	-DHXTAL_VALUE=8000000U
#Define directories to be searched for header files during pre-processing
INCLUDES := \
	-I. \
	-I$(HAL_DIR) \
	-I$(HAL_DIR)/Include \
	-I$(DRIVER_DIR) \
	-I$(STUBS_DIR) \
	-I$(SRC_DIR)
#Optimization. Optimize for size
OPT := -Os -ffunction-sections -fdata-sections

DEBUG := -g

WARN := -Wall -Wextra

CFLAGS := $(ARCH) $(PPFLAGS) $(INCLUDES) $(OPT) $(DEBUG) $(WARN) \
	  -std=gnu11 \
	  -ffreestanding

CPPFLAGS := $(ARCH) $(PPFLAGS) $(INCLUDES) $(OPT) $(DEBUG) $(WARN) \
	    -std=c++03 \
	    -ffreestanding
################################################################################
# Linker
################################################################################
LDSCRIPT := $(ENTRY_DIR)/GD32VF103xB.lds
#Wl= Extra option
#-Map=Placed .map file in specified folder
#--gc-sections=
#-nostartfiles=Do not use the standard system startup files when linking (i.e use our own _start function) 
#--specs=nano.specs
#-lc= C standard library
#-lm=Link Math.h
LDFLAGS := $(ARCH) -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map -Wl,--gc-sections -nostartfiles --specs=nano.specs -lc -lm

################################################################################
# Debug and flashing
################################################################################
UTIL_DIR := compiler/openocd
OPENOCD := $(UTIL_DIR)/openocd
OPENOCD_CONF := \
		-f $(UTIL_DIR)/ft232h.cfg \
		-f $(UTIL_DIR)/gd32vf103.cfg
################################################################################
# Sources
################################################################################
C_SOURCES := \
	$(shell find $(SRC_DIR) -name '*.c') \
	$(wildcard $(HAL_DIR)/*.c) \
	$(wildcard $(HAL_DIR)/Source/*.c) \
	$(wildcard $(FIRMWARE_DIR)/drivers/*.c) \
	$(wildcard $(FIRMWARE_DIR)/stubs/*.c) \
	$(ENTRY_DIR)/handlers.c \
	$(ENTRY_DIR)/init.c

CPP_SOURCES := \
	$(shell find $(SRC_DIR) -name '*.cpp')

ASM_SOURCES := \
	$(shell find $(SRC_DIR) -name '*.S') \
	$(ENTRY_DIR)/start.S \
	$(ENTRY_DIR)/entry.S

################################################################################
# Objects - Convert source files to object files
################################################################################
OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SOURCES))
CPP_OBJECTS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CPP_SOURCES))
OBJECTS += $(patsubst %.S,$(BUILD_DIR)/%.o,$(ASM_SOURCES))
################################################################################
# vpath - Tell the compiler to search for files here 
################################################################################
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

################################################################################
# CTAGS - Generate Tags for vim using ctags
CTAG_CMD := ctags -R $(SRC_DIR) $(HAL_DIR)

################################################################################
# Build rules
################################################################################
all: $(BUILD_DIR)/$(TARGET).elf ctags

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	@$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S 
	@mkdir -p $(dir $@)
	@echo "AS  $<"
	@$(CC) -c $(ARCH) $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.cpp 
	@mkdir -p $(dir $@)
	@echo "CPP $<"
	@$(CXX) -c $(ARCH) $(CPPFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(CPP_OBJECTS)
	@echo "LD  $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@$(SZ) -A $@ | awk '/\.text/ {flash=$$2} /\.data/ {data=$$2} /\.bss/ {bss=$$2} END {printf "\n========================================================================\nFlash: %.2f%% [%d/%d] bytes | RAM: %.2f%% [%d/%d] bytes\n========================================================================\n\n", flash/128000*100, flash, 128000, (data+bss)/32728*100, (data+bss), 32728}'

ctags:
	@$(CTAG_CMD)

$(BUILD_DIR):
	@mkdir -p $@

.PHONY: all clean debug flash
clean:
	@rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(TARGET).elf
	@echo "Flashing $(BUILD_DIR)/$(TARGET).elf via openocd..."
	$(OPENOCD) $(OPENOCD_CONF) \
		-c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
debug: $(BUILD_DIR)/$(TARGET).elf
	@echo "Starting debugging session...."
	$(OPENOCD) $(OPENOCD_CONF) &
	@sleep 1
	$(DBG) $(BUILD_DIR)/$(TARGET).elf \
		-ex "target extended-remote localhost:3333"
