###############################################################################
# Generic Makefile Template for C/C++ for use with STM32 Microcontrollers
#
# Copyright (c) 2016 - James Jackson
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the <organization> nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TARGET	?= flash

DEBUG ?= GDB

SERIAL_DEVICE ?= /dev/ttyUSB0

#################################
# GNU ARM Embedded Toolchain
#################################
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
NM=arm-none-eabi-nm
SIZE=arm-none-eabi-size
A2L=arm-none-eabi-addr2line

#################################
# Working directories
#################################
BREEZY_DIR = ../../..
ROOT		 = .
SRC_DIR		 = $(ROOT)
CMSIS_DIR	 = $(BREEZY_DIR)/lib/CMSIS
STDPERIPH_DIR	 = $(BREEZY_DIR)/lib/STM32F10x_StdPeriph_Driver
OBJECT_DIR	 = obj
BIN_DIR		 = obj


#################################
# Source Files
#################################
ASOURCES = $(BREEZY_DIR)/startup_stm32f10x_md_gcc.s

VPATH		:= $(SRC_DIR):$(SRC_DIR)/startups

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
				       $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

# Add linear Algebra Classes
VPATH := $(VPATH):$(BREEZY_DIR)/lib/math
MATH_SRC = $(notdir $(wildcard $(BREEZY_DIR)/lib/math/*.cpp))

CSOURCES =  $(CMSIS_SRC) \
	    $(STDPERIPH_SRC)

CXXSOURCES = $(ROOT)/main.cpp \
             $(MATH_SRC) \
             $(BREEZY_DIR)/board.cpp \
             $(BREEZY_DIR)/eeprom.cpp

INCLUDE_DIRS	 = $(SRC_DIR) \
		   $(BREEZY_DIR) \
                   $(BREEZY_DIR)/lib/math \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x


#################################
# Object List
#################################
OBJECTS=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(ASOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(CSOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(CXXSOURCES))))

#$(info $$OBJECTS is [${OBJECTS}])

#################################
# Target Output Files
#################################
TARGET_ELF=$(BIN_DIR)/$(TARGET).elf
TARGET_HEX=$(BIN_DIR)/$(TARGET).hex

#################################
# Debug Config
#################################
ifeq ($(DEBUG), GDB)
DEBUG_FLAGS = -ggdb3
OPTIMIZE = -Og
else
OPTIMIZE = -Os
LTO_FLAGS = -flto -fuse-linker-plugin $(OPTIMIZE)
endif


#################################
# Flags
#################################
MCFLAGS=-mcpu=cortex-m3 -mthumb
OPTIMIZE =
DEBUG = -DDebug -ggdb3
DEFS=-DTARGET_STM32F10X_MD -D__CORTEX_M4 -D__FPU_PRESENT -DWORDS_STACK_SIZE=200 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
CFLAGS=-c $(MCFLAGS) $(DEBUG) $(DEFS) $(OPTIMIZE) $(DEBUG_FLAGS) $(addprefix -I,$(INCLUDE_DIRS)) -std=c99
CXXFLAGS=-c $(MCFLAGS) $(DEFS) $(OPTIMIZE) $(DEBUG_FLAGS) $(addprefix -I,$(INCLUDE_DIRS)) -std=c++11
CXXFLAGS+=-U__STRICT_ANSI__
LDSCRIPT=$(BREEZY_DIR)/stm32_flash.ld
LDFLAGS =-T $(LDSCRIPT) $(MCFLAGS) -lm -nostartfiles -lc --specs=rdimon.specs $(ARCH_FLAGS)  $(LTO_FLAGS)  $(DEBUG_FLAGS) -static  -Wl,-gc-sections

#################################
# Build
#################################
$(TARGET_HEX): $(TARGET_ELF)
	$(CP) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CXX) -c -o $@ $(CXXFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<


#################################
# Recipes
#################################
.PHONY: all flash clean

clean:
	rm -f $(OBJECTS) $(TARGET_ELF) $(TARGET_HEX) $(BIN_DIR)/output.map

flash: $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 921600 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 921600 $(SERIAL_DEVICE)


