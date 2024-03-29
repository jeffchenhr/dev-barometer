##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

BINARY = main

# Comment the following line if you _don't_ have luftboot flashed!
LDFLAGS += -Wl,-Ttext=0x8002000
CFLAGS += -std=c99
LDSCRIPT = lisa-m.ld

### include ../../Makefile.include ###

PREFIX	?= arm-none-eabi
#PREFIX		?= arm-elf
CC		= $(PREFIX)-gcc
LD		= $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
GDB		= $(PREFIX)-gdb

TOOLCHAIN_DIR ?= ./libopencm3
ifeq ($(wildcard ./libopencm3/lib/libopencm3_stm32f1.a),)
ifneq ($(strip $(shell which $(CC))),)
TOOLCHAIN_DIR := $(shell dirname `which $(CC)`)/../$(PREFIX)
endif
else
ifeq ($(V),1)
$(info We seem to be building the example in the source directory. Using local library!)
endif
endif

# removed before -I$(TOOLCHAIN_DIR)/include		   -Wundef -Wshadow 
ARCH_FLAGS      = -mthumb -mcpu=cortex-m3 -msoft-float
# CFLAGS		+= -Os -g
CFLAGS		+= -Os -g \
		   -Wall -Wextra -Wimplicit-function-declaration \
		   -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes \
		   -I$(TOOLCHAIN_DIR)/include -I. \
		   -fno-common $(ARCH_FLAGS) -MD -DSTM32F1
LDSCRIPT	?= $(BINARY).ld
LDFLAGS		+= --static -Wl,--start-group -lm -lc -lgcc -lnosys -Wl,--end-group \
		   -L$(TOOLCHAIN_DIR)/lib \
		   -T$(LDSCRIPT) -nostartfiles -Wl,--gc-sections \
		   $(ARCH_FLAGS) -mfix-cortex-m3-ldrd

ifneq ($(OPENCM3_DIR),)
CFLAGS          += -I$(OPENCM3_DIR)/include
LDFLAGS         += -L$(OPENCM3_DIR)/lib -L$(OPENCM3_DIR)/lib/stm32/f1
SCRIPT_DIR      = $(OPENCM3_DIR)/share
else
SCRIPT_DIR      = $(shell dirname $(shell readlink -f $(shell which $(PREFIX)-gcc)))/../$(PREFIX)/share
endif


OBJS		+= $(BINARY).o actuators_pwm_arch.o spi.o imu_aspirin2.o spi_arch.o imu.o gpio_arch.o radio_control.o spektrum_arch.o uart.o uart_arch.o \
				ahrs.o ahrs_int_cmpl_quat.o math/pprz_trig_int.o ahrs_aligner.o controls.o i2c_arch.o i2c.o ms5611_spi.o baro_ms5611_spi.o ms5611.o

OOCD		?= openocd
OOCD_INTERFACE	?= flossjtag
OOCD_BOARD	?= olimex_stm32_h103
# Black magic probe specific variables
# Set the BMP_PORT to a serial port and then BMP is used for flashing
BMP_PORT        ?=
# texane/stlink can be used by uncommenting this...
# or defining it in your own makefiles
#STLINK_PORT ?= :4242

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
else
LDFLAGS += -Wl,--print-gc-sections
endif

.SUFFIXES: .elf .bin .hex .srec .list .images
.SECONDEXPANSION:
.SECONDARY:

all: images

images: $(BINARY).images
flash: $(BINARY).flash

%.images: %.bin %.hex %.srec %.list
	@echo "*** $* images generated ***"

%.bin: %.elf
	@printf "  OBJCOPY $(*).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@printf "  OBJCOPY $(*).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(*).elf $(*).hex

%.srec: %.elf
	@printf "  OBJCOPY $(*).srec\n"
	$(Q)$(OBJCOPY) -Osrec $(*).elf $(*).srec

%.list: %.elf
	@printf "  OBJDUMP $(*).list\n"
	$(Q)$(OBJDUMP) -S $(*).elf > $(*).list

%.elf: $(OBJS) $(LDSCRIPT) $(TOOLCHAIN_DIR)/lib/libopencm3_stm32f1.a
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) -o $(*).elf $(OBJS) -lopencm3_stm32f1 $(LDFLAGS)

%.o: %.c Makefile
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

clean:
	$(Q)rm -f *.o
	$(Q)rm -f *.d
	$(Q)rm -f *.elf
	$(Q)rm -f *.bin
	$(Q)rm -f *.hex
	$(Q)rm -f *.srec
	$(Q)rm -f *.list

ifeq ($(STLINK_PORT),)
ifeq ($(BMP_PORT),)
ifeq ($(OOCD_SERIAL),)
%.flash: %.hex
	@printf "  FLASH   $<\n"
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		    -f board/$(OOCD_BOARD).cfg \
		    -c "init" -c "reset init" \
		    -c "stm32f1x mass_erase 0" \
		    -c "flash write_image $(*).hex" \
		    -c "reset" \
		    -c "shutdown" $(NULL)
else
%.flash: %.hex
	@printf "  FLASH   $<\n"
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		    -f board/$(OOCD_BOARD).cfg \
		    -c "ft2232_serial $(OOCD_SERIAL)" \
		    -c "init" -c "reset init" \
		    -c "stm32f1x mass_erase 0" \
		    -c "flash write_image $(*).hex" \
		    -c "reset" \
		    -c "shutdown" $(NULL)
endif
else
%.flash: %.elf
	@echo "  GDB   $(*).elf (flash)"
	$(Q)$(GDB) --batch \
		   -ex 'target extended-remote $(BMP_PORT)' \
		   -x $(TOOLCHAIN_DIR)/scripts/black_magic_probe_flash.scr \
		   $(*).elf
endif
else
%.flash: %.elf
	@echo "  GDB   $(*).elf (flash)"
	$(Q)$(GDB) --batch \
		   -ex 'target extended-remote $(STLINK_PORT)' \
		   -x $(SCRIPT_DIR)/libopencm3/scripts/stlink_flash.scr \
		   $(*).elf
endif

.PHONY: images clean

-include $(OBJS:.o=.d)

