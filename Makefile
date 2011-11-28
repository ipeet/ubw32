#  Copyright 2011 Iain Peet
# 
#  Root makefile.  Includes subdir makefiles to get a complete list of
#  available sources; generates deps and objects for those sources, 
#  and compiles binaries from the objects.
# 
#  This program is distributed under the of the GNU Lesser Public License. 
# 
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>

.DEFAULT_GOAL=all
.SECONDEXPANSION:

# List of subdir from which to obtain source lists
SRCDIRS:= console \
	core \
	fileio \
	io \
	linux \
	ubw32 \
	usb \
	uwrt

# Include all the subdirs
include $(patsubst %,%/Makefile,$(SRCDIRS))

# Generated directories
GENDIR:=gen

# Linux compile config
# Specific gcc version until gcc-4.6 in *buntu 11.10 cleans up its act.
LINUX_CC:= gcc-4.4
LINUX_CCFLAGS:= -Wall -Wextra -g -O0
LINUX_INCLUDES:= -I. 
LINUX_LDFLAGS:= $(LINUX_CCFLAGS)
LINUX_LIBS:= -lm

# PIC32 compile config
C32_DIR:=/usr/local/lib/c32
MPROCESSOR:=32MX460F512L
PIC32_CC:=wine $(C32_DIR)/bin/pic32-gcc.exe
PIC32_LD:=$(PIC32_CC)
PIC32_BIN2HEX:=wine $(C32_DIR)/bin/pic32-bin2hex.exe
PIC32_CFLAGS:= -Wall -Wextra -mprocessor=$(MPROCESSOR)
PIC32_INCLUDES:= -I. -I./usb/include -I./usb -I$(C32_DIR)/include
PIC32_LDFLAGS:= -Wall -Wextra -mprocessor=$(MPROCESSOR) -Wl,--defsym,_min_heap_size=1024
PIC32_LIBS:=

# Suppress wine's noisiness.
export WINEDEBUG:=

# Linux binaries:
LINUX_BINS:= kalman_test air_functions_test acquire

# PIC32 binaries:
PIC32_BINS:= ubw32 uwrt_control console

# Kalman filter test binary:
kalman_test_OBJS:= \
	uwrt/est_apogee.o \
	uwrt/kalman.o \
	uwrt/kalman_test.o \
	linux/gnuplot.o
kalman_test_OBJS:= $(patsubst %.o,$(GENDIR)/linux/%.o,$(kalman_test_OBJS))

# Air functions test binary:
air_functions_test_OBJS:= \
	uwrt/air_functions.o \
	uwrt/air_functions_test.o
air_functions_test_OBJS:= $(patsubst %.o,$(GENDIR)/linux/%.o,$(air_functions_test_OBJS))

# Data acquisition binary:
acquire_OBJS:= gen/linux/linux/acquire_main.o

# Standard UBW32 Firmware:
ubw32_OBJS:= \
	$(patsubst %.c,$(GENDIR)/pic32/%.o,$(USB_PIC32_CC_SRCS)) \
	$(patsubst %.c,$(GENDIR)/pic32/%.o,$(UBW_PIC32_CC_SRCS))

# Core UBW32 code:
UBW32_COMMON_OBJS:= $(CORE_PIC32_CC_SRCS) \
	$(FILEIO_PIC32_CC_SRCS) \
	$(IO_PIC32_CC_SRCS) \
	$(USB_PIC32_CC_SRCS) 
UBW32_COMMON_OBJS:= $(patsubst %.c,$(GENDIR)/pic32/%.o,$(UBW32_COMMON_OBJS))

# Console firmware:
console_OBJS:= \
	$(UBW32_COMMON_OBJS) \
	gen/pic32/console/main.o

# UWRT rocket controller firmware:
uwrt_control_OBJS:= \
	$(UBW32_COMMON_OBJS) \
	gen/pic32/uwrt/control.o \
	gen/pic32/uwrt/control_main.o \
	gen/pic32/uwrt/est_apogee.o \
	gen/pic32/uwrt/igniters.o \
	gen/pic32/uwrt/kalman.o \

# Compilation of linux c objects:
LINUX_CC_OBJS:=$(patsubst %.c,$(GENDIR)/linux/%.o,$(LINUX_CC_SRCS))
$(LINUX_CC_OBJS): $$(patsubst $(GENDIR)/linux/%.o,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	@echo [LINUX] Compiling $<
	@$(LINUX_CC) $(LINUX_CCFLAGS) $(LINUX_INCLUDES) $< -c -o $@

# linux .c deps generation
LINUX_CC_DEPS:=$(patsubst %.c,$(GENDIR)/linux/%.d,$(LINUX_CC_SRCS))
$(LINUX_CC_DEPS): $$(patsubst $(GENDIR)/linux/%.d,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	@echo [LINUX] Generating deps for $<
	@$(LINUX_CC) $(LINUX_INCLUDES) -MM -MT $(<:%.c=$(GENDIR)/linux/%.o) -MF $@ $<
include $(LINUX_CC_DEPS)

# linux bin compilation:
$(LINUX_BINS): $$($$@_OBJS)
	@echo [LINUX] Linking $@
	@$(LINUX_CC) $(LINUX_LDFLAGS) $(LINUX_LIBS) $^ -o $@

# Compilation of PIC32 C objects:
PIC32_CC_OBJS:=$(patsubst %.c,$(GENDIR)/pic32/%.o,$(PIC32_CC_SRCS))
$(PIC32_CC_OBJS): $$(patsubst $(GENDIR)/pic32/%.o,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	@echo [PIC32] Compiling $<
	@$(PIC32_CC) $(PIC32_CFLAGS) $(PIC32_INCLUDES) $< -c -o $@

# PIC32 deps generation
PIC32_CC_DEPS:=$(patsubst %.c,$(GENDIR)/pic32/%.d,$(PIC32_CC_SRCS))
$(PIC32_CC_DEPS):$$(patsubst $(GENDIR)/pic32/%.d,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	@echo [PIC32] Generating deps for $<
	@$(PIC32_CC) $(PIC32_INCLUDES) -MM -MT $(<:%.c=$(GENDIR)/pic32/%.o) -MF $@ $<
include $(PIC32_CC_DEPS)

# Compilation of a pic32 binary:
PIC32_ELFS:= $(patsubst %,$(GENDIR)/%.elf,$(PIC32_BINS))
$(PIC32_ELFS): $$($$(patsubst $(GENDIR)/%.elf,%_OBJS,$$@))
	@echo [PIC32] Linking $@
	@$(PIC32_LD) $(PIC32_LDFLAGS) $(PIC32_LIBS) $^ -o $@

# Conversion of binary elf to a programmable hex:
PIC32_HEXES:= $(patsubst %,$(GENDIR)/%.hex,$(PIC32_BINS))
$(PIC32_HEXES): $$(patsubst %.hex,%.elf,$$@)
	@echo [PIC32] Generating $@
	@$(PIC32_BIN2HEX) $< 

# Make deps
deps: $(LINUX_CC_DEPS) $(PIC32_CC_DEPS)

# All linux targets:
linux: $(LINUX_BINS)

# All pic32 targets:
pic32: $(PIC32_HEXES)

tags: $(LINUX_CC_OBJS) $(PIC32_CC_OBJS)
	ctags -R .

all: linux pic32 tags

clean:
	rm -rf $(GENDIR)
	rm -f $(LINUX_BINS)
	rm -f tags

debugp:
	@echo LINUX_CC_SRCS: $(LINUX_CC_SRCS)
	@echo LINUX_CC_OBJS: $(LINUX_CC_OBJS)
	@echo LINUX_CC_DEPS: $(LINUX_CC_DEPS)
	@echo PIC32_CC_SRCS: $(PIC32_CC_SRCS)
	@echo PIC32_CC_OBJS: $(PIC32_CC_OBJS)
	@echo PIC32_CC_DEPS: $(PIC32_CC_DEPS)
	@echo PIC32_ELFS:    $(PIC32_ELFS)
	@echo PIC32_HEXES:   $(PIC32_HEXES)

