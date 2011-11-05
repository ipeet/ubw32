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
SRCDIRS:= ekf \
	linux \
	ubw32 \
	usb 

# Include all the subdirs
include $(patsubst %,%/Makefile,$(SRCDIRS))

# Generated directories
GENDIR:=gen

# Linux compile config
LINUX_CC:= gcc
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
LINUX_BINS:= kalman_test acquire

# PIC32 binaries:
PIC32_BINS:= ubw32

# Objects to include in each binary:
kalman_test_OBJS:= \
	ekf/kalman.o \
	ekf/kalman_test.o \
	linux/gnuplot.o
kalman_test_OBJS:= $(patsubst %.o,$(GENDIR)/linux/%.o,$(kalman_test_OBJS))

acquire_OBJS:= gen/linux/linux/acquire_main.o

ubw32_OBJS:= \
	$(patsubst %.c,$(GENDIR)/pic32/%.o,$(USB_PIC32_CC_SRCS)) \
	$(patsubst %.c,$(GENDIR)/pic32/%.o,$(UBW_PIC32_CC_SRCS))

# Compilation of linux c objects:
LINUX_CC_OBJS:=$(patsubst %.c,$(GENDIR)/linux/%.o,$(LINUX_CC_SRCS))
$(LINUX_CC_OBJS): $$(patsubst $(GENDIR)/linux/%.o,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	$(LINUX_CC) $(LINUX_CCFLAGS) $(LINUX_INCLUDES) $< -c -o $@

# linux .c deps generation
LINUX_CC_DEPS:=$(patsubst %.c,$(GENDIR)/linux/%.d,$(LINUX_CC_SRCS))
$(LINUX_CC_DEPS): $$(patsubst $(GENDIR)/linux/%.d,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	$(LINUX_CC) $(LINUX_INCLUDES) -MM -MT $(<:%.c=$(GENDIR)/linux/%.o) -MF $@ $<
include $(LINUX_CC_DEPS)

# linux bin compilation:
$(LINUX_BINS): $$($$@_OBJS)
	$(LINUX_CC) $(LINUX_LDFLAGS) $(LINUX_LIBS) $^ -o $@

# Compilation of PIC32 C objects:
PIC32_CC_OBJS:=$(patsubst %.c,$(GENDIR)/pic32/%.o,$(PIC32_CC_SRCS))
$(PIC32_CC_OBJS): $$(patsubst $(GENDIR)/pic32/%.o,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	$(PIC32_CC) $(PIC32_CFLAGS) $(PIC32_INCLUDES) $< -c -o $@

# PIC32 deps generation
PIC32_CC_DEPS:=$(patsubst %.c,$(GENDIR)/pic32/%.d,$(PIC32_CC_SRCS))
$(PIC32_CC_DEPS):$$(patsubst $(GENDIR)/pic32/%.d,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	$(PIC32_CC) $(PIC32_INCLUDES) -MM -MT $(<:%.c=$(GENDIR)/pic32/%.o) -MF $@ $<
include $(PIC32_CC_DEPS)

# Compilation of a pic32 binary:
PIC32_ELFS:= $(patsubst %,$(GENDIR)/%.elf,$(PIC32_BINS))
$(PIC32_ELFS): $$($$(patsubst $(GENDIR)/%.elf,%_OBJS,$$@))
	$(PIC32_LD) $(PIC32_LDFLAGS) $(PIC32_LIBS) $^ -o $@

# Conversion of binary elf to a programmable hex:
PIC32_HEXES:= $(patsubst %,$(GENDIR)/%.hex,$(PIC32_BINS))
$(PIC32_HEXES): $$(patsubst %.hex,%.elf,$$@)
	$(PIC32_BIN2HEX) $< 

# Make deps
deps: $(LINUX_CC_DEPS) $(PIC32_CC_DEPS)

# All linux targets:
linux: $(LINUX_BINS)

# All pic32 targets:
pic32: $(PIC32_HEXES)

all: linux pic32

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

