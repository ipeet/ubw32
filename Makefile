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
	linux

# Include all the subdirs
include $(patsubst %,%/Makefile,$(SRCDIRS))

# Generated directories
GENDIR:=gen

# Compile config
CC:= gcc
CCFLAGS:= -Wall -g -O0
INCLUDES:= -I. 
LDFLAGS:= $(CCFLAGS)
LIBS:= -lm 

# Linux binaries:
LINUX_BINS = kalman_test

# Objects to include in each binary:
kalman_test_OBJS:= \
	ekf/kalman.o \
	ekf/kalman_test.o \
	linux/gnuplot.o
kalman_test_OBJS:= $(patsubst %.o,$(GENDIR)/linux/%.o,$(kalman_test_OBJS))

# Compilation of linux c objects:
LINUX_CC_OBJS:=$(patsubst %.c,$(GENDIR)/linux/%.o,$(LINUX_CC_SRCS))
$(LINUX_CC_OBJS): $$(patsubst $(GENDIR)/linux/%.o,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	$(CC) $(CCFLAGS) $(INCLUDES) $< -c -o $@

# linux .c deps generation
LINUX_CC_DEPS:=$(patsubst %.c,$(GENDIR)/linux/%.d,$(LINUX_CC_SRCS))
$(LINUX_CC_DEPS): $$(patsubst $(GENDIR)/linux/%.d,%.c,$$@)
	@if test ! -e $(dir $@); then mkdir -p $(dir $@); fi
	$(CC) $(CCFLAGS) -MM -MT $(<:%.c=$(GENDIR)/linux/%.o) -MF $@ $<

# linux bin compilation:
$(LINUX_BINS): $$($$@_OBJS)
	$(CC) $(LDFLAGS) $(LIBS) $^ -o $@

deps: $(LINUX_CC_DEPS)
include $(LINUX_CC_DEPS)

all: $(LINUX_BINS)

clean:
	rm -rf $(GENDIR)
	rm -f $(LINUX_BINS)
	rm -f tags

debugp:
	@echo LINUX_CC_SRCS: $(LINUX_CC_SRCS)
	@echo LINUX_CC_OBJS: $(LINUX_CC_OBJS)
	@echo LINUX_CC_DEPS: $(LINUX_CC_DEPS)

