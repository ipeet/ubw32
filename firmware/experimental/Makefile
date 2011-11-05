# Copyright 2010 Iain Peet
#
# New sources can be linked into the executable simply by adding them to the
# SOURCES variable a little further on. 
#
# Notable targets:
# all - build everything (the default target)
# install - build, then write the program to the UBW32
# tags - build ctags for all sources and includes
# clean - delete all generated files
# depsclean - delete deps files
# reset - resets the ubw if it's in bootloader mode

# Directories
C32_DIR= /usr/local/lib/c32
OBJ_DIR= objs
DEP_DIR= deps

# Required programs:
CC= wine ${C32_DIR}/bin/pic32-gcc.exe
LD= $(CC)
BIN2HEX= wine ${C32_DIR}/bin/pic32-bin2hex.exe

# Configuration options:
CFLAGS= -Wall -Wextra -mprocessor=32MX460F512L
INCLUDES= -I. -I./usb -I${C32_DIR}/include
LDFLAGS= -Wall -mprocessor=32MX460F512L -Wl,--defsym,_min_heap_size=1024
# Mem usage report: -Wl,--report-mem
LIBS= 

# Environment:
export WINEDEBUG:= 

.SECONDEXPANSION:

# Source files to compile
SOURCES= \
    adc.c \
    control.c \
    dbg_buf.c \
    error.c \
    est_apogee.c \
    file_io.c \
    kalman.c \
    igniters.c \
    main.c \
    test_mock.c \
    test_data.c \
    timing.c \
    spi_sw.c \
    uart_file.c \
    usb_descriptors.c \
    usb_device.c \
    usb_file.c \
    usb_function_cdc.c

# Sources to compile into linux test program
LINUX_SOURCES = \
	est_apogee.c \
	kalman.c \
	test_main.c 

OBJECTS= $(patsubst %.c,${OBJ_DIR}/%.o,$(SOURCES))

LINUX_OBJECTS= $(patsubst %.c,obj.linux/%.o,$(LINUX_SOURCES))

# Binaries to build 
BINS= ubw32-firm.hex test-linux

# The binary with which to program the PIC32 (should be in BINS)
INSTALL_BIN= ubw32-firm.hex

.phony: all
all: ${BINS} tags

# Include dependency files produced in the last compile run
-include $(wildcard ${DEP_DIR}/*.d)

# Rule for compilation of objects (also generates a deps file)
${OBJECTS}: $$(patsubst ${OBJ_DIR}/%.o,%.c,$$@)
	${CC} ${CFLAGS} ${INCLUDES} -c $(patsubst ${OBJ_DIR}/%.o,%.c,$@) -o $@ -MMD -MF${DEP_DIR}/$(patsubst ${OBJ_DIR}/%.o,%.d,$@)

# Rule for compilation of linux objects
${LINUX_OBJECTS}: $$(patsubst obj.linux/%.o,%.c,$$@)
	gcc -Wall -Wextra -g -O0 -c $< -o $@

# Rule for linking a binary
ubw32-firm.elf: ${OBJECTS}
	${LD} ${LDFLAGS} ${OBJECTS} ${LIBS} -o $@ -Wl,-Map=$(@:.elf=.map)

# Rule for linking a linux binary
test-linux: ${LINUX_OBJECTS}
	gcc -lm $^ -o $@

# Rule for converting a linked binary to a programmable hex
ubw32-firm.hex: $$(patsubst %.hex,%.elf,$$@)
	${BIN2HEX} $(@:.hex=.elf)
	
# Generate tags
tags: $(SOURCES) $(patsubst -I%,%,$(INCLUDES))
	ctags -R $(SOURCES) $(patsubst -I%,%,$(INCLUDES))

# Programs the PIC32 after compiling
.phony: install
install: all $(INSTALL_BIN)
	-echo -e "\nreset" > $(wildcard /dev/ttyACM*)
	@sleep 1
	sudo ubw32 -w $(INSTALL_BIN) -r

.phony: reset
reset: ; sudo ubw32 -r

# A good guess for bringing up the device's terminal
.phony: tty
tty: ; gtkterm -p $(wildcard /dev/ttyACM*)

# Delete all generated files
.phony: clean
clean: depsclean
	rm -f objs/*.o 
	rm -f obj.linux/*.o
	rm -f *~
	rm -f *.elf *.map *.hex 
	rm -f tags
	rm -f test-linux

# Delete only dependency files 
.phony: depsclean
depsclean:
	rm -f deps/*.d

# Print variables, for debugging
.phony: vars
vars:
	@echo "SOURCES: ${SOURCES}"
	@echo "OBJECTS: ${OBJECTS}"
	@echo "BINS: ${BINS}"

