# Directories
C32_DIR= /usr/local/lib/c32
OBJ_DIR= objs
DEP_DIR= deps

# Compiler Config:
CC= wine ${C32_DIR}/bin/pic32-gcc.exe
CFLAGS= -mprocessor=32MX460F512L 
INCLUDES= -I. -I./usb -I${C32_DIR}/include
LD= $(CC)
LDFLAGS= -mprocessor=32MX460F512L
LIBS= 
BIN2HEX= wine ${C32_DIR}/bin/pic32-bin2hex.exe

# Environment:
export WINEDEBUG:= 

.SECONDEXPANSION:


# Source files to compile
SOURCES= \
    main.c \
    usb_api.c \
    usb_descriptors.c \
    usb_device.c \
    usb_function_cdc.c

OBJECTS= $(patsubst %.c,${OBJ_DIR}/%.o,$(SOURCES))

# Binaries to build 
BINS= ubw32-firm.hex
INSTALL_BIN= ubw32-firm.hex

.phony: all
all: ${BINS} tags

# Include any compiled dependencies
-include $(wildcard ${DEP_DIR}/*.d)

${OBJECTS}: $$(patsubst ${OBJ_DIR}/%.o,%.c,$$@)
	${CC} ${CFLAGS} ${INCLUDES} -c $(patsubst ${OBJ_DIR}/%.o,%.c,$@) -o $@ -MMD -MF${DEP_DIR}/$(patsubst ${OBJ_DIR}/%.o,%.d,$@)

ubw32-firm.elf: ${OBJECTS}
	${LD} ${LDFLAGS} ${OBJECTS} ${LIBS} -o $@ -Map=$(@:.elf=.map)

ubw32-firm.hex: $$(patsubst %.hex,%.elf,$$@)
	${BIN2HEX} $(@:.hex=.elf)
	
# Update tags
tags: $(SOURCES)
	ctags $(SOURCES)

.phony: install
install: $(INSTALL_BIN)
	sudo ubw32 -w $(INSTALL_BIN) -r

.phony: clean
clean: depsclean
	rm -f objs/*.o 
	rm -f *.elf *.map *.hex 
	rm -f tags
	
.phony: depsclean
depsclean:
	rm -f deps/*.d

.phony: vars
vars:
	@echo "SOURCES: ${SOURCES}"
	@echo "OBJECTS: ${OBJECTS}"
	@echo "BINS: ${BINS}"
