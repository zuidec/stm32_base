
# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q		:= @
NULL		:= 2>/dev/null
endif

###############################################################################
# Directories

OPENCM3_DIR    	= libopencm3

ROOT_DIR		= ./
DRIVERS_DIR		= drivers
DEVICES_DIR		= devices
CORE_DIR		= core
COMMS_DIR		= comms
TIMERS_DIR		= timers
OBJ_DIR			= obj

TARGET_LIBNAME = stm32_base
OPENCM3_LIBNAME		= opencm3_stm32f4

###############################################################################
# Basic Device Setup

DEFS				+= -DSTM32F4
FP_FLAGS			?= -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARCH_FLAGS			= -mthumb -mcpu=cortex-m4 $(FP_FLAGS)
ARFLAGS 			= rcs

###############################################################################
# Linkerscript

LDLIBS		+= -l$(OPENCM3_LIBNAME)
LDFLAGS		+= -L$(OPENCM3_DIR)/lib

###############################################################################
# Includes

DEFS		+= -I$(OPENCM3_DIR)/include
DEFS		+= -I$(ROOT_DIR)

###############################################################################
# Executables

PREFIX		?= arm-none-eabi-

CC			:= $(PREFIX)gcc
CXX			:= $(PREFIX)g++
LD			:= $(PREFIX)gcc
AR			:= $(PREFIX)ar
AS			:= $(PREFIX)as
OBJCOPY		:= $(PREFIX)objcopy
OBJDUMP		:= $(PREFIX)objdump
GDB			:= $(PREFIX)gdb
STFLASH		= $(shell which st-flash)
OPT			:= -Os
DEBUG		:= -ggdb3
CSTD		?= -std=c99


###############################################################################
# Source files


OBJS		+= $(CORE_DIR)/system.o
OBJS		+= $(TIMERS_DIR)/timer.o
OBJS		+= $(TIMERS_DIR)/simple-timer.o
OBJS		+= $(COMMS_DIR)/crc.o
OBJS		+= $(COMMS_DIR)/comms.o
OBJS		+= $(DRIVERS_DIR)/fifo.o
OBJS		+= $(DRIVERS_DIR)/uart/uart.o
OBJS		+= $(DRIVERS_DIR)/spi/spi.o
OBJS		+= $(DEVICES_DIR)/mpu60X0/mpu60X0.o


###############################################################################
# C flags

TGT_CFLAGS	+= $(OPT) $(CSTD) $(DEBUG)
TGT_CFLAGS	+= $(ARCH_FLAGS)
TGT_CFLAGS	+= -Wextra -Wshadow -Wimplicit-function-declaration
TGT_CFLAGS	+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
TGT_CFLAGS	+= -fno-common -ffunction-sections -fdata-sections

###############################################################################
# C++ flags

TGT_CXXFLAGS	+= $(OPT) $(CXXSTD) $(DEBUG)
TGT_CXXFLAGS	+= $(ARCH_FLAGS)
TGT_CXXFLAGS	+= -Wextra -Wshadow -Wredundant-decls  -Weffc++
TGT_CXXFLAGS	+= -fno-common -ffunction-sections -fdata-sections

###############################################################################
# C & C++ preprocessor common flags

TGT_CPPFLAGS	+= -MD
TGT_CPPFLAGS	+= -Wall -Wundef
TGT_CPPFLAGS	+= $(DEFS)

###############################################################################
# Used libraries

LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

###############################################################################
###############################################################################
###############################################################################

.SUFFIXES: .elf .bin .hex .srec .list .map .images
.SECONDEXPANSION:
.SECONDARY:


all: lib
lib: $(TARGET_LIBNAME).a $(OPENCM3_DIR)/lib/lib$(OPENCM3_LIBNAME).a

$(OPENCM3_DIR)/lib/lib$(OPENCM3_LIBNAME).a:
ifeq (,$(wildcard $@))
	$(warning $(OPENCM3_LIBNAME).a not found, attempting to rebuild in $(OPENCM3_DIR))
	$(MAKE) -C $(OPENCM3_DIR)
endif

# Define a helper macro for debugging make errors online
# you can type "make print-OPENCM3_DIR" and it will show you
# how that ended up being resolved by all of the included
# makefiles.
print-%:
	@echo $*=$($*)

$(TARGET_LIBNAME).a: $(OBJS)
	@#printf "  AR      $(*).o\n"
	$(Q)$(AR) $(ARFLAGS) -o $(TARGET_LIBNAME).a $(OBJS)
	
%.o: %.c
	@#printf "  CC      $(*).c\n"
	$(Q)$(CC) $(TGT_CFLAGS) $(CFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $(*).o -c $(*).c

%.o: %.cxx
	@#printf "  CXX     $(*).cxx\n"
	$(Q)$(CXX) $(TGT_CXXFLAGS) $(CXXFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $(OBJ_DIR)/$(*).o -c $(*).cxx

%.o: %.cpp
	@#printf "  CXX     $(*).cpp\n"
	$(Q)$(CXX) $(TGT_CXXFLAGS) $(CXXFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $(*).o -c $(*).cpp

clean:
	@#printf "  CLEAN\n"
	$(Q)$(RM) $(GENERATED_BINARIES) generated.* $(OBJS) $(OBJS:%.o=%.d)


.PHONY: images clean elf bin hex srec list

-include $(OBJS:.o=.d)