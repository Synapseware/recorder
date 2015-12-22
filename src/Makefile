
# MCU name
MCU = atmega32u4

# Fuse values
L_FUSE = 0xCD
H_FUSE = 0xD1
E_FUSE = 0xFB


#
#             LUFA Library
#     Copyright (C) Dean Camera, 2014.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.
FUSE_1			= 0xCD
FUSE_2			= 0xD1
FUSE_3			= 0xFB


MCU				= atmega32u4
ARCH			= AVR8
BOARD			= BOARD_ADAFRUITU4
F_CPU			= 16000000
F_USB			= 16000000
OPTIMIZATION	= s
TARGET			= recorder
SRC				= $(TARGET).c \
				Descriptors.c \
				$(LUFA_SRC_USB) \
				$(LUFA_SRC_USBCLASS) \
				$(LUFA_SRC_SERIAL) \
				$(LUFA_SRC_SPI)
LUFA_PATH		= ../LUFA
CC_FLAGS		= -DUSE_LUFA_CONFIG_HEADER -IConfig/
LD_FLAGS		= 

PROG			= stk500v2
PORT			= /dev/ttyACM1

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk


program: all
	avrdude -p $(MCU) -P $(PORT) -c $(PROG) -U flash:w:$(TARGET).hex

