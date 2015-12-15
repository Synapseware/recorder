
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
BOARD			= ADAFRUITU4
F_CPU			= 16000000
F_USB			= $(F_CPU)
OPTIMIZATION	= s
TARGET			= recorder
SRC				= $(TARGET).c \
				$(LUFA_SRC_USB) \
				$(LUFA_SRC_USBCLASS) \
				$(LUFA_SRC_SERIAL) \
				$(LUFA_SRC_SPI)
LUFA_PATH		= ../LUFA
CC_FLAGS		= -DUSE_LUFA_CONFIG_HEADER -IConfig/
LD_FLAGS		= 

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
	avrdude -p $(MCU) -P /dev/ttyACM0 -c avrisp2 -U flash:w:$(TARGET).hex

