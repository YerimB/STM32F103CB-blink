DEVICE          = STM32F103x8
FLASH           = 0x08000000

USE_ST_CMSIS    = true

# Use globally installed Arm GNU Toolchain
TOOLCHAIN_PATH  =


# Include the main makefile
include ./STM32-base/make/common.mk

re: clean all

.PHONY: all clean flash re
