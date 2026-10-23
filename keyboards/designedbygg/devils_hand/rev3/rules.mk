SRC += matrix.c
SRC += polling_rate.c
SRC += polling_rate_selftest.c

POINTING_DEVICE_DRIVER = paw3323

CUSTOM_MATRIX = lite

MCU_LDSCRIPT = AT32F405xC

OPT_DEFS += -DPOLLING_RATE_SELFTEST
