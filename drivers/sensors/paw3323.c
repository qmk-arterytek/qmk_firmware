// Copyright 2024 Colin Lam (Ploopy Corporation)
// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#include <stddef.h>
#include <string.h>
#include "paw3323.h"
#include "print.h"
#include "timer.h"
#include "wait.h"
#include "gpio.h"
#include "spi_master.h"
#include "pointing_device_internal.h"

#define MSB1 0x80
#define MSB0 0x7F

typedef struct {
    uint8_t reg;
    uint8_t val;
} paw3323_reg_pair_t;

#define PAW_CHAIN(status, call)           \
    do {                                      \
        if ((status) == SPI_STATUS_SUCCESS) { \
            (status) = (call);                \
        }                                     \
    } while (0)

#define PAW_CHAIN_BOOL(status, call)      \
    do {                                      \
        if ((status) == SPI_STATUS_SUCCESS) { \
            if (!(call)) {                    \
                (status) = SPI_STATUS_ERROR;  \
            }                                 \
        }                                     \
    } while (0)

static paw3323_debug_t paw3323_dbg;

#define PAW3323_LOG_FAIL(step)                                                   \
    do {                                                                         \
        paw3323_dbg.ok       = false;                                            \
        paw3323_dbg.t_end_ms = timer_read32();                                   \
        pd_dprintf("paw3323: init FAILED at %s (line %d)\n", (step), __LINE__); \
    } while (0)

#define PAW3323_STAGE(s) (paw3323_dbg.stage = (s))

#define PAW3323_DBG_NO_INDEX 0xFFFE /* failing access was not an entry of a table */

static void paw3323_dbg_fail(size_t index, uint8_t reg, uint8_t val, spi_status_t status) {
    paw3323_dbg.fail_index = (uint16_t)index;
    paw3323_dbg.fail_reg   = reg;
    paw3323_dbg.fail_val   = val;
    paw3323_dbg.spi_status = status;
}

/* Low/high CPI register-set boundary used by the operation mode tables (7.2) */
#define PAW3323_CPI_HIGH_THRESHOLD 6000

static uint16_t paw3323_last_cpi     = 0;     /* last CPI programmed into the chip, 0 = unknown */
static bool     paw3323_mode_high    = false; /* which CPI-band register set is loaded */
static bool     paw3323_mode_applied = false; /* mode sequence loaded since the last init */

static bool paw3323_apply_cpi(uint16_t cpi);
static bool paw3323_signature_valid(uint8_t product_id, uint8_t inv_product_id);

const pointing_device_driver_t paw3323_pointing_device_driver = {
    .init       = paw3323_init,
    .get_report = paw3323_get_report,
    .set_cpi    = paw3323_set_cpi,
    .get_cpi    = paw3323_get_cpi,
};

// Convert a 16-bit twos complement binary-represented number into a
// signed 16-bit integer.
static int16_t convert_twoscomp_16(uint8_t high, uint8_t low) {
    uint16_t data = (high << 8) | low;

    if ((data & 0x8000) == 0x8000) {
        return -32768 + (data & 0x7FFF);
    } else {
        return data;
    }
}

static spi_status_t paw3323_write(uint8_t reg_addr, uint8_t data) {
    spi_status_t status = SPI_STATUS_ERROR;

    if (!spi_start(PAW3323_CS_PIN, false, 3, PAW3323_SPI_DIVISOR)) {
        return status;
    }

    wait_us(1); // tNCS_SCLK
    status = spi_write(reg_addr | MSB1);
    wait_us(180); // tSRAD
    if (status >= SPI_STATUS_SUCCESS) {
        status = spi_write(data);
    }
    if (status >= SPI_STATUS_SUCCESS) {
        status = SPI_STATUS_SUCCESS;
    }
    wait_us(1); // tSCLK_NCS_W
    spi_stop();
    wait_us(20); // tSRW

    return status;
}

static spi_status_t paw3323_read(uint8_t reg_addr, uint8_t *value) {
    spi_status_t status = SPI_STATUS_ERROR;

    if (value == NULL) {
        return status;
    }

    if (!spi_start(PAW3323_CS_PIN, false, 3, PAW3323_SPI_DIVISOR)) {
        return status;
    }

    wait_us(1); // tNCS_SCLK
    status = spi_write(reg_addr & MSB0);
    wait_us(180); // tSRAD
    if (status >= SPI_STATUS_SUCCESS) {
        status = spi_read();
        if (status >= SPI_STATUS_SUCCESS) {
            *value = (uint8_t)status;
            status = SPI_STATUS_SUCCESS;
        }
    }
    wait_us(1); // tSCLK_NCS_R
    spi_stop();
    wait_us(20); // tSRR

    return status;
}

#ifdef PAW3323_READ_BURST
static spi_status_t paw3323_read_dump(uint8_t reg_addr, uint8_t *buf, uint8_t len) {
    spi_status_t status = SPI_STATUS_ERROR;

    if (buf == NULL) {
        return status;
    }

    if (!spi_start(PAW3323_CS_PIN, false, 3, PAW3323_SPI_DIVISOR)) {
        return status;
    }

    wait_us(1); // tNCS_SCLK
    status = spi_write(reg_addr & MSB0);
    wait_us(180); // tSRAD
    if (status >= SPI_STATUS_SUCCESS) {
        status = spi_receive(buf, len);
    }
    wait_us(1); // tSCLK_NCS_R
    spi_stop();
    wait_us(20); // tSRR

    return status;
}
#endif

/* Single write that records the register/value/status if it fails (init report) */
static spi_status_t paw3323_write_logged(uint8_t reg, uint8_t val) {
    spi_status_t status = paw3323_write(reg, val);

    if (status != SPI_STATUS_SUCCESS) {
        paw3323_dbg_fail(PAW3323_DBG_NO_INDEX, reg, val, status);
    }

    return status;
}

static bool paw3323_write_sequence(const paw3323_reg_pair_t *seq, size_t len) {
    spi_status_t status = SPI_STATUS_SUCCESS;

    for (size_t i = 0; i < len; i++) {
        PAW_CHAIN(status, paw3323_write(seq[i].reg, seq[i].val));
        if (status != SPI_STATUS_SUCCESS) {
            paw3323_dbg_fail(i, seq[i].reg, seq[i].val, status);
            break;
        }
    }

    return status == SPI_STATUS_SUCCESS;
}

static bool paw3323_read_discard_sequence(const uint8_t *regs, size_t len) {
    spi_status_t status = SPI_STATUS_SUCCESS;
    uint8_t      discard;

    for (size_t i = 0; i < len; i++) {
        PAW_CHAIN(status, paw3323_read(regs[i], &discard));
        if (status != SPI_STATUS_SUCCESS) {
            paw3323_dbg_fail(i, regs[i], 0, status);
            break;
        }
    }

    return status == SPI_STATUS_SUCCESS;
}

static bool paw3323_init_ready(void) {
    for (uint8_t i = 0; i < 55; i++) {
        uint8_t      value;
        spi_status_t status = paw3323_read(REG_Undocumented_20, &value);

        paw3323_dbg.ready_polls = (uint8_t)(i + 1);
        if (status != SPI_STATUS_SUCCESS) {
            paw3323_dbg_fail(i, REG_Undocumented_20, 0, status);
            return false;
        }

        paw3323_dbg.ready_last = value;
        if (value == 0x0F) {
            paw3323_dbg.ready_ok = true;
            return true;
        }

        wait_ms(1);
    }

    return false;
}

/* 7.1.1 step 4: Power_Up_Reset */
static const paw3323_reg_pair_t paw3323_init_seq_power[] = {
    {REG_Power_Up_Reset, 0x5A},
};

/* 7.1.2 steps 1-116 (labels are datasheet step numbers) */
static const paw3323_reg_pair_t paw3323_init_seq[] = {
    /*   1 */ {0x7F, 0x00},
    /*   2 */ {0x40, 0x80},
    /*   3 */ {0x7F, 0x07},
    /*   4 */ {0x40, 0xC3},
    /*   5 */ {0x7F, 0x00},
    /*   6 */ {0x50, 0xFF},
    /*   7 */ {0x51, 0xFF},
    /*   8 */ {0x52, 0x0B},
    /*   9 */ {0x4E, 0x42},
    /*  10 */ {0x4B, 0x44},
    /*  11 */ {0x4A, 0x00},
    /*  12 */ {0x4D, 0x10},
    /*  13 */ {0x4F, 0x8D},
    /*  14 */ {0x64, 0xB0},
    /*  15 */ {0x65, 0x04},
    /*  16 */ {0x77, 0x5E},
    /*  17 */ {0x7D, 0x09},
    /*  18 */ {0x7F, 0x05},
    /*  19 */ {0x42, 0x0E},
    /*  20 */ {0x43, 0xE0},
    /*  21 */ {0x44, 0xA8},
    /*  22 */ {0x4A, 0x14},
    /*  23 */ {0x53, 0x0C},
    /*  24 */ {0x51, 0x10},
    /*  25 */ {0x55, 0xC9},
    /*  26 */ {0x5B, 0xEA},
    /*  27 */ {0x60, 0x0F},
    /*  28 */ {0x61, 0x13},
    /*  29 */ {0x62, 0x0B},
    /*  30 */ {0x64, 0xD8},
    /*  31 */ {0x6C, 0xEA},
    /*  32 */ {0x6D, 0xEA},
    /*  33 */ {0x7D, 0x84},
    /*  34 */ {0x7F, 0x06},
    /*  35 */ {0x49, 0x40},
    /*  36 */ {0x4D, 0x3F},
    /*  37 */ {0x52, 0xFF},
    /*  38 */ {0x60, 0xB0},
    /*  39 */ {0x61, 0x00},
    /*  40 */ {0x7E, 0x40},
    /*  41 */ {0x74, 0x40},
    /*  42 */ {0x7F, 0x07},
    /*  43 */ {0x42, 0x53},
    /*  44 */ {0x43, 0x00},
    /*  45 */ {0x4A, 0x00},
    /*  46 */ {0x46, 0x56},
    /*  47 */ {0x7F, 0x08},
    /*  48 */ {0x6F, 0x60},
    /*  49 */ {0x7F, 0x09},
    /*  50 */ {0x40, 0x03},
    /*  51 */ {0x42, 0x10},
    /*  52 */ {0x4E, 0x10},
    /*  53 */ {0x4F, 0x0B},
    /*  54 */ {0x7F, 0x0A},
    /*  55 */ {0x48, 0x60},
    /*  56 */ {0x49, 0x00},
    /*  57 */ {0x4A, 0x23},
    /*  58 */ {0x4C, 0x28},
    /*  59 */ {0x4D, 0x08},
    /*  60 */ {0x4E, 0x20},
    /*  61 */ {0x4F, 0x02},
    /*  62 */ {0x44, 0x44},
    /*  63 */ {0x55, 0x1C},
    /*  64 */ {0x7F, 0x0C},
    /*  65 */ {0x40, 0xA0},
    /*  66 */ {0x41, 0x78},
    /*  67 */ {0x42, 0x14},
    /*  68 */ {0x43, 0x38},
    /*  69 */ {0x44, 0x05},
    /*  70 */ {0x45, 0x03},
    /*  71 */ {0x4A, 0x0D},
    /*  72 */ {0x4B, 0x0C},
    /*  73 */ {0x4C, 0x60},
    /*  74 */ {0x4E, 0xCB},
    /*  75 */ {0x50, 0x01},
    /*  76 */ {0x51, 0x02},
    /*  77 */ {0x55, 0x00},
    /*  78 */ {0x56, 0x00},
    /*  79 */ {0x57, 0x40},
    /*  80 */ {0x58, 0x0A},
    /*  81 */ {0x59, 0x3C},
    /*  82 */ {0x7F, 0x0D},
    /*  83 */ {0x48, 0x8F},
    /*  84 */ {0x4F, 0x02},
    /*  85 */ {0x4E, 0x6B},
    /*  86 */ {0x5E, 0xC3},
    /*  87 */ {0x7F, 0x14},
    /*  88 */ {0x61, 0x88},
    /*  89 */ {0x75, 0x87},
    /*  90 */ {0x71, 0x87},
    /*  91 */ {0x54, 0x01},
    /*  92 */ {0x62, 0x28},
    /*  93 */ {0x63, 0x1C},
    /*  94 */ {0x64, 0x00},
    /*  95 */ {0x6F, 0x00},
    /*  96 */ {0x52, 0x0A},
    /*  97 */ {0x73, 0x03},
    /*  98 */ {0x74, 0x00},
    /*  99 */ {0x7A, 0x16},
    /* 100 */ {0x7F, 0x00},
    /* 101 */ {0x61, 0xAD},
    /* 102 */ {0x76, 0x3F},
    /* 103 */ {0x7F, 0x10},
    /* 104 */ {0x45, 0x03},
    /* 105 */ {0x46, 0x6E},
    /* 106 */ {0x66, 0x00},
    /* 107 */ {0x67, 0xEA},
    /* 108 */ {0x68, 0x60},
    /* 109 */ {0x6D, 0x05},
    /* 110 */ {0x6E, 0xA4},
    /* 111 */ {0x73, 0x87},
    /* 112 */ {0x74, 0x6A},
    /* 113 */ {0x76, 0x3E},
    /* 114 */ {0x77, 0x1A},
    /* 115 */ {0x7F, 0x00},
    /* 116 */ {0x19, 0x9F},
};

/* 7.1.2 steps 120-126, written after the 0x20 ready poll (steps 117-118) */
static const paw3323_reg_pair_t paw3323_post_init_seq[] = {
    /* 119 */ {0x19, 0x10},
    /* 120 */ {0x7F, 0x00},
    /* 121 */ {0x5B, 0x50},
    /* 122 */ {0x40, 0x00},
    /* 123 */ {0x61, 0xD5},
    /* 124 */ {0x7F, 0x07},
    /* 125 */ {0x40, 0xC2},
    /* 126 */ {0x7F, 0x00},
};

static const uint8_t paw3323_init_discard_regs[] = {
    REG_Motion, REG_Delta_X_L, REG_Delta_X_H, REG_Delta_Y_L, REG_Delta_Y_H,
};

/* Operation mode tables (7.2): labels are datasheet step numbers. Low CPI is < 6000, high CPI is >= 6000. */
#if (PAW3323_GAMING_MODE == MODE_1) // LPM

static const paw3323_reg_pair_t paw3323_mode_seq_low[] = {
    /*   1 */ {0x7F, 0x05},
    /*   2 */ {0x61, 0x13},
    /*   3 */ {0x62, 0x0B},
    /*   4 */ {0x53, 0x0C},
    /*   5 */ {0x55, 0xC9},
    /*   6 */ {0x7D, 0x84},
    /*   7 */ {0x7E, 0x03},
    /*   8 */ {0x7F, 0x06},
    /*   9 */ {0x60, 0xB0},
    /*  10 */ {0x61, 0x00},
    /*  11 */ {0x62, 0x10},
    /*  12 */ {0x63, 0x00},
    /*  13 */ {0x7A, 0x10},
    /*  14 */ {0x79, 0x00},
    /*  15 */ {0x4C, 0x8A},
    /*  16 */ {0x7F, 0x0A},
    /*  17 */ {0x4F, 0x02},
    /*  18 */ {0x4D, 0x08},
    /*  19 */ {0x48, 0x60},
    /*  20 */ {0x55, 0x1C},
    /*  21 */ {0x7F, 0x07},
    /*  22 */ {0x46, 0x56},
    /*  23 */ {0x42, 0x53},
    /*  24 */ {0x7F, 0x09},
    /*  25 */ {0x40, 0x03},
    /*  26 */ {0x41, 0x01},
    /*  27 */ {0x47, 0x00},
    /*  28 */ {0x4E, 0x10},
    /*  29 */ {0x4F, 0x08},
    /*  30 */ {0x50, 0x0A},
    /*  31 */ {0x51, 0x10},
    /*  32 */ {0x52, 0x0A},
    /*  33 */ {0x7F, 0x00},
    /*  34 */ {0x66, 0x00},
    /*  35 */ {0x78, 0x01},
    /*  36 */ {0x79, 0x4F},
    /*  37 */ {0x77, 0x5E},
    /*  38 */ {0x7A, 0x08},
    /*  39 */ {0x7B, 0x4A},
};

static const paw3323_reg_pair_t paw3323_mode_seq_high[] = {
    /*   1 */ {0x7F, 0x05},
    /*   2 */ {0x61, 0x13},
    /*   3 */ {0x62, 0x0B},
    /*   4 */ {0x53, 0x0C},
    /*   5 */ {0x55, 0xC9},
    /*   6 */ {0x7D, 0x8A},
    /*   7 */ {0x7E, 0x04},
    /*   8 */ {0x7F, 0x06},
    /*   9 */ {0x60, 0xB0},
    /*  10 */ {0x61, 0x00},
    /*  11 */ {0x62, 0x10},
    /*  12 */ {0x63, 0x00},
    /*  13 */ {0x7A, 0x10},
    /*  14 */ {0x79, 0x00},
    /*  15 */ {0x4C, 0x8A},
    /*  16 */ {0x7F, 0x0A},
    /*  17 */ {0x4F, 0x00},
    /*  18 */ {0x4D, 0x08},
    /*  19 */ {0x48, 0x60},
    /*  20 */ {0x55, 0x1C},
    /*  21 */ {0x7F, 0x07},
    /*  22 */ {0x46, 0x56},
    /*  23 */ {0x42, 0x53},
    /*  24 */ {0x7F, 0x09},
    /*  25 */ {0x40, 0x04},
    /*  26 */ {0x41, 0x02},
    /*  27 */ {0x47, 0x10},
    /*  28 */ {0x4E, 0x00},
    /*  29 */ {0x4F, 0x00},
    /*  30 */ {0x50, 0x0F},
    /*  31 */ {0x51, 0x08},
    /*  32 */ {0x52, 0x0C},
    /*  33 */ {0x7F, 0x00},
    /*  34 */ {0x66, 0x40},
    /*  35 */ {0x78, 0x01},
    /*  36 */ {0x79, 0x4F},
    /*  37 */ {0x77, 0x5E},
    /*  38 */ {0x7A, 0x08},
    /*  39 */ {0x7B, 0x4A},
};

#elif (PAW3323_GAMING_MODE == MODE_2) // CORDED

static const paw3323_reg_pair_t paw3323_mode_seq_low[] = {
    /*   1 */ {0x7F, 0x05},
    /*   2 */ {0x61, 0x13},
    /*   3 */ {0x62, 0x04},
    /*   4 */ {0x53, 0x0C},
    /*   5 */ {0x55, 0xC9},
    /*   6 */ {0x7D, 0x84},
    /*   7 */ {0x7E, 0x00},
    /*   8 */ {0x7F, 0x06},
    /*   9 */ {0x60, 0x30},
    /*  10 */ {0x61, 0x00},
    /*  11 */ {0x62, 0x20},
    /*  12 */ {0x63, 0x00},
    /*  13 */ {0x7A, 0x20},
    /*  14 */ {0x79, 0x00},
    /*  15 */ {0x4C, 0x8A},
    /*  16 */ {0x7F, 0x0A},
    /*  17 */ {0x4F, 0x02},
    /*  18 */ {0x4D, 0x08},
    /*  19 */ {0x48, 0x60},
    /*  20 */ {0x55, 0x1C},
    /*  21 */ {0x7F, 0x07},
    /*  22 */ {0x46, 0x26},
    /*  23 */ {0x42, 0x29},
    /*  24 */ {0x7F, 0x09},
    /*  25 */ {0x40, 0x03},
    /*  26 */ {0x41, 0x01},
    /*  27 */ {0x47, 0x00},
    /*  28 */ {0x4E, 0x10},
    /*  29 */ {0x4F, 0x08},
    /*  30 */ {0x50, 0x0A},
    /*  31 */ {0x51, 0x10},
    /*  32 */ {0x52, 0x0A},
    /*  33 */ {0x7F, 0x00},
    /*  34 */ {0x66, 0x00},
    /*  35 */ {0x78, 0x01},
    /*  36 */ {0x79, 0x4F},
    /*  37 */ {0x77, 0x5E},
    /*  38 */ {0x7A, 0x08},
    /*  39 */ {0x7B, 0x4A},
};

static const paw3323_reg_pair_t paw3323_mode_seq_high[] = {
    /*   1 */ {0x7F, 0x05},
    /*   2 */ {0x61, 0x13},
    /*   3 */ {0x62, 0x04},
    /*   4 */ {0x53, 0x0C},
    /*   5 */ {0x55, 0xC9},
    /*   6 */ {0x7D, 0x8A},
    /*   7 */ {0x7E, 0x04},
    /*   8 */ {0x7F, 0x06},
    /*   9 */ {0x60, 0x30},
    /*  10 */ {0x61, 0x00},
    /*  11 */ {0x62, 0x20},
    /*  12 */ {0x63, 0x00},
    /*  13 */ {0x7A, 0x20},
    /*  14 */ {0x79, 0x00},
    /*  15 */ {0x4C, 0x8A},
    /*  16 */ {0x7F, 0x0A},
    /*  17 */ {0x4F, 0x00},
    /*  18 */ {0x4D, 0x08},
    /*  19 */ {0x48, 0x60},
    /*  20 */ {0x55, 0x1C},
    /*  21 */ {0x7F, 0x07},
    /*  22 */ {0x46, 0x26},
    /*  23 */ {0x42, 0x29},
    /*  24 */ {0x7F, 0x09},
    /*  25 */ {0x40, 0x04},
    /*  26 */ {0x41, 0x02},
    /*  27 */ {0x47, 0x10},
    /*  28 */ {0x4E, 0x00},
    /*  29 */ {0x4F, 0x00},
    /*  30 */ {0x50, 0x0F},
    /*  31 */ {0x51, 0x08},
    /*  32 */ {0x52, 0x0C},
    /*  33 */ {0x7F, 0x00},
    /*  34 */ {0x66, 0x40},
    /*  35 */ {0x78, 0x01},
    /*  36 */ {0x79, 0x4F},
    /*  37 */ {0x77, 0x5E},
    /*  38 */ {0x7A, 0x08},
    /*  39 */ {0x7B, 0x4A},
};

#elif (PAW3323_GAMING_MODE == MODE_3) // OFFICE

static const paw3323_reg_pair_t paw3323_mode_seq_low[] = {
    /*   1 */ {0x7F, 0x05},
    /*   2 */ {0x61, 0x11},
    /*   3 */ {0x62, 0x60},
    /*   4 */ {0x53, 0x10},
    /*   5 */ {0x55, 0x84},
    /*   6 */ {0x7D, 0x84},
    /*   7 */ {0x7E, 0x23},
    /*   8 */ {0x7F, 0x06},
    /*   9 */ {0x60, 0x00},
    /*  10 */ {0x61, 0x06},
    /*  11 */ {0x62, 0x80},
    /*  12 */ {0x63, 0x00},
    /*  13 */ {0x7A, 0x10},
    /*  14 */ {0x79, 0x00},
    /*  15 */ {0x4C, 0x8A},
    /*  16 */ {0x7F, 0x0A},
    /*  17 */ {0x4F, 0x02},
    /*  18 */ {0x4D, 0x00},
    /*  19 */ {0x48, 0x60},
    /*  20 */ {0x55, 0x1C},
    /*  21 */ {0x7F, 0x07},
    /*  22 */ {0x46, 0x53},
    /*  23 */ {0x42, 0x56},
    /*  24 */ {0x7F, 0x09},
    /*  25 */ {0x40, 0x03},
    /*  26 */ {0x41, 0x01},
    /*  27 */ {0x47, 0x00},
    /*  28 */ {0x4E, 0x10},
    /*  29 */ {0x4F, 0x08},
    /*  30 */ {0x50, 0x0A},
    /*  31 */ {0x51, 0x10},
    /*  32 */ {0x52, 0x0A},
    /*  33 */ {0x7F, 0x00},
    /*  34 */ {0x66, 0x00},
    /*  35 */ {0x78, 0x0A},
    /*  36 */ {0x79, 0x02},
    /*  37 */ {0x77, 0x2F},
    /*  38 */ {0x7A, 0x19},
    /*  39 */ {0x7B, 0x0C},
};

/* Office mode uses a single register set for both CPI bands (7.2) */
#define paw3323_mode_seq_high paw3323_mode_seq_low

#else
#    error "Invalid PAW3323_GAMING_MODE. Use MODE_1, MODE_2, or MODE_3."
#endif

static bool paw3323_apply_mode_sequence(bool high) {
    const paw3323_reg_pair_t *seq = high ? paw3323_mode_seq_high : paw3323_mode_seq_low;
    size_t                    len = high ? ARRAY_SIZE(paw3323_mode_seq_high) : ARRAY_SIZE(paw3323_mode_seq_low);
    return paw3323_write_sequence(seq, len);
}

bool paw3323_init(void) {
    spi_status_t status = SPI_STATUS_SUCCESS;

    uint16_t runs = paw3323_dbg.init_count + 1;
    memset(&paw3323_dbg, 0, sizeof(paw3323_dbg));
    paw3323_dbg.init_count = runs;
    paw3323_dbg.fail_index = 0xFFFF;
    paw3323_dbg.t_start_ms = timer_read32();

    /* A reset reverts the chip to LPM / low-CPI settings */
    paw3323_mode_applied = false;
    paw3323_mode_high    = false;
    paw3323_last_cpi     = 0;

    wait_ms(50);
    gpio_set_pin_output(PAW3323_CS_PIN);
    gpio_write_pin_high(PAW3323_CS_PIN);
    wait_us(1);
    gpio_write_pin_low(PAW3323_CS_PIN);

    spi_init();

    /* Power-up reboot */
    PAW3323_STAGE(PAW3323_STAGE_RESET);
    PAW_CHAIN_BOOL(status, paw3323_write_sequence(paw3323_init_seq_power, ARRAY_SIZE(paw3323_init_seq_power)));
    if (status != SPI_STATUS_SUCCESS) {
        PAW3323_LOG_FAIL("reboot");
        return false;
    }
    wait_ms(5);

    /* Signature check (raw values are kept for the init report) */
    PAW3323_STAGE(PAW3323_STAGE_SIGNATURE);
    paw3323_dbg.sig_spi_ok = paw3323_read_signature(&paw3323_dbg.pid, &paw3323_dbg.inv_pid);
    if (!paw3323_dbg.sig_spi_ok) {
        status = SPI_STATUS_ERROR;
    } else if (!paw3323_signature_valid(paw3323_dbg.pid, paw3323_dbg.inv_pid)) {
        status = SPI_STATUS_ERROR;
    }
    if (status != SPI_STATUS_SUCCESS) {
        PAW3323_LOG_FAIL("signature");
        return false;
    }

    /* Common init sequence */
    PAW3323_STAGE(PAW3323_STAGE_INIT_SEQ);
    PAW_CHAIN_BOOL(status, paw3323_write_sequence(paw3323_init_seq, ARRAY_SIZE(paw3323_init_seq)));
    if (status != SPI_STATUS_SUCCESS) {
        PAW3323_LOG_FAIL("init_seq");
        return false;
    }
    wait_ms(1);

    PAW3323_STAGE(PAW3323_STAGE_READY_POLL);
    if (!paw3323_init_ready()) {
        pd_dprintf("paw3323: ready timeout, writing 0x19=0x00\n");
        PAW_CHAIN(status, paw3323_write_logged(REG_Undocumented_19, 0x00));
    } else {
        PAW_CHAIN(status, paw3323_write_logged(REG_Undocumented_19, 0x10));
    }

    PAW3323_STAGE(PAW3323_STAGE_POST_INIT);
    PAW_CHAIN_BOOL(status, paw3323_write_sequence(paw3323_post_init_seq, ARRAY_SIZE(paw3323_post_init_seq)));
    if (status != SPI_STATUS_SUCCESS) {
        PAW3323_LOG_FAIL("post-init");
        return false;
    }

    /* Load the operation mode tables and program the configured CPI. This is
     * done after the 7.1.2 sequence (steps 1-126) so that sequence stays intact. */
    PAW3323_STAGE(PAW3323_STAGE_MODE_CPI);
    if (!paw3323_apply_cpi(PAW3323_CPI)) {
        PAW3323_LOG_FAIL("mode/cpi");
        return false;
    }

    PAW3323_STAGE(PAW3323_STAGE_DISCARD);
    PAW_CHAIN_BOOL(status, paw3323_read_discard_sequence(paw3323_init_discard_regs, ARRAY_SIZE(paw3323_init_discard_regs)));
    if (status != SPI_STATUS_SUCCESS) {
        PAW3323_LOG_FAIL("discard");
        return false;
    }

    PAW3323_STAGE(PAW3323_STAGE_DONE);
    paw3323_dbg.ok       = true;
    paw3323_dbg.t_end_ms = timer_read32();
    pd_dprintf("paw3323: init OK\n");
    return true;
}

static void paw3323_stat_motion(uint8_t motion) {
    paw3323_dbg.polls++;
    paw3323_dbg.last_motion = motion;
    paw3323_dbg.opmode[motion & 0x03]++;
    if ((motion & MSB1) == MSB1) {
        paw3323_dbg.mot_hits++;
    }
}

report_paw3323_t paw3323_read_burst(void) {
    report_paw3323_t report = {0};

#ifdef PAW3323_READ_BURST
    uint8_t data[12] = {0};
    if (paw3323_read_dump(REG_Burst_Motion_Read, data, (uint8_t)sizeof(data)) == SPI_STATUS_SUCCESS) {
        paw3323_stat_motion(data[0]);
        if ((data[0] & MSB1) == MSB1) {
            // Motion detected
            report.dx = convert_twoscomp_16(data[3], data[2]);
            report.dy = convert_twoscomp_16(data[5], data[4]);
            paw3323_dbg.sum_dx += report.dx;
            paw3323_dbg.sum_dy += report.dy;
        }
    } else {
        paw3323_dbg.spi_errors++;
    }
#else
    spi_status_t status = SPI_STATUS_SUCCESS;
    uint8_t      motion;
    uint8_t      dx_l;
    uint8_t      dx_h;
    uint8_t      dy_l;
    uint8_t      dy_h;

    status = paw3323_read(REG_Motion, &motion);
    if (status == SPI_STATUS_SUCCESS) {
        paw3323_stat_motion(motion);
    } else {
        paw3323_dbg.spi_errors++;
    }
    if (status == SPI_STATUS_SUCCESS && (motion & MSB1) == MSB1) {
        PAW_CHAIN(status, paw3323_read(REG_Delta_X_L, &dx_l));
        PAW_CHAIN(status, paw3323_read(REG_Delta_X_H, &dx_h));
        PAW_CHAIN(status, paw3323_read(REG_Delta_Y_L, &dy_l));
        PAW_CHAIN(status, paw3323_read(REG_Delta_Y_H, &dy_h));

        if (status == SPI_STATUS_SUCCESS) {
            report.dx = convert_twoscomp_16(dx_h, dx_l);
            report.dy = convert_twoscomp_16(dy_h, dy_l);
            paw3323_dbg.sum_dx += report.dx;
            paw3323_dbg.sum_dy += report.dy;
        }
    }
#endif

    return report;
}

#if (PAW3323_INIT_REG_SET == LENS_1)
/* Resolution Setting 1: L0AL-LSG1  (datasheet v0.8, section 7.5.1)
 * Row n holds the registers for a target of n * 50 CPI. Values are copied verbatim.
 * The datasheet repeats a value on adjacent rows: X at 2900/2950 and 3400/3450,
 * Y at 3400/3450. */

static const uint8_t paw3323_res_x_lut[200] = {
    /*    50 -   500 */ 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    /*   550 -  1000 */ 0x0B, 0x0C, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
    /*  1050 -  1500 */ 0x16, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x20, 0x21,
    /*  1550 -  2000 */ 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x29, 0x2A, 0x2B, 0x2C,
    /*  2050 -  2500 */ 0x2D, 0x2E, 0x2F, 0x30, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    /*  2550 -  3000 */ 0x38, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x40, 0x41,
    /*  3050 -  3500 */ 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4B, 0x4B, 0x4C,
    /*  3550 -  4000 */ 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
    /*  4050 -  4500 */ 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62,
    /*  4550 -  5000 */ 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D,
    /*  5050 -  5500 */ 0x6F, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
    /*  5550 -  6000 */ 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F, 0x80, 0x81, 0x82, 0x83,
    /*  6050 -  6500 */ 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E,
    /*  6550 -  7000 */ 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
    /*  7050 -  7500 */ 0x9B, 0x9C, 0x9D, 0x9E, 0x9F, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4,
    /*  7550 -  8000 */ 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
    /*  8050 -  8500 */ 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA,
    /*  8550 -  9000 */ 0xBC, 0xBD, 0xBE, 0xBF, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5,
    /*  9050 -  9500 */ 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD0,
    /*  9550 - 10000 */ 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB,
};

static const uint8_t paw3323_res_y_lut[200] = {
    /*    50 -   500 */ 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    /*   550 -  1000 */ 0x0B, 0x0C, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
    /*  1050 -  1500 */ 0x16, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1F, 0x21, 0x22,
    /*  1550 -  2000 */ 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x2A, 0x2B, 0x2C, 0x2D,
    /*  2050 -  2500 */ 0x2E, 0x2F, 0x30, 0x31, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
    /*  2550 -  3000 */ 0x39, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43,
    /*  3050 -  3500 */ 0x44, 0x45, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4D, 0x4D, 0x4E,
    /*  3550 -  4000 */ 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x59, 0x5A,
    /*  4050 -  4500 */ 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x61, 0x62, 0x63, 0x64, 0x65,
    /*  4550 -  5000 */ 0x66, 0x67, 0x68, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70,
    /*  5050 -  5500 */ 0x71, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7C,
    /*  5550 -  6000 */ 0x7D, 0x7E, 0x7F, 0x80, 0x81, 0x82, 0x83, 0x85, 0x86, 0x87,
    /*  6050 -  6500 */ 0x88, 0x89, 0x8A, 0x8B, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92,
    /*  6550 -  7000 */ 0x93, 0x94, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D,
    /*  7050 -  7500 */ 0x9F, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA8, 0xA9,
    /*  7550 -  8000 */ 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB1, 0xB2, 0xB3, 0xB4,
    /*  8050 -  8500 */ 0xB5, 0xB6, 0xB7, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
    /*  8550 -  9000 */ 0xC0, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCB,
    /*  9050 -  9500 */ 0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD4, 0xD5, 0xD6,
    /*  9550 - 10000 */ 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDD, 0xDE, 0xDF, 0xE0, 0xE1,
};

#elif (PAW3323_INIT_REG_SET == LENS_2)
/* Resolution Setting 2: L0AJ-LSG1  (datasheet v0.8, section 7.5.2)
 * Row n holds the registers for a target of n * 50 CPI. Values are copied verbatim. */

static const uint8_t paw3323_res_x_lut[200] = {
    /*    50 -   500 */ 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    /*   550 -  1000 */ 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x15, 0x16,
    /*  1050 -  1500 */ 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1E, 0x1F, 0x20, 0x21,
    /*  1550 -  2000 */ 0x22, 0x23, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    /*  2050 -  2500 */ 0x2D, 0x2E, 0x2F, 0x30, 0x32, 0x33, 0x35, 0x36, 0x37, 0x38,
    /*  2550 -  3000 */ 0x39, 0x3A, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43,
    /*  3050 -  3500 */ 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4C, 0x4D, 0x4E,
    /*  3550 -  4000 */ 0x4F, 0x50, 0x51, 0x52, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    /*  4050 -  4500 */ 0x5A, 0x5B, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x64,
    /*  4550 -  5000 */ 0x65, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x70,
    /*  5050 -  5500 */ 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x79, 0x7A, 0x7B,
    /*  5550 -  6000 */ 0x7C, 0x7D, 0x7E, 0x7F, 0x80, 0x81, 0x83, 0x84, 0x85, 0x86,
    /*  6050 -  6500 */ 0x87, 0x88, 0x89, 0x8A, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91,
    /*  6550 -  7000 */ 0x92, 0x93, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C,
    /*  7050 -  7500 */ 0x9E, 0x9F, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA8,
    /*  7550 -  8000 */ 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB1, 0xB2, 0xB3,
    /*  8050 -  8500 */ 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE,
    /*  8550 -  9000 */ 0xBF, 0xC0, 0xC1, 0xC2, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9,
    /*  9050 -  9500 */ 0xCA, 0xCB, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4,
    /*  9550 - 10000 */ 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDF, 0xE0,
};

static const uint8_t paw3323_res_y_lut[200] = {
    /*    50 -   500 */ 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    /*   550 -  1000 */ 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x15, 0x16,
    /*  1050 -  1500 */ 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1E, 0x20, 0x21, 0x22,
    /*  1550 -  2000 */ 0x23, 0x24, 0x25, 0x26, 0x27, 0x29, 0x2A, 0x2B, 0x2C, 0x2D,
    /*  2050 -  2500 */ 0x2E, 0x2F, 0x30, 0x31, 0x33, 0x34, 0x36, 0x37, 0x38, 0x39,
    /*  2550 -  3000 */ 0x3A, 0x3B, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44,
    /*  3050 -  3500 */ 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4E, 0x4F, 0x50,
    /*  3550 -  4000 */ 0x51, 0x52, 0x53, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B,
    /*  4050 -  4500 */ 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x64, 0x65, 0x66, 0x67,
    /*  4550 -  5000 */ 0x68, 0x69, 0x6A, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72,
    /*  5050 -  5500 */ 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7B, 0x7C, 0x7D, 0x7E,
    /*  5550 -  6000 */ 0x7F, 0x80, 0x81, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    /*  6050 -  6500 */ 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x92, 0x93, 0x94, 0x95,
    /*  6550 -  7000 */ 0x96, 0x97, 0x98, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F, 0xA0,
    /*  7050 -  7500 */ 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA9, 0xAA, 0xAB, 0xAC,
    /*  7550 -  8000 */ 0xAD, 0xAE, 0xAF, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7,
    /*  8050 -  8500 */ 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xC0, 0xC1, 0xC2, 0xC3,
    /*  8550 -  9000 */ 0xC4, 0xC5, 0xC6, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE,
    /*  9050 -  9500 */ 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD7, 0xD8, 0xD9, 0xDA,
    /*  9550 - 10000 */ 0xDB, 0xDC, 0xDD, 0xDF, 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5,
};

#elif (PAW3323_INIT_REG_SET == LENS_3)
/* Resolution Setting 3: LM31-LNG, LM33-LSG  (datasheet v0.8, section 7.5.3)
 * Row n holds the registers for a target of n * 50 CPI. Values are copied verbatim.
 * The datasheet repeats X and Y at 6000/6050, and rows 6050-8000 (40 rows) sit one
 * count below floor(cpi / divisor) before resyncing at 8050. This may be intentional
 * calibration for the high-CPI band or a datasheet typo; confirm with PixArt. */

static const uint8_t paw3323_res_x_lut[200] = {
    /*    50 -   500 */ 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0B,
    /*   550 -  1000 */ 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x12, 0x13, 0x14, 0x15, 0x16,
    /*  1050 -  1500 */ 0x17, 0x18, 0x19, 0x1B, 0x1C, 0x1D, 0x1E, 0x20, 0x21, 0x22,
    /*  1550 -  2000 */ 0x23, 0x24, 0x25, 0x26, 0x27, 0x29, 0x2A, 0x2B, 0x2C, 0x2D,
    /*  2050 -  2500 */ 0x2E, 0x2F, 0x30, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
    /*  2550 -  3000 */ 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x42, 0x43, 0x44,
    /*  3050 -  3500 */ 0x45, 0x46, 0x47, 0x48, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
    /*  3550 -  4000 */ 0x50, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x5A, 0x5B,
    /*  4050 -  4500 */ 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x63, 0x64, 0x65, 0x66,
    /*  4550 -  5000 */ 0x67, 0x68, 0x69, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71,
    /*  5050 -  5500 */ 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7B, 0x7C, 0x7D,
    /*  5550 -  6000 */ 0x7E, 0x7F, 0x80, 0x81, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
    /*  6050 -  6500 */ 0x88, 0x89, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x93,
    /*  6550 -  7000 */ 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9B, 0x9C, 0x9D, 0x9E,
    /*  7050 -  7500 */ 0x9F, 0xA0, 0xA1, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9,
    /*  7550 -  8000 */ 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 0xB1, 0xB3, 0xB4, 0xB5,
    /*  8050 -  8500 */ 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBD, 0xBE, 0xBF, 0xC0, 0xC1,
    /*  8550 -  9000 */ 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCD,
    /*  9050 -  9500 */ 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD5, 0xD6, 0xD7, 0xD8,
    /*  9550 - 10000 */ 0xD9, 0xDA, 0xDB, 0xDD, 0xDE, 0xDF, 0xE0, 0xE1, 0xE2, 0xE3,
};

static const uint8_t paw3323_res_y_lut[200] = {
    /*    50 -   500 */ 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0B,
    /*   550 -  1000 */ 0x0C, 0x0D, 0x0E, 0x0F, 0x11, 0x12, 0x13, 0x14, 0x15, 0x17,
    /*  1050 -  1500 */ 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1F, 0x20, 0x21, 0x22,
    /*  1550 -  2000 */ 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2C, 0x2D, 0x2E,
    /*  2050 -  2500 */ 0x2F, 0x30, 0x31, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x3A,
    /*  2550 -  3000 */ 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x41, 0x42, 0x43, 0x44, 0x45,
    /*  3050 -  3500 */ 0x46, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x50, 0x51,
    /*  3550 -  4000 */ 0x52, 0x53, 0x54, 0x55, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C,
    /*  4050 -  4500 */ 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x65, 0x66, 0x67, 0x68,
    /*  4550 -  5000 */ 0x69, 0x6A, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72, 0x74,
    /*  5050 -  5500 */ 0x75, 0x76, 0x77, 0x78, 0x79, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
    /*  5550 -  6000 */ 0x80, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x89, 0x8A, 0x8B,
    /*  6050 -  6500 */ 0x8B, 0x8C, 0x8D, 0x8F, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95,
    /*  6550 -  7000 */ 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9E, 0x9F, 0xA0, 0xA1,
    /*  7050 -  7500 */ 0xA2, 0xA3, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAC, 0xAD,
    /*  7550 -  8000 */ 0xAE, 0xAF, 0xB0, 0xB1, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8,
    /*  8050 -  8500 */ 0xBA, 0xBC, 0xBD, 0xBE, 0xBF, 0xC0, 0xC1, 0xC3, 0xC4, 0xC5,
    /*  8550 -  9000 */ 0xC6, 0xC7, 0xC8, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD1,
    /*  9050 -  9500 */ 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC,
    /*  9550 - 10000 */ 0xDD, 0xDE, 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE7, 0xE8,
};

#else
#    error "Invalid PAW3323_INIT_REG_SET. Use LENS_1, LENS_2, or LENS_3."
#endif

/* Extended-range (10050..18000 CPI) divisors, scaled x1000.
 * Res_X = floor(cpi / div_x), Res_Y = floor(cpi / div_y) */
#if (PAW3323_INIT_REG_SET == LENS_1)
#    define PAW3323_EXT_DIV_X 45475u /* 45.475 */
#    define PAW3323_EXT_DIV_Y 44323u /* 44.323 */
#elif (PAW3323_INIT_REG_SET == LENS_2)
#    define PAW3323_EXT_DIV_X 44616u /* 44.616 */
#    define PAW3323_EXT_DIV_Y 43482u /* 43.482 */
#elif (PAW3323_INIT_REG_SET == LENS_3)
#    define PAW3323_EXT_DIV_X 43885u /* 43.885 */
#    define PAW3323_EXT_DIV_Y 43050u /* 43.050 */
#endif

/* Programs the X/Y resolution registers and, whenever the low/high CPI band
 * (boundary: PAW3323_CPI_HIGH_THRESHOLD) changes or the mode tables have not
 * been loaded yet, loads the matching operation mode sequence first. */
static bool paw3323_apply_cpi(uint16_t cpi) {
    spi_status_t status = SPI_STATUS_SUCCESS;
    uint8_t      res_hi, res_x_lo, res_y_lo;

    cpi = CONSTRAIN(cpi, PAW3323_CPI_MIN, PAW3323_CPI_EXT_MAX);
    cpi -= cpi % PAW3323_CPI_STEP; /* target resolution must be a multiple of 50 */

    if (cpi <= PAW3323_CPI_MAX) {
        uint8_t idx = (uint8_t)((cpi / PAW3323_CPI_STEP) - 1U);

        res_hi   = 0x00;
        res_x_lo = paw3323_res_x_lut[idx];
        res_y_lo = paw3323_res_y_lut[idx];
    } else {
        uint16_t res_x = (uint16_t)(((uint32_t)cpi * 1000u) / PAW3323_EXT_DIV_X);
        uint16_t res_y = (uint16_t)(((uint32_t)cpi * 1000u) / PAW3323_EXT_DIV_Y);

        res_x_lo = (uint8_t)(res_x & 0xFF);
        res_y_lo = (uint8_t)(res_y & 0xFF);
        res_hi   = (uint8_t)((((res_y >> 8) & 0x0F) << 4) | ((res_x >> 8) & 0x0F));
    }

    const bool high = (cpi >= PAW3323_CPI_HIGH_THRESHOLD);
    if (!paw3323_mode_applied || high != paw3323_mode_high) {
        paw3323_mode_applied = false; /* state unknown if the sequence fails part-way */
        if (!paw3323_apply_mode_sequence(high)) {
            pd_dprintf("paw3323: mode sequence (%s CPI) failed\n", high ? "high" : "low");
            return false;
        }
        paw3323_mode_high    = high;
        paw3323_mode_applied = true;
    }

    PAW_CHAIN(status, paw3323_write_logged(REG_Res_XY_High, res_hi));
    PAW_CHAIN(status, paw3323_write_logged(REG_Res_X_Low, res_x_lo));
    PAW_CHAIN(status, paw3323_write_logged(REG_Res_Y_Low, res_y_lo));

    if (status != SPI_STATUS_SUCCESS) {
        return false;
    }

    paw3323_last_cpi       = cpi;
    paw3323_dbg.exp_res_hi = res_hi;
    paw3323_dbg.exp_res_x  = res_x_lo;
    paw3323_dbg.exp_res_y  = res_y_lo;
    return true;
}

void paw3323_set_cpi(uint16_t cpi) {
    if (!paw3323_apply_cpi(cpi)) {
        pd_dprintf("paw3323: set_cpi(%u) failed\n", cpi);
    }
}

/* The X/Y resolution LUTs contain duplicate entries and the extended range
 * cannot be recovered from 0x4E alone, so report what was last programmed. */
uint16_t paw3323_get_cpi(void) {
    return paw3323_last_cpi;
}

report_mouse_t paw3323_get_report(report_mouse_t mouse_report) {
    report_paw3323_t data = paw3323_read_burst();

    if (data.dx != 0 || data.dy != 0) {
        pd_dprintf("Raw ] X: %d, Y: %d\n", data.dx, data.dy);
        mouse_report.x = CONSTRAIN_HID_XY(data.dx);
        mouse_report.y = CONSTRAIN_HID_XY(data.dy);
    }

    return mouse_report;
}

/* Debug aid: returns the raw Product_ID / Inv_Product_ID register values.
 * Returns false if an SPI read failed (the matching output is then 0). */
bool paw3323_read_signature(uint8_t *product_id, uint8_t *inv_product_id) {
    spi_status_t status = SPI_STATUS_SUCCESS;
    uint8_t      checkval_1 = 0;
    uint8_t      checkval_2 = 0;

    PAW_CHAIN(status, paw3323_read(REG_Product_ID, &checkval_1));
    PAW_CHAIN(status, paw3323_read(REG_Inv_Product_ID, &checkval_2));

    if (product_id != NULL) {
        *product_id = checkval_1;
    }
    if (inv_product_id != NULL) {
        *inv_product_id = checkval_2;
    }

    return status == SPI_STATUS_SUCCESS;
}

static bool paw3323_signature_valid(uint8_t product_id, uint8_t inv_product_id) {
    return (product_id == 0x2F) && (inv_product_id == (uint8_t)~product_id);
}

bool paw3323_check_signature(void) {
    uint8_t checkval_1;
    uint8_t checkval_2;

    return paw3323_read_signature(&checkval_1, &checkval_2) && paw3323_signature_valid(checkval_1, checkval_2);
}

const paw3323_debug_t *paw3323_get_debug(void) {
    return &paw3323_dbg;
}

void paw3323_debug_dump(void) {
#ifdef CONSOLE_ENABLE
    static const char *const stage_name[] = {"not run", "reset", "signature", "init_seq", "ready_poll", "post_init", "mode_cpi", "discard", "DONE"};
    const paw3323_debug_t    *d              = &paw3323_dbg;

    uprintf("PAW3323 init report: runs=%u result=%s stage=%s t=%lu..%lu ms\n", d->init_count, d->ok ? "OK" : "FAILED", stage_name[d->stage], (unsigned long)d->t_start_ms, (unsigned long)d->t_end_ms);
    if (d->stage >= PAW3323_STAGE_SIGNATURE) {
        uprintf("  signature: spi %s, Product_ID=0x%02X (want 0x2F), Inv_Product_ID=0x%02X (want 0xD0)\n", d->sig_spi_ok ? "ok" : "FAILED", d->pid, d->inv_pid);
    }
    if (d->stage >= PAW3323_STAGE_READY_POLL) {
        uprintf("  ready poll: %u reads, last 0x20=0x%02X (want 0x0F) -> %s\n", d->ready_polls, d->ready_last, d->ready_ok ? "ready" : "timed out / failed");
    }
    if (d->fail_index != 0xFFFF) {
        if (d->fail_index == PAW3323_DBG_NO_INDEX) {
            uprintf("  failed access: stage %s, reg 0x%02X, val 0x%02X, spi_status %d\n", stage_name[d->stage], d->fail_reg, d->fail_val, d->spi_status);
        } else {
            // Only these tables are numbered in the datasheet: init steps 1-116, steps 120-126, mode table steps 1-39
            unsigned step = 0;
            if (d->stage == PAW3323_STAGE_INIT_SEQ || d->stage == PAW3323_STAGE_MODE_CPI) {
                step = d->fail_index + 1U;
            } else if (d->stage == PAW3323_STAGE_POST_INIT) {
                step = d->fail_index + 120U;
            }
            if (step != 0) {
                uprintf("  failed access: stage %s, entry %u (datasheet step %u), reg 0x%02X, val 0x%02X, spi_status %d\n", stage_name[d->stage], d->fail_index, step, d->fail_reg, d->fail_val, d->spi_status);
            } else {
                uprintf("  failed access: stage %s, entry %u, reg 0x%02X, val 0x%02X, spi_status %d\n", stage_name[d->stage], d->fail_index, d->fail_reg, d->fail_val, d->spi_status);
            }
        }
    }
    if (d->polls == 0) {
        uprintf("  motion: NO POLLS since init (get_report not running, or init failed)\n");
    } else {
        uint32_t up_ms = timer_read32() - d->t_end_ms;
        uprintf("  motion: polls=%lu (~%lu/s) mot_hits=%lu spi_err=%lu last_motion=0x%02X sum=(%ld,%ld)\n", (unsigned long)d->polls, up_ms ? (unsigned long)((uint64_t)d->polls * 1000u / up_ms) : 0UL, (unsigned long)d->mot_hits, (unsigned long)d->spi_errors, d->last_motion, (long)d->sum_dx, (long)d->sum_dy);
        uprintf("  op_mode polls: run=%lu rest1=%lu rest2=%lu rest3=%lu\n", (unsigned long)d->opmode[0], (unsigned long)d->opmode[1], (unsigned long)d->opmode[2], (unsigned long)d->opmode[3]);
    }
    uprintf("  now: cpi=%u band=%s mode_loaded=%u\n", paw3323_last_cpi, paw3323_mode_high ? "high" : "low", paw3323_mode_applied);
#endif
}

/* On-demand sensor health check. Blocks ~0.6 s (Chip_Observation needs > the longest frame period). */
void paw3323_debug_snapshot(void) {
#ifdef CONSOLE_ENABLE
    static const char *const opmode_name[] = {"run", "rest1", "rest2", "rest3"};
    const paw3323_debug_t    *d             = &paw3323_dbg;
    spi_status_t              status        = SPI_STATUS_SUCCESS;
    uint8_t                   motion = 0, squal = 0, raw_sum = 0, raw_max = 0, raw_min = 0, sh_u = 0, sh_l = 0;
    uint8_t                   perf = 0, axis = 0, res_hi = 0, res_x = 0, res_y = 0, obs = 0;

    PAW_CHAIN(status, paw3323_read(REG_Motion, &motion));
    PAW_CHAIN(status, paw3323_read(REG_SQUAL, &squal));
    PAW_CHAIN(status, paw3323_read(REG_Raw_Data_Sum, &raw_sum));
    PAW_CHAIN(status, paw3323_read(REG_Maximum_RawData, &raw_max));
    PAW_CHAIN(status, paw3323_read(REG_Minimum_RawData, &raw_min));
    PAW_CHAIN(status, paw3323_read(REG_Shutter_Upper, &sh_u)); // Upper first, then Lower, back to back
    PAW_CHAIN(status, paw3323_read(REG_Shutter_Lower, &sh_l));
    PAW_CHAIN(status, paw3323_read(REG_Performance, &perf));
    PAW_CHAIN(status, paw3323_read(REG_Axis_Control, &axis));
    PAW_CHAIN(status, paw3323_read(REG_Res_XY_High, &res_hi));
    PAW_CHAIN(status, paw3323_read(REG_Res_X_Low, &res_x));
    PAW_CHAIN(status, paw3323_read(REG_Res_Y_Low, &res_y));
    // Chip_Observation self-check: clear, wait longer than the Rest3 frame period (504 ms +10%), read back
    PAW_CHAIN(status, paw3323_write(REG_Chip_Observation, 0x00));
    if (status == SPI_STATUS_SUCCESS) {
        wait_ms(600);
    }
    PAW_CHAIN(status, paw3323_read(REG_Chip_Observation, &obs));

    uprintf("PAW3323 snapshot: spi %s\n", status == SPI_STATUS_SUCCESS ? "ok" : "FAILED");
    uprintf("  motion=0x%02X MOT=%u op_mode=%s (SQUAL/raw/shutter only mean something in run)\n", motion, (unsigned)((motion >> 7) & 1), opmode_name[motion & 3]);
    uprintf("  SQUAL=%u (%u features, max 182)  raw_sum=%u (avg pixel %u)  raw_max=%u raw_min=%u  shutter=%u\n", squal, (unsigned)squal * 4U, raw_sum, (unsigned)raw_sum * 512U / 900U, raw_max, raw_min, (unsigned)(((sh_u & 0x0F) << 8) | sh_l));
    uprintf("  performance(0x40)=0x%02X (init writes 0x00)  axis_control(0x5B)=0x%02X (init writes 0x50)\n", perf, axis);
    uprintf("  res 4A/4E/4B read back %02X/%02X/%02X, last written %02X/%02X/%02X -> %s\n", res_hi, res_x, res_y, d->exp_res_hi, d->exp_res_x, d->exp_res_y, (res_hi == d->exp_res_hi && res_x == d->exp_res_x && res_y == d->exp_res_y) ? "match" : "MISMATCH (writes not sticking?)");
    uprintf("  chip_observation=0x%02X (want 0x77 or 0x7F) -> %s\n", obs, (obs == 0x77 || obs == 0x7F) ? "ok" : "BAD");
#endif
}
