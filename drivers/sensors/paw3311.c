// Copyright 2024 Colin Lam (Ploopy Corporation)
// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#include <stddef.h>
#include <string.h>
#include "paw3311.h"
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
} paw3311_reg_pair_t;

#define PAW3311_CHAIN(status, call)           \
    do {                                      \
        if ((status) == SPI_STATUS_SUCCESS) { \
            (status) = (call);                \
        }                                     \
    } while (0)

#define PAW3311_CHAIN_BOOL(status, call)      \
    do {                                      \
        if ((status) == SPI_STATUS_SUCCESS) { \
            if (!(call)) {                    \
                (status) = SPI_STATUS_ERROR;  \
            }                                 \
        }                                     \
    } while (0)

static paw3311_debug_t paw3311_dbg;

#define PAW3311_LOG_FAIL(step)                                                   \
    do {                                                                         \
        paw3311_dbg.ok       = false;                                            \
        paw3311_dbg.t_end_ms = timer_read32();                                   \
        pd_dprintf("PAW3311: init FAILED at %s (line %d)\n", (step), __LINE__); \
    } while (0)

#define PAW3311_STAGE(s) (paw3311_dbg.stage = (s))

#define PAW3311_DBG_NO_INDEX 0xFFFE /* failing access was not an entry of a table */

static void paw3311_dbg_fail(size_t index, uint8_t reg, uint8_t val, spi_status_t status) {
    paw3311_dbg.fail_index = (uint16_t)index;
    paw3311_dbg.fail_reg   = reg;
    paw3311_dbg.fail_val   = val;
    paw3311_dbg.spi_status = status;
}

static uint16_t paw3311_last_cpi     = 0;     /* last CPI programmed into the chip, 0 = unknown */
static bool     paw3311_mode_applied = false; /* mode sequence loaded since the last init */

static bool paw3311_apply_cpi(uint16_t cpi);
static bool paw3311_signature_valid(uint8_t product_id, uint8_t inv_product_id);

const pointing_device_driver_t paw3311_pointing_device_driver = {
    .init       = paw3311_init,
    .get_report = paw3311_get_report,
    .set_cpi    = paw3311_set_cpi,
    .get_cpi    = paw3311_get_cpi,
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

static spi_status_t paw3311_write(uint8_t reg_addr, uint8_t data) {
    spi_status_t status = SPI_STATUS_ERROR;

    if (!spi_start(PAW3311_CS_PIN, false, 3, PAW3311_SPI_DIVISOR)) {
        return status;
    }

    wait_us(1); // tNCS_SCLK
    status = spi_write(reg_addr | MSB1);
    wait_us(2); // tSRAD
    if (status >= SPI_STATUS_SUCCESS) {
        status = spi_write(data);
    }
    if (status >= SPI_STATUS_SUCCESS) {
        status = SPI_STATUS_SUCCESS;
    }
    wait_us(1); // tSCLK_NCS_W
    spi_stop();
    wait_us(2); // tSRW

    return status;
}

static spi_status_t paw3311_read(uint8_t reg_addr, uint8_t *value) {
    spi_status_t status = SPI_STATUS_ERROR;

    if (value == NULL) {
        return status;
    }

    if (!spi_start(PAW3311_CS_PIN, false, 3, PAW3311_SPI_DIVISOR)) {
        return status;
    }

    wait_us(1); // tNCS_SCLK
    status = spi_write(reg_addr & MSB0);
    wait_us(2); // tSRAD
    if (status >= SPI_STATUS_SUCCESS) {
        status = spi_read();
        if (status >= SPI_STATUS_SUCCESS) {
            *value = (uint8_t)status;
            status = SPI_STATUS_SUCCESS;
        }
    }
    wait_us(1); // tSCLK_NCS_R
    spi_stop();
    wait_us(2); // tSRR

    return status;
}

#ifdef PAW3311_READ_BURST
static spi_status_t paw3311_read_dump(uint8_t reg_addr, uint8_t *buf, uint8_t len) {
    spi_status_t status = SPI_STATUS_ERROR;

    if (buf == NULL) {
        return status;
    }

    if (!spi_start(PAW3311_CS_PIN, false, 3, PAW3311_SPI_DIVISOR)) {
        return status;
    }

    wait_us(1); // tNCS_SCLK
    status = spi_write(reg_addr & MSB0);
    wait_us(2); // tSRAD

    if (status >= SPI_STATUS_SUCCESS) {
        status = spi_receive(buf, len);
    }

    wait_us(1); // tSCLK_NCS_R
    spi_stop();
    wait_us(2); // tSRR

    return status;
}
#endif

/* Single write that records the register/value/status if it fails (init report) */
static spi_status_t paw3311_write_logged(uint8_t reg, uint8_t val) {
    spi_status_t status = paw3311_write(reg, val);

    if (status != SPI_STATUS_SUCCESS) {
        paw3311_dbg_fail(PAW3311_DBG_NO_INDEX, reg, val, status);
    }

    return status;
}

static bool paw3311_write_sequence(const paw3311_reg_pair_t *seq, size_t len) {
    spi_status_t status = SPI_STATUS_SUCCESS;

    for (size_t i = 0; i < len; i++) {
        PAW3311_CHAIN(status, paw3311_write(seq[i].reg, seq[i].val));
        if (status != SPI_STATUS_SUCCESS) {
            paw3311_dbg_fail(i, seq[i].reg, seq[i].val, status);
            break;
        }
    }

    return status == SPI_STATUS_SUCCESS;
}

static bool paw3311_read_discard_sequence(const uint8_t *regs, size_t len) {
    spi_status_t status = SPI_STATUS_SUCCESS;
    uint8_t      discard;

    for (size_t i = 0; i < len; i++) {
        PAW3311_CHAIN(status, paw3311_read(regs[i], &discard));
        if (status != SPI_STATUS_SUCCESS) {
            paw3311_dbg_fail(i, regs[i], 0, status);
            break;
        }
    }

    return status == SPI_STATUS_SUCCESS;
}

static bool paw3311_init_ready(void) {
    for (uint8_t i = 0; i < 55; i++) {
        uint8_t      value;
        spi_status_t status = paw3311_read(REG_Undocumented_20, &value);

        paw3311_dbg.ready_polls = (uint8_t)(i + 1);
        if (status != SPI_STATUS_SUCCESS) {
            paw3311_dbg_fail(i, REG_Undocumented_20, 0, status);
            return false;
        }

        paw3311_dbg.ready_last = value;
        if (value == 0x0F) {
            paw3311_dbg.ready_ok = true;
            return true;
        }

        wait_ms(1);
    }

    return false;
}

static const paw3311_reg_pair_t paw3311_init_seq_power[] = {
    {REG_Power_Up_Reset, 0x5A},
};

static const paw3311_reg_pair_t paw3311_early_init_seq[] = {
    /* 1 */ {0x40, 0x80},
    /* 2 */ {0x55, 0x01},
};

static const paw3311_reg_pair_t paw3311_pre_init_seq[] = {
    /* 12 */ {0x7F, 0x00},
    /* 13 */ {0x55, 0x00},
    /* 14 */ {0x4D, 0x50},
};

#if (PAW3311_INIT_REG_SET == LENS_1) // LM31-LNG, LM33-LSG, L0AL-LSG1

static const paw3311_reg_pair_t paw3311_lens_seq[] = {
    /* 15 */ {0x4E, 0x46},
    /* 16 */ {0x4F, 0x46},
    /* 17 */ {0x77, 0x24},
    /* 18 */ {0x7F, 0x05},
    /* 19 */ {0x44, 0xA8},
    /* 20 */ {0x4A, 0x14},
    /* 21 */ {0x53, 0x0C},
    /* 22 */ {0x5B, 0xEA},
    /* 23 */ {0x61, 0x13},
    /* 24 */ {0x62, 0x07},
    /* 25 */ {0x64, 0xD8},
    /* 26 */ {0x6D, 0x86},
    /* 27 */ {0x7D, 0x84},
    /* 28 */ {0x7E, 0x00},
    /* 29 */ {0x7F, 0x06},
    /* 30 */ {0x60, 0x70},
    /* 31 */ {0x61, 0x00},
    /* 32 */ {0x7E, 0x40},
    /* 33 */ {0x7F, 0x07},
    /* 34 */ {0x42, 0x16},
    /* 35 */ {0x7F, 0x09},
    /* 36 */ {0x40, 0x03},
    /* 37 */ {0x7F, 0x0A},
    /* 38 */ {0x49, 0x00},
    /* 39 */ {0x4A, 0x23},
    /* 40 */ {0x4C, 0x28},
    /* 41 */ {0x4F, 0x02},
    /* 42 */ {0x7F, 0x0C},
    /* 43 */ {0x40, 0xA0},
    /* 44 */ {0x41, 0x70},
    /* 45 */ {0x42, 0x20},
    /* 46 */ {0x43, 0xC5},
    /* 47 */ {0x44, 0x44},
    /* 48 */ {0x45, 0x04},
    /* 49 */ {0x4C, 0x60},
};

#elif (PAW3311_INIT_REG_SET == LENS_2) // L0AJ-LSG1

static const paw3311_reg_pair_t paw3311_lens_seq[] = {
    /* 15 */ {0x4E, 0x47},
    /* 16 */ {0x4F, 0x46},
    /* 17 */ {0x77, 0x24},
    /* 18 */ {0x7F, 0x05},
    /* 19 */ {0x44, 0xA8},
    /* 20 */ {0x4A, 0x14},
    /* 21 */ {0x53, 0x0C},
    /* 22 */ {0x5B, 0xEA},
    /* 23 */ {0x61, 0x13},
    /* 24 */ {0x62, 0x07},
    /* 25 */ {0x64, 0xD8},
    /* 26 */ {0x6D, 0x86},
    /* 27 */ {0x7D, 0x84},
    /* 28 */ {0x7E, 0x00},
    /* 29 */ {0x7F, 0x06},
    /* 30 */ {0x60, 0x70},
    /* 31 */ {0x61, 0x00},
    /* 32 */ {0x7E, 0x40},
    /* 33 */ {0x7F, 0x07},
    /* 34 */ {0x42, 0x16},
    /* 35 */ {0x7F, 0x09},
    /* 36 */ {0x40, 0x03},
    /* 37 */ {0x7F, 0x0A},
    /* 38 */ {0x49, 0x00},
    /* 39 */ {0x4A, 0x23},
    /* 40 */ {0x4C, 0x28},
    /* 41 */ {0x4F, 0x02},
    /* 42 */ {0x7F, 0x0C},
    /* 43 */ {0x40, 0xB8},
    /* 44 */ {0x41, 0x70},
    /* 45 */ {0x42, 0x1E},
    /* 46 */ {0x43, 0xA8},
    /* 47 */ {0x44, 0x44},
    /* 48 */ {0x45, 0x01},
    /* 49 */ {0x4A, 0x13},
    /* 50 */ {0x4B, 0x12},
    /* 51 */ {0x4C, 0x44},
    /* 52 */ {0x4E, 0x1B},
    /* 53 */ {0x58, 0x2C},
    /* 54 */ {0x6D, 0x7F},
};

#else
#    error "Invalid PAW3311_INIT_REG_SET. Use LENS_1 or LENS_2."
#endif

static const paw3311_reg_pair_t paw3311_init_seq[] = {
    /* 50/55 */ {0x54, 0x00},
    /* 51/56 */ {0x55, 0x40},
    /* 52/57 */ {0x59, 0x93},
    /* 53/58 */ {0x7F, 0x0D},
    /* 54/59 */ {0x4F, 0x02},
    /* 55/60 */ {0x4E, 0x6B},
    /* 56/61 */ {0x7F, 0x14},
    /* 57/62 */ {0x4A, 0x67},
    /* 58/63 */ {0x62, 0x1C},
    /* 59/64 */ {0x63, 0x1C},
    /* 60/65 */ {0x6D, 0x82},
    /* 61/66 */ {0x6F, 0xD8},
    /* 62/67 */ {0x73, 0x83},
    /* 63/68 */ {0x74, 0x00},
    /* 64/69 */ {0x7A, 0x16},
    /* 65/70 */ {0x7F, 0x10},
    /* 66/71 */ {0x48, 0x0F},
    /* 67/72 */ {0x49, 0x88},
    /* 68/73 */ {0x4C, 0x1D},
    /* 69/74 */ {0x4F, 0x08},
    /* 70/75 */ {0x51, 0x6F},
    /* 71/76 */ {0x52, 0x90},
    /* 72/77 */ {0x54, 0x64},
    /* 73/78 */ {0x55, 0xF0},
    /* 74/79 */ {0x5C, 0x40},
    /* 75/80 */ {0x61, 0xEE},
    /* 76/81 */ {0x62, 0xE5},
    /* 77/82 */ {0x7F, 0x00},
    /* 78/83 */ {0x5B, 0x40},
    /* 79/84 */ {0x61, 0xAD},
    /* 80/85 */ {0x51, 0xEA},
    /* 81/86 */ {0x19, 0x9F},
};

static const paw3311_reg_pair_t paw3311_post_init_seq[] = {
    /* 83/88 */ {0x19, 0x10},
    /* 84/89 */ {0x40, 0x00},
    /* 85/90 */ {0x61, 0xD5},
    /* 86/91 */ {0x7F, 0x00},
};

static const uint8_t paw3311_init_discard_regs[] = {
    REG_Motion, REG_Delta_X_L, REG_Delta_X_H, REG_Delta_Y_L, REG_Delta_Y_H,
};

#if (PAW3311_GAMING_MODE == MODE_1) // Low power

static const paw3311_reg_pair_t paw3311_mode_seq[] = {
    /*  1 */ {0x7F, 0x05},
    /*  2 */ {0x62, 0x07},
    /*  3 */ {0x7E, 0x00},
    /*  4 */ {0x61, 0x13},
    /*  5 */ {0x55, 0x87},
    /*  6 */ {0x51, 0x0C},
    /*  7 */ {0x53, 0x0C},
    /*  8 */ {0x7F, 0x06},
    /*  9 */ {0x60, 0x70},
    /* 10 */ {0x61, 0x00},
    /* 11 */ {0x62, 0x10},
    /* 12 */ {0x63, 0x00},
    /* 13 */ {0x7F, 0x00},
    /* 14 */ {0x77, 0x24},
    /* 15 */ {0x78, 0x01},
    /* 16 */ {0x79, 0x4F},
    /* 17 */ {0x7A, 0x08},
    /* 18 */ {0x7B, 0x4A},
};

#elif (PAW3311_GAMING_MODE == MODE_2) // Corded

static const paw3311_reg_pair_t paw3311_mode_seq[] = {
    /*  1 */ {0x7F, 0x05},
    /*  2 */ {0x62, 0x04},
    /*  3 */ {0x7E, 0x00},
    /*  4 */ {0x61, 0x13},
    /*  5 */ {0x55, 0x87},
    /*  6 */ {0x51, 0x0C},
    /*  7 */ {0x53, 0x0C},
    /*  8 */ {0x7F, 0x06},
    /*  9 */ {0x60, 0x40},
    /* 10 */ {0x61, 0x00},
    /* 11 */ {0x62, 0x10},
    /* 12 */ {0x63, 0x00},
    /* 13 */ {0x7F, 0x00},
    /* 14 */ {0x77, 0x24},
    /* 15 */ {0x78, 0x01},
    /* 16 */ {0x79, 0x4F},
    /* 17 */ {0x7A, 0x08},
    /* 18 */ {0x7B, 0x4A},
};

#else
#    error "Invalid PAW3311_GAMING_MODE. Use MODE_1 or MODE_2."
#endif

bool paw3311_init(void) {
    uint8_t VarA, VarB;
    spi_status_t status = SPI_STATUS_SUCCESS;

    uint16_t runs = paw3311_dbg.init_count + 1;
    memset(&paw3311_dbg, 0, sizeof(paw3311_dbg));
    paw3311_dbg.init_count = runs;
    paw3311_dbg.fail_index = 0xFFFF;
    paw3311_dbg.t_start_ms = timer_read32();

    /* A reset reverts the chip to LPM / low-CPI settings */
    paw3311_mode_applied = false;
    paw3311_last_cpi     = 0;

    wait_ms(50);
    gpio_set_pin_output(PAW3311_CS_PIN);

    // CS must be kept high at power-up stage for at least 1us
    gpio_write_pin_high(PAW3311_CS_PIN);
    wait_us(1);
    gpio_write_pin_low(PAW3311_CS_PIN);

    spi_init();

    /* Power-up reboot */
    PAW3311_STAGE(PAW3311_STAGE_RESET);
    PAW3311_CHAIN_BOOL(status, paw3311_write_sequence(paw3311_init_seq_power, ARRAY_SIZE(paw3311_init_seq_power)));

    if (status != SPI_STATUS_SUCCESS) {
        PAW3311_LOG_FAIL("reboot");
        return false;
    }

    wait_ms(5);

    /* Signature check (raw values are kept for the init report) */
    PAW3311_STAGE(PAW3311_STAGE_SIGNATURE);
    paw3311_dbg.sig_spi_ok = paw3311_read_signature(&paw3311_dbg.pid, &paw3311_dbg.inv_pid);

    if (!paw3311_dbg.sig_spi_ok) {
        status = SPI_STATUS_ERROR;
    } else if (!paw3311_signature_valid(paw3311_dbg.pid, paw3311_dbg.inv_pid)) {
        status = SPI_STATUS_ERROR;
    }

    if (status != SPI_STATUS_SUCCESS) {
        PAW3311_LOG_FAIL("signature");
        return false;
    }

    /* Common init sequence */
    PAW3311_STAGE(PAW3311_STAGE_EARLY_INIT);
    PAW3311_CHAIN_BOOL(status, paw3311_write_sequence(paw3311_early_init_seq, ARRAY_SIZE(paw3311_early_init_seq)));

    if (status != SPI_STATUS_SUCCESS) {
        PAW3311_LOG_FAIL("early-init");
        return false;
    }

    wait_ms(1);
    PAW3311_CHAIN(status, paw3311_write(0x7F, 0x0E));
    PAW3311_CHAIN(status, paw3311_write(0x43, 0x1D));
    PAW3311_CHAIN(status, paw3311_read(0x46, &VarA));
    PAW3311_CHAIN(status, paw3311_write(0x43, 0x1E));
    PAW3311_CHAIN(status, paw3311_read(0x46, &VarB));
    PAW3311_CHAIN(status, paw3311_write(0x7F, 0x14));
    PAW3311_CHAIN(status, paw3311_write(0x6A, VarA));
    PAW3311_CHAIN(status, paw3311_write(0x6C, VarB));

    PAW3311_STAGE(PAW3311_STAGE_PRE_INIT);
    PAW3311_CHAIN_BOOL(status, paw3311_write_sequence(paw3311_pre_init_seq, ARRAY_SIZE(paw3311_pre_init_seq)));

    if (status != SPI_STATUS_SUCCESS) {
        PAW3311_LOG_FAIL("pre-init");
        return false;
    }

    PAW3311_STAGE(PAW3311_STAGE_LENS);
    PAW3311_CHAIN_BOOL(status, paw3311_write_sequence(paw3311_lens_seq, ARRAY_SIZE(paw3311_lens_seq)));

    if (status != SPI_STATUS_SUCCESS) {
        PAW3311_LOG_FAIL("lens");
        return false;
    }

    PAW3311_STAGE(PAW3311_STAGE_INIT_SEQ);
    PAW3311_CHAIN_BOOL(status, paw3311_write_sequence(paw3311_init_seq, ARRAY_SIZE(paw3311_init_seq)));

    if (status != SPI_STATUS_SUCCESS) {
        PAW3311_LOG_FAIL("init_seq");
        return false;
    }

    wait_ms(1);

    PAW3311_STAGE(PAW3311_STAGE_READY_POLL);
    paw3311_init_ready();

    PAW3311_STAGE(PAW3311_STAGE_POST_INIT);
    PAW3311_CHAIN_BOOL(status, paw3311_write_sequence(paw3311_post_init_seq, ARRAY_SIZE(paw3311_post_init_seq)));

    if (status != SPI_STATUS_SUCCESS) {
        PAW3311_LOG_FAIL("post-init");
        return false;
    }

    /* Load the operation mode tables and program the configured CPI. This is
     * done after the 7.1.2 sequence (steps 1-126) so that sequence stays intact. */
    PAW3311_STAGE(PAW3311_STAGE_MODE_CPI);
    PAW3311_CHAIN_BOOL(status, paw3311_write_sequence(paw3311_mode_seq, ARRAY_SIZE(paw3311_mode_seq)));

    if (!paw3311_apply_cpi(PAW3311_CPI) || (status != SPI_STATUS_SUCCESS)) {
        PAW3311_LOG_FAIL("mode/cpi");
        return false;
    }

    PAW3311_STAGE(PAW3311_STAGE_DISCARD);
    PAW3311_CHAIN_BOOL(status, paw3311_read_discard_sequence(paw3311_init_discard_regs, ARRAY_SIZE(paw3311_init_discard_regs)));

    if (status != SPI_STATUS_SUCCESS) {
        PAW3311_LOG_FAIL("discard");
        return false;
    }

    PAW3311_STAGE(PAW3311_STAGE_DONE);
    paw3311_dbg.ok       = true;
    paw3311_dbg.t_end_ms = timer_read32();
    pd_dprintf("PAW3311: init OK\n");
    return true;
}

report_paw3311_t paw3311_read_burst(void) {
    report_paw3311_t report = {0};

#ifdef PAW3311_READ_BURST
    uint8_t data[12] = {0};
    if (paw3311_read_dump(REG_Burst_Motion_Read, data, (uint8_t)sizeof(data)) == SPI_STATUS_SUCCESS) {
        if ((data[0] & MSB1) == MSB1) {
            // Motion detected
            report.dx = convert_twoscomp_16(data[3], data[2]);
            report.dy = convert_twoscomp_16(data[5], data[4]);
        }
    }
#else
    spi_status_t status = SPI_STATUS_SUCCESS;
    uint8_t      motion;
    uint8_t      dx_l;
    uint8_t      dx_h;
    uint8_t      dy_l;
    uint8_t      dy_h;

    status = paw3311_read(REG_Motion, &motion);
    if (status == SPI_STATUS_SUCCESS && (motion & MSB1) == MSB1) {
        // Motion detected
        PAW3311_CHAIN(status, paw3311_read(REG_Delta_X_L, &dx_l));
        PAW3311_CHAIN(status, paw3311_read(REG_Delta_X_H, &dx_h));
        PAW3311_CHAIN(status, paw3311_read(REG_Delta_Y_L, &dy_l));
        PAW3311_CHAIN(status, paw3311_read(REG_Delta_Y_H, &dy_h));

        if (status == SPI_STATUS_SUCCESS) {
            report.dx = convert_twoscomp_16(dx_h, dx_l);
            report.dy = convert_twoscomp_16(dy_h, dy_l);
        }
    }
#endif

    return report;
}

static const uint8_t paw3311_cpi_lut[200] = {
#if (PAW3311_INIT_REG_SET == LENS_1)
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2F,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x43, 0x44, 0x45, 0x46,
    0x47, 0x48, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x51, 0x52, 0x53, 0x54, 0x55, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5E,
    0x5F, 0x60, 0x61, 0x62, 0x63, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x72, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7F, 0x80, 0x81, 0x82, 0x83, 0x84, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8D,
    0x8E, 0x8F, 0x90, 0x91, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F, 0xA1, 0xA2, 0xA3, 0xA4,
    0xA5, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAE, 0xAF, 0xB0, 0xB1, 0xB2, 0xB3, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBB, 0xBC,
    0xBD, 0xBE, 0xBF, 0xC0, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3,
    0xD4, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDD, 0xDE, 0xDF, 0xE0, 0xE1, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xEA, 0xEB
#elif (PAW3311_INIT_REG_SET == LENS_2)
    0x01, 0x02, 0x03, 0x04, 0x05, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x20, 0x21, 0x22, 0x23, 0x24, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2D, 0x2E, 0x2F,
    0x30, 0x31, 0x33, 0x34, 0x35, 0x36, 0x37, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x40, 0x41, 0x42, 0x43, 0x44, 0x46, 0x47,
    0x48, 0x49, 0x4A, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x53, 0x54, 0x55, 0x56, 0x57, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5F,
    0x60, 0x61, 0x62, 0x63, 0x64, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x72, 0x73, 0x74, 0x75, 0x76,
    0x77, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7F, 0x80, 0x81, 0x82, 0x83, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8C, 0x8D, 0x8E,
    0x8F, 0x90, 0x92, 0x93, 0x94, 0x95, 0x96, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9F, 0xA0, 0xA1, 0xA2, 0xA3, 0xA5, 0xA6,
    0xA7, 0xA8, 0xA9, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBE,
    0xBF, 0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5,
    0xD6, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDE, 0xDF, 0xE0, 0xE1, 0xE2, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEB, 0xEC, 0xED
#endif
};

static bool paw3311_cpi_ext = false;

static bool paw3311_apply_cpi(uint16_t cpi) {
    spi_status_t status = SPI_STATUS_SUCCESS;
    uint8_t      value;
    bool         ext = false;

    cpi = CONSTRAIN(cpi, PAW3311_CPI_MIN, PAW3311_CPI_EXT_MAX);

    if (cpi > PAW3311_CPI_MAX) {
        /* Extended CPI range */
        if (cpi < PAW3311_CPI_EXT_MIN) {
            cpi = PAW3311_CPI_EXT_MIN;
        }
        ext   = true;
        value = (cpi / PAW3311_CPI_EXT_STEP) - 1U; // 10100 - 12000 CPI
    } else {
        value = (cpi / PAW3311_CPI_STEP) - 1U; // 50 - 10000 CPI
    }

    PAW3311_CHAIN(status, paw3311_write_logged(0x4D, ext ? 0xD0 : 0x50));
    PAW3311_CHAIN(status, paw3311_write_logged(REG_Resolution, paw3311_cpi_lut[value]));
    PAW3311_CHAIN(status, paw3311_write_logged(REG_Ripple_Control, (cpi >= 5000) ? 0x90 : 0x10));

    if (status == SPI_STATUS_SUCCESS) {
        paw3311_cpi_ext = ext;
    }

    paw3311_last_cpi = cpi;
    return true;
}

void paw3311_set_cpi(uint16_t cpi) {
    if (!paw3311_apply_cpi(cpi)) {
        pd_dprintf("PAW3311: set_cpi(%u) failed\n", cpi);
    }
}

uint16_t paw3311_get_cpi(void) {
//    uint8_t cpival;
//
//    if (paw3311_read(0x4E, &cpival) != SPI_STATUS_SUCCESS) {
//        return 0;
//    }
//
//    for (uint8_t cpi = 0; cpi < ARRAY_SIZE(paw3311_cpi_lut); cpi++) {
//        if (paw3311_cpi_lut[cpi] == cpival) {
//            return (cpi + 1) * (paw3311_cpi_ext ? PAW3311_CPI_EXT_STEP : PAW3311_CPI_STEP);
//        }
//    }
//
//    return 0;
    return paw3311_last_cpi;
}

report_mouse_t paw3311_get_report(report_mouse_t mouse_report) {
    report_paw3311_t data = paw3311_read_burst();

    if (data.dx != 0 || data.dy != 0) {
        pd_dprintf("Raw ] X: %d, Y: %d\n", data.dx, data.dy);
        mouse_report.x = CONSTRAIN_HID_XY(data.dx);
        mouse_report.y = CONSTRAIN_HID_XY(data.dy);
    }

    return mouse_report;
}

/* Debug aid: returns the raw Product_ID / Inv_Product_ID register values.
 * Returns false if an SPI read failed (the matching output is then 0). */
bool paw3311_read_signature(uint8_t *product_id, uint8_t *inv_product_id) {
    spi_status_t status = SPI_STATUS_SUCCESS;
    uint8_t      checkval_1 = 0;
    uint8_t      checkval_2 = 0;

    PAW3311_CHAIN(status, paw3311_read(REG_Product_ID, &checkval_1));
    PAW3311_CHAIN(status, paw3311_read(REG_Inv_Product_ID, &checkval_2));

    if (product_id != NULL) {
        *product_id = checkval_1;
    }
    if (inv_product_id != NULL) {
        *inv_product_id = checkval_2;
    }

    return status == SPI_STATUS_SUCCESS;
}

static bool paw3311_signature_valid(uint8_t product_id, uint8_t inv_product_id) {
    return (product_id == 0x50) && (inv_product_id == (uint8_t)~product_id);
}

bool paw3311_check_signature(void) {
    uint8_t checkval_1;
    uint8_t checkval_2;

    return paw3311_read_signature(&checkval_1, &checkval_2) && paw3311_signature_valid(checkval_1, checkval_2);
}

const paw3311_debug_t *paw3311_get_debug(void) {
    return &paw3311_dbg;
}

void paw3311_debug_dump(void) {
#ifdef CONSOLE_ENABLE
    static const char *const stage_name[] = {"not run", "reset", "signature", "early_init", "pre_init", "lens", "init_seq", "ready_poll", "post_init", "mode_cpi", "discard", "DONE"};
    const paw3311_debug_t    *d           = &paw3311_dbg;

    uprintf("PAW3311 init report: runs=%u result=%s stage=%s t=%lu..%lu ms\n", d->init_count, d->ok ? "OK" : "FAILED", stage_name[d->stage], (unsigned long)d->t_start_ms, (unsigned long)d->t_end_ms);
    if (d->stage >= PAW3311_STAGE_SIGNATURE) {
        uprintf("  signature: spi %s, Product_ID=0x%02X (want 0x2F), Inv_Product_ID=0x%02X (want 0xD0)\n", d->sig_spi_ok ? "ok" : "FAILED", d->pid, d->inv_pid);
    }
    if (d->stage >= PAW3311_STAGE_READY_POLL) {
        uprintf("  ready poll: %u reads, last 0x20=0x%02X (want 0x0F) -> %s\n", d->ready_polls, d->ready_last, d->ready_ok ? "ready" : "timed out / failed");
    }
    if (d->fail_index != 0xFFFF) {
        if (d->fail_index == PAW3311_DBG_NO_INDEX) {
            uprintf("  failed access: stage %s, reg 0x%02X, val 0x%02X, spi_status %d\n", stage_name[d->stage], d->fail_reg, d->fail_val, d->spi_status);
        } else {
            // Only these tables are numbered in the datasheet: init steps 1-116, steps 120-126, mode table steps 1-39
            unsigned step = 0;
            if (d->stage == PAW3311_STAGE_EARLY_INIT || d->stage == PAW3311_STAGE_MODE_CPI) {
                step = d->fail_index + 1U;
            } else if (d->stage == PAW3311_STAGE_PRE_INIT) {
                step = d->fail_index + 12U;
            } else if (d->stage == PAW3311_STAGE_LENS) {
                step = d->fail_index + 15U;
            } else if (d->stage == PAW3311_STAGE_INIT_SEQ) {
#if (PAW3311_INIT_REG_SET == LENS_1) // LM31-LNG, LM33-LSG, L0AL-LSG1
                step = d->fail_index + 50U;
            } else if (d->stage == PAW3311_STAGE_POST_INIT) {
                step = d->fail_index + 83U;
            }
#elif (PAW3311_INIT_REG_SET == LENS_2) // L0AJ-LSG1
                step = d->fail_index + 55U;
            } else if (d->stage == PAW3311_STAGE_POST_INIT) {
                step = d->fail_index + 88U;
            }
#endif
            if (step != 0) {
                uprintf("  failed access: stage %s, entry %u (datasheet step %u), reg 0x%02X, val 0x%02X, spi_status %d\n", stage_name[d->stage], d->fail_index, step, d->fail_reg, d->fail_val, d->spi_status);
            } else {
                uprintf("  failed access: stage %s, entry %u, reg 0x%02X, val 0x%02X, spi_status %d\n", stage_name[d->stage], d->fail_index, d->fail_reg, d->fail_val, d->spi_status);
            }
        }
    }
    uprintf("  now: cpi=%u mode_loaded=%u\n", paw3311_last_cpi, paw3311_mode_applied);
#endif
}
