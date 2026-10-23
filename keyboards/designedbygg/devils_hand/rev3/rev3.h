// Copyright 2019 Sunjun Kim
// Copyright 2020 Christopher Courtney, aka Drashna Jael're  (@drashna) <drashna@live.com>
// Copyright 2020 Ploopy Corporation
// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "quantum.h"

/*
 * EEPROM layout change.
 *
 * polling_rate joins dpi_config in the existing 32-bit kb block, rather than
 * opening a second EEPROM region. Two bytes of four are now in use.
 *
 * Adding a field does not invalidate what is already stored - dpi_config keeps
 * its offset, so your saved DPI survives the update.
 *
 * polling_rate is stored as index + 1, with 0 reserved for "never saved". A
 * block written by the old firmware has 0 here because nothing ever wrote it,
 * and 0 is also a perfectly valid rate index (250 Hz) - so without the offset,
 * updating would silently drop every existing mouse from 8 kHz to the slowest
 * rate.
 */
typedef union {
    uint32_t raw;
    struct {
        uint8_t dpi_config;
        uint8_t polling_rate;
    };
} user_config_t;

extern user_config_t user_config;
extern uint16_t dpi_array[];

enum custom_keycodes {
    DPI_CFG = QK_KB_0,
    PR_CFG  = QK_KB_1,
    /* Only does anything in a POLLING_RATE_SELFTEST=yes build; the keycode
       always exists so the enum, and therefore VIA, stays stable either way. */
    PR_TEST = QK_KB_2
};

void cycle_dpi(void);
void cycle_polling_rate(void);
