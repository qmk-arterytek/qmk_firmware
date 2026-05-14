// Copyright 2019 Sunjun Kim
// Copyright 2020 Christopher Courtney, aka Drashna Jael're  (@drashna) <drashna@live.com>
// Copyright 2020 Ploopy Corporation
// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "quantum.h"

typedef union {
    uint32_t raw;
    struct {
        uint8_t dpi_config;
    };
} user_config_t;

extern user_config_t user_config;
extern uint16_t dpi_array[];

enum custom_keycodes {
    DPI_CFG = QK_KB_0
};

void cycle_dpi(void);
