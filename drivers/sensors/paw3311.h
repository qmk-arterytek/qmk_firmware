// Copyright 2021 Colin Lam (Ploopy Corporation)
// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "pointing_device.h"

#ifndef PAW3311_CS_PIN
#    ifdef POINTING_DEVICE_CS_PIN
#        define PAW3311_CS_PIN POINTING_DEVICE_CS_PIN
#    else
#        error "No chip select pin defined -- missing POINTING_DEVICE_CS_PIN or PAW3311_CS_PIN define"
#    endif
#endif

#ifndef PAW3311_SPI_DIVISOR
#    error "No PAW3311 SPI divisor defined -- missing PAW3311_SPI_DIVISOR"
#endif

#ifndef PAW3311_INIT_REG_SET
#    define PAW3311_INIT_REG_SET LENS_1
#endif

#ifndef PAW3311_GAMING_MODE
#    define PAW3311_GAMING_MODE MODE_1
#endif

typedef enum {
    LENS_1, // LM31-LNG, LM33-LSG, L0AL-LSG1
    LENS_2  // L0AJ-LSG1
} paw3311_lens_t;

typedef enum {
    MODE_1, // Low power mode
    MODE_2  // Corded mode
} paw3311_mode_t;

typedef struct {
    int16_t dx;
    int16_t dy;
} report_paw3311_t;

extern const pointing_device_driver_t paw3311_pointing_device_driver;

bool             paw3311_init(void);
report_paw3311_t paw3311_read_burst(void);
void             paw3311_set_cpi(uint16_t cpi);
uint16_t         paw3311_get_cpi(void);
report_mouse_t   paw3311_get_report(report_mouse_t mouse_report);
bool             paw3311_check_signature(void);

#if !defined(PAW3311_CPI)
#    define PAW3311_CPI 3000
#endif

#define PAW3311_CPI_STEP 50
#define PAW3311_CPI_MIN 50
#define PAW3311_CPI_MAX 10000
#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
