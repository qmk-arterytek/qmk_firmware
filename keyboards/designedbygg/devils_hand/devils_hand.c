// Copyright 2019 Sunjun Kim
// Copyright 2020 Christopher Courtney, aka Drashna Jael're  (@drashna) <drashna@live.com>
// Copyright 2020 Ploopy Corporation
// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#include "devils_hand.h"
#include "quantum.h"

#ifdef POINTING_DEVICE_ENABLE

#ifndef MOUSE_DPI_OPTIONS
#    define MOUSE_DPI_OPTIONS \
        { 400, 800, 1200, 1600, 3200, 6400, 10000 }
#    ifndef MOUSE_DPI_DEFAULT
#        define MOUSE_DPI_DEFAULT 1
#    endif
#endif

user_config_t user_config;
uint16_t dpi_array[] = MOUSE_DPI_OPTIONS;
#define DPI_OPTION_SIZE ARRAY_SIZE(dpi_array)

static bool dpi_indicator_active = false;
static uint16_t dpi_indicator_timer;

void dpi_indicator_trigger(void) {
    dpi_indicator_timer = timer_read();
    dpi_indicator_active = true;
}

void cycle_dpi(void) {
    user_config.dpi_config = (user_config.dpi_config + 1) % DPI_OPTION_SIZE;
    eeconfig_update_kb(user_config.raw);
    pointing_device_set_cpi(dpi_array[user_config.dpi_config]);
    dpi_indicator_trigger();
}

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    if (!rgb_matrix_indicators_advanced_user(led_min, led_max)) {
        return false;
    }

    rgb_t dpi_colors[] = {
        {255,   0,   0},
        {255, 255,   0},
        {  0, 255,   0},
        {  0, 255, 255},
        {  0,   0, 255},
        {255,   0, 255},
        {255, 255, 255},
    };

    if (dpi_indicator_active) {
        if (timer_elapsed(dpi_indicator_timer) > 2000) {
            dpi_indicator_active = false;
        } else if (user_config.dpi_config < DPI_OPTION_SIZE) {
            rgb_t colors = dpi_colors[user_config.dpi_config];
            rgb_matrix_set_color_all(colors.r, colors.g, colors.b);
        }
    }

    return true;
}

report_mouse_t pointing_device_task_kb(report_mouse_t mouse_report) {
    mouse_report = pointing_device_task_user(mouse_report);

    return mouse_report;
}

void pointing_device_init_kb(void) {
    user_config.raw = eeconfig_read_kb();
    if (user_config.dpi_config > DPI_OPTION_SIZE) {
        eeconfig_init_kb();
    }
    pointing_device_set_cpi(dpi_array[user_config.dpi_config]);
}
#endif

void eeconfig_init_kb(void) {
    user_config.dpi_config = MOUSE_DPI_DEFAULT;
    eeconfig_update_kb(user_config.raw);
    eeconfig_init_user();
}

void keyboard_post_init_kb(void) {
    user_config.raw = eeconfig_read_kb();
}

bool process_record_kb(uint16_t keycode, keyrecord_t* record) {
    if (!process_record_user(keycode, record)) {
        return false;
    }

    switch (keycode) {
        case DPI_CFG:
            if (record->event.pressed) {
                cycle_dpi();
            }

            return false;
    }

    return true;
}
