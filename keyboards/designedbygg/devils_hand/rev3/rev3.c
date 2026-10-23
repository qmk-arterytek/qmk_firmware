// Copyright 2019 Sunjun Kim
// Copyright 2020 Christopher Courtney, aka Drashna Jael're  (@drashna) <drashna@live.com>
// Copyright 2020 Ploopy Corporation
// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#include "rev3.h"
#include "quantum.h"
#include "polling_rate.h"

#ifdef POLLING_RATE_SELFTEST
#    include "polling_rate_selftest.h"
#endif

#ifdef POINTING_DEVICE_ENABLE

#ifndef MOUSE_DPI_OPTIONS
#    define MOUSE_DPI_OPTIONS \
        { 400, 800, 1200, 1600, 10000, 11000, 12000 }
#    ifndef MOUSE_DPI_DEFAULT
#        define MOUSE_DPI_DEFAULT 1
#    endif
#endif

/*
 * Hold DPI_CFG this long to cycle the polling rate instead of the DPI.
 *
 * The board has six buttons and all six are spoken for, so the rate needs a
 * gesture rather than a key. Set this to 0 to disable the gesture entirely and
 * keep DPI_CFG exactly as it was - PR_CFG still works as a keycode you can map
 * in VIA or in the keymap.
 *
 * Note the gesture moves the DPI action from press to release. That is the cost
 * of telling a tap from a hold at all; there is no way to know which one it is
 * until the key comes back up.
 */
#ifndef DEVILS_HAND_RATE_HOLD_MS
#    define DEVILS_HAND_RATE_HOLD_MS 500
#endif

user_config_t user_config;
uint16_t dpi_array[] = MOUSE_DPI_OPTIONS;
#define DPI_OPTION_SIZE ARRAY_SIZE(dpi_array)

static bool dpi_indicator_active = false;
static uint16_t dpi_indicator_timer;

static bool rate_indicator_active = false;
static uint16_t rate_indicator_timer;

/* Indicator window, and half-period of the blink inside it. */
#define RATE_INDICATOR_MS   1800
#define RATE_BLINK_HALF_MS  150

#if DEVILS_HAND_RATE_HOLD_MS > 0
static uint16_t dpi_key_timer;
#endif

void dpi_indicator_trigger(void) {
    dpi_indicator_timer = timer_read();
    dpi_indicator_active = true;
    rate_indicator_active = false;
}

void rate_indicator_trigger(void) {
    rate_indicator_timer = timer_read();
    rate_indicator_active = true;
    dpi_indicator_active = false;
}

void cycle_dpi(void) {
    user_config.dpi_config = (user_config.dpi_config + 1) % DPI_OPTION_SIZE;
    eeconfig_update_kb(user_config.raw);
    pointing_device_set_cpi(dpi_array[user_config.dpi_config]);
    dpi_indicator_trigger();
}

void cycle_polling_rate(void) {
    polling_rate_cycle();
    /* stored offset by one - see the note in the header on why 0 is reserved */
    user_config.polling_rate = (uint8_t)polling_rate_get() + 1;
    eeconfig_update_kb(user_config.raw);
    rate_indicator_trigger();

#ifdef CONSOLE_ENABLE
    uprintf("polling rate: %u Hz\n", polling_rate_get_hz());
#endif
}

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    static const rgb_t dpi_colors[] = {
        {RGB_RED},
        {RGB_YELLOW},
        {RGB_GREEN},
        {RGB_CYAN},
        {RGB_BLUE},
        {RGB_MAGENTA},
        {RGB_WHITE},
    };

    /*
     * One colour per rate. Blinking rather than solid because DPI already owns
     * the solid-colour indicator and shares four of these six colours - with
     * two LEDs, colour alone could not tell you which setting you had just
     * changed. Blink says "rate", colour says which one; no counting needed.
     */
    static const rgb_t rate_colors[] = {
        {RGB_RED},      /*  250 Hz */
        {RGB_YELLOW},   /*  500 Hz */
        {RGB_GREEN},    /*    1 kHz */
        {RGB_CYAN},     /*    2 kHz */
        {RGB_BLUE},     /*    4 kHz */
        {RGB_PURPLE},   /*    8 kHz */
    };

    if (rate_indicator_active) {
        uint16_t elapsed = timer_elapsed(rate_indicator_timer);

        if (elapsed > RATE_INDICATOR_MS) {
            rate_indicator_active = false;
        } else if ((elapsed / RATE_BLINK_HALF_MS) % 2 == 0) {
            rgb_t colors = rate_colors[(uint8_t)polling_rate_get()];
            rgb_matrix_set_color_all(colors.r, colors.g, colors.b);
        } else {
            rgb_matrix_set_color_all(0, 0, 0);
        }
    } else if (dpi_indicator_active) {
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

    /*
     * Throttle last, after every hook has had its say, because this is about
     * how often a report leaves - not about what is in it.
     *
     * This is also the only safe place to do it. pointing_device_send() looks
     * like the obvious hook and is weak, but the only way to clear the report
     * from outside is pointing_device_set_report(), whose
     * has_mouse_report_changed(&local_mouse_report, &mouse_report) passes the
     * CURRENT report as new_report - so zeroing motion sets
     * pointing_device_force_send, pointing_device_task() returns true, and
     * keyboard.c:748 fires last_pointing_device_activity_trigger() on every
     * throttled cycle. The mouse would never idle: no RGB timeout, no suspend.
     */
#ifdef POLLING_RATE_SELFTEST
    polling_rate_selftest_mark_pointing();
    mouse_report = polling_rate_selftest_inject(mouse_report);
    mouse_report = polling_rate_filter(mouse_report);
    polling_rate_selftest_count(mouse_report);
    return mouse_report;
#else
    return polling_rate_filter(mouse_report);
#endif
}

#ifdef POLLING_RATE_SELFTEST
void matrix_scan_kb(void) {
    polling_rate_selftest_mark_matrix();
    matrix_scan_user();
}

void housekeeping_task_kb(void) {
    polling_rate_selftest_mark_housekeeping();
    polling_rate_selftest_task();

    static const uint8_t dump_at_s[] = {3, 8, 15, 30};
    static uint8_t       next        = 0;

    if (next < ARRAY_SIZE(dump_at_s) && timer_read32() >= (uint32_t)dump_at_s[next] * 1000UL) {
        paw3323_debug_dump();
        next++;
    }
}
#endif

void pointing_device_init_kb(void) {
    user_config.raw = eeconfig_read_kb();

    /*
     * Was `> DPI_OPTION_SIZE`, which let the boundary value through: with seven
     * entries the valid indices are 0..6, and a stored 7 satisfies `7 > 7 ==
     * false`, so dpi_array[7] got read one past the end. `>=` is the correct
     * test.
     */
    if (user_config.dpi_config >= DPI_OPTION_SIZE) {
        eeconfig_init_kb();
    }

    pointing_device_set_cpi(dpi_array[user_config.dpi_config]);

    polling_rate_init();

    /*
     * Stored as index + 1, so that 0 means "no rate was ever saved" rather than
     * meaning 250 Hz.
     *
     * A block written by the previous firmware has 0 in this byte simply
     * because nothing wrote it. Treating that as a literal index would drop
     * every existing mouse from 8 kHz to the slowest rate on the first boot
     * after updating, which looks exactly like the update broke the sensor.
     * Offsetting
     * by one keeps "unset" and "deliberately 250 Hz" distinguishable.
     */
    if (user_config.polling_rate >= 1 && user_config.polling_rate <= PR_RATE_COUNT) {
        polling_rate_set((polling_rate_t)(user_config.polling_rate - 1));
    } else {
        user_config.polling_rate = (uint8_t)polling_rate_get() + 1;
        eeconfig_update_kb(user_config.raw);
    }
}
#endif

void eeconfig_init_kb(void) {
    user_config.dpi_config   = MOUSE_DPI_DEFAULT;
    user_config.polling_rate = POLLING_RATE_DEFAULT + 1;
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
#if DEVILS_HAND_RATE_HOLD_MS > 0
            if (record->event.pressed) {
                dpi_key_timer = timer_read();
            } else if (timer_elapsed(dpi_key_timer) >= DEVILS_HAND_RATE_HOLD_MS) {
                cycle_polling_rate();
            } else {
                cycle_dpi();
#if defined(CONSOLE_ENABLE) && defined(POINTING_DEVICE_DRIVER_paw3323)
                paw3323_debug_dump(); // what happened at boot
                paw3323_debug_snapshot();
                uint8_t pid, inv_pid; // and what the registers return right now
                bool    sig_ok = paw3323_read_signature(&pid, &inv_pid);
                uprintf("PAW3323 live: spi %s, Product_ID=0x%02X (want 0x2F), Inv_Product_ID=0x%02X (want 0xD0)\n", sig_ok ? "ok" : "FAILED", pid, inv_pid);
#endif
            }
#else
            if (record->event.pressed) {
                cycle_dpi();
#if defined(CONSOLE_ENABLE) && defined(POINTING_DEVICE_DRIVER_paw3323)
                paw3323_debug_dump(); // what happened at boot
                paw3323_debug_snapshot();
                uint8_t pid, inv_pid; // and what the registers return right now
                bool    sig_ok = paw3323_read_signature(&pid, &inv_pid);
                uprintf("PAW3323 live: spi %s, Product_ID=0x%02X (want 0x2F), Inv_Product_ID=0x%02X (want 0xD0)\n", sig_ok ? "ok" : "FAILED", pid, inv_pid);
#endif
            }
#endif

            return false;

        case PR_CFG:
            if (record->event.pressed) {
                cycle_polling_rate();
            }

            return false;

        case PR_TEST:
#ifdef POLLING_RATE_SELFTEST
            if (record->event.pressed) {
                polling_rate_selftest_toggle();
            }
#endif

            return false;
    }

    return true;
}
