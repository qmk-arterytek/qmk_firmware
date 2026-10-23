// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Runtime-switchable mouse report rate for QMK: 250 Hz to 8 kHz.
 *
 * No reflash, no re-enumeration, no USB dropout, and no edits to the QMK tree -
 * this hooks the weak pointing_device_task_user()/_kb() extension point.
 *
 * HOW THE RATES ARE REACHED
 *
 * The endpoint enumerates once, at whatever bInterval USB_POLLING_INTERVAL_MS
 * sets, and never changes. On a high-speed device with the default of 1 that is
 * a 125 us interval - the host asks 8000 times a second and keeps asking.
 *
 * What this module changes is whether a report is *ready* when the host asks.
 * Below the top rate the endpoint NAKs on the polls in between. Every tool that
 * measures a mouse counts delivered reports, so they read the throttled figure.
 * The top rate is free-running: nothing is gated, and the endpoint's own
 * interval sets the pace.
 *
 * Two things therefore cap the real ceiling, neither of them here:
 *
 *   1. The enumerated bInterval. Leave USB_POLLING_INTERVAL_MS at 1 for 8 kHz.
 *      On a high-speed interrupt endpoint the value is an exponent, not a
 *      count of milliseconds - see the table below - so raising it costs more
 *      than it looks like it should.
 *
 *   2. The main loop. A report can only go out on a pass of
 *      pointing_device_task(), so 8 kHz needs the loop running faster than
 *      8 kHz, sensor read included. DEBUG_MATRIX_SCAN_RATE will tell you what
 *      you actually have.
 *
 * BINTERVAL, FOR REFERENCE
 *
 * QMK writes USB_POLLING_INTERVAL_MS straight into bInterval with no awareness
 * of speed (usb_descriptor.c:615), and LUFA names the field PollingIntervalMS
 * because LUFA is full-speed-centric. The host applies the encoding, and per
 * USB 2.0 section 9.6.6 a high-speed interrupt endpoint's bInterval is an
 * exponent: the period is 2^(bInterval-1) microframes of 125 us each.
 *
 *      bInterval   high speed    full speed
 *          1        8000 Hz       1000 Hz
 *          2        4000 Hz        500 Hz
 *          3        2000 Hz        333 Hz
 *          4        1000 Hz        250 Hz
 *          5         500 Hz        200 Hz
 *          6         250 Hz        167 Hz
 *          7         125 Hz        143 Hz
 *          8        62.5 Hz        125 Hz
 *
 * Full speed is the linear one: bInterval is a plain frame count, 1 to 255.
 *
 * WHICH SPEED YOU GET ON AT32
 *
 * hal_usb_lld.c picks DCFG_DEVSPD_HS only when BOARD_OTG2_USES_ULPI is defined.
 * For AT32F405 that comes from the QMK board file, not from the HAL port:
 *
 *      platforms/chibios/boards/GENERIC_AT32_F405XX/board/board.h:40
 *          #define BOARD_OTG2_USES_ULPI
 *
 * with AT32_USB_USE_OTG2 TRUE and USB_DRIVER USBD2 in that board's mcuconf.h
 * and config.h, and AT32_USE_USB_OTG2_HS defaulting TRUE and never overridden.
 * So that board is high speed and the left-hand column applies.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "report.h"

/* ------------------------------------------------------------------------ */
/* Rates                                                                     */
/* ------------------------------------------------------------------------ */

typedef enum {
    PR_RATE_250HZ = 0,
    PR_RATE_500HZ,
    PR_RATE_1000HZ,
    PR_RATE_2000HZ,
    PR_RATE_4000HZ,
    PR_RATE_8000HZ,
    PR_RATE_COUNT
} polling_rate_t;

/*
 * Default at power-on. 8 kHz, matching what the board already does before this
 * module is added - installing it should not quietly slow the mouse down.
 */
#ifndef POLLING_RATE_DEFAULT
#    define POLLING_RATE_DEFAULT PR_RATE_8000HZ
#endif

/*
 * This module stores nothing.
 *
 * The rate is deliberately not persisted here, because on a real board it
 * belongs next to that board's other settings rather than in a region of its
 * own - two separate blocks for one mouse's config is how they end up sitting
 * on top of each other. Keep it wherever DPI already lives and call
 * polling_rate_set() once you have read it back.
 *
 * Devils Hand does exactly that: the rate is one byte in the keyboard's
 * existing user_config union, written by eeconfig_update_kb() alongside
 * dpi_config.
 */

/* ------------------------------------------------------------------------ */
/* API                                                                       */
/* ------------------------------------------------------------------------ */

/**
 * @brief Reset to POLLING_RATE_DEFAULT and re-anchor the release schedule.
 *
 * Call from keyboard_post_init_user() or pointing_device_init_kb(), then
 * polling_rate_set() if you have a saved value to apply.
 */
void polling_rate_init(void);

/**
 * @brief The throttle itself. Call from pointing_device_task_user()/_kb().
 *
 * Accumulates x/y/v/h across skipped cycles so no motion is lost, and releases
 * the total on the cycle the period expires. Button changes flush immediately
 * and are never delayed, so click latency is unaffected by the rate setting.
 *
 * At the top rate nothing is gated and reports pass straight through.
 *
 * @param mouse_report the report as handed to pointing_device_task_user()
 * @return the report to pass on - zeroed motion when throttling, accumulated
 *         motion when releasing
 */
report_mouse_t polling_rate_filter(report_mouse_t mouse_report);

/** @brief Current rate as an enum index. */
polling_rate_t polling_rate_get(void);

/** @brief Current rate in Hz: 250, 500, 1000, 2000, 4000 or 8000. */
uint16_t polling_rate_get_hz(void);

/**
 * @brief Set the rate. Out-of-range values and no-op changes are ignored.
 *
 * Does not save anything - persist it yourself if it should survive a power
 * cycle.
 */
void polling_rate_set(polling_rate_t rate);

/** @brief 250 -> 500 -> 1000 -> 2000 -> 4000 -> 8000 -> 250. */
void polling_rate_cycle(void);

/** @brief One step up, stopping at 8000 Hz. */
void polling_rate_step_up(void);

/** @brief One step down, stopping at 250 Hz. */
void polling_rate_step_down(void);
