// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Firmware-side measurement of the actual report rate.
 *
 * WHY BOTHER, WHEN HOST TOOLS EXIST
 *
 * Host tools tell you what arrived. They cannot tell you *why* a number is low,
 * and at these rates that is the whole question. If you ask for 8 kHz and
 * measure 3 kHz, the cause is one of:
 *
 *   - the main loop cannot go faster (sensor read, RGB, matrix scan)
 *   - the host is not actually polling at 125 us (wrong bInterval, or the
 *     device enumerated at full speed without anyone noticing)
 *   - the measurement is wrong
 *
 * This module separates the first from the rest with a LOOP PROBE: the first
 * second of every run injects no motion at all, so nothing is sent and the loop
 * runs unblocked. That pass count is the loop's true ceiling.
 *
 * The probe is necessary, not decorative. Comparing loop/s against reports/s
 * during normal measurement proves nothing at the top rate, because
 * send_report() -> usb_endpoint_in_send() -> obqWriteTimeout() blocks when the
 * endpoint queue is full (usb_driver.c:273). Free-running is *supposed* to be
 * paced by USB, so loop/s equals reports/s whether the loop or the bus is the
 * constraint. Only the unblocked probe rate answers the question.
 *
 * It also reports the min and max gap between releases, because a mean of
 * 8000/s hides a lot. A clean 8 kHz shows min and max both near 125 us; one
 * that stalls periodically shows a max in the milliseconds.
 *
 * WHAT THE COUNT ACTUALLY MEANS
 *
 * It counts reports *released* by polling_rate_filter, not reports the host
 * acknowledged. Those converge, and not by luck: send_report() posts to the
 * endpoint queue with a 100 ms blocking timeout (usb_main.c:393), so once the
 * queue is full the loop waits. Sustained release rate cannot exceed the rate
 * the host drains the queue. Over a one-second window the two agree.
 *
 * Where they would NOT agree is a short burst into an empty queue, which is
 * why the window is a full second and why min/max gap is reported too.
 *
 * SYNTHETIC MOTION - A VISIBLE SWEEP
 *
 * A still mouse sends nothing, so measuring by hand means waving it about for
 * the whole window and hoping you were consistent. Instead the self-test drives
 * the cursor itself: right for PR_ST_SWEEP_COUNTS counts, then left, then right
 * again, one count per report.
 *
 * One count per report is what makes the sweep a measurement rather than a
 * light show. Counts and reports are then the same number, so
 *
 *      leg time = counts / rate
 *
 * and the rate follows from a stopwatch. 2000 counts takes 250 ms at 8 kHz and
 * 8 s at 250 Hz - you can watch the cursor crawl when you drop the rate, and
 * the printed leg timing gives you the number without one.
 *
 * It also makes the measurement independent of the sensor, which separates "my
 * firmware cannot emit 8 kHz" from "my sensor cannot report 8 kHz".
 *
 * BUILD
 *
 *     make <kb>:<km> POLLING_RATE_SELFTEST=yes
 *
 * Needs CONSOLE_ENABLE. Read it with `qmk console`. Costs nothing when the
 * flag is absent - the whole thing compiles out.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "report.h"

/** @brief Start/stop measuring. Resets the counters on start. */
void polling_rate_selftest_toggle(void);

/** @brief Whether measurement is currently running. */
bool polling_rate_selftest_active(void);

/**
 * @brief Replace the report with one count of sweep motion.
 *
 * Call immediately BEFORE polling_rate_filter(). Emits a single +/-1 on the
 * first pass after each release and zero in between, so exactly one count
 * reaches every report whatever the rate. Returns the report untouched when
 * not active.
 */
report_mouse_t polling_rate_selftest_inject(report_mouse_t mouse_report);

/**
 * @brief Count one loop pass, and one release if the report carries motion.
 *
 * Call immediately AFTER polling_rate_filter(), with what it returned.
 */
void polling_rate_selftest_count(report_mouse_t mouse_report);

/**
 * @brief Split the loop in two, to find where the time goes.
 *
 * Call mark_matrix() from matrix_scan_kb(), mark_pointing() at the top of
 * pointing_device_task_kb(), and mark_housekeeping() at the top of
 * housekeeping_task_kb(). All three weak hooks run exactly once per main-loop
 * pass, in that order - matrix_scan_kb() unconditionally from matrix_scan()
 * on a non-split board (matrix_common.c:171) - so the arcs between them cover
 * the whole loop:
 *
 *   MTX arc   (housekeeping -> matrix)   = matrix_scan_custom + debounce
 *   REST arc  (matrix -> pointing)      = quantum_task, rgb_matrix_task,
 *                                          encoder_task, and the
 *                                          pointing-device driver read
 *   USB arc   (pointing -> housekeeping) = the report send, oled_task,
 *                                          raw_hid_task, console_task,
 *                                          deferred_exec_task
 *
 * Neither is free to guess at: the matrix here is six GPIO reads with no
 * matrix_io_delay(), and rgb_matrix only flushes every RGB_MATRIX_LED_FLUSH_LIMIT
 * ms, so the obvious suspects are not obviously guilty. Measure instead.
 */
void polling_rate_selftest_mark_matrix(void);
void polling_rate_selftest_mark_pointing(void);
void polling_rate_selftest_mark_housekeeping(void);

/**
 * @brief Print the per-second stats, and each sweep leg as it completes.
 *
 * Call from housekeeping_task_kb() or housekeeping_task_user(). Legs are timed
 * in the hot path but printed here - uprintf at 8 kHz would stall the loop
 * being measured.
 */
void polling_rate_selftest_task(void);
