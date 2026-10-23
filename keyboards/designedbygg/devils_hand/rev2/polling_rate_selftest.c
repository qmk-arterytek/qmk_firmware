// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#include "polling_rate_selftest.h"
#include "polling_rate.h"

#include "timer.h"
#include "print.h"

#if !defined(PROTOCOL_CHIBIOS)
#    error "polling_rate_selftest needs the ChibiOS system timer to resolve gaps at 8 kHz."
#endif

#include <ch.h>

/* Counts per leg. 2000 at 1600 CPI is about 1.25 inch of travel, and takes
   250 ms at 8 kHz or 8 s at 250 Hz - a difference you can see without a stopwatch. */
#ifndef PR_ST_SWEEP_COUNTS
#    define PR_ST_SWEEP_COUNTS 2000
#endif

#ifndef PR_ST_WINDOW_MS
#    define PR_ST_WINDOW_MS 1000
#endif

/* 100 kHz tick -> 10 us. Computed, not hardcoded, so a different board is right too. */
#define PR_ST_US_PER_TICK (1000000UL / CH_CFG_ST_FREQUENCY)

static bool   pr_st_on    = false;
static bool   pr_st_armed = true;
static int8_t pr_st_dir   = 1; /* +1 = rightward, -1 = leftward */

/*
 * The loop probe.
 *
 * The first window of every run injects nothing. With no motion the filter
 * releases nothing, pointing_device_send() finds no change and never calls
 * host_mouse_send(), so the loop runs completely unblocked and the pass count
 * is the loop's true ceiling.
 *
 * Without this the measurement cannot answer its own question at the top rate.
 * send_report() -> usb_endpoint_in_send() -> obqWriteTimeout() BLOCKS when the
 * endpoint queue is full (usb_driver.c:273), so free-running is supposed to be
 * paced by USB - loop/s and reports/s come out equal whether the loop is the
 * limit or the bus is. Comparing them proves nothing. Comparing the unblocked
 * loop rate against the target does.
 */
static bool     pr_st_probing   = false;
static uint32_t pr_st_loop_free = 0;

/* per-second window */
static uint32_t pr_st_passes   = 0;
static uint32_t pr_st_releases = 0;
static uint32_t pr_st_window;

/* release-to-release jitter */
static systime_t pr_st_last_release;
static bool      pr_st_have_last = false;
static uint32_t  pr_st_gap_min;
static uint32_t  pr_st_gap_max;

/* current leg */
static uint16_t  pr_st_leg_pos = 0;
static systime_t pr_st_leg_start;

/*
 * A finished leg, handed to the housekeeping task to print.
 *
 * The leg completes inside polling_rate_selftest_count(), which runs on the
 * pointing-device path up to 8000 times a second. Calling uprintf from there
 * would stall the very loop being measured and corrupt the next window's
 * numbers, so the hot path only stores and flags.
 */
static volatile bool     pr_st_leg_ready = false;
static uint32_t          pr_st_leg_ticks;
static int8_t            pr_st_leg_dir;
static uint32_t          pr_st_leg_missed = 0;

/*
 * Loop split. Two weak hooks fire once per pass, in this order:
 *   ... -> pointing_device_task_kb -> housekeeping_task_kb -> (next pass) ...
 * so arc_scan is housekeeping->pointing and arc_usb is pointing->housekeeping.
 */
static systime_t pr_st_t_mtx;
static systime_t pr_st_t_point;
static systime_t pr_st_t_house;
static bool      pr_st_have_mtx   = false;
static bool      pr_st_have_point = false;
static bool      pr_st_have_house = false;
static uint32_t  pr_st_mtx_sum  = 0, pr_st_mtx_n  = 0, pr_st_mtx_max  = 0;
static uint32_t  pr_st_scan_sum = 0, pr_st_scan_n = 0, pr_st_scan_max = 0;
static uint32_t  pr_st_usb_sum  = 0, pr_st_usb_n  = 0, pr_st_usb_max  = 0;

/* totals for the run */
static uint32_t pr_st_total_reports = 0;
static uint32_t pr_st_total_legs    = 0;
static uint32_t pr_st_start_ms      = 0;

static void pr_st_window_reset(void) {
    pr_st_mtx_sum   = 0; pr_st_mtx_n  = 0; pr_st_mtx_max  = 0;
    pr_st_scan_sum  = 0; pr_st_scan_n = 0; pr_st_scan_max = 0;
    pr_st_usb_sum   = 0; pr_st_usb_n  = 0; pr_st_usb_max  = 0;
    pr_st_have_mtx   = false;
    pr_st_have_point = false;
    pr_st_have_house = false;
    pr_st_passes    = 0;
    pr_st_releases  = 0;
    pr_st_window    = timer_read32();
    pr_st_have_last = false;
    pr_st_gap_min   = UINT32_MAX;
    pr_st_gap_max   = 0;
}

void polling_rate_selftest_toggle(void) {
    pr_st_on = !pr_st_on;

    if (pr_st_on) {
        pr_st_probing       = true;
        pr_st_loop_free     = 0;
        pr_st_armed         = true;
        pr_st_dir           = 1;
        pr_st_leg_pos       = 0;
        pr_st_leg_start     = chVTGetSystemTimeX();
        pr_st_leg_ready     = false;
        pr_st_leg_missed    = 0;
        pr_st_total_reports = 0;
        pr_st_total_legs    = 0;
        pr_st_start_ms      = timer_read32();
        pr_st_window_reset();

#ifdef CONSOLE_ENABLE
        uprintf("PR selftest: START at %u Hz, sweeping %u counts per leg\n",
                polling_rate_get_hz(), (unsigned)PR_ST_SWEEP_COUNTS);
        uprintf("PR: probing unblocked loop rate for %u ms (no reports sent)...\n",
                (unsigned)PR_ST_WINDOW_MS);
#endif
        return;
    }

#ifdef CONSOLE_ENABLE
    {
        const uint32_t run_ms = timer_elapsed32(pr_st_start_ms);

        uprintf("PR selftest: STOP after %lu.%lu s | %lu reports | %lu counts | %lu legs\n",
                (unsigned long)(run_ms / 1000), (unsigned long)((run_ms % 1000) / 100),
                (unsigned long)pr_st_total_reports,
                (unsigned long)pr_st_total_reports, /* one count per report, by construction */
                (unsigned long)pr_st_total_legs);
    }
#endif

    pr_st_window_reset();
}

bool polling_rate_selftest_active(void) {
    return pr_st_on;
}

void polling_rate_selftest_mark_matrix(void) {
    if (!pr_st_on) {
        return;
    }

    const systime_t now = chVTGetSystemTimeX();

    if (pr_st_have_house) {
        const uint32_t us = (uint32_t)chTimeDiffX(pr_st_t_house, now) * PR_ST_US_PER_TICK;
        pr_st_mtx_sum += us;
        pr_st_mtx_n++;
        if (us > pr_st_mtx_max) {
            pr_st_mtx_max = us;
        }
    }

    pr_st_t_mtx    = now;
    pr_st_have_mtx = true;
}

void polling_rate_selftest_mark_pointing(void) {
    if (!pr_st_on) {
        return;
    }

    const systime_t now = chVTGetSystemTimeX();

    /* Measure from the matrix mark when we have one, so this arc is
       quantum+rgb+encoder+sensor rather than the whole pre-pointing half. */
    if (pr_st_have_mtx) {
        const uint32_t us = (uint32_t)chTimeDiffX(pr_st_t_mtx, now) * PR_ST_US_PER_TICK;
        pr_st_scan_sum += us;
        pr_st_scan_n++;
        if (us > pr_st_scan_max) {
            pr_st_scan_max = us;
        }
    }

    pr_st_t_point    = now;
    pr_st_have_point = true;
}

void polling_rate_selftest_mark_housekeeping(void) {
    if (!pr_st_on) {
        return;
    }

    const systime_t now = chVTGetSystemTimeX();

    if (pr_st_have_point) {
        const uint32_t us = (uint32_t)chTimeDiffX(pr_st_t_point, now) * PR_ST_US_PER_TICK;
        pr_st_usb_sum += us;
        pr_st_usb_n++;
        if (us > pr_st_usb_max) {
            pr_st_usb_max = us;
        }
    }

    pr_st_t_house    = now;
    pr_st_have_house = true;
}

report_mouse_t polling_rate_selftest_inject(report_mouse_t mouse_report) {
    if (!pr_st_on) {
        return mouse_report;
    }

    /*
     * Exactly one count per RELEASE, not one per loop pass.
     *
     * polling_rate_filter accumulates between releases, so injecting on every
     * pass would put however-many-passes-fit into each report - roughly 80 at
     * 250 Hz with a 20 kHz loop. The cursor would then travel at the loop rate
     * no matter what the setting was, and a sweep would take the same wall time
     * at 250 Hz as at 8 kHz, measuring nothing.
     *
     * Arming once per release instead makes counts and reports the same number,
     * so leg time is exactly counts / rate and the sweep visibly slows down as
     * you drop the rate.
     *
     * Holding one direction for a whole leg is safe: pointing_device_send()
     * memsets local_mouse_report and only then copies it into old_report
     * (pointing_device.c:246-250), so old_report.x is always 0 and
     * has_mouse_report_changed() treats any nonzero x as a change. Repeated
     * identical reports are not suppressed.
     *
     * Replacing rather than adding keeps the sensor out of the result, which is
     * the point - this measures the firmware and the bus.
     */
    mouse_report.x = (pr_st_probing || !pr_st_armed) ? 0 : pr_st_dir;
    mouse_report.y = 0;
    mouse_report.v = 0;
    mouse_report.h = 0;
    if (!pr_st_probing) {
        pr_st_armed = false;
    }

    return mouse_report;
}

void polling_rate_selftest_count(report_mouse_t mouse_report) {
    if (!pr_st_on) {
        return;
    }

    pr_st_passes++;

    if (mouse_report.x == 0 && mouse_report.y == 0 && mouse_report.v == 0 && mouse_report.h == 0) {
        return;
    }

    pr_st_releases++;
    pr_st_total_reports++;
    pr_st_armed = true;

    const systime_t now = chVTGetSystemTimeX();

    if (pr_st_have_last) {
        const uint32_t gap_us = (uint32_t)chTimeDiffX(pr_st_last_release, now) * PR_ST_US_PER_TICK;

        if (gap_us < pr_st_gap_min) {
            pr_st_gap_min = gap_us;
        }
        if (gap_us > pr_st_gap_max) {
            pr_st_gap_max = gap_us;
        }
    }

    pr_st_last_release = now;
    pr_st_have_last    = true;

    /* Leg complete? Turn around and hand the timing to the printer. */
    if (++pr_st_leg_pos < PR_ST_SWEEP_COUNTS) {
        return;
    }

    if (pr_st_leg_ready) {
        /* The previous leg has not been printed yet. Count it rather than
           overwrite silently - if this is ever nonzero the window is too short
           or housekeeping is starved, and the printed rates are suspect. */
        pr_st_leg_missed++;
    }

    pr_st_leg_ticks = (uint32_t)chTimeDiffX(pr_st_leg_start, now);
    pr_st_leg_dir   = pr_st_dir;
    pr_st_leg_ready = true;

    pr_st_total_legs++;
    pr_st_dir       = (int8_t)-pr_st_dir;
    pr_st_leg_pos   = 0;
    pr_st_leg_start = now;
}

void polling_rate_selftest_task(void) {
    if (!pr_st_on) {
        return;
    }

#ifdef CONSOLE_ENABLE
    if (pr_st_leg_ready) {
        const uint32_t ticks = pr_st_leg_ticks;
        const uint32_t ms    = (ticks * PR_ST_US_PER_TICK) / 1000UL;
        /* counts * tick_hz / ticks, exact in integers: 2000 * 100000 fits in u32 */
        const uint32_t rate = ticks ? ((uint32_t)PR_ST_SWEEP_COUNTS * CH_CFG_ST_FREQUENCY) / ticks : 0;

        uprintf("PR: sweep %s %u counts in %lu ms -> %lu reports/s\n",
                pr_st_leg_dir > 0 ? "-->" : "<--",
                (unsigned)PR_ST_SWEEP_COUNTS,
                (unsigned long)ms,
                (unsigned long)rate);

        if (pr_st_leg_missed) {
            uprintf("    ^ %lu leg(s) finished before this printed - timings are suspect.\n",
                    (unsigned long)pr_st_leg_missed);
            pr_st_leg_missed = 0;
        }

        pr_st_leg_ready = false;
    }
#endif

    const uint32_t elapsed = timer_elapsed32(pr_st_window);

    if (elapsed < PR_ST_WINDOW_MS) {
        return;
    }

    if (pr_st_probing) {
        pr_st_loop_free = (pr_st_passes * 1000UL) / elapsed;

#ifdef CONSOLE_ENABLE
        uprintf("PR: loop probe -> %lu passes/s unblocked (ceiling for any rate)\n",
                (unsigned long)pr_st_loop_free);
#endif

        pr_st_probing   = false;
        pr_st_armed     = true;
        pr_st_leg_pos   = 0;
        pr_st_leg_start = chVTGetSystemTimeX();
        pr_st_leg_ready = false;
        pr_st_window_reset();
        return;
    }

#ifdef CONSOLE_ENABLE
    /*
     * Scale to per-second rather than assuming the window landed exactly on
     * 1000 ms - housekeeping runs when it runs, and at 8 kHz a 3 ms overshoot
     * is 24 reports.
     */
    const uint32_t reports_s = (pr_st_releases * 1000UL) / elapsed;
    const uint32_t loop_s    = (pr_st_passes * 1000UL) / elapsed;

    if (pr_st_scan_n && pr_st_usb_n) {
        uprintf("PR: arc mtx %lu (max %lu) | rgb+enc+sensor %lu (max %lu) | usb+tail %lu (max %lu) us\n",
                (unsigned long)(pr_st_mtx_n ? pr_st_mtx_sum / pr_st_mtx_n : 0), (unsigned long)pr_st_mtx_max,
                (unsigned long)(pr_st_scan_sum / pr_st_scan_n), (unsigned long)pr_st_scan_max,
                (unsigned long)(pr_st_usb_sum / pr_st_usb_n), (unsigned long)pr_st_usb_max);
    }

    uprintf("PR: set %u Hz | reports %lu/s | loop %lu/s | gap %lu..%lu us\n",
            polling_rate_get_hz(),
            (unsigned long)reports_s,
            (unsigned long)loop_s,
            (unsigned long)(pr_st_gap_min == UINT32_MAX ? 0 : pr_st_gap_min),
            (unsigned long)pr_st_gap_max);

    /*
     * Blame the loop or the bus - using the PROBE, not loop_s.
     *
     * loop_s is useless for this at the top rate: the loop blocks inside
     * send_report() waiting for the endpoint queue to drain, so loop_s tracks
     * reports_s no matter which one is the real constraint. pr_st_loop_free was
     * measured with nothing being sent, so it is the only number here that says
     * what the loop can actually do.
     */
    const uint32_t target = polling_rate_get_hz();

    if (reports_s < target * 9 / 10) {
        if (pr_st_loop_free == 0) {
            uprintf("    ^ short of target; no loop probe yet.\n");
        } else if (pr_st_loop_free < target) {
            uprintf("    ^ LOOP-BOUND: unblocked loop is %lu/s, below the %lu/s asked for.\n"
                    "      Speed the loop up; the bus is not the problem.\n",
                    (unsigned long)pr_st_loop_free, (unsigned long)target);
        } else {
            uprintf("    ^ BUS-BOUND: loop can do %lu/s unblocked but only %lu reports/s land.\n"
                    "      Check enumerated speed (480M?) and bInterval (1?), and the hub chain.\n",
                    (unsigned long)pr_st_loop_free, (unsigned long)reports_s);
        }
    }
#endif

    pr_st_window_reset();
}
