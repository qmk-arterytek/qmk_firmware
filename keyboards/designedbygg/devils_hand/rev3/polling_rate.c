// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#include "polling_rate.h"

#include "pointing_device.h"

/* ------------------------------------------------------------------------ */
/* Time base                                                                 */
/*                                                                           */
/* Not QMK's timer_read32(). That converts ChibiOS ticks to milliseconds and  */
/* throws away everything finer, which is enough for 1000 Hz and useless for  */
/* 2000, 4000 or 8000. The ChibiOS system time underneath it is already at    */
/* CH_CFG_ST_FREQUENCY - 100000 on this platform, a 10 us tick.               */
/*                                                                           */
/* chTimeDiffX/chTimeAddX are the wrap-safe helpers, which matters because    */
/* systime_t is 32 bits at 100 kHz and rolls over every ~11.9 hours.          */
/* ------------------------------------------------------------------------ */

#if !defined(PROTOCOL_CHIBIOS)
#    error "polling_rate needs the ChibiOS system timer for its sub-millisecond rates. On another platform, supply a >=100 kHz counter here and rebuild the period table around it - QMK's millisecond timer_read32() cannot express 125/250/500 us."
#endif

#include <ch.h>

typedef systime_t     pr_time_t;
typedef sysinterval_t pr_ticks_t;

static inline pr_time_t  pr_now(void) { return chVTGetSystemTimeX(); }
static inline pr_ticks_t pr_since(pr_time_t start, pr_time_t now) { return chTimeDiffX(start, now); }
static inline pr_time_t  pr_advance(pr_time_t t, pr_ticks_t d) { return chTimeAddX(t, d); }

#define PR_TICKS_FOR(hz) ((pr_ticks_t)(CH_CFG_ST_FREQUENCY / (hz)))

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */

static const uint16_t pr_hz[PR_RATE_COUNT] = {250, 500, 1000, 2000, 4000, 8000};

/*
 * Period between releases, in system ticks.
 *
 * The top entry is 0, meaning free-running: do not gate at all. Throttling to
 * 8 kHz on an endpoint the host already polls at 8 kHz could only add jitter,
 * and 125 us is 12.5 ticks at 100 kHz - not representable, so any attempt to
 * gate it would drift. Letting the endpoint set the pace is both simpler and
 * exact.
 *
 * Nothing is lost if the main loop outruns the host. send_report() in
 * usb_main.c:393 posts to the endpoint's output queue with a 100 ms blocking
 * timeout, so once that queue fills the loop simply waits - the mouse paces
 * itself to the host's polling interval rather than discarding reports.
 *
 * Every other entry must divide the tick frequency exactly, or the rate would
 * be quietly wrong. Checked rather than assumed, because a board with a
 * different CH_CFG_ST_FREQUENCY would otherwise fail silently.
 */
static const pr_ticks_t pr_period_ticks[PR_RATE_COUNT] = {
    PR_TICKS_FOR(250),
    PR_TICKS_FOR(500),
    PR_TICKS_FOR(1000),
    PR_TICKS_FOR(2000),
    PR_TICKS_FOR(4000),
    0,
};

_Static_assert(CH_CFG_ST_FREQUENCY % 250 == 0, "tick frequency cannot express 250 Hz exactly");
_Static_assert(CH_CFG_ST_FREQUENCY % 500 == 0, "tick frequency cannot express 500 Hz exactly");
_Static_assert(CH_CFG_ST_FREQUENCY % 1000 == 0, "tick frequency cannot express 1000 Hz exactly");
_Static_assert(CH_CFG_ST_FREQUENCY % 2000 == 0, "tick frequency cannot express 2000 Hz exactly");
_Static_assert(CH_CFG_ST_FREQUENCY % 4000 == 0, "tick frequency cannot express 4000 Hz exactly");
_Static_assert(CH_CFG_ST_FREQUENCY >= 100000, "tick frequency too coarse for 4 kHz; need >= 100 kHz");
_Static_assert(POLLING_RATE_DEFAULT < PR_RATE_COUNT, "POLLING_RATE_DEFAULT is out of range");

static uint8_t   pr_rate = POLLING_RATE_DEFAULT;
static pr_time_t pr_last_release;

/*
 * Accumulators are int32 rather than the report width on purpose.
 *
 * At 250 Hz a fast flick can produce far more counts in 4 ms than a report can
 * carry. Accumulating at report width would saturate and silently throw the
 * excess away, which shows up as the cursor under-travelling on fast swipes at
 * low rates - the exact thing someone lowering the rate is not expecting.
 * Accumulating wide and carrying the remainder forward (see the release path)
 * means the counts arrive a report or two later instead of vanishing.
 */
static int32_t pr_acc_x = 0;
static int32_t pr_acc_y = 0;
static int32_t pr_acc_v = 0;
static int32_t pr_acc_h = 0;

static uint8_t pr_last_buttons = 0;

/* CONSTRAIN_HID_XY covers x/y. Wheel has its own width and its own bounds. */
#define PR_CLAMP_HV(amt) ((amt) < MOUSE_REPORT_HV_MIN ? MOUSE_REPORT_HV_MIN : ((amt) > MOUSE_REPORT_HV_MAX ? MOUSE_REPORT_HV_MAX : (amt)))

/* ------------------------------------------------------------------------ */
/* API                                                                       */
/* ------------------------------------------------------------------------ */

void polling_rate_init(void) {
    /*
     * Unconditional reset to the compiled default. This module stores nothing
     * itself - whoever owns the setting owns saving it - so "init" can only
     * mean "back to the default".
     *
     * Apply a saved value by calling polling_rate_set() after this. On Devils
     * Hand that is pointing_device_init_kb(), which reads the rate out of the
     * keyboard's existing user_config block.
     */
    pr_rate = POLLING_RATE_DEFAULT;

    pr_last_release = pr_now();
    pr_acc_x        = 0;
    pr_acc_y        = 0;
    pr_acc_v        = 0;
    pr_acc_h        = 0;
    pr_last_buttons = 0;
}

polling_rate_t polling_rate_get(void) {
    return (polling_rate_t)pr_rate;
}

uint16_t polling_rate_get_hz(void) {
    return pr_hz[pr_rate];
}

void polling_rate_set(polling_rate_t rate) {
    if (rate >= PR_RATE_COUNT || rate == (polling_rate_t)pr_rate) {
        return;
    }

    pr_rate = (uint8_t)rate;

    /*
     * Re-anchor rather than leaving a release time computed against the old
     * period. Dropping from 8000 Hz to 250 Hz with a stale anchor would
     * otherwise release one report immediately and then wait a full 4 ms.
     *
     * Accumulated motion is deliberately NOT discarded here: whatever is
     * pending belongs to the user's hand, not to the old setting, and the next
     * release will carry it out at the new rate.
     */
    pr_last_release = pr_now();
}

void polling_rate_cycle(void) {
    polling_rate_set((polling_rate_t)((pr_rate + 1) % PR_RATE_COUNT));
}

void polling_rate_step_up(void) {
    if (pr_rate + 1 < PR_RATE_COUNT) {
        polling_rate_set((polling_rate_t)(pr_rate + 1));
    }
}

void polling_rate_step_down(void) {
    if (pr_rate > 0) {
        polling_rate_set((polling_rate_t)(pr_rate - 1));
    }
}

/* ------------------------------------------------------------------------ */
/* The throttle                                                              */
/* ------------------------------------------------------------------------ */

report_mouse_t polling_rate_filter(report_mouse_t mouse_report) {
    const pr_ticks_t period = pr_period_ticks[pr_rate];
    const pr_time_t  now    = pr_now();

    /*
     * Fold this cycle's deltas in.
     *
     * Safe to do unconditionally because stock pointing_device_send() memsets
     * local_mouse_report after every call - sent or not - so x/y/v/h are always
     * this cycle's values and never last cycle's leftovers. That holds even
     * with POINTING_DEVICE_MOTION_PIN, where the driver is not called at all on
     * a quiet cycle and the fields are simply zero.
     */
    pr_acc_x += mouse_report.x;
    pr_acc_y += mouse_report.y;
    pr_acc_v += mouse_report.v;
    pr_acc_h += mouse_report.h;

    /* A button edge always goes out now. Nobody wants click latency traded for report rate. */
    const bool buttons_changed = (mouse_report.buttons != pr_last_buttons);

    /*
     * period == 0 is free-running, so every pass is due. Note this still runs
     * the accumulate-and-release path rather than short-circuiting: that way a
     * switch up from a slower rate flushes whatever carry was pending instead
     * of stranding it, and clamping stays consistent across all six rates.
     */
    const bool due = (period == 0) || (pr_since(pr_last_release, now) >= period);

    if (!due && !buttons_changed) {
        /*
         * Hold. Zeroed motion plus unchanged buttons means
         * has_mouse_report_changed() reports no change, so pointing_device_send()
         * does not transmit and returns false - which matters, because
         * keyboard.c feeds that return into last_pointing_device_activity_trigger().
         * Returning a spurious "something happened" here would keep the device
         * permanently awake and defeat every idle timeout on the board.
         */
        mouse_report.x = 0;
        mouse_report.y = 0;
        mouse_report.v = 0;
        mouse_report.h = 0;
        return mouse_report;
    }

    if (period == 0) {
        pr_last_release = now;
    } else {
        /*
         * Advance by exactly one period rather than re-anchoring to now, so the
         * rate does not drift with main-loop jitter. If we have fallen more
         * than a period behind - a long SPI read, a flash write, USB suspend -
         * resync instead of trying to catch up with a burst.
         */
        pr_last_release = pr_advance(pr_last_release, period);
        if (pr_since(pr_last_release, now) >= period) {
            pr_last_release = now;
        }
    }

    pr_last_buttons = mouse_report.buttons;

    /* Release what fits; keep the rest for the next report. */
    const mouse_xy_report_t out_x = CONSTRAIN_HID_XY(pr_acc_x);
    const mouse_xy_report_t out_y = CONSTRAIN_HID_XY(pr_acc_y);
    const mouse_hv_report_t out_v = PR_CLAMP_HV(pr_acc_v);
    const mouse_hv_report_t out_h = PR_CLAMP_HV(pr_acc_h);

    mouse_report.x = out_x;
    mouse_report.y = out_y;
    mouse_report.v = out_v;
    mouse_report.h = out_h;

    pr_acc_x -= out_x;
    pr_acc_y -= out_y;
    pr_acc_v -= out_v;
    pr_acc_h -= out_h;

    return mouse_report;
}
