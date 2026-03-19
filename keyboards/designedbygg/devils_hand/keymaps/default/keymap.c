// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

// Each layer gets a name for readability, which is then used in the keymap matrix below.
// The underscores don't mean anything - you can have a layer called STUFF or any other name.
// Layer names don't all need to be of the same length, obviously, and you can also skip them
// entirely and just use numbers.

enum layer_names {
    _BASE,
    _FN
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

/*
       ┌───┐  ┌───┐  ┌───┐
       │LMB│  │MMB│  │RMB│
       │   │  └───┘  │   │
       └───┘  ┌───┐  └───┘
       ┌───┐  │DPI│
	   │SB1│  └───┘
       ├───┤
       │SB2│
       └───┘
*/
    /*  Row:    0        1        2        3        4        5      */
    [_BASE] = LAYOUT(
                MS_BTN1, MS_BTN3, MS_BTN2, MS_BTN5, DPI_CFG, MS_BTN4
            ),

/*
       ┌───┐  ┌───┐  ┌───┐
       │   │  │   │  │   │
       │   │  └───┘  │   │
       └───┘  ┌───┐  └───┘
       ┌───┐  │   │
	   │   │  └───┘
       ├───┤
       │   │
       └───┘
*/
    /*  Row:    0        1        2        3        4        5      */
    [_FN] = LAYOUT(
                _______, _______, _______, _______, _______, _______
            )
};

#ifdef ENCODER_MAP_ENABLE
const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
    [_BASE] = { ENCODER_CCW_CW(MS_WHLD, MS_WHLU) },
    [_FN]   = { ENCODER_CCW_CW(_______, _______) },
};
#endif

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
#ifdef CONSOLE_ENABLE
    uprintf("KL: kc: 0x%04X, col: %2u, row: %2u, pressed: %u, time: %5u, int: %u, count: %u\n", keycode, record->event.key.col, record->event.key.row, record->event.pressed, record->event.time, record->tap.interrupted, record->tap.count);
#endif
    return true;
}
