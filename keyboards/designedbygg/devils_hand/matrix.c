// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#include "matrix.h"

static const pin_t direct_pins[MATRIX_ROWS][MATRIX_COLS] = DIRECT_PINS;

static inline uint8_t readMatrixPin(pin_t pin) {
    if (pin == F11) {
        return (gpio_read_pin(pin) == 1) ? 0 : 1;
    } else if (pin != NO_PIN) {
        return (gpio_read_pin(pin) == 0) ? 0 : 1;
    } else {
        return 1;
    }
}

// matrix code

__attribute__((weak)) void matrix_init_custom(void) {
    for (int row = 0; row < MATRIX_ROWS; row++) {
        for (int col = 0; col < MATRIX_COLS; col++) {
            pin_t pin = direct_pins[row][col];
            if (pin == F11) {
                gpio_set_pin_input_low(pin);
            } else if (pin != NO_PIN) {
                gpio_set_pin_input_high(pin);
            }
        }
    }
}

__attribute__((weak)) bool matrix_read_cols_on_row(matrix_row_t current_matrix[], uint8_t current_row) {
    // Start with a clear matrix row
    matrix_row_t last_row_value = current_matrix[current_row];
    current_matrix[current_row] = 0;

    matrix_row_t row_shifter = MATRIX_ROW_SHIFTER;
    for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++, row_shifter <<= 1) {
        pin_t pin = direct_pins[current_row][col_index];
        current_matrix[current_row] |= readMatrixPin(pin) ? 0 : row_shifter;
    }

    return (last_row_value != current_matrix[current_row]);
}

bool matrix_scan_custom(matrix_row_t current_matrix[]) {
    bool matrix_has_changed = false;

    // Set row, read cols
    for (uint8_t current_row = 0; current_row < MATRIX_ROWS; current_row++) {
        matrix_has_changed |= matrix_read_cols_on_row(current_matrix, current_row);
    }

    return matrix_has_changed;
}
