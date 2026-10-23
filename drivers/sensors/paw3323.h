// Copyright 2021 Colin Lam (Ploopy Corporation)
// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "pointing_device.h"

#ifndef PAW3323_CS_PIN
#    ifdef POINTING_DEVICE_CS_PIN
#        define PAW3323_CS_PIN POINTING_DEVICE_CS_PIN
#    else
#        error "No chip select pin defined -- missing POINTING_DEVICE_CS_PIN or PAW3323_CS_PIN define"
#    endif
#endif

#ifndef PAW3323_SPI_DIVISOR
#    error "No PAW3323 SPI divisor defined -- missing PAW3323_SPI_DIVISOR"
#endif

#ifndef PAW3323_INIT_REG_SET
#    define PAW3323_INIT_REG_SET LENS_2
#endif

#ifndef PAW3323_GAMING_MODE
#    define PAW3323_GAMING_MODE MODE_1
#endif

#ifndef PAW3323_CPI
#    define PAW3323_CPI 3000
#endif

#define LENS_1 0 /* L0AL-LSG1  (Resolution Setting 1) */
#define LENS_2 1 /* L0AJ-LSG1  (Resolution Setting 2) */
#define LENS_3 2 /* LM31-LNG, LM33-LSG (Resolution Setting 3) */

typedef enum {
    SMD_L0AL      = LENS_1, // L0AL-LSG1
    SMD_L0AJ      = LENS_2, // L0AJ-LSG1
    THT_LM31_LM33 = LENS_3, // LM31-LNG, LM33-LSG
} paw3323_lens_t;

#define MODE_1 0
#define MODE_2 1
#define MODE_3 2

typedef enum {
    LPM    = MODE_1, // Low power mode
    CORDED = MODE_2, // Corded mode
    OFFICE = MODE_3  // Office mode
} paw3323_mode_t;

// PAW3323DB-T4MU register addresses (datasheet v0.8, Table 20)
// clang-format off
#define REG_Product_ID           0x00
#define REG_Motion               0x02
#define REG_Delta_X_L            0x03
#define REG_Delta_X_H            0x04
#define REG_Delta_Y_L            0x05
#define REG_Delta_Y_H            0x06
#define REG_SQUAL                0x07
#define REG_Raw_Data_Sum         0x08
#define REG_Maximum_RawData      0x09
#define REG_Minimum_RawData      0x0a
#define REG_Shutter_Lower        0x0b
#define REG_Shutter_Upper        0x0c
#define REG_Chip_Observation     0x15
#define REG_Burst_Motion_Read    0x16
#define REG_Power_Up_Reset       0x3a
#define REG_Shutdown             0x3b
#define REG_Performance          0x40
#define REG_Res_XY_High          0x4a
#define REG_Res_Y_Low            0x4b
#define REG_Res_X_Low            0x4e
#define REG_Angle_Snap           0x56
#define REG_RawData_Grab         0x58
#define REG_RawData_Grab_Status  0x59
#define REG_Ripple_Control       0x5a
#define REG_Axis_Control         0x5b
#define REG_Inv_Product_ID       0x5f
#define REG_Run_DownShift        0x77
#define REG_Rest1_Period         0x78
#define REG_Rest1_Downshift      0x79
#define REG_Rest2_Period         0x7a
#define REG_Rest2_Downshift      0x7b
#define REG_Rest3_Period         0x7c
#define REG_Run_Downshift_Mult   0x7d
#define REG_Rest_Downshift_Mult  0x7e

// Not listed in Table 20; only used by the 7.1.2 init flow (steps 116-119)
#define REG_Undocumented_19      0x19
#define REG_Undocumented_20      0x20
// clang-format on

typedef struct {
    int16_t dx;
    int16_t dy;
} report_paw3323_t;

/* Init report kept in RAM so it can be read after USB/console is up */
typedef enum {
    PAW3323_STAGE_NOT_RUN = 0,
    PAW3323_STAGE_RESET,      // 7.1.1 step 4: Power_Up_Reset
    PAW3323_STAGE_SIGNATURE,  // 7.1.1 step 6: Product_ID check
    PAW3323_STAGE_INIT_SEQ,   // 7.1.2 steps 1-116
    PAW3323_STAGE_READY_POLL, // 7.1.2 step 117: poll 0x20 for 0x0F
    PAW3323_STAGE_POST_INIT,  // 7.1.2 steps 118-126
    PAW3323_STAGE_MODE_CPI,   // operation mode tables + CPI registers
    PAW3323_STAGE_DISCARD,    // 7.1.1 step 8: discard reads
    PAW3323_STAGE_DONE,
} paw3323_init_stage_t;

typedef struct {
    paw3323_init_stage_t stage;      // last stage entered; PAW3323_STAGE_DONE on success
    bool                 ok;         // paw3323_init() returned true
    uint16_t             init_count; // times paw3323_init() has run since boot
    uint32_t             t_start_ms; // timer_read32() when init started / ended
    uint32_t             t_end_ms;
    uint8_t              pid;        // raw Product_ID read during init
    uint8_t              inv_pid;    // raw Inv_Product_ID read during init
    bool                 sig_spi_ok; // both signature reads completed at the SPI level
    uint16_t             fail_index; // entry of the failing sequence, 0xFFFF = none
    uint8_t              fail_reg;   // register of the failing access
    uint8_t              fail_val;   // value being written (writes only)
    int16_t              spi_status; // SPI status code of the failing access, 0 = none
    uint8_t              ready_polls;// reads of the ready register (0x20)
    uint8_t              ready_last; // last value read from it (want 0x0F)
    bool                 ready_ok;   // 0x0F seen within the 55 polls
    // Motion polling statistics since init, filled by the normal get_report path
    uint32_t             polls;      // Motion polls that completed
    uint32_t             mot_hits;   // ... of which had the MOT bit set
    uint32_t             spi_errors; // Motion polls where the SPI read failed
    uint32_t             opmode[4];  // polls seen in Run / Rest1 / Rest2 / Rest3 (Motion[1:0])
    int32_t              sum_dx;     // accumulated counts returned to QMK
    int32_t              sum_dy;
    uint8_t              last_motion;// last raw Motion register value
    uint8_t              exp_res_hi; // resolution registers as last written (0x4A / 0x4E / 0x4B)
    uint8_t              exp_res_x;
    uint8_t              exp_res_y;
} paw3323_debug_t;

extern const pointing_device_driver_t paw3323_pointing_device_driver;

bool             paw3323_init(void);
report_paw3323_t paw3323_read_burst(void);
void             paw3323_set_cpi(uint16_t cpi);
uint16_t         paw3323_get_cpi(void);
report_mouse_t   paw3323_get_report(report_mouse_t mouse_report);
bool             paw3323_check_signature(void);
bool             paw3323_read_signature(uint8_t *product_id, uint8_t *inv_product_id);
const paw3323_debug_t *paw3323_get_debug(void);
void             paw3323_debug_dump(void);
void             paw3323_debug_snapshot(void);

#define PAW3323_CPI_STEP 50
#define PAW3323_CPI_MIN 50
#define PAW3323_CPI_MAX 10000
#define PAW3323_CPI_EXT_STEP 50
#define PAW3323_CPI_EXT_MIN 10050
#define PAW3323_CPI_EXT_MAX 18000
#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
