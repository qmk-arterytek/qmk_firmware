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

#ifndef PAW3311_CPI
#    define PAW3311_CPI 3000
#endif

#define LENS_1 0
#define LENS_2 1

typedef enum {
    THT_LENS = LENS_1, // LM31-LNG, LM33-LSG, L0AL-LSG1
    SMD_LENS = LENS_2  // L0AJ-LSG1
} paw3311_lens_t;

#define MODE_1 0
#define MODE_2 1

typedef enum {
    LPM    = MODE_1, // Low power mode
    CORDED = MODE_2  // Corded mode
} paw3311_mode_t;

// PAW3311DB-T2MU register addresses
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
#define REG_Minimum_RawData      0x0A
#define REG_Shutter_Lower        0x0B
#define REG_Shutter_Upper        0x0C
#define REG_Chip_Observation     0x15
#define REG_Burst_Motion_Read    0x16
#define REG_Power_Up_Reset       0x3A
#define REG_Shutdown             0x3B
#define REG_Performance          0x40
#define REG_Resolution           0x4E
#define REG_Angle_Snap           0x56
#define REG_RawData_Grab         0x58
#define REG_RawData_Grab_Status  0x59
#define REG_Ripple_Control       0x5A
#define REG_Axis_Control         0x5B
#define REG_Inv_Product_ID       0x5F
#define REG_Run_DownShift        0x77
#define REG_Rest1_Period         0x78
#define REG_Rest1_Downshift      0x79
#define REG_Rest2_Period         0x7A
#define REG_Rest2_Downshift      0x7B
#define REG_Rest3_Period         0x7C
#define REG_Run_Downshift_Mult   0x7D
#define REG_Rest_Downshift_Mult  0x7E

// Not listed in Table 20; only used by the 7.1.2 init flow (steps 116-119)
#define REG_Undocumented_19      0x19
#define REG_Undocumented_20      0x20
// clang-format on

typedef struct {
    int16_t dx;
    int16_t dy;
} report_paw3311_t;

/* Init report kept in RAM so it can be read after USB/console is up */
typedef enum {
    PAW3311_STAGE_NOT_RUN = 0,
    PAW3311_STAGE_RESET,      // 7.1.1 step 4: Power_Up_Reset
    PAW3311_STAGE_SIGNATURE,  // 7.1.1 step 6: Product_ID check
    PAW3311_STAGE_EARLY_INIT,
    PAW3311_STAGE_PRE_INIT,
    PAW3311_STAGE_LENS,
    PAW3311_STAGE_INIT_SEQ,   // 7.1.2 steps 1-116
    PAW3311_STAGE_READY_POLL, // 7.1.2 step 117: poll 0x20 for 0x0F
    PAW3311_STAGE_POST_INIT,  // 7.1.2 steps 118-126
    PAW3311_STAGE_MODE_CPI,   // operation mode tables + CPI registers
    PAW3311_STAGE_DISCARD,    // 7.1.1 step 8: discard reads
    PAW3311_STAGE_DONE,
} paw3311_init_stage_t;

typedef struct {
    paw3311_init_stage_t stage;      // last stage entered; PAW3311_STAGE_DONE on success
    bool                 ok;         // paw3311_init() returned true
    uint16_t             init_count; // times paw3311_init() has run since boot
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
} paw3311_debug_t;

extern const pointing_device_driver_t paw3311_pointing_device_driver;

bool                  paw3311_init(void);
report_paw3311_t      paw3311_read_burst(void);
void                  paw3311_set_cpi(uint16_t cpi);
uint16_t              paw3311_get_cpi(void);
report_mouse_t        paw3311_get_report(report_mouse_t mouse_report);
bool                  paw3311_check_signature(void);
bool                  paw3311_read_signature(uint8_t *product_id, uint8_t *inv_product_id);
const paw3311_debug_t *paw3311_get_debug(void);
void                  paw3311_debug_dump(void);

#define PAW3311_CPI_STEP 50
#define PAW3311_CPI_MIN 50
#define PAW3311_CPI_MAX 10000
#define PAW3311_CPI_EXT_STEP 100
#define PAW3311_CPI_EXT_MIN 10100
#define PAW3311_CPI_EXT_MAX 12000
#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
