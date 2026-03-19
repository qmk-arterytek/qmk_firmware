// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/* Sensor Driver */
#define SPI_DRIVER SPID1
#define SPI_SCK_PIN B3
#define SPI_MISO_PIN B4
#define SPI_MOSI_PIN B5
#define PAW3311_CS_PIN A15
#define PAW3311_SPI_DIVISOR 16
#define PAW3311_GAMING_MODE MODE_2
#define MOUSE_EXTENDED_REPORT
#define POINTING_DEVICE_DEBUG // remove when done

/* WS2812 PWM Driver */
#define WS2812_PWM_DRIVER PWMD3
#define WS2812_PWM_PAL_MODE 2
#define WS2812_PWM_CHANNEL 1
#define WS2812_PWM_DMA_STREAM AT32_DMA1_STREAM2
#define WS2812_PWM_DMAMUX_ID AT32_DMAMUX_TMR3_OVERFLOW

/* Debug Settings (remove when done)*/
#define DEBUG_ENABLE
#define DEBUG_MATRIX_SCAN_RATE
