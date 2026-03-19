// Copyright 2026 HorrorTroll <https://github.com/HorrorTroll>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include_next <mcuconf.h>

#undef AT32_PWM_USE_TMR3
#define AT32_PWM_USE_TMR3 TRUE

#undef AT32_SPI_USE_SPI1
#define AT32_SPI_USE_SPI1 TRUE

#undef AT32_PLL_NS_VALUE
#define AT32_PLL_NS_VALUE 64

#undef AT32_PLL_FP_VALUE
#define AT32_PLL_FP_VALUE 6

#undef AT32_PLL_FU_VALUE
#define AT32_PLL_FU_VALUE 16
