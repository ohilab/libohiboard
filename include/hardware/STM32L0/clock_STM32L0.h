/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019-2021 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file libohiboard/include/hardware/STM32L0/clock_STM32L0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Clock useful definitions for STM32L0 series
 */

#ifndef __CLOCK_STM32L0_H
#define __CLOCK_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_STM32L0)

#if defined (LIBOHIBOARD_STM32L0x1)

#define CLOCK_MAX_FREQ_SYSCLK                 32000000u

#define CLOCK_STARTUP_FREQ                     2097000u

#define CLOCK_MIN_FREQ_MSI                       65536u
#define CLOCK_MAX_FREQ_MSI                     4194000u
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
#define CLOCK_MIN_FREQ_HSE                     1000000u
#define CLOCK_MAX_FREQ_HSE                    24000000u
#define CLOCK_MAX_FREQ_EXTERNAL_CLOCK_HSE     32000000u
#endif
#define CLOCK_MIN_INPUT_FREQ_PLL               2000000u
#define CLOCK_MAX_INPUT_FREQ_PLL              24000000u
#define CLOCK_MAX_FREQ_PLL                    96000000u
#define CLOCK_FREQ_HSI                        16000000u
#define CLOCK_FREQ_LSE               ((uint32_t)32768u)
#define CLOCK_FREQ_LSI               ((uint32_t)37000u)

// Useful timeout
#define CLOCK_MSI_TIMEOUT_VALUE                    (2u)
#define CLOCK_HSI_TIMEOUT_VALUE                    (2u)
#define CLOCK_LSI_TIMEOUT_VALUE                    (2u)
#define CLOCK_LSE_TIMEOUT_VALUE                 (5000u)
#define CLOCK_WRITE_ACCESS_TIMEOUT_VALUE         (100u)
#define CLOCK_PLL_TIMEOUT_VALUE                    (2u)
#define CLOCK_SOURCE_SWITCH_TIMEOUT_VALUE       (5000u)

#define CLOCK_VOLTAGERANGE1_MAX_FREQ_WO_WAIT (16000000u)
#define CLOCK_VOLTAGERANGE2_MAX_FREQ_WO_WAIT  (8000000u)
#define CLOCK_VOLTAGERANGE3_MAX_FREQ_WO_WAIT  (4200000u)

// Useful default calibration value
#define CLOCK_MSI_DEFAULT_TRIM_VALUE ((uint32_t)0x00000000u)
#define CLOCK_HSI_DEFAULT_TRIM_VALUE ((uint32_t)0x00001000u)

#elif defined (LIBOHIBOARD_STM32L0x2)

#define CLOCK_MAX_FREQ_SYSCLK                 32000000u

#define CLOCK_STARTUP_FREQ                     2097000u

#define CLOCK_MIN_FREQ_MSI                       65536u
#define CLOCK_MAX_FREQ_MSI                     4194000u
#define CLOCK_MIN_FREQ_HSE                     1000000u
#define CLOCK_MAX_FREQ_HSE                    24000000u
#define CLOCK_MAX_FREQ_EXTERNAL_CLOCK_HSE     32000000u
#define CLOCK_MIN_INPUT_FREQ_PLL               2000000u
#define CLOCK_MAX_INPUT_FREQ_PLL              24000000u
#define CLOCK_MAX_FREQ_PLL                    96000000u
#define CLOCK_FREQ_HSI                        16000000u
#define CLOCK_FREQ_LSE               ((uint32_t)32768u)
#define CLOCK_FREQ_LSI               ((uint32_t)37000u)

// Useful timeout
#define CLOCK_MSI_TIMEOUT_VALUE                    (2u)
#define CLOCK_HSI_TIMEOUT_VALUE                    (2u)
#define CLOCK_LSI_TIMEOUT_VALUE                    (2u)
#define CLOCK_LSE_TIMEOUT_VALUE                 (5000u)
#define CLOCK_WRITE_ACCESS_TIMEOUT_VALUE         (100u)
#define CLOCK_PLL_TIMEOUT_VALUE                    (2u)
#define CLOCK_SOURCE_SWITCH_TIMEOUT_VALUE       (5000u)

#define CLOCK_VOLTAGERANGE1_MAX_FREQ_WO_WAIT (16000000u)
#define CLOCK_VOLTAGERANGE2_MAX_FREQ_WO_WAIT  (8000000u)
#define CLOCK_VOLTAGERANGE3_MAX_FREQ_WO_WAIT  (4200000u)

// Useful default calibration value
#define CLOCK_MSI_DEFAULT_TRIM_VALUE ((uint32_t)0x00000000u)
#define CLOCK_HSI_DEFAULT_TRIM_VALUE ((uint32_t)0x00001000u)

#elif defined (LIBOHIBOARD_STM32L0x3)

#define CLOCK_MAX_FREQ_SYSCLK                 32000000u

#define CLOCK_STARTUP_FREQ                     2097000u

#define CLOCK_MIN_FREQ_MSI                       65536u
#define CLOCK_MAX_FREQ_MSI                     4194000u
#define CLOCK_MIN_FREQ_HSE                     1000000u
#define CLOCK_MAX_FREQ_HSE                    24000000u
#define CLOCK_MIN_INPUT_FREQ_PLL               2000000u
#define CLOCK_MAX_INPUT_FREQ_PLL              24000000u
#define CLOCK_MAX_FREQ_PLL                    96000000u
#define CLOCK_FREQ_HSI                        16000000u
#define CLOCK_FREQ_LSE               ((uint32_t)32768u)
#define CLOCK_FREQ_LSI               ((uint32_t)37000u)

#define CLOCK_VOLTAGERANGE1_MAX_FREQ_WO_WAIT (16000000u)
#define CLOCK_VOLTAGERANGE2_MAX_FREQ_WO_WAIT  (8000000u)
#define CLOCK_VOLTAGERANGE3_MAX_FREQ_WO_WAIT  (4200000u)

// Useful timeout
#define CLOCK_MSI_TIMEOUT_VALUE                    (2u)
#define CLOCK_HSI_TIMEOUT_VALUE                    (2u)
#define CLOCK_LSI_TIMEOUT_VALUE                    (2u)
#define CLOCK_LSE_TIMEOUT_VALUE                 (5000u)
#define CLOCK_WRITE_ACCESS_TIMEOUT_VALUE         (100u)
#define CLOCK_PLL_TIMEOUT_VALUE                    (2u)
#define CLOCK_SOURCE_SWITCH_TIMEOUT_VALUE       (5000u)

// Useful default calibration value
#define CLOCK_MSI_DEFAULT_TRIM_VALUE ((uint32_t)0x00000000u)
#define CLOCK_HSI_DEFAULT_TRIM_VALUE ((uint32_t)0x00001000u)

#endif

#define CLOCK_ENABLE_SYSCFG() do { \
                                UTILITY_SET_REGISTER_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN); \
                                asm("nop"); \
                                (void) UTILITY_READ_REGISTER_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN); \
                              } while (0)

#define CLOCK_DISABLE_SYSCFG() do { \
                                 UTILITY_CLEAR_REGISTER_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN); \
                               } while (0)

#define CLOCK_ENABLE_PWR()    do { \
                                UTILITY_SET_REGISTER_BIT(RCC->APB1ENR,RCC_APB1ENR_PWREN); \
                                asm("nop"); \
                                (void) UTILITY_READ_REGISTER_BIT(RCC->APB1ENR,RCC_APB1ENR_PWREN); \
                              } while (0)

#define CLOCK_DISABLE_PWR()   do { \
                                UTILITY_CLEAR_REGISTER_BIT(RCC->APB1ENR,RCC_APB1ENR_PWREN); \
                              } while (0)

#define CLOCK_IS_ENABLE_PWR() ((UTILITY_READ_REGISTER_BIT(RCC->APB1ENR,RCC_APB1ENR_PWREN) == 0) ? FALSE : TRUE)

#define CLOCK_BACKUP_DISABLE_WRITE_ACCESS() (UTILITY_CLEAR_REGISTER_BIT(PWR->CR,PWR_CR_DBP))

#define CLOCK_BACKUP_ENABLE_WRITE_ACCESS() (UTILITY_SET_REGISTER_BIT(PWR->CR,PWR_CR_DBP))

#define CLOCK_BACKUP_IS_ENABLE_WRITE_ACCESS() ((UTILITY_READ_REGISTER_BIT(PWR->CR,PWR_CR_DBP) == 0) ? FALSE : TRUE)

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // __CLOCK_STM32L4_H
