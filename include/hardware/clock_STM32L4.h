/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/clock_STM32L4.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Clock useful definitions for STM32L4 series
 */

#ifndef __CLOCK_STM32L4_H
#define __CLOCK_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_STM32L4)

#if defined (LIBOHIBOARD_STM32L476)

#define CLOCK_MIN_FREQ_HSE                     4000000u
#define CLOCK_MAX_FREQ_HSE                    48000000u
#define CLOCK_MAX_FREQ_PLL                    80000000u
#define CLOCK_FREQ_HSI                        16000000u
#define CLOCK_FREQ_LSE                ((uint32_t)32768u)
#define CLOCK_FREQ_LSI                ((uint32_t)32000u)

#endif

#define CLOCK_ENABLE_SYSCFG() do { \
                                UTILITY_SET_REGISTER_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN); \
                                asm("nop"); \
                                (void) UTILITY_READ_REGISTER_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN); \
                              } while (0)

#define CLOCK_DISABLE_SYSCFG()do { \
                                UTILITY_CLEAR_REGISTER_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN); \
                              } while (0)

#define CLOCK_ENABLE_PWR()    do { \
                                UTILITY_SET_REGISTER_BIT(RCC->APB1ENR1,RCC_APB1ENR1_PWREN); \
                                asm("nop"); \
                                (void) UTILITY_READ_REGISTER_BIT(RCC->APB1ENR1,RCC_APB1ENR1_PWREN); \
                              } while (0)

#define CLOCK_DISABLE_PWR()   do { \
                                UTILITY_CLEAR_REGISTER_BIT(RCC->APB1ENR1,RCC_APB1ENR1_PWREN); \
                              } while (0)

#define CLOCK_IS_ENABLE_PWR() ((UTILITY_READ_REGISTER_BIT(RCC->APB1ENR1,RCC_APB1ENR1_PWREN) == 0) ? FALSE : TRUE)

#define CLOCK_BACKUP_DISABLE_WRITE_PROTECTION() UTILITY_SET_REGISTER_BIT(PWR->CR1,PWR_CR1_DBP)

#define CLOCK_BACKUP_ENABLE_WRITE_PROTECTION() UTILITY_CLEAR_REGISTER_BIT(PWR->CR1,PWR_CR1_DBP)

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // __CLOCK_STM32L4_H
