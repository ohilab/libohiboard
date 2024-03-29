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
 * @file libohiboard/include/hardware/STM32L0/lowpower-timer_STM32L0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Low-Power Timer useful definitions for STM32L0 series
 */

#ifndef __LOWPOWER_TIMER_STM32L0_H
#define __LOWPOWER_TIMER_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L0) && defined (LIBOHIBOARD_LOWPOWER_TIMER)

#if defined (LIBOHIBOARD_STM32L0x1) || \
    defined (LIBOHIBOARD_STM32L0x2) || \
    defined (LIBOHIBOARD_STM32L0x3)

extern LowPowerTimer_DeviceHandle OB_LPTIM1;

void LPTIM1_IRQHandler (void);

#endif // LIBOHIBOARD_STM32L0x1 || LIBOHIBOARD_STM32L0x2 || LIBOHIBOARD_STM32L0x3

#endif // LIBOHIBOARD_STM32L0 && LIBOHIBOARD_LOWPOWER_TIMER

#ifdef __cplusplus
}
#endif

#endif // __LOWPOWER_TIMER_STM32L0_H
