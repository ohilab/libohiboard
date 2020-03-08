/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/include/hardware/PIC24FJ/lowpower-timer_PIC24FJ.h
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief Low-Power Timer useful definitions for PIC24FJ
 */

#ifndef __LOWPOWER_TIMER_PIC24FJ_H
#define __LOWPOWER_TIMER_PIC24FJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_LOWPOWER_TIMER) && defined(LIBOHIBOARD_PIC24FJ)

/**
 * @addtogroup LOWPOWER TIMER
 * @{
 */

/**
 * @defgroup LOWPOWER TIMER_Hardware LOWPOWER TIMER specific hardware types
 * @{
 */

void __attribute__ ( ( interrupt, no_auto_psv ) ) _T1Interrupt (  );

#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)

extern LowPowerTimer_DeviceHandle OB_LPTIM1;

#endif

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_LOWPOWER_TIMER && LIBOHIBOARD_PIC24FJ

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __LOWPOWER_TIMER_PIC24FJ_H
