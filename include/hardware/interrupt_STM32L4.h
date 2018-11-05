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
 * @file libohiboard/include/hardware/interrupt_STM32L4.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief INTERRUPT vector definitions for STM32L4 series
 */

#ifndef __INTERRUPT_STM32L4_H
#define __INTERRUPT_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_STM32L4)

typedef enum  _Interrupt_Vector
{

    INTERRUPT_SYSTICK          = -1,
    INTERRUPT_WWDG             = 0,
    INTERRUPT_RTC_TAMP_STAMP   = 2,
    INTERRUPT_RTC_WKUP         = 3,
    INTERRUPT_EXTI0            = 6,
    INTERRUPT_EXTI1            = 7,
    INTERRUPT_EXTI2            = 8,
    INTERRUPT_EXTI3            = 9,
    INTERRUPT_EXTI4            = 10,

    INTERRUPT_EXTI9_5          = 23,

    INTERRUPT_UART1            = 37,
    INTERRUPT_UART2            = 38,
    INTERRUPT_UART3            = 39,
    INTERRUPT_EXTI15_10        = 40,

    INTERRUPT_UART4            = 52,
    INTERRUPT_UART5            = 53,

    INTERRUPT_LPUART1          = 70,

    // TODO: Add all interrupts

} Interrupt_Vector;

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // __INTERRUPT_STM32L4_H
