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
 * @file libohiboard/include/hardware/interrupt_MKL.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief INTERRUPT vector definitions for NXP MKL series
 */

#ifndef __INTERRUPT_MKL_H
#define __INTERRUPT_MKL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_MKL)

typedef enum  _Interrupt_Vector
{

    INTERRUPT_SYSTICK    = -1,

    INTERRUPT_DMA0       = 0,
    INTERRUPT_DMA1       = 1,
    INTERRUPT_DMA2       = 2,
    INTERRUPT_DMA3       = 3,
    INTERRUPT_FTFA       = 5,
    INTERRUPT_PMC        = 6,
    INTERRUPT_LLWU       = 7,
    INTERRUPT_IIC0       = 8,
    INTERRUPT_IIC1       = 9,
    INTERRUPT_SPI0       = 10,
    INTERRUPT_SPI1       = 11,
    INTERRUPT_UART0      = 12,
    INTERRUPT_UART1      = 13,
    INTERRUPT_UART2      = 14,
    INTERRUPT_ADC0       = 15,
    INTERRUPT_CMP0       = 16,
    INTERRUPT_TPM0       = 17,
    INTERRUPT_TPM1       = 18,
    INTERRUPT_TPM2       = 19,
    INTERRUPT_RTC_ALARM  = 20,
    INTERRUPT_RTC_SECOND = 21,
    INTERRUPT_PIT        = 22,
    INTERRUPT_DAC0       = 25,
    INTERRUPT_TSI0       = 26,
    INTERRUPT_MCG        = 27,
    INTERRUPT_LPTMR0     = 28,
    INTERRUPT_PORTA      = 30,
    INTERRUPT_PORTD      = 31,

} Interrupt_Vector;

/**
 * List of all possible value for interrupt priority.
 */
typedef enum  _Interrupt_Priority
{
    INTERRUPT_PRIORITY_0 = 0x03,
    INTERRUPT_PRIORITY_1 = 0x02,
    INTERRUPT_PRIORITY_2 = 0x01,
    INTERRUPT_PRIORITY_3 = 0x00,

} Interrupt_Priority;

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // __INTERRUPT_MKL_H
