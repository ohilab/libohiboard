/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2020-2021 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/STM32G0/interrupt_STM32G0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief INTERRUPT vector definitions for STM32G0 series
 */

#ifndef __INTERRUPT_STM32G0_H
#define __INTERRUPT_STM32G0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_STM32G0)

typedef enum  _Interrupt_Vector
{

    INTERRUPT_SYSTICK          = -1,
    INTERRUPT_WWDG             = 0,
    INTERRUPT_PWD              = 1,
    INTERRUPT_RTC              = 2,
    INTERRUPT_FLASH            = 3,
    INTERRUPT_RCC_CRS          = 4,
    INTERRUPT_EXTI1_0          = 5,
    INTERRUPT_EXTI3_2          = 6,
    INTERRUPT_EXTI15_4         = 7,
//    INTERRUPT_TSC              = 8,
//    INTERRUPT_DMA1_CH1         = 9,
//    INTERRUPT_DMA1_CH3_2       = 10,
//    INTERRUPT_DMA1_CH7_4       = 11,
//    INTERRUPT_ADC_COMP         = 12,
//    INTERRUPT_LPTIM1           = 13,
//    INTERRUPT_USART4_5         = 14,
//    INTERRUPT_TIM2             = 15,
//    INTERRUPT_TIM3             = 16,
//    INTERRUPT_TIM6_DAC         = 17,
//    INTERRUPT_TIM7             = 18,
////    INTERRUPT_RESERVED         = 19,
//    INTERRUPT_TIM21            = 20,
//    INTERRUPT_I2C3             = 21,
//    INTERRUPT_TIM22            = 22,
//    INTERRUPT_I2C1             = 23,
//    INTERRUPT_I2C2             = 24,
//    INTERRUPT_SPI1             = 25,
//    INTERRUPT_SPI2             = 26,
//    INTERRUPT_USART1           = 27,
//    INTERRUPT_USART2           = 28,
//    INTERRUPT_LPUART1          = 29,
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

#endif // LIBOHIBOARD_STM32G0

#ifdef __cplusplus
}
#endif

#endif // __INTERRUPT_STM32G0_H
