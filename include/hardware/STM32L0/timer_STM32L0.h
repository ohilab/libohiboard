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
 * @file libohiboard/include/hardware/STM32L0/timer_STM32L0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Timer useful definitions for STM32L0 series
 */

#ifndef __TIMER_STM32L0_H
#define __TIMER_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L0) && \
    defined (LIBOHIBOARD_TIMER)

typedef enum _Timer_Channels
{
    TIMER_CHANNELS_CH1  = 0x00u,
    TIMER_CHANNELS_CH2  = 0x04u,
    TIMER_CHANNELS_CH3  = 0x08u,
    TIMER_CHANNELS_CH4  = 0x0Cu,

    TIMER_CHANNELS_NONE = 0xFFu,

} Timer_Channels;

typedef enum
{
    TIMER_PINS_NONE,

#if defined (LIBOHIBOARD_STM32L0x1)

#if defined (LIBOHIBOARD_STM32L081)
    TIMER_PINS_PA0,
    TIMER_PINS_PA1,
    TIMER_PINS_PA2,
    TIMER_PINS_PA3,
    TIMER_PINS_PA5,
    TIMER_PINS_PA6,
    TIMER_PINS_PA7,
#if defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    TIMER_PINS_PA15,
#endif

    TIMER_PINS_PB0,
    TIMER_PINS_PB1,
#if defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    TIMER_PINS_PB3,
#endif
    TIMER_PINS_PB4,
    TIMER_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    TIMER_PINS_PB10,
    TIMER_PINS_PB11,
    TIMER_PINS_PB13,
    TIMER_PINS_PB14,
#endif

#endif // LIBOHIBOARD_STM32L081

#elif defined (LIBOHIBOARD_STM32L0x2)

#if defined (LIBOHIBOARD_STM32L072)

    TIMER_PINS_PA0,
    TIMER_PINS_PA1,
    TIMER_PINS_PA2,
    TIMER_PINS_PA3,
    TIMER_PINS_PA5,
    TIMER_PINS_PA6,
    TIMER_PINS_PA7,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    TIMER_PINS_PA15,
#endif

    TIMER_PINS_PB0,
    TIMER_PINS_PB1,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    TIMER_PINS_PB3,
#endif
    TIMER_PINS_PB4,
    TIMER_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    TIMER_PINS_PB10,
    TIMER_PINS_PB11,
    TIMER_PINS_PB13,
    TIMER_PINS_PB14,
#endif

#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    TIMER_PINS_PC6,
    TIMER_PINS_PC7,
    TIMER_PINS_PC8,
    TIMER_PINS_PC9,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    TIMER_PINS_PD0,
    TIMER_PINS_PD7,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    TIMER_PINS_PE3,
    TIMER_PINS_PE4,
    TIMER_PINS_PE5,
    TIMER_PINS_PE6,
    TIMER_PINS_PE9,
    TIMER_PINS_PE10,
    TIMER_PINS_PE11,
    TIMER_PINS_PE12,
#endif

#endif // LIBOHIBOARD_STM32L072

#endif

    TIMER_PINS_NUMBER,

} Timer_Pins;

#if defined (LIBOHIBOARD_STM32L0x1)

extern Timer_DeviceHandle OB_TIM2;
extern Timer_DeviceHandle OB_TIM3;
extern Timer_DeviceHandle OB_TIM6;
extern Timer_DeviceHandle OB_TIM7;
extern Timer_DeviceHandle OB_TIM21;
extern Timer_DeviceHandle OB_TIM22;

void TIM2_IRQHandler (void);
void TIM3_IRQHandler (void);
void TIM6_IRQHandler (void);
void TIM7_IRQHandler (void);
void TIM21_IRQHandler (void);
void TIM22_IRQHandler (void);

#elif defined (LIBOHIBOARD_STM32L0x2)

extern Timer_DeviceHandle OB_TIM2;
extern Timer_DeviceHandle OB_TIM3;
extern Timer_DeviceHandle OB_TIM6;
extern Timer_DeviceHandle OB_TIM7;
extern Timer_DeviceHandle OB_TIM21;
extern Timer_DeviceHandle OB_TIM22;

void TIM2_IRQHandler (void);
void TIM3_IRQHandler (void);
void TIM6_DAC_IRQHandler (void);
void TIM7_IRQHandler (void);
void TIM21_IRQHandler (void);
void TIM22_IRQHandler (void);

#endif

#endif // LIBOHIBOARD_STM32L0 && LIBOHIBOARD_TIMER

#ifdef __cplusplus
}
#endif

#endif // __TIMER_STM32L4_H
