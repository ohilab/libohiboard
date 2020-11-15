/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018-2020 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/STM32L4/timer_STM32L4.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Timer useful definitions for STM32L4 and STM32WB series
 */

#ifndef __TIMER_STM32L4_H
#define __TIMER_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if (defined (LIBOHIBOARD_STM32L4)  || \
     defined (LIBOHIBOARD_STM32WB)) && \
     defined (LIBOHIBOARD_TIMER)

typedef enum _Timer_Channels
{
    TIMER_CHANNELS_CH1 = 0x00u,
    TIMER_CHANNELS_CH2 = 0x04u,
    TIMER_CHANNELS_CH3 = 0x08u,
    TIMER_CHANNELS_CH4 = 0x0Cu,
    TIMER_CHANNELS_CH5 = 0x10u,
    TIMER_CHANNELS_CH6 = 0x14u,

} Timer_Channels;

typedef enum
{
#if defined (LIBOHIBOARD_STM32L4x6) ||\
    defined (LIBOHIBOARD_STM32WB55)

#if defined (LIBOHIBOARD_STM32L476) ||\
    defined (LIBOHIBOARD_STM32WB55)

    TIMER_PINS_PA0,
    TIMER_PINS_PA1,
    TIMER_PINS_PA2,
    TIMER_PINS_PA3,
    TIMER_PINS_PA5,
    TIMER_PINS_PA6,
    TIMER_PINS_PA7,
    TIMER_PINS_PA8,
    TIMER_PINS_PA9,
    TIMER_PINS_PA10,
    TIMER_PINS_PA11,
    TIMER_PINS_PA15,

#if !defined (LIBOHIBOARD_STM32WB55)
    TIMER_PINS_PB0,
    TIMER_PINS_PB1,
#endif
    TIMER_PINS_PB3,
#if !defined (LIBOHIBOARD_STM32WB55)
    TIMER_PINS_PB4,
    TIMER_PINS_PB5,
#endif
    TIMER_PINS_PB6,
    TIMER_PINS_PB7,
    TIMER_PINS_PB8,
    TIMER_PINS_PB9,
    TIMER_PINS_PB10,
    TIMER_PINS_PB11,
    TIMER_PINS_PB13,
    TIMER_PINS_PB14,
    TIMER_PINS_PB15,

#if !defined (LIBOHIBOARD_STM32WB55)
    TIMER_PINS_PC6,
    TIMER_PINS_PC7,
    TIMER_PINS_PC8,
    TIMER_PINS_PC9,
#endif

#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
    TIMER_PINS_PD12,
    TIMER_PINS_PD13,
    TIMER_PINS_PD14,
    TIMER_PINS_PD15,
#endif

#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
    TIMER_PINS_PE2,
    TIMER_PINS_PE3,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ) || \
    defined (LIBOHIBOARD_STM32WB55Rx)
    TIMER_PINS_PE4,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
    TIMER_PINS_PE5,
    TIMER_PINS_PE6,
#endif
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
    TIMER_PINS_PE8,
#endif
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
    TIMER_PINS_PE9,
    TIMER_PINS_PE10,
    TIMER_PINS_PE11,
    TIMER_PINS_PE12,
    TIMER_PINS_PE13,
    TIMER_PINS_PE14,
#endif

#if defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
    TIMER_PINS_PF6,
    TIMER_PINS_PF7,
    TIMER_PINS_PF8,
    TIMER_PINS_PF9,
    TIMER_PINS_PF10,
#endif

#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
    TIMER_PINS_PG9,
    TIMER_PINS_PG10,
    TIMER_PINS_PG11,
#endif

#endif

#endif

    TIMER_PINS_NONE,

} Timer_Pins;


#if defined (LIBOHIBOARD_STM32L4x6) ||\
    defined (LIBOHIBOARD_STM32WB55)

extern Timer_DeviceHandle OB_TIM1;
extern Timer_DeviceHandle OB_TIM2;

#if !defined (LIBOHIBOARD_STM32WB55)
extern Timer_DeviceHandle OB_TIM3;
extern Timer_DeviceHandle OB_TIM4;
extern Timer_DeviceHandle OB_TIM5;
extern Timer_DeviceHandle OB_TIM6;
extern Timer_DeviceHandle OB_TIM7;
extern Timer_DeviceHandle OB_TIM8;
extern Timer_DeviceHandle OB_TIM15;
#endif

extern Timer_DeviceHandle OB_TIM16;
extern Timer_DeviceHandle OB_TIM17;

#if defined (LIBOHIBOARD_STM32L4x6)
void TIM1_BRK_TIM15_IRQHandler (void);
#elif defined (LIBOHIBOARD_STM32WB55)
void TIM1_BRK_IRQHandler (void);
#endif

void TIM1_UP_TIM16_IRQHandler (void);
void TIM1_TRG_COM_TIM17_IRQHandler (void);
void TIM1_CC_IRQHandler (void);
void TIM2_IRQHandler (void);

#if !defined (LIBOHIBOARD_STM32WB55)
void TIM3_IRQHandler (void);
void TIM4_IRQHandler (void);
void TIM5_IRQHandler (void);
void TIM6_DAC_IRQHandler (void);
void TIM7_IRQHandler (void);
void TIM8_BRK_IRQHandler (void);
void TIM8_UP_IRQHandler (void);
void TIM8_TRG_COM_IRQHandler (void);
void TIM8_CC_IRQHandler (void);
#endif

#endif

#endif // LIBOHIBOARD_STM32L4-WB && LIBOHIBOARD_TIMER

#ifdef __cplusplus
}
#endif

#endif // __TIMER_STM32L4_H
