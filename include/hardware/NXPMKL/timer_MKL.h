/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/timer_MKL.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Timer hardware definitions for NXP MKL series
 */

#ifndef __TIMER_MKL_H
#define __TIMER_MKL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_MKL) && defined (LIBOHIBOARD_TIMER)

/**
 * List of possible Timer channels.
 */
typedef enum _Timer_Channels
{
    TIMER_CHANNELS_CH0,
    TIMER_CHANNELS_CH1,
    TIMER_CHANNELS_CH2,
    TIMER_CHANNELS_CH3,
    TIMER_CHANNELS_CH4,
    TIMER_CHANNELS_CH5,

    TIMER_CHANNELS_NUMBER,

} Timer_Channels;

/**
 * List of possible Timer pins.
 */
typedef enum _Timer_Pins
{
#if defined (LIBOHIBOARD_MKL15)

    TIMER_PINS_PTA0,
    TIMER_PINS_PTA1,
    TIMER_PINS_PTA2,
    TIMER_PINS_PTA3,
    TIMER_PINS_PTA4,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    TIMER_PINS_PTA5,
    TIMER_PINS_PTA12,
    TIMER_PINS_PTA13,
#endif

    TIMER_PINS_PTB0,
    TIMER_PINS_PTB1,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    TIMER_PINS_PTB2,
    TIMER_PINS_PTB3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    TIMER_PINS_PTB18,
    TIMER_PINS_PTB19,
#endif

    TIMER_PINS_PTC1,
    TIMER_PINS_PTC2,
    TIMER_PINS_PTC3,
    TIMER_PINS_PTC4,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    TIMER_PINS_PTC8,
    TIMER_PINS_PTC9,
#endif

#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    TIMER_PINS_PTD0,
    TIMER_PINS_PTD1,
    TIMER_PINS_PTD2,
    TIMER_PINS_PTD3,
#endif
    TIMER_PINS_PTD4,
    TIMER_PINS_PTD5,

#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    TIMER_PINS_PTE20,
    TIMER_PINS_PTE21,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    TIMER_PINS_PTE22,
    TIMER_PINS_PTE23,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    TIMER_PINS_PTE24,
    TIMER_PINS_PTE25,
    TIMER_PINS_PTE29,
#endif
    TIMER_PINS_PTE30,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    TIMER_PINS_PTE31,
#endif

#endif // LIBOHIBOARD_MKL15

    TIMER_PINS_NUMBER,
    TIMER_PINS_NONE = 0xFFFF,

} Timer_Pins;

#if defined (LIBOHIBOARD_MKL15) || \
    defined (LIBOHIBOARD_MKL25)
extern Timer_DeviceHandle OB_TIM0;
extern Timer_DeviceHandle OB_TIM1;
extern Timer_DeviceHandle OB_TIM2;

void TPM0_DriverIRQHandler (void);
void TPM1_DriverIRQHandler (void);
void TPM2_DriverIRQHandler (void);
#endif

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // __TIMER_MKL_H
