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
 * @file libohiboard/include/hardware/PIC24FJ/timer_PIC24FJ.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Timer useful definitions for PIC24FJ series
 */

#ifndef __TIMER_PIC24FJ_H
#define __TIMER_PIC24FJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_TIMER) && defined (LIBOHIBOARD_PIC24FJ)

/**
 *
 */
typedef enum _Timer_Channels
{
    TIMER_CHANNELS_NONE = 0,

    TIMER_CHANNELS_CH1  = 1u,
    TIMER_CHANNELS_CH2  = 2u,
    TIMER_CHANNELS_CH3  = 3u,
    TIMER_CHANNELS_CH4  = 4u,
    TIMER_CHANNELS_CH5  = 5u,
    TIMER_CHANNELS_CH6  = 6u,

} Timer_Channels;

/**
 * 
 */
typedef enum _Timer_Pins
{
    TIMER_PINS_NONE  = -1,

    TIMER_PINS_RP0  = 0,  /**< RB0 */
    TIMER_PINS_RP1  = 1,  /**< RB1 */
    TIMER_PINS_RP2  = 2,  /**< RD8 */    
    TIMER_PINS_RP3  = 3,  /**< RD10 */
    TIMER_PINS_RP4  = 4,  /**< RD9 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    TIMER_PINS_RP5  = 5,  /**< RD15 */        
#endif
    TIMER_PINS_RP6  = 6,  /**< RB6 */
    TIMER_PINS_RP7  = 7,  /**< RB7 */
    TIMER_PINS_RP8  = 8,  /**< RB8 */
    TIMER_PINS_RP9  = 9,  /**< RB9 */
    TIMER_PINS_RP10 = 10, /**< RF4 */
    TIMER_PINS_RP11 = 11, /**< RD0 */
    TIMER_PINS_RP12 = 12, /**< RD11 */
    TIMER_PINS_RP13 = 13, /**< RB2 */
    TIMER_PINS_RP14 = 14, /**< RB14 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    TIMER_PINS_RP15 = 15, /**< RF8 */        
#endif
    TIMER_PINS_RP16 = 16, /**< RF3 */
    TIMER_PINS_RP17 = 17, /**< RF5 */
    TIMER_PINS_RP18 = 18, /**< RB5 */
    TIMER_PINS_RP19 = 19, /**< RG8 */
    TIMER_PINS_RP20 = 20, /**< RD5 */
    TIMER_PINS_RP21 = 21, /**< RG6 */
    TIMER_PINS_RP22 = 22, /**< RD3 */
    TIMER_PINS_RP23 = 23, /**< RD2 */
    TIMER_PINS_RP24 = 24, /**< RD1 */
    TIMER_PINS_RP25 = 25, /**< RD4 */
    TIMER_PINS_RP26 = 26, /**< RG7 */ 
    TIMER_PINS_RP27 = 27, /**< RG9 */
    TIMER_PINS_RP28 = 28, /**< RB4 */
    TIMER_PINS_RP29 = 29, /**< RB15 */
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    TIMER_PINS_RP30 = 30, /**< RF2 */
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    TIMER_PINS_RP31 = 31, /**< RF13 */        
#endif
    
    TIMER_PINS_PD6,
    TIMER_PINS_PD7,
    TIMER_PINS_PF0,

} Timer_Pins;


extern Timer_DeviceHandle OB_TIM23;
extern Timer_DeviceHandle OB_TIM45;

extern Timer_DeviceHandle OB_TIM2;
extern Timer_DeviceHandle OB_TIM3;
extern Timer_DeviceHandle OB_TIM4;
extern Timer_DeviceHandle OB_TIM5;

extern Timer_DeviceHandle OB_TIMPWM;

#endif // LIBOHIBOARD_TIMER && LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // __TIMER_PIC24FJ_H
