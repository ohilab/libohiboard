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
 * @file libohiboard/include/hardware/dac_STM32L4.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief DAC pins and device definitions for STM32L4 series
 */

#ifndef __DAC_STM32L4_H
#define __DAC_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_DAC) & defined(LIBOHIBOARD_STM32L4)

/**
 * @addtogroup DAC
 * @{
 */

/**
 * List of possible pin where DAC channel can be used.
 */
typedef enum _Dac_Pins
{
#if defined (LIBOHIBOARD_STM32L476)

    DAC_PINS_PA4,
    DAC_PINS_PA5,

#endif

} Dac_Pins;

#define DAC_CHANNEL_NUMBER_MASK     (0x000F0000)
#define DAC_CHANNEL_NUMBER_POS      (16u)
#define DAC_CHANNEL_NUMBER(CHANNEL) (((CHANNEL) & DAC_CHANNEL_NUMBER_MASK) >> DAC_CHANNEL_NUMBER_POS)

#define DAC_CHANNEL_SHIFT_MASK      (0x000000FF)

/**
 * List of possible DAC channel.
 */
typedef enum _Dac_Channels
{
    DAC_CHANNELS_CH1         = (0x00000000u),
    DAC_CHANNELS_CH2         = (0x00010010u),

} Dac_Channels;

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

extern Dac_DeviceHandle OB_DAC1;

#endif // LIBOHIBOARD_STM32L476Jx || LIBOHIBOARD_STM32L476Rx

/**
 * @}
 */

#endif // LIBOHIBOARD_DAC & LIBOHIBOARD_STM32L4

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __DAC_STM32L4_H
