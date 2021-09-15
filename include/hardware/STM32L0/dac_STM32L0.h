/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
 *   Niccolò Paolinelli <nico.paolinelli@gmail.com>
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
 * @file libohiboard/include/hardware/STM32L4/dac_STM32L0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief DAC pins and device definitions for STM32L0 series
 */

#ifndef __DAC_STM32L0_H
#define __DAC_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_DAC) & defined(LIBOHIBOARD_STM32L0)

/**
 * @addtogroup DAC
 * @{
 */

/**
 * @defgroup DAC_Hardware DAC specific hardware types
 * @{
 */

/**
 * List of possible pin where DAC channel can be used.
 */
typedef enum _Dac_Pins
{
#if defined (LIBOHIBOARD_STM32L072)

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


#if defined (LIBOHIBOARD_STM32L072)
/**
 * DAC Device Handle Number 1.
 */
extern Dac_DeviceHandle OB_DAC1;

#endif

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_DAC & LIBOHIBOARD_STM32L0

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __DAC_STM32L0_H
