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
 * @file libohiboard/include/hardware/gpio_STM32L4.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO pins and device definitions for STM32L4 series
 */

#ifndef __GPIO_STM32L4_H
#define __GPIO_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_STM32L4)

typedef enum
{
    GPIO_PINS_NONE = 0,

#if defined (LIBOHIBOARD_STM32L476)

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

    GPIO_PINS_PA0,
    GPIO_PINS_PA1,
    GPIO_PINS_PA2,
    GPIO_PINS_PA3,
    GPIO_PINS_PA4,
    GPIO_PINS_PA5,
    GPIO_PINS_PA6,
    GPIO_PINS_PA7,
    GPIO_PINS_PA8,
    GPIO_PINS_PA9,
    GPIO_PINS_PA10,
    GPIO_PINS_PA11,
    GPIO_PINS_PA12,
    GPIO_PINS_PA13, // JTMS-SWDIO
    GPIO_PINS_PA14, // JTCK-SWCLK
    GPIO_PINS_PA15, // JTDI

    GPIO_PINS_PB0,
    GPIO_PINS_PB1,
    GPIO_PINS_PB2,
    GPIO_PINS_PB3, // JTDO-TRACE SWO
    GPIO_PINS_PB4, // NJTRST
    GPIO_PINS_PB5,
    GPIO_PINS_PB6,
    GPIO_PINS_PB7,
    GPIO_PINS_PB8,
    GPIO_PINS_PB9,
    GPIO_PINS_PB10,
    GPIO_PINS_PB11,
    GPIO_PINS_PB12,
    GPIO_PINS_PB13,
    GPIO_PINS_PB14,
    GPIO_PINS_PB15,

    GPIO_PINS_PC0,
    GPIO_PINS_PC1,
    GPIO_PINS_PC2,
    GPIO_PINS_PC3,
    GPIO_PINS_PC4,
    GPIO_PINS_PC5,
    GPIO_PINS_PC6,
    GPIO_PINS_PC7,
    GPIO_PINS_PC8,
    GPIO_PINS_PC9,
    GPIO_PINS_PC10,
    GPIO_PINS_PC11,
    GPIO_PINS_PC12,
    GPIO_PINS_PC13,
    GPIO_PINS_PC14,
    GPIO_PINS_PC15,

    GPIO_PINS_PD2,

#if defined (LIBOHIBOARD_STM32L476Jx)
    GPIO_PINS_PG9,
    GPIO_PINS_PG10,
    GPIO_PINS_PG11,
    GPIO_PINS_PG12,
    GPIO_PINS_PG13,
    GPIO_PINS_PG14,
#endif

    GPIO_PINS_PH0,
    GPIO_PINS_PH1,

#endif

#endif // LIBOHIBOARD_STM32L476

} Gpio_Pins;

/**
 * List of possible GPIO ports.
 */
typedef enum
{
    GPIO_PORTS_A,
    GPIO_PORTS_B,
    GPIO_PORTS_C,
    GPIO_PORTS_D,
    GPIO_PORTS_E,
    GPIO_PORTS_F,
    GPIO_PORTS_G,
    GPIO_PORTS_H,

} Gpio_Ports;

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // __GPIO_STM32L4_H
