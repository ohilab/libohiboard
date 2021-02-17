/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2021 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/STM32G0/gpio_STM32G0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO pins and device definitions for STM32G0 series
 */

#ifndef __GPIO_STM32G0_H
#define __GPIO_STM32G0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_STM32G0)

/**
 * @addtogroup GPIO
 * @{
 */

/**
 * @defgroup GPIO_Hardware GPIO specific hardware types
 * @{
 */

/**
 * List of possible GPIO pins.
 */
typedef enum
{
    GPIO_PINS_NONE = 0,

#if defined (LIBOHIBOARD_STM32G0x1)

#if defined (LIBOHIBOARD_STM32G031)

    GPIO_PINS_PA0,
    GPIO_PINS_PA1,
    GPIO_PINS_PA2,
#if !defined (LIBOHIBOARD_STM32G031JxM)
    GPIO_PINS_PA3,
    GPIO_PINS_PA4,
    GPIO_PINS_PA5,
    GPIO_PINS_PA6,
    GPIO_PINS_PA7,
#endif
    GPIO_PINS_PA8,
#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU) || \
    defined (LIBOHIBOARD_STM32G031KxT) || \
    defined (LIBOHIBOARD_STM32G031KxU)
    GPIO_PINS_PA9,
    GPIO_PINS_PA10,
#endif
    GPIO_PINS_PA11,
    GPIO_PINS_PA12,
    GPIO_PINS_PA13, // JTMS-SWDIO
    GPIO_PINS_PA14, // JTCK-SWCLK, BOOT0
    GPIO_PINS_PA15,

    GPIO_PINS_PB0,
    GPIO_PINS_PB1,
#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU) || \
    defined (LIBOHIBOARD_STM32G031KxT) || \
    defined (LIBOHIBOARD_STM32G031KxU) || \
    defined (LIBOHIBOARD_STM32G031FxP) || \
    defined (LIBOHIBOARD_STM32G031YxY)
    GPIO_PINS_PB2,
#endif
#if !defined (LIBOHIBOARD_STM32G031JxM)
    GPIO_PINS_PB3,
    GPIO_PINS_PB4,
#endif
    GPIO_PINS_PB5,
    GPIO_PINS_PB6,
    GPIO_PINS_PB7,
    GPIO_PINS_PB8,
#if !defined (LIBOHIBOARD_STM32G031GxU)
    GPIO_PINS_PB9,
#endif
#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU)
    GPIO_PINS_PB10,
    GPIO_PINS_PB11,
    GPIO_PINS_PB12,
    GPIO_PINS_PB13,
    GPIO_PINS_PB14,
    GPIO_PINS_PB15,
#endif

#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU) || \
    defined (LIBOHIBOARD_STM32G031KxT) || \
    defined (LIBOHIBOARD_STM32G031KxU) || \
    defined (LIBOHIBOARD_STM32G031GxU)
    GPIO_PINS_PC6,
#endif
#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU)
    GPIO_PINS_PC7,
#endif
#endif
#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU)
    GPIO_PINS_PC13,
#endif
    GPIO_PINS_PC14,
#if !defined (LIBOHIBOARD_STM32G031JxM)
    GPIO_PINS_PC15,
#endif

#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU)
    GPIO_PINS_PD0,
    GPIO_PINS_PD1,
    GPIO_PINS_PD2,
    GPIO_PINS_PD3,
#endif

#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU)
    GPIO_PINS_PF0,
    GPIO_PINS_PF1,
#endif
    GPIO_PINS_PF2,

#endif // LIBOHIBOARD_STM32G031
#endif // LIBOHIBOARD_STM32G0x1

} Gpio_Pins;

/**
 * List of possible GPIO ports.
 */
typedef enum _Gpio_Ports
{

    GPIO_PORTS_A,
    GPIO_PORTS_B,
    GPIO_PORTS_C,
    GPIO_PORTS_D,
    GPIO_PORTS_F,

    GPIO_PORTS_NUMBER,

} Gpio_Ports;

#define GPIO_MAX_PINS_NUMBER_FOR_PORT  16
#define GPIO_MAX_PORTS_NUMBER          GPIO_PORTS_NUMBER

#define GPIO_PORTS_BASE               ((GPIO_TypeDef *) GPIOA_BASE)

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_STM32G0

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __GPIO_STM32G0_H
