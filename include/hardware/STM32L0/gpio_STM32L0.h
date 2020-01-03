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
 * @file libohiboard/include/hardware/STM32L0/gpio_STM32L0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO pins and device definitions for STM32L4 series
 */

#ifndef __GPIO_STM32L0_H
#define __GPIO_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_STM32L0)

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

#if defined (LIBOHIBOARD_STM32L0x2)

#if defined (LIBOHIBOARD_STM32L072)

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
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PA15,
#endif

    GPIO_PINS_PB0,
    GPIO_PINS_PB1,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PB2,
#endif
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PB3,
#endif
    GPIO_PINS_PB4,
    GPIO_PINS_PB5,
    GPIO_PINS_PB6,
    GPIO_PINS_PB7,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PB8,
    GPIO_PINS_PB9,
    GPIO_PINS_PB10,
    GPIO_PINS_PB11,
    GPIO_PINS_PB12,
    GPIO_PINS_PB13,
    GPIO_PINS_PB14,
    GPIO_PINS_PB15,
#endif

#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PC0,
    GPIO_PINS_PC1,
    GPIO_PINS_PC2,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PC3,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
	defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PC4,
    GPIO_PINS_PC5,
    GPIO_PINS_PC6,
    GPIO_PINS_PC7,
    GPIO_PINS_PC8,
    GPIO_PINS_PC9,
    GPIO_PINS_PC10,
    GPIO_PINS_PC11,
    GPIO_PINS_PC12,
#endif
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PC13,
#endif
    GPIO_PINS_PC14,
    GPIO_PINS_PC15,

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PD0,
    GPIO_PINS_PD1,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
	defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PD2,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PD3,
    GPIO_PINS_PD4,
    GPIO_PINS_PD5,
    GPIO_PINS_PD6,
    GPIO_PINS_PD7,
    GPIO_PINS_PD8,
    GPIO_PINS_PD9,
    GPIO_PINS_PD10,
    GPIO_PINS_PD11,
    GPIO_PINS_PD12,
    GPIO_PINS_PD13,
    GPIO_PINS_PD14,
    GPIO_PINS_PD15,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PE0,
    GPIO_PINS_PE1,
    GPIO_PINS_PE2,
    GPIO_PINS_PE3,
    GPIO_PINS_PE4,
    GPIO_PINS_PE5,
    GPIO_PINS_PE6,
    GPIO_PINS_PE7,
    GPIO_PINS_PE8,
    GPIO_PINS_PE9,
    GPIO_PINS_PE10,
    GPIO_PINS_PE11,
    GPIO_PINS_PE12,
    GPIO_PINS_PE13,
    GPIO_PINS_PE14,
    GPIO_PINS_PE15,
#endif

#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
	defined (LIBOHIBOARD_STM32L072RxI) || \
	defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PH0,
    GPIO_PINS_PH1,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    GPIO_PINS_PH9,
    GPIO_PINS_PH10,
#endif

#endif // LIBOHIBOARD_STM32L072

#elif defined (LIBOHIBOARD_STM32L0x3)

#if defined (LIBOHIBOARD_STM32L073)

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
    GPIO_PINS_PA15,

    GPIO_PINS_PB0,
    GPIO_PINS_PB1,
    GPIO_PINS_PB2,
    GPIO_PINS_PB3,
    GPIO_PINS_PB4,
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

#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    GPIO_PINS_PC0,
    GPIO_PINS_PC1,
    GPIO_PINS_PC2,
#endif
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    GPIO_PINS_PC3,
#endif
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    GPIO_PINS_PC4,
    GPIO_PINS_PC5,
    GPIO_PINS_PC6,
    GPIO_PINS_PC7,
    GPIO_PINS_PC8,
    GPIO_PINS_PC9,
    GPIO_PINS_PC10,
    GPIO_PINS_PC11,
    GPIO_PINS_PC12,
#endif
    GPIO_PINS_PC13,
    GPIO_PINS_PC14,
    GPIO_PINS_PC15,

#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    GPIO_PINS_PD0,
    GPIO_PINS_PD1,
#endif
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    GPIO_PINS_PD2,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    GPIO_PINS_PD3,
    GPIO_PINS_PD4,
    GPIO_PINS_PD5,
    GPIO_PINS_PD6,
    GPIO_PINS_PD7,
    GPIO_PINS_PD8,
    GPIO_PINS_PD9,
    GPIO_PINS_PD10,
    GPIO_PINS_PD11,
    GPIO_PINS_PD12,
    GPIO_PINS_PD13,
    GPIO_PINS_PD14,
    GPIO_PINS_PD15,
#endif

#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    GPIO_PINS_PE0,
    GPIO_PINS_PE1,
    GPIO_PINS_PE2,
    GPIO_PINS_PE3,
    GPIO_PINS_PE4,
    GPIO_PINS_PE5,
    GPIO_PINS_PE6,
    GPIO_PINS_PE7,
    GPIO_PINS_PE8,
    GPIO_PINS_PE9,
    GPIO_PINS_PE10,
    GPIO_PINS_PE11,
    GPIO_PINS_PE12,
    GPIO_PINS_PE13,
    GPIO_PINS_PE14,
    GPIO_PINS_PE15,
#endif

    GPIO_PINS_PH0,
    GPIO_PINS_PH1,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    GPIO_PINS_PH9,
    GPIO_PINS_PH10,
#endif

#endif // LIBOHIBOARD_STM32L073

#endif

} Gpio_Pins;

/**
 * List of possible GPIO ports.
 */
typedef enum _Gpio_Ports
{
#if defined (LIBOHIBOARD_STM32L0x2) || \
    defined (LIBOHIBOARD_STM32L0x3)
    GPIO_PORTS_A,
    GPIO_PORTS_B,
    GPIO_PORTS_C,
    GPIO_PORTS_D,
    GPIO_PORTS_E,
    GPIO_PORTS_H,

#endif

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

#endif // LIBOHIBOARD_STM32L0

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __GPIO_STM32L0_H
