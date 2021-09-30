/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019-2021 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/source/STM32L0/gpio_STM32L0.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO implementations for STM32L0 Series.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L0)

#include "gpio.h"

#include "interrupt.h"
#include "clock.h"
#include "utility.h"

#define GPIO_ENABLE_CLOCK_PORTA() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOAEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOAEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTB() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOBEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOBEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTC() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOCEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOCEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTD() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIODEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIODEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTE() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOEEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOEEN); \
                                  } while (0)

#if 0
#define GPIO_ENABLE_CLOCK_PORTF() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOFEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOFEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTG() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOGEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOGEN); \
                                  } while (0)
#endif

#define GPIO_ENABLE_CLOCK_PORTH() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOHEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOHEN); \
                                  } while (0)

#define GPIO_DISABLE_CLOCK_PORTA() do { \
                                     UTILITY_CLEAR_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOAEN); \
                                     asm("nop"); \
                                     (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOAEN); \
                                   } while (0)

#define GPIO_DISABLE_CLOCK_PORTB() do { \
                                     UTILITY_CLEAR_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOBEN); \
                                     asm("nop"); \
                                     (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOBEN); \
                                   } while (0)

#define GPIO_DISABLE_CLOCK_PORTC() do { \
                                     UTILITY_CLEAR_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOCEN); \
                                     asm("nop"); \
                                     (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOCEN); \
                                   } while (0)

#define GPIO_DISABLE_CLOCK_PORTD() do { \
                                     UTILITY_CLEAR_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIODEN); \
                                     asm("nop"); \
                                     (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIODEN); \
                                   } while (0)

#define GPIO_DISABLE_CLOCK_PORTE() do { \
                                     UTILITY_CLEAR_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOEEN); \
                                     asm("nop"); \
                                     (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOEEN); \
                                   } while (0)
#if 0
#define GPIO_DISABLE_CLOCK_PORTF() do { \
                                     UTILITY_CLEAR_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOFEN); \
                                     asm("nop"); \
                                     (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOFEN); \
                                   } while (0)

#define GPIO_DISABLE_CLOCK_PORTG() do { \
                                     UTILITY_CLEAR_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOGEN); \
                                     asm("nop"); \
                                     (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOGEN); \
                                   } while (0)
#endif

#define GPIO_DISABLE_CLOCK_PORTH() do { \
                                     UTILITY_CLEAR_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOHEN); \
                                     asm("nop"); \
                                     (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOHEN); \
                                   } while (0)

typedef void (*Gpio_ExtCallback_t)(Gpio_Pins pin);

typedef struct _Gpio_ExtIsrCallback
{
	bool enabled;
	Gpio_Pins pin;
	Gpio_ExtCallback_t callback;
} Gpio_ExtIsrCallback_t;

static Gpio_ExtIsrCallback_t Gpio_isrPortRequestVector[GPIO_MAX_PINS_NUMBER_FOR_PORT] = {0};

static uint32_t Gpio_isrRegister = 0x0;

typedef struct _Gpio_PinDevice
{
    Gpio_Ports port;                      /**< The port of the selected pins */
    uint8_t pinNumber;       /**< The number of the pin of the relative port */
    uint8_t portIndex;       /**< Current port number for interrupt managing */
} Gpio_PinDevice;

static const Gpio_PinDevice GPIO_AVAILABLE_PINS[] =
{
    {0xFF,0xFF,0},

#if defined (LIBOHIBOARD_STM32L0x1)

#if defined (LIBOHIBOARD_STM32L071)

    {GPIO_PORTS_A,0,0},
    {GPIO_PORTS_A,1,0},
    {GPIO_PORTS_A,2,0},
    {GPIO_PORTS_A,3,0},
    {GPIO_PORTS_A,4,0},
    {GPIO_PORTS_A,5,0},
    {GPIO_PORTS_A,6,0},
    {GPIO_PORTS_A,7,0},
    {GPIO_PORTS_A,8,0},
    {GPIO_PORTS_A,9,0},
    {GPIO_PORTS_A,10,0},
    {GPIO_PORTS_A,11,0},
    {GPIO_PORTS_A,12,0},
    {GPIO_PORTS_A,13,0},
    {GPIO_PORTS_A,14,0},
#if !defined (LIBOHIBOARD_STM32L071KxU)
    {GPIO_PORTS_A,15,0},
#endif

    {GPIO_PORTS_B,0,1},
    {GPIO_PORTS_B,1,1},
#if defined (LIBOHIBOARD_STM32L071CxT) || \
    defined (LIBOHIBOARD_STM32L071CxY) || \
    defined (LIBOHIBOARD_STM32L071CxU) || \
    defined (LIBOHIBOARD_STM32L071RxT) || \
    defined (LIBOHIBOARD_STM32L071RxH) || \
    defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_B,2,1},
#endif
#if !defined (LIBOHIBOARD_STM32L071KxU)
    {GPIO_PORTS_B,3,1},
#endif
    {GPIO_PORTS_B,4,1},
    {GPIO_PORTS_B,5,1},
    {GPIO_PORTS_B,6,1},
    {GPIO_PORTS_B,7,1},
#if defined (LIBOHIBOARD_STM32L071CxT) || \
    defined (LIBOHIBOARD_STM32L071CxY) || \
    defined (LIBOHIBOARD_STM32L071CxU) || \
    defined (LIBOHIBOARD_STM32L071RxT) || \
    defined (LIBOHIBOARD_STM32L071RxH) || \
    defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_B,8,1},
    {GPIO_PORTS_B,9,1},
    {GPIO_PORTS_B,10,1},
    {GPIO_PORTS_B,11,1},
    {GPIO_PORTS_B,12,1},
    {GPIO_PORTS_B,13,1},
    {GPIO_PORTS_B,14,1},
    {GPIO_PORTS_B,15,1},
#endif

#if defined (LIBOHIBOARD_STM32L071CxY) || \
    defined (LIBOHIBOARD_STM32L071RxT) || \
    defined (LIBOHIBOARD_STM32L071RxH) || \
    defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_C,0,2},
    {GPIO_PORTS_C,1,2},
    {GPIO_PORTS_C,2,2},
#endif
#if defined (LIBOHIBOARD_STM32L071RxT) || \
    defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI))
    {GPIO_PORTS_C,3,2},
#endif
#if defined (LIBOHIBOARD_STM32L071RxT) || \
    defined (LIBOHIBOARD_STM32L071RxH) || \
    defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_C,4,2},
    {GPIO_PORTS_C,5,2},
    {GPIO_PORTS_C,6,2},
    {GPIO_PORTS_C,7,2},
    {GPIO_PORTS_C,8,2},
    {GPIO_PORTS_C,9,2},
    {GPIO_PORTS_C,10,2},
    {GPIO_PORTS_C,11,2},
    {GPIO_PORTS_C,12,2},
#endif
#if defined (LIBOHIBOARD_STM32L071CxT) || \
    defined (LIBOHIBOARD_STM32L071CxY) || \
    defined (LIBOHIBOARD_STM32L071CxU) || \
    defined (LIBOHIBOARD_STM32L071RxT) || \
    defined (LIBOHIBOARD_STM32L071RxH) || \
    defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_C,13,2},
#endif
    {GPIO_PORTS_C,14,2},
    {GPIO_PORTS_C,15,2},

#if defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_D,0,3},
    {GPIO_PORTS_D,1,3},
#endif
#if defined (LIBOHIBOARD_STM32L071RxT) || \
    defined (LIBOHIBOARD_STM32L071RxH) || \
    defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_D,2,3},
#endif
#if defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_D,3,3},
    {GPIO_PORTS_D,4,3},
    {GPIO_PORTS_D,5,3},
    {GPIO_PORTS_D,6,3},
    {GPIO_PORTS_D,7,3},
    {GPIO_PORTS_D,8,3},
    {GPIO_PORTS_D,9,3},
    {GPIO_PORTS_D,10,3},
    {GPIO_PORTS_D,11,3},
    {GPIO_PORTS_D,12,3},
    {GPIO_PORTS_D,13,3},
    {GPIO_PORTS_D,14,3},
    {GPIO_PORTS_D,15,3},
#endif

#if defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_E,0,4},
    {GPIO_PORTS_E,1,4},
    {GPIO_PORTS_E,2,4},
    {GPIO_PORTS_E,3,4},
    {GPIO_PORTS_E,4,4},
    {GPIO_PORTS_E,5,4},
    {GPIO_PORTS_E,6,4},
    {GPIO_PORTS_E,7,4},
    {GPIO_PORTS_E,8,4},
    {GPIO_PORTS_E,9,4},
    {GPIO_PORTS_E,10,4},
    {GPIO_PORTS_E,11,4},
    {GPIO_PORTS_E,12,4},
    {GPIO_PORTS_E,13,4},
    {GPIO_PORTS_E,14,4},
    {GPIO_PORTS_E,15,4},
#endif

#if defined (LIBOHIBOARD_STM32L071CxT) || \
    defined (LIBOHIBOARD_STM32L071CxY) || \
    defined (LIBOHIBOARD_STM32L071CxU) || \
    defined (LIBOHIBOARD_STM32L071RxT) || \
    defined (LIBOHIBOARD_STM32L071RxH) || \
    defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_H,0,7},
    {GPIO_PORTS_H,1,7},
#endif
#if defined (LIBOHIBOARD_STM32L071VxT) || \
    defined (LIBOHIBOARD_STM32L071VxI)
    {GPIO_PORTS_H,9,7},
    {GPIO_PORTS_H,10,7},
#endif

#endif // LIBOHIBOARD_STM32L071

#if defined (LIBOHIBOARD_STM32L081)

    {GPIO_PORTS_A,0,0},
    {GPIO_PORTS_A,1,0},
    {GPIO_PORTS_A,2,0},
    {GPIO_PORTS_A,3,0},
    {GPIO_PORTS_A,4,0},
    {GPIO_PORTS_A,5,0},
    {GPIO_PORTS_A,6,0},
    {GPIO_PORTS_A,7,0},
    {GPIO_PORTS_A,8,0},
    {GPIO_PORTS_A,9,0},
    {GPIO_PORTS_A,10,0},
    {GPIO_PORTS_A,11,0},
    {GPIO_PORTS_A,12,0},
    {GPIO_PORTS_A,13,0},
    {GPIO_PORTS_A,14,0},
#if defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    {GPIO_PORTS_A,15,0},
#endif

    {GPIO_PORTS_B,0,1},
    {GPIO_PORTS_B,1,1},
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    {GPIO_PORTS_B,2,1},
#endif
#if defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    {GPIO_PORTS_B,3,1},
#endif
    {GPIO_PORTS_B,4,1},
    {GPIO_PORTS_B,5,1},
    {GPIO_PORTS_B,6,1},
    {GPIO_PORTS_B,7,1},
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    {GPIO_PORTS_B,8,1},
    {GPIO_PORTS_B,9,1},
    {GPIO_PORTS_B,10,1},
    {GPIO_PORTS_B,11,1},
    {GPIO_PORTS_B,12,1},
    {GPIO_PORTS_B,13,1},
    {GPIO_PORTS_B,14,1},
    {GPIO_PORTS_B,15,1},
#endif

#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    {GPIO_PORTS_C,13,2},
#endif
    {GPIO_PORTS_C,14,2},
    {GPIO_PORTS_C,15,2},

#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    {GPIO_PORTS_H,0,7},
    {GPIO_PORTS_H,1,7},
#endif

#endif // LIBOHIBOARD_STM32L081

#elif defined (LIBOHIBOARD_STM32L0x2)

#if defined (LIBOHIBOARD_STM32L072)

    {GPIO_PORTS_A,0,0},
    {GPIO_PORTS_A,1,0},
    {GPIO_PORTS_A,2,0},
    {GPIO_PORTS_A,3,0},
    {GPIO_PORTS_A,4,0},
    {GPIO_PORTS_A,5,0},
    {GPIO_PORTS_A,6,0},
    {GPIO_PORTS_A,7,0},
    {GPIO_PORTS_A,8,0},
    {GPIO_PORTS_A,9,0},
    {GPIO_PORTS_A,10,0},
    {GPIO_PORTS_A,11,0},
    {GPIO_PORTS_A,12,0},
    {GPIO_PORTS_A,13,0},
    {GPIO_PORTS_A,14,0},
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    {GPIO_PORTS_A,15,0},
#endif

    {GPIO_PORTS_B,0,1},
    {GPIO_PORTS_B,1,1},
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    {GPIO_PORTS_B,2,1},
#endif
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    {GPIO_PORTS_B,3,1},
#endif
    {GPIO_PORTS_B,4,1},
    {GPIO_PORTS_B,5,1},
    {GPIO_PORTS_B,6,1},
    {GPIO_PORTS_B,7,1},
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    {GPIO_PORTS_B,8,1},
    {GPIO_PORTS_B,9,1},
#if !defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    {GPIO_PORTS_B,10,1},
    {GPIO_PORTS_B,11,1},
#endif
    {GPIO_PORTS_B,12,1},
    {GPIO_PORTS_B,13,1},
    {GPIO_PORTS_B,14,1},
    {GPIO_PORTS_B,15,1},
#endif

#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    {GPIO_PORTS_C,0,2},
    {GPIO_PORTS_C,1,2},
    {GPIO_PORTS_C,2,2},
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    {GPIO_PORTS_C,3,2},
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    {GPIO_PORTS_C,4,2},
    {GPIO_PORTS_C,5,2},
    {GPIO_PORTS_C,6,2},
    {GPIO_PORTS_C,7,2},
    {GPIO_PORTS_C,8,2},
    {GPIO_PORTS_C,9,2},
    {GPIO_PORTS_C,10,2},
    {GPIO_PORTS_C,11,2},
    {GPIO_PORTS_C,12,2},
#endif
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    {GPIO_PORTS_C,13,2},
#endif
#if !defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    {GPIO_PORTS_C,14,2},
    {GPIO_PORTS_C,15,2},
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    {GPIO_PORTS_D,0,3},
    {GPIO_PORTS_D,1,3},
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    {GPIO_PORTS_D,2,3},
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    {GPIO_PORTS_D,3,3},
    {GPIO_PORTS_D,4,3},
    {GPIO_PORTS_D,5,3},
    {GPIO_PORTS_D,6,3},
    {GPIO_PORTS_D,7,3},
    {GPIO_PORTS_D,8,3},
    {GPIO_PORTS_D,9,3},
    {GPIO_PORTS_D,10,3},
    {GPIO_PORTS_D,11,3},
    {GPIO_PORTS_D,12,3},
    {GPIO_PORTS_D,13,3},
    {GPIO_PORTS_D,14,3},
    {GPIO_PORTS_D,15,3},
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    {GPIO_PORTS_E,0,4},
    {GPIO_PORTS_E,1,4},
    {GPIO_PORTS_E,2,4},
    {GPIO_PORTS_E,3,4},
    {GPIO_PORTS_E,4,4},
    {GPIO_PORTS_E,5,4},
    {GPIO_PORTS_E,6,4},
    {GPIO_PORTS_E,7,4},
    {GPIO_PORTS_E,8,4},
    {GPIO_PORTS_E,9,4},
    {GPIO_PORTS_E,10,4},
    {GPIO_PORTS_E,11,4},
    {GPIO_PORTS_E,12,4},
    {GPIO_PORTS_E,13,4},
    {GPIO_PORTS_E,14,4},
    {GPIO_PORTS_E,15,4},
#endif

#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    {GPIO_PORTS_H,0,7},
    {GPIO_PORTS_H,1,7},
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    {GPIO_PORTS_H,9,7},
    {GPIO_PORTS_H,10,7},
#endif

#endif // LIBOHIBOARD_STM32L072

#elif defined (LIBOHIBOARD_STM32L0x3)

#if defined (LIBOHIBOARD_STM32L073)

    {GPIO_PORTS_A,0,0},
    {GPIO_PORTS_A,1,0},
    {GPIO_PORTS_A,2,0},
    {GPIO_PORTS_A,3,0},
    {GPIO_PORTS_A,4,0},
    {GPIO_PORTS_A,5,0},
    {GPIO_PORTS_A,6,0},
    {GPIO_PORTS_A,7,0},
    {GPIO_PORTS_A,8,0},
    {GPIO_PORTS_A,9,0},
    {GPIO_PORTS_A,10,0},
    {GPIO_PORTS_A,11,0},
    {GPIO_PORTS_A,12,0},
    {GPIO_PORTS_A,13,0},
    {GPIO_PORTS_A,14,0},
    {GPIO_PORTS_A,15,0},

    {GPIO_PORTS_B,0,1},
    {GPIO_PORTS_B,1,1},
    {GPIO_PORTS_B,2,1},
    {GPIO_PORTS_B,3,1},
    {GPIO_PORTS_B,4,1},
    {GPIO_PORTS_B,5,1},
    {GPIO_PORTS_B,6,1},
    {GPIO_PORTS_B,7,1},
    {GPIO_PORTS_B,8,1},
    {GPIO_PORTS_B,9,1},
    {GPIO_PORTS_B,10,1},
    {GPIO_PORTS_B,11,1},
    {GPIO_PORTS_B,12,1},
    {GPIO_PORTS_B,13,1},
    {GPIO_PORTS_B,14,1},
    {GPIO_PORTS_B,15,1},

#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    {GPIO_PORTS_C,0,2},
    {GPIO_PORTS_C,1,2},
    {GPIO_PORTS_C,2,2},
#endif
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    {GPIO_PORTS_C,3,2},
#endif
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    {GPIO_PORTS_C,4,2},
    {GPIO_PORTS_C,5,2},
    {GPIO_PORTS_C,6,2},
    {GPIO_PORTS_C,7,2},
    {GPIO_PORTS_C,8,2},
    {GPIO_PORTS_C,9,2},
    {GPIO_PORTS_C,10,2},
    {GPIO_PORTS_C,11,2},
    {GPIO_PORTS_C,12,2},
#endif
    {GPIO_PORTS_C,13,2},
    {GPIO_PORTS_C,14,2},
    {GPIO_PORTS_C,15,2},

#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    {GPIO_PORTS_D,0,3},
    {GPIO_PORTS_D,1,3},
#endif
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    {GPIO_PORTS_D,2,3},
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    {GPIO_PORTS_D,3,3},
    {GPIO_PORTS_D,4,3},
    {GPIO_PORTS_D,5,3},
    {GPIO_PORTS_D,6,3},
    {GPIO_PORTS_D,7,3},
    {GPIO_PORTS_D,8,3},
    {GPIO_PORTS_D,9,3},
    {GPIO_PORTS_D,10,3},
    {GPIO_PORTS_D,11,3},
    {GPIO_PORTS_D,12,3},
    {GPIO_PORTS_D,13,3},
    {GPIO_PORTS_D,14,3},
    {GPIO_PORTS_D,15,3},
#endif

#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    {GPIO_PORTS_E,0,4},
    {GPIO_PORTS_E,1,4},
    {GPIO_PORTS_E,2,4},
    {GPIO_PORTS_E,3,4},
    {GPIO_PORTS_E,4,4},
    {GPIO_PORTS_E,5,4},
    {GPIO_PORTS_E,6,4},
    {GPIO_PORTS_E,7,4},
    {GPIO_PORTS_E,8,4},
    {GPIO_PORTS_E,9,4},
    {GPIO_PORTS_E,10,4},
    {GPIO_PORTS_E,11,4},
    {GPIO_PORTS_E,12,4},
    {GPIO_PORTS_E,13,4},
    {GPIO_PORTS_E,14,4},
    {GPIO_PORTS_E,15,4},
#endif

    {GPIO_PORTS_H,0,7},
    {GPIO_PORTS_H,1,7},
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    {GPIO_PORTS_H,9,7},
    {GPIO_PORTS_H,10,7},
#endif

#endif // LIBOHIBOARD_STM32L073

#endif

};

static const uint8_t GPIO_AVAILABLE_PINS_COUNT = UTILITY_DIMOF(GPIO_AVAILABLE_PINS);

static GPIO_TypeDef* Gpio_getPort (Gpio_Ports port)
{
    switch (port)
    {
#if defined (LIBOHIBOARD_STM32L0x1)
    case GPIO_PORTS_A:
        return GPIOA;

    case GPIO_PORTS_B:
        return GPIOB;

    case GPIO_PORTS_C:
        return GPIOC;
#if defined (LIBOHIBOARD_STM32L071)
    case GPIO_PORTS_D:
        return GPIOD;

    case GPIO_PORTS_E:
        return GPIOE;
#endif
#if defined (LIBOHIBOARD_STM32L071)    || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    case GPIO_PORTS_H:
        return GPIOH;
#endif
#elif defined (LIBOHIBOARD_STM32L0x2) || \
      defined (LIBOHIBOARD_STM32L0x3)
    case GPIO_PORTS_A:
        return GPIOA;

    case GPIO_PORTS_B:
        return GPIOB;

    case GPIO_PORTS_C:
        return GPIOC;

    case GPIO_PORTS_D:
        return GPIOD;

    case GPIO_PORTS_E:
        return GPIOE;

    case GPIO_PORTS_H:
        return GPIOH;
#endif

    default:
        ohiassert(0);
        return 0;
    }
}

void Gpio_enablePortClock (Gpio_Ports port)
{
    switch (port)
    {
#if defined (LIBOHIBOARD_STM32L0x1)
    case GPIO_PORTS_A:
        GPIO_ENABLE_CLOCK_PORTA();
        break;

    case GPIO_PORTS_B:
        GPIO_ENABLE_CLOCK_PORTB();
        break;

    case GPIO_PORTS_C:
        GPIO_ENABLE_CLOCK_PORTC();
        break;
#if defined (LIBOHIBOARD_STM32L071)

    case GPIO_PORTS_D:
        GPIO_ENABLE_CLOCK_PORTD();
        break;

    case GPIO_PORTS_E:
        GPIO_ENABLE_CLOCK_PORTE();
        break;
#endif
#if defined (LIBOHIBOARD_STM32L071)    || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    case GPIO_PORTS_H:
        GPIO_ENABLE_CLOCK_PORTH();
        break;
#endif
#elif defined (LIBOHIBOARD_STM32L0x2) || \
      defined (LIBOHIBOARD_STM32L0x3)
    case GPIO_PORTS_A:
        GPIO_ENABLE_CLOCK_PORTA();
        break;

    case GPIO_PORTS_B:
        GPIO_ENABLE_CLOCK_PORTB();
        break;

    case GPIO_PORTS_C:
        GPIO_ENABLE_CLOCK_PORTC();
        break;

    case GPIO_PORTS_D:
        GPIO_ENABLE_CLOCK_PORTD();
        break;

    case GPIO_PORTS_E:
        GPIO_ENABLE_CLOCK_PORTE();
        break;

    case GPIO_PORTS_H:
        GPIO_ENABLE_CLOCK_PORTH();
        break;
#endif

    default:
        ohiassert(0);
        break;
    }

}

void Gpio_disablePortClock (Gpio_Ports port)
{
    switch (port)
    {
#if defined (LIBOHIBOARD_STM32L0x1)
    case GPIO_PORTS_A:
        GPIO_DISABLE_CLOCK_PORTA();
        break;

    case GPIO_PORTS_B:
        GPIO_DISABLE_CLOCK_PORTB();
        break;

    case GPIO_PORTS_C:
        GPIO_DISABLE_CLOCK_PORTC();
        break;
#if defined (LIBOHIBOARD_STM32L071)

    case GPIO_PORTS_D:
        GPIO_DISABLE_CLOCK_PORTD();
        break;

    case GPIO_PORTS_E:
        GPIO_DISABLE_CLOCK_PORTE();
        break;
#endif
#if defined (LIBOHIBOARD_STM32L071)    || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    case GPIO_PORTS_H:
        GPIO_DISABLE_CLOCK_PORTH();
        break;
#endif
#elif defined (LIBOHIBOARD_STM32L0x2) || \
      defined (LIBOHIBOARD_STM32L0x3)
    case GPIO_PORTS_A:
        GPIO_DISABLE_CLOCK_PORTA();
        break;

    case GPIO_PORTS_B:
        GPIO_DISABLE_CLOCK_PORTB();
        break;

    case GPIO_PORTS_C:
        GPIO_DISABLE_CLOCK_PORTC();
        break;

    case GPIO_PORTS_D:
        GPIO_DISABLE_CLOCK_PORTD();
        break;

    case GPIO_PORTS_E:
        GPIO_DISABLE_CLOCK_PORTE();
        break;

    case GPIO_PORTS_H:
        GPIO_DISABLE_CLOCK_PORTH();
        break;
#endif

    default:
        ohiassert(0);
        break;
    }
}

void Gpio_configAlternate (Gpio_Pins pin, Gpio_Alternate alternate, uint16_t options)
{
    if (pin == GPIO_PINS_NONE)
    {
        return; //ERRORS_GPIO_NULL_PIN;
    }

    GPIO_TypeDef* port;
    uint32_t temp = 0x00;
    // Pin number into the current port
    uint8_t number = 0;

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

    // Check the alternate value: it have 16 possibility
    ohiassert(alternate < 16);

    // Enable clock and save current port
    Gpio_enablePortClock(GPIO_AVAILABLE_PINS[pin].port);
    port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);

    number = GPIO_AVAILABLE_PINS[pin].pinNumber;

    if (alternate >= 0)
    {
        // Configure alternate function
        // Select AFR0 or AFR1
        temp = port->AFR[number >> 3];
        temp &= ~((uint32_t)0xF << ((uint32_t)(number & (uint32_t)0x07) * 4)) ;
        temp |= ((uint32_t)(alternate) << (((uint32_t)number & (uint32_t)0x07) * 4));
        port->AFR[number >> 3] = temp;

        // Configure pin as alternate mode
        temp = port->MODER;
        temp &= ~(GPIO_MODER_MODE0 << (number * 2));
        temp |= ((0x2u) << (number * 2));
        port->MODER = temp;

        if (options)
        {
            // Only one type of configuration is possible
            ohiassert(!(((options & GPIO_PINS_ENABLE_OUTPUT_PUSHPULL) == GPIO_PINS_ENABLE_OUTPUT_PUSHPULL) &&
                        ((options & GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN) == GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN)));

            // Configure IO output type
            // One value must be selected, anyway use PUSH-PULL as default value
            temp = port->OTYPER;
            temp &= ~(GPIO_OTYPER_OT_0 << number) ;
            temp |= (((options & GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN) ? 0x01 : 0x00) << number);
            port->OTYPER = temp;

            // Set pull-up or pull-down resistor if requested
            if (options & GPIO_PINS_PULL)
            {
                // One type must be configured
                ohiassert(((options & GPIO_PINS_ENABLE_PULLUP) == GPIO_PINS_ENABLE_PULLUP) ^
                          ((options & GPIO_PINS_ENABLE_PULLDOWN) == GPIO_PINS_ENABLE_PULLDOWN));

                temp = port->PUPDR;
                temp &= ~(GPIO_PUPDR_PUPD0 << (number * 2));
                temp |= (((options & GPIO_PINS_ENABLE_PULLUP) ? 0x01 : 0x02) << (number * 2));
                port->PUPDR = temp;
            }

            // Configure output speed
            // Default speed is low speed
            uint32_t speed = 0U;
            if (options & GPIO_PINS_SPEED_MEDIUM)
            {
                speed = 0x01;
            }
            else if (options & GPIO_PINS_SPEED_HIGH)
            {
                speed = 0x02;
            }
            else if (options & GPIO_PINS_SPEED_VERY_HIGH)
            {
                speed = 0x03;
            }
            temp = port->OSPEEDR;
            temp &= ~(GPIO_OSPEEDER_OSPEED0 << (number * 2));
            temp |= (speed << (number * 2));
            port->OSPEEDR = temp;
        }
    }
    else
    {
        // Configure pin as analog
        temp = port->MODER;
        temp &= ~(GPIO_MODER_MODE0 << (number * 2));
        temp |= ((0x3u) << (number * 2));
        port->MODER = temp;
    }
}

System_Errors Gpio_config (Gpio_Pins pin, uint16_t options)
{
    if (pin == GPIO_PINS_NONE)
    {
        return ERRORS_GPIO_NULL_PIN;
    }

    GPIO_TypeDef* port;
    uint32_t temp = 0x00;

    // Pin number into the current port
    uint8_t number = 0;

    // Only one type of configuration is possible
    ohiassert(((options & GPIO_PINS_OUTPUT) == GPIO_PINS_OUTPUT) ^
              ((options & GPIO_PINS_INPUT) == GPIO_PINS_INPUT));

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

    number = GPIO_AVAILABLE_PINS[pin].pinNumber;

    // Enable clock and save current port
    Gpio_enablePortClock(GPIO_AVAILABLE_PINS[pin].port);
    port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);

    // Configure direction mode
    temp = port->MODER;
    temp &= ~(GPIO_MODER_MODE0 << (number * 2));
    temp |= (((options & GPIO_PINS_INPUT) ? 0x00 : 0x01) << (number * 2));
    port->MODER = temp;

    if (options & GPIO_PINS_OUTPUT)
    {
        // Only one type of configuration is possible
        ohiassert(!(((options & GPIO_PINS_ENABLE_OUTPUT_PUSHPULL) == GPIO_PINS_ENABLE_OUTPUT_PUSHPULL) &&
                    ((options & GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN) == GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN)));

        // Configure IO output type
        // One value must be selected, anyway use PUSH-PULL as default value
        temp = port->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << number) ;
        temp |= (((options & GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN) ? 0x01 : 0x00) << number);
        port->OTYPER = temp;

        // Configure output speed
        // Default speed is low speed
        uint32_t speed = 0U;
        if (options & GPIO_PINS_SPEED_MEDIUM)
        {
            speed = 0x01;
        }
        else if (options & GPIO_PINS_SPEED_HIGH)
        {
            speed = 0x02;
        }
        else if (options & GPIO_PINS_SPEED_VERY_HIGH)
        {
            speed = 0x03;
        }
        temp = port->OSPEEDR;
        temp &= ~(GPIO_OSPEEDER_OSPEED0 << (number * 2));
        temp |= (speed << (number * 2));
        port->OSPEEDR = temp;
    }

    // Set pull-up or pull-down resistor if requested
    if (options & GPIO_PINS_PULL)
    {
        // One type must be configured
        ohiassert(((options & GPIO_PINS_ENABLE_PULLUP) == GPIO_PINS_ENABLE_PULLUP) ^
                  ((options & GPIO_PINS_ENABLE_PULLDOWN) == GPIO_PINS_ENABLE_PULLDOWN));

        temp = port->PUPDR;
        temp &= ~(GPIO_PUPDR_PUPD0 << (number * 2));
        temp |= (((options & GPIO_PINS_ENABLE_PULLUP) ? 0x01 : 0x02) << (number * 2));
        port->PUPDR = temp;
    }

    return ERRORS_NO_ERROR;
}

void Gpio_set (Gpio_Pins pin)
{
    if (pin != GPIO_PINS_NONE)
    {
        //Check if pin definition exist
        ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

        GPIO_TypeDef* port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);
        port->BSRR = GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber);
    }
}

void Gpio_clear (Gpio_Pins pin)
{
    if (pin != GPIO_PINS_NONE)
    {
        //Check if pin definition exist
        ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

        GPIO_TypeDef* port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);
        port->BRR = GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber);
    }
}

void Gpio_toggle (Gpio_Pins pin)
{
    if (pin != GPIO_PINS_NONE)
    {
        //Check if pin definition exist
        ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

        GPIO_TypeDef* port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);
        port->ODR ^= GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber);
    }
}

Gpio_Level Gpio_get (Gpio_Pins pin)
{
    if (pin == GPIO_PINS_NONE)
    {
        return GPIO_TOGGLE;
    }

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

    GPIO_TypeDef* port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);
    return ((port->IDR & GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber)) > 0) ? GPIO_HIGH : GPIO_LOW;
}

System_Errors Gpio_configInterrupt (Gpio_Pins pin, Interrupt_Priority priority, void* callback)
{
	(void)priority; //FIXMENOW: manage interrupt priority

    if (pin == GPIO_PINS_NONE)
    {
        return ERRORS_GPIO_NULL_PIN;
    }

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);
    if (pin >= GPIO_AVAILABLE_PINS_COUNT)
        return ERRORS_GPIO_WRONG_PIN;

    //Check callback is empty
	if (Gpio_isrPortRequestVector[GPIO_AVAILABLE_PINS[pin].pinNumber].callback != nullptr)
	{
		asm("NOP");
	}

	Gpio_isrPortRequestVector[GPIO_AVAILABLE_PINS[pin].pinNumber].pin = pin;
    Gpio_isrPortRequestVector[GPIO_AVAILABLE_PINS[pin].pinNumber].callback = callback;
    Gpio_isrRegister |= 1 << GPIO_AVAILABLE_PINS[pin].pinNumber;

    return ERRORS_NO_ERROR;
}

System_Errors Gpio_enableInterrupt (Gpio_Pins pin, Gpio_EventType event)
{
    if (pin == GPIO_PINS_NONE)
    {
        return ERRORS_GPIO_NULL_PIN;
    }

    uint32_t pinNumber = GPIO_AVAILABLE_PINS[pin].pinNumber;
    uint32_t portIndex = GPIO_AVAILABLE_PINS[pin].portIndex;
    uint32_t temp = 0x00;

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

    if (pin > GPIO_AVAILABLE_PINS_COUNT)
        return ERRORS_GPIO_WRONG_PIN;

    // At least one type of controller must be choose
    ohiassert(((event & GPIO_EVENT_USE_INTERRUPT) == GPIO_EVENT_USE_INTERRUPT) ||
              ((event & GPIO_EVENT_USE_EVENT) == GPIO_EVENT_USE_EVENT));
    if (!(((event & GPIO_EVENT_USE_INTERRUPT) == GPIO_EVENT_USE_INTERRUPT) ||
         ((event & GPIO_EVENT_USE_EVENT) == GPIO_EVENT_USE_EVENT)))
        return ERRORS_GPIO_WRONG_CONFIG;

    // At least one type of configuration must be choose
    ohiassert(((event & GPIO_EVENT_ON_RISING) == GPIO_EVENT_ON_RISING) ||
              ((event & GPIO_EVENT_ON_FALLING) == GPIO_EVENT_ON_FALLING));

    // Enable clock to SYSCFG module
    CLOCK_ENABLE_SYSCFG();

    // Set the current pin as interrupt source for the corresponding channel
    temp = SYSCFG->EXTICR[pinNumber >> 2];
    temp &= ~(((uint32_t)0x0F) << (4 * (pinNumber & 0x03)));
    temp |= portIndex << (4 * (pinNumber & 0x03));
    SYSCFG->EXTICR[pinNumber >> 2] = temp;

    // Enable interrupt if request
    temp = EXTI->IMR;
    temp &= ~((uint32_t)GPIO_PIN(pinNumber));
    if ((event & GPIO_EVENT_USE_INTERRUPT) == GPIO_EVENT_USE_INTERRUPT)
    {
      temp |= (uint32_t)GPIO_PIN(pinNumber);
    }
    EXTI->IMR = temp;

    // Enable event if request
    temp = EXTI->EMR;
    temp &= ~((uint32_t)GPIO_PIN(pinNumber));
    if ((event & GPIO_EVENT_USE_EVENT) == GPIO_EVENT_USE_EVENT)
    {
      temp |= (uint32_t)GPIO_PIN(pinNumber);
    }
    EXTI->EMR = temp;

    // Set-up falling and rising edge
    temp = EXTI->RTSR;
    temp &= ~((uint32_t)GPIO_PIN(pinNumber));
    if ((event & GPIO_EVENT_ON_RISING) == GPIO_EVENT_ON_RISING)
    {
      temp |= (uint32_t)GPIO_PIN(pinNumber);
    }
    EXTI->RTSR = temp;

    temp = EXTI->FTSR;
    temp &= ~((uint32_t)GPIO_PIN(pinNumber));
    if ((event & GPIO_EVENT_ON_FALLING) == GPIO_EVENT_ON_FALLING)
    {
      temp |= (uint32_t)GPIO_PIN(pinNumber);
    }
    EXTI->FTSR = temp;

    // Enable callback
    Gpio_isrPortRequestVector[pinNumber].enabled = true;

    // Enable NVIC interrupt
    switch (pinNumber)
    {
    case 0:
    case 1:
        Interrupt_enable(INTERRUPT_EXTI1_0);
        break;
    case 2:
    case 3:
        Interrupt_enable(INTERRUPT_EXTI3_2);
        break;
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        Interrupt_enable(INTERRUPT_EXTI15_4);
        break;
    }

    return ERRORS_NO_ERROR;
}

System_Errors Gpio_disableInterrupt (Gpio_Pins pin)
{
    if (pin == GPIO_PINS_NONE)
    {
        return ERRORS_GPIO_NULL_PIN;
    }

    uint32_t pinNumber = GPIO_AVAILABLE_PINS[pin].pinNumber;
    uint32_t portIndex = GPIO_AVAILABLE_PINS[pin].portIndex;
    uint32_t temp = 0x00;

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);
    if (pin >= GPIO_AVAILABLE_PINS_COUNT)
        return ERRORS_GPIO_WRONG_PIN;

    temp = SYSCFG->EXTICR[pinNumber >> 2];
    temp &= (((uint32_t)0x0F) << (4 * (pinNumber & 0x03)));
    // Disable interrupt only if this pin is enable
    if (temp == (portIndex << (4 * (pinNumber & 0x03))))
    {
        temp = ((uint32_t)0x0F) << (4 * (pinNumber & 0x03));
        SYSCFG->EXTICR[pinNumber >> 2] &= ~temp;

        // Clear EXTI line configuration
        EXTI->IMR &= ~((uint32_t)GPIO_PIN(pinNumber));
        EXTI->EMR &= ~((uint32_t)GPIO_PIN(pinNumber));

        // Clear Rising-Falling edge configuration
        EXTI->RTSR &= ~((uint32_t)GPIO_PIN(pinNumber));
        EXTI->FTSR &= ~((uint32_t)GPIO_PIN(pinNumber));

        // Disable callback
        Gpio_isrPortRequestVector[pinNumber].enabled = false;

        // Disable NVIC interrupt
        switch (pinNumber)
        {
            case 0:
            case 1:
            {
                bool disable = true;
                for (uint8_t i = 0; i < 2; i++)
                {
                    if (Gpio_isrPortRequestVector[i].enabled == true)
                    {
                        disable = false;
                    }
                }
                if (disable == true)
                {
                    Interrupt_disable(INTERRUPT_EXTI1_0);
                }
            }
                break;
            case 2:
            case 3:
            {
                bool disable = true;
                for (uint8_t i = 2; i < 4; i++)
                {
                    if (Gpio_isrPortRequestVector[i].enabled == true)
                    {
                        disable = false;
                    }
                }
                if (disable == true)
                {
                    Interrupt_disable(INTERRUPT_EXTI3_2);
                }
            }
                break;
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
            {
                bool disable = true;
                for (uint8_t i = 4; i < 16; i++)
                {
                    if (Gpio_isrPortRequestVector[i].enabled == true)
                    {
                        disable = false;
                    }
                }
                if (disable == true)
                {
                    Interrupt_disable(INTERRUPT_EXTI15_4);
                }
            }
                break;
        }
    }
    return ERRORS_NO_ERROR;
}

void EXTI0_1_IRQHandler (void)
{
    uint32_t pending = EXTI->PR;
    if (pending & GPIO_PIN(0))
    {
        // Clear pending flag
        EXTI->PR = GPIO_PIN(0);

        if ((Gpio_isrPortRequestVector[0].callback != nullptr) &&
            (Gpio_isrPortRequestVector[0].enabled == true))
        {
            Gpio_isrPortRequestVector[0].callback(Gpio_isrPortRequestVector[0].pin);
        }
    }

    if (pending & GPIO_PIN(1))
    {
        // Clear pending flag
        EXTI->PR = GPIO_PIN(1);

        if ((Gpio_isrPortRequestVector[1].callback != nullptr) &&
            (Gpio_isrPortRequestVector[1].enabled == true))
        {
            Gpio_isrPortRequestVector[1].callback(Gpio_isrPortRequestVector[1].pin);
        }
    }
}

void EXTI2_3_IRQHandler (void)
{
    uint32_t pending = EXTI->PR;
    if (pending & GPIO_PIN(2))
    {
        // Clear pending flag
        EXTI->PR = GPIO_PIN(2);

        if ((Gpio_isrPortRequestVector[2].callback != nullptr) &&
            (Gpio_isrPortRequestVector[2].enabled == true))
        {
            Gpio_isrPortRequestVector[2].callback(Gpio_isrPortRequestVector[2].pin);
        }
    }

    if (pending & GPIO_PIN(3))
    {
        // Clear pending flag
        EXTI->PR = GPIO_PIN(3);

        if ((Gpio_isrPortRequestVector[3].callback != nullptr) &&
            (Gpio_isrPortRequestVector[3].enabled == true))
        {
            Gpio_isrPortRequestVector[3].callback(Gpio_isrPortRequestVector[3].pin);
        }
    }
}

void EXTI4_15_IRQHandler (void)
{
    uint32_t pending = EXTI->PR;

    for (int i = 4; i < 16; i++)
    {
        if (pending & GPIO_PIN(i))
        {
            // Clear flag interrupt
            EXTI->PR = GPIO_PIN(i);
            if ((Gpio_isrPortRequestVector[i].callback != nullptr) &&
                (Gpio_isrPortRequestVector[i].enabled == true))
            {
                Gpio_isrPortRequestVector[i].callback(Gpio_isrPortRequestVector[i].pin);
            }
        }
    }
}

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif
