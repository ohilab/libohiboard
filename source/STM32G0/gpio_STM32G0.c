/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2021 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/STM32G0/gpio_STM32G0.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO implementations for STM32G0 Series.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32G0)

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

#define GPIO_ENABLE_CLOCK_PORTF() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOFEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOFEN); \
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

#define GPIO_DISABLE_CLOCK_PORTF() do { \
                                     UTILITY_CLEAR_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOFEN); \
                                     asm("nop"); \
                                     (void) UTILITY_READ_REGISTER_BIT(RCC->IOPENR,RCC_IOPENR_GPIOFEN); \
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

#if defined (LIBOHIBOARD_STM32G0x1)

#if defined (LIBOHIBOARD_STM32G031)

    {GPIO_PORTS_A,0,0},
    {GPIO_PORTS_A,1,0},
    {GPIO_PORTS_A,2,0},
#if !defined (LIBOHIBOARD_STM32G031JxM)
    {GPIO_PORTS_A,3,0},
    {GPIO_PORTS_A,4,0},
    {GPIO_PORTS_A,5,0},
    {GPIO_PORTS_A,6,0},
    {GPIO_PORTS_A,7,0},
#endif
    {GPIO_PORTS_A,8,0},
#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU) || \
    defined (LIBOHIBOARD_STM32G031KxT) || \
    defined (LIBOHIBOARD_STM32G031KxU)
    {GPIO_PORTS_A,9,0},
    {GPIO_PORTS_A,10,0},
#endif
    {GPIO_PORTS_A,11,0},
    {GPIO_PORTS_A,12,0},
    {GPIO_PORTS_A,13,0},
    {GPIO_PORTS_A,14,0},
    {GPIO_PORTS_A,15,0},

    {GPIO_PORTS_B,0,1},
    {GPIO_PORTS_B,1,1},
#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU) || \
    defined (LIBOHIBOARD_STM32G031KxT) || \
    defined (LIBOHIBOARD_STM32G031KxU) || \
    defined (LIBOHIBOARD_STM32G031FxP) || \
    defined (LIBOHIBOARD_STM32G031YxY)
    {GPIO_PORTS_B,2,1},
#endif
#if !defined (LIBOHIBOARD_STM32G031JxM)
    {GPIO_PORTS_B,3,1},
    {GPIO_PORTS_B,4,1},
#endif
    {GPIO_PORTS_B,5,1},
    {GPIO_PORTS_B,6,1},
    {GPIO_PORTS_B,7,1},
    {GPIO_PORTS_B,8,1},
#if !defined (LIBOHIBOARD_STM32G031GxU)
    {GPIO_PORTS_B,9,1},
#endif
#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU)
    {GPIO_PORTS_B,10,1},
    {GPIO_PORTS_B,11,1},
    {GPIO_PORTS_B,12,1},
    {GPIO_PORTS_B,13,1},
    {GPIO_PORTS_B,14,1},
    {GPIO_PORTS_B,15,1},
#endif

#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU) || \
    defined (LIBOHIBOARD_STM32G031KxT) || \
    defined (LIBOHIBOARD_STM32G031KxU) || \
    defined (LIBOHIBOARD_STM32G031GxU)
    {GPIO_PORTS_C,6,2},
#endif
#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU)
    {GPIO_PORTS_C,7,2},
#endif
#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU)
    {GPIO_PORTS_C,13,2},
#endif
    {GPIO_PORTS_C,14,2},
#if !defined (LIBOHIBOARD_STM32G031JxM)
    {GPIO_PORTS_C,15,2},
#endif

#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU)
    {GPIO_PORTS_D,0,3},
    {GPIO_PORTS_D,1,3},
    {GPIO_PORTS_D,2,3},
    {GPIO_PORTS_D,3,3},
#endif

#if defined (LIBOHIBOARD_STM32G031CxT) || \
    defined (LIBOHIBOARD_STM32G031CxU)
    {GPIO_PORTS_F,0,5},
    {GPIO_PORTS_F,1,5},
#endif
    {GPIO_PORTS_F,2,5},

#endif // LIBOHIBOARD_STM32G031

#endif // LIBOHIBOARD_STM32G0x1

};

static const uint8_t GPIO_AVAILABLE_PINS_COUNT = UTILITY_DIMOF(GPIO_AVAILABLE_PINS);

static GPIO_TypeDef* Gpio_getPort (Gpio_Ports port)
{
    switch (port)
    {
    case GPIO_PORTS_A:
        return GPIOA;

    case GPIO_PORTS_B:
        return GPIOB;

    case GPIO_PORTS_C:
        return GPIOC;

    case GPIO_PORTS_D:
        return GPIOD;

    case GPIO_PORTS_F:
        return GPIOF;

    default:
        ohiassert(0);
        return 0;
    }
}

void Gpio_enablePortClock (Gpio_Ports port)
{
    switch (port)
    {
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

    case GPIO_PORTS_F:
        GPIO_ENABLE_CLOCK_PORTF();
        break;

    default:
        ohiassert(0);
        break;
    }

}

void Gpio_disablePortClock (Gpio_Ports port)
{
    switch (port)
    {
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

    case GPIO_PORTS_F:
        GPIO_DISABLE_CLOCK_PORTF();
        break;

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
            temp &= ~(GPIO_OTYPER_OT0 << number) ;
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
            temp &= ~(GPIO_OSPEEDR_OSPEED0 << (number * 2));
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
        temp &= ~(GPIO_OTYPER_OT0 << number) ;
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
        temp &= ~(GPIO_OSPEEDR_OSPEED0 << (number * 2));
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
        //port->ODR ^= GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber);
        // From RM0444 (pg 243): or atomic bit set/reset, the OD bits can be individually
        // set and/or reset by writing to the GPIOx_BSRR register
        uint32_t odr = port->ODR;
        port->BSRR = ((odr  & GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber)) << 16) |
                      (~odr & GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber));
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

    // Set the current pin as interrupt source for the corresponding channel
    temp = EXTI->EXTICR[pinNumber >> 2];
    temp &= ~(((uint32_t)0x0F) << (8u * (pinNumber & 0x03)));
    temp |= portIndex << (8u * (pinNumber & 0x03));
    EXTI->EXTICR[pinNumber >> 2] = temp;

    // Enable interrupt if request
    temp = EXTI->IMR1;
    temp &= ~((uint32_t)GPIO_PIN(pinNumber));
    if ((event & GPIO_EVENT_USE_INTERRUPT) == GPIO_EVENT_USE_INTERRUPT)
    {
      temp |= (uint32_t)GPIO_PIN(pinNumber);
    }
    EXTI->IMR1 = temp;

    // Enable event if request
    temp = EXTI->EMR1;
    temp &= ~((uint32_t)GPIO_PIN(pinNumber));
    if ((event & GPIO_EVENT_USE_EVENT) == GPIO_EVENT_USE_EVENT)
    {
      temp |= (uint32_t)GPIO_PIN(pinNumber);
    }
    EXTI->EMR1 = temp;

    // Set-up falling and rising edge
    temp = EXTI->RTSR1;
    temp &= ~((uint32_t)GPIO_PIN(pinNumber));
    if ((event & GPIO_EVENT_ON_RISING) == GPIO_EVENT_ON_RISING)
    {
      temp |= (uint32_t)GPIO_PIN(pinNumber);
    }
    EXTI->RTSR1 = temp;

    temp = EXTI->FTSR1;
    temp &= ~((uint32_t)GPIO_PIN(pinNumber));
    if ((event & GPIO_EVENT_ON_FALLING) == GPIO_EVENT_ON_FALLING)
    {
      temp |= (uint32_t)GPIO_PIN(pinNumber);
    }
    EXTI->FTSR1 = temp;

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

    temp = EXTI->EXTICR[pinNumber >> 2];
    temp &= (((uint32_t)0x0F) << (8u * (pinNumber & 0x03)));
    // Disable interrupt only if this pin is enable
    if (temp == (portIndex << (8u * (pinNumber & 0x03))))
    {
        temp = ((uint32_t)0x0F) << (8u * (pinNumber & 0x03));
        EXTI->EXTICR[pinNumber >> 2] &= ~temp;

        // Clear EXTI line configuration
        EXTI->IMR1 &= ~((uint32_t)GPIO_PIN(pinNumber));
        EXTI->EMR1 &= ~((uint32_t)GPIO_PIN(pinNumber));

        // Clear Rising-Falling edge configuration
        EXTI->RTSR1 &= ~((uint32_t)GPIO_PIN(pinNumber));
        EXTI->FTSR1 &= ~((uint32_t)GPIO_PIN(pinNumber));

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

#define GPIO_CLEAR_PENDING_RISING(__PIN_NUMBER__)  (EXTI->RPR1 = (__PIN_NUMBER__))
#define GPIO_CLEAR_PENDING_FALLING(__PIN_NUMBER__) (EXTI->FPR1 = (__PIN_NUMBER__))

#define GPIO_IS_PENDING_RISING(__PIN_NUMBER__)     (EXTI->RPR1 & __PIN_NUMBER__)
#define GPIO_IS_PENDING_FALLING(__PIN_NUMBER__)    (EXTI->FPR1 & __PIN_NUMBER__)

void EXTI0_1_IRQHandler (void)
{
    if ((GPIO_IS_PENDING_RISING(GPIO_PIN(0))) ||
        (GPIO_IS_PENDING_FALLING(GPIO_PIN(0))))
    {
        // Clear pending flag
        GPIO_CLEAR_PENDING_RISING(GPIO_PIN(0));
        GPIO_CLEAR_PENDING_FALLING(GPIO_PIN(0));

        if ((Gpio_isrPortRequestVector[0].callback != nullptr) &&
            (Gpio_isrPortRequestVector[0].enabled == true))
        {
            Gpio_isrPortRequestVector[0].callback(Gpio_isrPortRequestVector[0].pin);
        }
    }

    if ((GPIO_IS_PENDING_RISING(GPIO_PIN(1))) ||
        (GPIO_IS_PENDING_FALLING(GPIO_PIN(1))))
    {
        // Clear pending flag
        GPIO_CLEAR_PENDING_RISING(GPIO_PIN(1));
        GPIO_CLEAR_PENDING_FALLING(GPIO_PIN(1));

        if ((Gpio_isrPortRequestVector[1].callback != nullptr) &&
            (Gpio_isrPortRequestVector[1].enabled == true))
        {
            Gpio_isrPortRequestVector[1].callback(Gpio_isrPortRequestVector[1].pin);
        }
    }
}

void EXTI2_3_IRQHandler (void)
{
    if ((GPIO_IS_PENDING_RISING(GPIO_PIN(2))) ||
        (GPIO_IS_PENDING_FALLING(GPIO_PIN(2))))
    {
        // Clear pending flag
        GPIO_CLEAR_PENDING_RISING(GPIO_PIN(2));
        GPIO_CLEAR_PENDING_FALLING(GPIO_PIN(2));

        if ((Gpio_isrPortRequestVector[2].callback != nullptr) &&
            (Gpio_isrPortRequestVector[2].enabled == true))
        {
            Gpio_isrPortRequestVector[2].callback(Gpio_isrPortRequestVector[2].pin);
        }
    }

    if ((GPIO_IS_PENDING_RISING(GPIO_PIN(3))) ||
        (GPIO_IS_PENDING_FALLING(GPIO_PIN(3))))
    {
        // Clear pending flag
        GPIO_CLEAR_PENDING_RISING(GPIO_PIN(3));
        GPIO_CLEAR_PENDING_FALLING(GPIO_PIN(3));

        if ((Gpio_isrPortRequestVector[3].callback != nullptr) &&
            (Gpio_isrPortRequestVector[3].enabled == true))
        {
            Gpio_isrPortRequestVector[3].callback(Gpio_isrPortRequestVector[3].pin);
        }
    }
}

void EXTI4_15_IRQHandler (void)
{
    for (int i = 4; i < 16; i++)
    {
        if ((GPIO_IS_PENDING_RISING(GPIO_PIN(i))) ||
            (GPIO_IS_PENDING_FALLING(GPIO_PIN(i))))
        {
            // Clear flag interrupt
            GPIO_CLEAR_PENDING_RISING(GPIO_PIN(i));
            GPIO_CLEAR_PENDING_FALLING(GPIO_PIN(i));

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
