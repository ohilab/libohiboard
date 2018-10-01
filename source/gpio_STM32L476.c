/*
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/gpio_STM32L476.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO implementations for STM32L476.
 */

#if defined (LIBOHIBOARD_STM32L476)

#include "gpio.h"
#include "platforms.h"
#include "interrupt.h"
#include "clock.h"
#include "utility.h"

#define GPIO_ENABLE_CLOCK_PORTA() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOAEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOAEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTB() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOBEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOBEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTC() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOCEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOCEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTD() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIODEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIODEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTE() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOEEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOEEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTF() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOFEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOFEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTG() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOGEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOGEN); \
                                  } while (0)

#define GPIO_ENABLE_CLOCK_PORTH() do { \
                                    UTILITY_SET_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOHEN); \
                                    asm("nop"); \
                                    (void) UTILITY_READ_REGISTER_BIT(RCC->AHB2ENR,RCC_AHB2ENR_GPIOHEN); \
                                  } while (0)

#define  PORT_MAX_PIN  16

static void (*Gpio_isrPortRequestVector[PORT_MAX_PIN]) (void);
static uint32_t Gpio_isrRegister = 0x0;


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

typedef struct _Gpio_PinDevice
{
    Gpio_Ports port;                      /**< The port of the selected pins */
    uint8_t pinNumber;       /**< The number of the pin of the relative port */
    uint8_t portIndex;       /**< Current port number for interrupt managing */
} Gpio_PinDevice;

static Gpio_PinDevice Gpio_availablePins[] =
{
    {0xFF,0xFF,0},

#if defined (LIBOHIBOARD_STM32L476Jx) // WLCSP72 ballout

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

    {GPIO_PORTS_C,0,2},
    {GPIO_PORTS_C,1,2},
    {GPIO_PORTS_C,2,2},
    {GPIO_PORTS_C,3,2},
    {GPIO_PORTS_C,4,2},
    {GPIO_PORTS_C,5,2},
    {GPIO_PORTS_C,6,2},
    {GPIO_PORTS_C,7,2},
    {GPIO_PORTS_C,8,2},
    {GPIO_PORTS_C,9,2},
    {GPIO_PORTS_C,10,2},
    {GPIO_PORTS_C,11,2},
    {GPIO_PORTS_C,12,2},
    {GPIO_PORTS_C,13,2},
    {GPIO_PORTS_C,14,2},
    {GPIO_PORTS_C,15,2},

    {GPIO_PORTS_D,2,3},

    {GPIO_PORTS_G,9,6},
    {GPIO_PORTS_G,10,6},
    {GPIO_PORTS_G,11,6},
    {GPIO_PORTS_G,12,6},
    {GPIO_PORTS_G,13,6},
    {GPIO_PORTS_G,14,6},

    {GPIO_PORTS_H,0,7},
    {GPIO_PORTS_H,1,7},

#endif
};

static uint8_t Gpio_availablePinsCount = 0;

static GPIO_TypeDef* Gpio_getPort (Gpio_Pins pin)
{
    switch (Gpio_availablePins[pin].port)
    {
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

    case GPIO_PORTS_F:
        return GPIOF;

    case GPIO_PORTS_G:
        return GPIOG;

    case GPIO_PORTS_H:
        return GPIOH;

    default:
        ohiassert(0);
        return 0;
    }
}

System_Errors Gpio_config (Gpio_Pins pin, uint16_t options)
{
    GPIO_TypeDef* port;
    uint32_t temp = 0x00;

    // Pin number into the current port
    uint8_t number = 0;

    // Only one type of configuration is possible
    ohiassert(((options & GPIO_PINS_OUTPUT) == GPIO_PINS_OUTPUT) ^
              ((options & GPIO_PINS_INPUT) == GPIO_PINS_INPUT));


    if (Gpio_availablePinsCount == 0)
        Gpio_availablePinsCount = sizeof(Gpio_availablePins)/sizeof(Gpio_availablePins[0]);
    //Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);

    number = Gpio_availablePins[pin].pinNumber;

    // Enable clock and save current port */
    switch (Gpio_availablePins[pin].port)
    {
    case GPIO_PORTS_A:
        GPIO_ENABLE_CLOCK_PORTA();
        port = GPIOA;
        break;
    case GPIO_PORTS_B:
        GPIO_ENABLE_CLOCK_PORTB();
        port = GPIOB;
        break;
    case GPIO_PORTS_C:
        GPIO_ENABLE_CLOCK_PORTC();
        port = GPIOC;
        break;
    case GPIO_PORTS_D:
        GPIO_ENABLE_CLOCK_PORTD();
        port = GPIOD;
        break;
    case GPIO_PORTS_E:
        GPIO_ENABLE_CLOCK_PORTE();
        port = GPIOE;
        break;
    case GPIO_PORTS_F:
        GPIO_ENABLE_CLOCK_PORTF();
        port = GPIOF;
        break;
    case GPIO_PORTS_G:
        GPIO_ENABLE_CLOCK_PORTG();
        port = GPIOG;
        break;
    case GPIO_PORTS_H:
        GPIO_ENABLE_CLOCK_PORTH();
        port = GPIOH;
        break;
    default:
        ohiassert(0);
    }

    // Configure direction mode
    temp = port->MODER;
    temp &= ~(GPIO_MODER_MODE0 << (number * 2));
    temp |= (((options & GPIO_PINS_INPUT) ? 0x00 : 0x01) << (number * 2));
    port->MODER = temp;

    if (options & GPIO_PINS_OUTPUT)
    {
        // Only one type of configuration is possible
        ohiassert(((options & GPIO_PINS_ENABLE_OUTPUT_PUSHPULL) == GPIO_PINS_ENABLE_OUTPUT_PUSHPULL) ^
                  ((options & GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN) == GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN));

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
    //Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);

    GPIO_TypeDef* port = Gpio_getPort(pin);
    port->BSRR = GPIO_PIN(Gpio_availablePins[pin].pinNumber);
}

void Gpio_clear (Gpio_Pins pin)
{
    //Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);

    GPIO_TypeDef* port = Gpio_getPort(pin);
    port->BRR = GPIO_PIN(Gpio_availablePins[pin].pinNumber);
}

void Gpio_toggle (Gpio_Pins pin)
{
    //Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);

    GPIO_TypeDef* port = Gpio_getPort(pin);
    port->ODR ^= GPIO_PIN(Gpio_availablePins[pin].pinNumber);
}

Gpio_Level Gpio_get (Gpio_Pins pin)
{
    //Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);

    GPIO_TypeDef* port = Gpio_getPort(pin);
    return ((port->IDR & GPIO_PIN(Gpio_availablePins[pin].pinNumber)) > 0) ? GPIO_HIGH : GPIO_LOW;
}

System_Errors Gpio_configInterrupt (Gpio_Pins pin, void* callback)
{
    //Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);
    if (pin < Gpio_availablePinsCount)
        return ERRORS_GPIO_WRONG_PIN;

    Gpio_isrPortRequestVector[Gpio_availablePins[pin].pinNumber] = callback;
    Gpio_isrRegister |= 1 << Gpio_availablePins[pin].pinNumber;

    return ERRORS_NO_ERROR;
}

System_Errors Gpio_enableInterrupt (Gpio_Pins pin, Gpio_EventType event)
{
    uint32_t pinNumber = Gpio_availablePins[pin].pinNumber;
    uint32_t portIndex = Gpio_availablePins[pin].portIndex;
    uint32_t temp = 0x00;

    //Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);
    if (pin < Gpio_availablePinsCount)
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

    // Enable NVIC interrupt
    switch (pinNumber)
    {
    case 0:
        Interrupt_enable(INTERRUPT_EXTI0);
        break;
    case 1:
        Interrupt_enable(INTERRUPT_EXTI1);
        break;
    case 2:
        Interrupt_enable(INTERRUPT_EXTI2);
        break;
    case 3:
        Interrupt_enable(INTERRUPT_EXTI3);
        break;
    case 4:
        Interrupt_enable(INTERRUPT_EXTI4);
        break;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
        Interrupt_enable(INTERRUPT_EXTI9_5);
        break;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        Interrupt_enable(INTERRUPT_EXTI15_10);
        break;
    }

    return ERRORS_NO_ERROR;
}

System_Errors Gpio_disableInterrupt (Gpio_Pins pin)
{
    uint32_t pinNumber = Gpio_availablePins[pin].pinNumber;
    uint32_t portIndex = Gpio_availablePins[pin].portIndex;
    uint32_t temp = 0x00;

    //Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);
    if (pin < Gpio_availablePinsCount)
        return ERRORS_GPIO_WRONG_PIN;

    temp = SYSCFG->EXTICR[pinNumber >> 2];
    temp &= (((uint32_t)0x0F) << (4 * (pinNumber & 0x03)));
    // Disable interrupt only if this pin is enable
    if(temp == (portIndex << (4 * (pinNumber & 0x03))))
    {
        temp = ((uint32_t)0x0F) << (4 * (pinNumber & 0x03));
        SYSCFG->EXTICR[pinNumber >> 2] &= ~temp;

        // Clear EXTI line configuration
        EXTI->IMR1 &= ~((uint32_t)GPIO_PIN(pinNumber));
        EXTI->EMR1 &= ~((uint32_t)GPIO_PIN(pinNumber));

        // Clear Rising-Falling edge configuration
        EXTI->RTSR1 &= ~((uint32_t)GPIO_PIN(pinNumber));
        EXTI->FTSR1 &= ~((uint32_t)GPIO_PIN(pinNumber));

        // Disable NVIC interrupt
        switch (pinNumber)
        {
        case 0:
            Interrupt_disable(INTERRUPT_EXTI0);
            break;
        case 1:
            Interrupt_disable(INTERRUPT_EXTI1);
            break;
        case 2:
            Interrupt_disable(INTERRUPT_EXTI2);
            break;
        case 3:
            Interrupt_disable(INTERRUPT_EXTI3);
            break;
        case 4:
            Interrupt_disable(INTERRUPT_EXTI4);
            break;
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            Interrupt_disable(INTERRUPT_EXTI9_5);
            break;
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
            Interrupt_disable(INTERRUPT_EXTI15_10);
            break;
        }
    }
    return ERRORS_NO_ERROR;
}

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_EXTI0
void EXTI0_IRQHandler (void)
{

}
#endif

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_EXTI1
void EXTI1_IRQHandler (void)
{

}
#endif

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_EXTI2
void EXTI2_IRQHandler (void)
{

}
#endif

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_EXTI3
void EXTI3_IRQHandler (void)
{

}
#endif

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_EXTI4
void EXTI4_IRQHandler (void)
{

}
#endif

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_EXTI9_5
void EXTI9_5_IRQHandler (void)
{

}
#endif

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_EXTI15_10
void EXTI15_10_IRQHandler (void)
{

}
#endif


#if 0


void PORTA_IRQHandler (void)
{
    uint8_t i=0;

    while (i < PORTA_MAX_PIN)
    {
        if(INT_REG_A & (1 << i))
        {
            if (PORTA_PCR(i) & PORT_PCR_ISF_MASK)
            {
                Gpio_isrPortARequestVector[i]();
                //reset interrupt
                PORTA_PCR(i) |= PORT_PCR_ISF_MASK;
            }
        }
        i++;
    }
}

void PORTD_IRQHandler (void)
{
    uint8_t i=0;

    while (i < PORTD_MAX_PIN)
    {
        if(INT_REG_D & (1 << i))
        {
            if (PORTD_PCR(i) & PORT_PCR_ISF_MASK)
            {
                Gpio_isrPortDRequestVector[i]();
                //reset interrupt
                PORTD_PCR(i) |= PORT_PCR_ISF_MASK;
            }
        }
        i++;
    }
}
#endif

#endif // LIBOHIBOARD_STM32L476
