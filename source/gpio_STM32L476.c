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

#if 0
//* TODO: Verificare che siano giuste le define */
#define  PORTA_MAX_PIN  21
#define  PORTD_MAX_PIN  8

static void (*Gpio_isrPortARequestVector[PORTA_MAX_PIN]) (void);
static void (*Gpio_isrPortDRequestVector[PORTD_MAX_PIN]) (void);

static uint32_t INT_REG_A = 0x0;
static uint32_t INT_REG_D = 0x0;
#endif

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
} Gpio_PinDevice;

static Gpio_PinDevice Gpio_availablePins[] =
{
    {0xFF,0xFF},

#if defined (LIBOHIBOARD_STM32L476Jx) // WLCSP72 ballout

    {GPIO_PORTS_A,0},
    {GPIO_PORTS_A,1},
    {GPIO_PORTS_A,2},
    {GPIO_PORTS_A,3},
    {GPIO_PORTS_A,4},
    {GPIO_PORTS_A,5},
    {GPIO_PORTS_A,6},
    {GPIO_PORTS_A,7},
    {GPIO_PORTS_A,8},
    {GPIO_PORTS_A,9},
    {GPIO_PORTS_A,10},
    {GPIO_PORTS_A,11},
    {GPIO_PORTS_A,12},
    {GPIO_PORTS_A,13},
    {GPIO_PORTS_A,14},
    {GPIO_PORTS_A,15},

    {GPIO_PORTS_B,0},
    {GPIO_PORTS_B,1},
    {GPIO_PORTS_B,2},
    {GPIO_PORTS_B,3},
    {GPIO_PORTS_B,4},
    {GPIO_PORTS_B,5},
    {GPIO_PORTS_B,6},
    {GPIO_PORTS_B,7},
    {GPIO_PORTS_B,8},
    {GPIO_PORTS_B,9},
    {GPIO_PORTS_B,10},
    {GPIO_PORTS_B,11},
    {GPIO_PORTS_B,12},
    {GPIO_PORTS_B,13},
    {GPIO_PORTS_B,14},
    {GPIO_PORTS_B,15},

    {GPIO_PORTS_C,0},
    {GPIO_PORTS_C,1},
    {GPIO_PORTS_C,2},
    {GPIO_PORTS_C,3},
    {GPIO_PORTS_C,4},
    {GPIO_PORTS_C,5},
    {GPIO_PORTS_C,6},
    {GPIO_PORTS_C,7},
    {GPIO_PORTS_C,8},
    {GPIO_PORTS_C,9},
    {GPIO_PORTS_C,10},
    {GPIO_PORTS_C,11},
    {GPIO_PORTS_C,12},
    {GPIO_PORTS_C,13},
    {GPIO_PORTS_C,14},
    {GPIO_PORTS_C,15},

    {GPIO_PORTS_D,2},

    {GPIO_PORTS_G,9},
    {GPIO_PORTS_G,10},
    {GPIO_PORTS_G,11},
    {GPIO_PORTS_G,12},
    {GPIO_PORTS_G,13},
    {GPIO_PORTS_G,14},


    {GPIO_PORTS_H,0},
    {GPIO_PORTS_H,1},

#endif
};

static uint8_t Gpio_availablePinsCount = sizeof(Gpio_availablePins)/sizeof(Gpio_availablePins[0]);

static void Gpio_getPort (Gpio_Pins pin, GPIO_TypeDef* port)
{
    switch (Gpio_availablePins[pin].port)
    {
    case GPIO_PORTS_A:
        port = GPIOA;
        break;
    case GPIO_PORTS_B:
        port = GPIOB;
        break;
    case GPIO_PORTS_C:
        port = GPIOC;
        break;
    case GPIO_PORTS_D:
        port = GPIOD;
        break;
    case GPIO_PORTS_E:
        port = GPIOE;
        break;
    case GPIO_PORTS_F:
        port = GPIOF;
        break;
    case GPIO_PORTS_G:
        port = GPIOG;
        break;
    case GPIO_PORTS_H:
        port = GPIOH;
        break;
    default:
        ohiassert(0);
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
        temp = port->OTYPER;
        temp &= ~(GPIO_OTYPER_OT0 << number) ;
        temp |= (((options & GPIO_PINS_ENABLE_OUTPUT_PUSHPULL) ? 0x00 : 0x01) << number);
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
    GPIO_TypeDef* port;

    // Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);

    Gpio_getPort(pin,port);
    port->BSRR = GPIO_PIN(Gpio_availablePins[pin].pinNumber);
}

void Gpio_clear (Gpio_Pins pin)
{
    GPIO_TypeDef* port;

    // Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);

    Gpio_getPort(pin,port);
    port->BRR = GPIO_PIN(Gpio_availablePins[pin].pinNumber);
}

void Gpio_toggle (Gpio_Pins pin)
{
    GPIO_TypeDef* port;

    // Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);

    Gpio_getPort(pin,port);
    port->ODR = GPIO_PIN(Gpio_availablePins[pin].pinNumber);
}

Gpio_Level Gpio_get (Gpio_Pins pin)
{
    GPIO_TypeDef* port;

    // Check if pin definition exist
    ohiassert(pin < Gpio_availablePinsCount);

    Gpio_getPort(pin,port);
    return ((port->IDR & GPIO_PIN(Gpio_availablePins[pin].pinNumber)) > 0) ? GPIO_HIGH : GPIO_LOW;
}

#if 0

System_Errors Gpio_configInterrupt (Gpio_Pins pin, void* callback)
{
    GPIO_MemMapPtr port;
    Gpio_getPort(pin,&port);

    switch(Gpio_availablePins[pin].port)
    {
    case GPIO_PORTS_A:
        Gpio_isrPortARequestVector[Gpio_availablePins[pin].pinNumber] = callback;
        INT_REG_A |= 1 << Gpio_availablePins[pin].pinNumber;
        break;
    case GPIO_PORTS_D:
        Gpio_isrPortDRequestVector[Gpio_availablePins[pin].pinNumber] = callback;
        INT_REG_D |= 1 << Gpio_availablePins[pin].pinNumber;
        break;
    default:
        assert(0);
        return ERRORS_GPIO_WRONG_PORT;
    }

    return ERRORS_NO_ERROR;
}

System_Errors Gpio_enableInterrupt (Gpio_Pins pin, Gpio_EventType event)
{
    GPIO_MemMapPtr port;
    Gpio_getPort(pin,&port);

    switch(Gpio_availablePins[pin].port)
    {
    case GPIO_PORTS_A:
        PORTA_PCR(Gpio_availablePins[pin].pinNumber) &= ~PORT_PCR_IRQC_MASK;
        PORTA_PCR(Gpio_availablePins[pin].pinNumber)|=PORT_PCR_IRQC(event)|PORT_PCR_MUX(0x1);
        INT_REG_A |= 1 << Gpio_availablePins[pin].pinNumber;
        Interrupt_enable (INTERRUPT_PORTA);
        break;
    case GPIO_PORTS_D:
        PORTD_PCR(Gpio_availablePins[pin].pinNumber) &= ~PORT_PCR_IRQC_MASK;
        PORTD_PCR(Gpio_availablePins[pin].pinNumber) |= PORT_PCR_IRQC(event)|PORT_PCR_MUX(0x1);
        INT_REG_D |= 1 << Gpio_availablePins[pin].pinNumber;
        Interrupt_enable (INTERRUPT_PORTD);
        break;
    default:
        assert(0);
        return ERRORS_GPIO_WRONG_PORT;
    }

    return ERRORS_NO_ERROR;
}

System_Errors Gpio_disableInterrupt (Gpio_Pins pin)
{
    GPIO_MemMapPtr port;
    Gpio_getPort(pin,&port);

    switch(Gpio_availablePins[pin].port)
    {
    case GPIO_PORTS_A:
        PORTA_PCR(Gpio_availablePins[pin].pinNumber) &= ~PORT_PCR_IRQC_MASK;
        INT_REG_A &= ~(1 << Gpio_availablePins[pin].pinNumber);
        if (!INT_REG_A) Interrupt_disable(INTERRUPT_PORTA);
        break;
    case GPIO_PORTS_D:
        PORTD_PCR(Gpio_availablePins[pin].pinNumber) &= ~PORT_PCR_IRQC_MASK;
        INT_REG_D &= ~(1 << Gpio_availablePins[pin].pinNumber);
        if (!INT_REG_D) Interrupt_disable(INTERRUPT_PORTD);
        break;
    default:
        assert(0);
        return ERRORS_GPIO_WRONG_PORT;
    }

    return ERRORS_NO_ERROR;
}

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
