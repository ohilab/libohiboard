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

    {GPIO_PORTS_C,0},
    {GPIO_PORTS_C,1},
    {GPIO_PORTS_C,13},
    {GPIO_PORTS_C,14},
    {GPIO_PORTS_C,15},

    {GPIO_PORTS_H,0},
    {GPIO_PORTS_H,1},

#endif
};

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

    // Enable clock and save current port */
    switch (Gpio_availablePins[pin].port)
    {
    case GPIO_PORTS_A:
        GPIO_ENABLE_CLOCK_PORTA();
        port       = GPIOA;
        break;
    }
}


#if 0
System_Errors Gpio_config (Gpio_Pins pin, uint16_t options)
{
    PORT_MemMapPtr port;
    GPIO_MemMapPtr gpioPort;
    uint32_t controlBits = 0;

    /* Enable clock */
    switch (Gpio_availablePins[pin].port)
    {
    case GPIO_PORTS_A:
        SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
        port       = PORTA_BASE_PTR;
        gpioPort   = PTA_BASE_PTR;
        break;
    case GPIO_PORTS_B:
        SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
        port       = PORTB_BASE_PTR;
        gpioPort   = PTB_BASE_PTR;
        break;
    case GPIO_PORTS_C:
        SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
        port       = PORTC_BASE_PTR;
        gpioPort   = PTC_BASE_PTR;
        break;
    case GPIO_PORTS_D:
        SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
        port       = PORTD_BASE_PTR;
        gpioPort   = PTD_BASE_PTR;
        break;
    case GPIO_PORTS_E:
        SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
        port       = PORTE_BASE_PTR;
        gpioPort   = PTE_BASE_PTR;
        break;
    default:
        assert(0);
        return ERRORS_GPIO_WRONG_PORT;
    }

    controlBits = PORT_PCR_MUX(1) | 0;

    /* TODO: Interrupt? */
    if (options & GPIO_PINS_OUTPUT)
    {
        if (options & GPIO_PINS_ENABLE_DRIVE_STRENGTH)
            controlBits |= PORT_PCR_DSE_MASK;

        if (options & GPIO_PINS_ENABLE_SLEW_RATE)
            controlBits |= PORT_PCR_SRE_MASK;
    }
    else if (options & GPIO_PINS_INPUT)
    {
        if (options & GPIO_PINS_ENABLE_PASSIVE_FILTER)
            controlBits |= PORT_PCR_PFE_MASK;

        if (options & GPIO_PINS_PULL)
        {
            controlBits |= PORT_PCR_PE_MASK;
            if (options & GPIO_PINS_ENABLE_PULLUP)
            {
                controlBits |= PORT_PCR_PS_MASK;
            }
            else if (options & GPIO_PINS_ENABLE_PULLDOWN)
            {
                controlBits &= ~PORT_PCR_PS_MASK;
            }
        }
    }
    else
    {
        assert(0);
        return ERRORS_GPIO_WRONG_CONFIG;
    }

    PORT_PCR_REG(port,Gpio_availablePins[pin].pinNumber) = controlBits;

    if (options & GPIO_PINS_OUTPUT)
    {
        gpioPort->PDDR |= GPIO_PIN(Gpio_availablePins[pin].pinNumber);
    }
    else if(options & GPIO_PINS_INPUT)
	{
        gpioPort->PDDR &= ~GPIO_PIN(Gpio_availablePins[pin].pinNumber);
	}

    return ERRORS_NO_ERROR;
}

void Gpio_set (Gpio_Pins pin)
{
    GPIO_MemMapPtr port;
    Gpio_getPort(pin,&port);

    port->PSOR = GPIO_PIN(Gpio_availablePins[pin].pinNumber);
}

void Gpio_clear (Gpio_Pins pin)
{
    GPIO_MemMapPtr port;
    Gpio_getPort(pin,&port);

    port->PCOR = GPIO_PIN(Gpio_availablePins[pin].pinNumber);
}

void Gpio_toggle (Gpio_Pins pin)
{
    GPIO_MemMapPtr port;
    Gpio_getPort(pin,&port);

    port->PTOR = GPIO_PIN(Gpio_availablePins[pin].pinNumber);
}

Gpio_Level Gpio_get (Gpio_Pins pin)
{
    GPIO_MemMapPtr port;
    Gpio_getPort(pin,&port);

    return ((port->PDIR & GPIO_PIN(Gpio_availablePins[pin].pinNumber)) > 0) ? GPIO_HIGH : GPIO_LOW;
}

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
