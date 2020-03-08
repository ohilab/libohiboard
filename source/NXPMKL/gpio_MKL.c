/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2014-2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/gpio_MKL.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO implementations for NXP MKL series.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_MKL)

#include "gpio.h"
#include "interrupt.h"
#include "utility.h"

#define  PORT_MAX_PIN  32

#define GPIO_VALID_ISR_EVENT(EVENT) (((EVENT) == GPIO_EVENT_WHEN_0)     || \
                                     ((EVENT) == GPIO_EVENT_ON_RISING)  || \
                                     ((EVENT) == GPIO_EVENT_ON_FALLING) || \
                                     ((EVENT) == GPIO_EVENT_WHEN_1)     || \
                                     ((EVENT) == GPIO_EVENT_ON_BOOTH))

typedef void (*Gpio_IsrCallback)(Gpio_Pins pin);

typedef struct _Gpio_IsrPinCallback
{
    bool enabled;
    Gpio_Pins pin;
    Gpio_IsrCallback callback;
} Gpio_IsrPinCallback;

// Callback array for each usable port
static Gpio_IsrPinCallback Gpio_isrPortARequestVector[GPIO_MAX_PINS_NUMBER_FOR_PORT] = {0};
static Gpio_IsrPinCallback Gpio_isrPortDRequestVector[GPIO_MAX_PINS_NUMBER_FOR_PORT] = {0};

// Useful variable to store enabled ISR pins
static uint32_t Gpio_isrRegisterPortA = 0x0;
static uint32_t Gpio_isrRegisterPortD = 0x0;

/**
 * Struct to represent each pins.
 */
typedef struct _Gpio_PinDevice
{
    Gpio_Ports port;                      /**< The port of the selected pins */
    uint8_t pinNumber;       /**< The number of the pin of the relative port */
} Gpio_PinDevice;

/**
 * List of available couple pin/port.
 */
static const Gpio_PinDevice GPIO_AVAILABLE_PINS[] =
{
    {0xFF,0xFF},

#if defined (LIBOHIBOARD_MKL15)

    {GPIO_PORTS_A,0},
    {GPIO_PORTS_A,1},
    {GPIO_PORTS_A,2},
    {GPIO_PORTS_A,3},
    {GPIO_PORTS_A,4},
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_A,5},
    {GPIO_PORTS_A,12},
    {GPIO_PORTS_A,13},
#endif
#if defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_A,14},
    {GPIO_PORTS_A,15},
    {GPIO_PORTS_A,16},
    {GPIO_PORTS_A,17},
#endif
    {GPIO_PORTS_A,18},
    {GPIO_PORTS_A,19},
    {GPIO_PORTS_A,20},

    {GPIO_PORTS_B,0},
    {GPIO_PORTS_B,1},
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_B,2},
    {GPIO_PORTS_B,3},
#endif
#if defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_B,8},
    {GPIO_PORTS_B,9},
    {GPIO_PORTS_B,10},
    {GPIO_PORTS_B,11},
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_B,16},
    {GPIO_PORTS_B,17},
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_B,18},
    {GPIO_PORTS_B,19},
#endif

#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_C,0},
#endif
    {GPIO_PORTS_C,1},
    {GPIO_PORTS_C,2},
    {GPIO_PORTS_C,3},
    {GPIO_PORTS_C,4},
    {GPIO_PORTS_C,5},
    {GPIO_PORTS_C,6},
    {GPIO_PORTS_C,7},
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_C,8},
    {GPIO_PORTS_C,9},
    {GPIO_PORTS_C,10},
    {GPIO_PORTS_C,11},
#endif
#if defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_C,12},
    {GPIO_PORTS_C,13},
    {GPIO_PORTS_C,16},
    {GPIO_PORTS_C,17},
#endif

#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_D,0},
    {GPIO_PORTS_D,1},
    {GPIO_PORTS_D,2},
    {GPIO_PORTS_D,3},
#endif
    {GPIO_PORTS_D,4},
    {GPIO_PORTS_D,5},
    {GPIO_PORTS_D,6},
    {GPIO_PORTS_D,7},

#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_E,0},
    {GPIO_PORTS_E,1},
#endif
#if defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_E,2},
    {GPIO_PORTS_E,3},
    {GPIO_PORTS_E,4},
    {GPIO_PORTS_E,5},
#endif
    {GPIO_PORTS_E,16},
    {GPIO_PORTS_E,17},
    {GPIO_PORTS_E,18},
    {GPIO_PORTS_E,19},
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_E,20},
    {GPIO_PORTS_E,21},
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_E,22},
    {GPIO_PORTS_E,23},
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_E,24},
    {GPIO_PORTS_E,25},
    {GPIO_PORTS_E,29},
#endif
    {GPIO_PORTS_E,30},
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    {GPIO_PORTS_E,31},
#endif

// ENDIF LIBOHIBOARD_MKL15
#elif defined (LIBOHIBOARD_MKL25)

    {GPIO_PORTS_A,0},
    {GPIO_PORTS_A,1},
    {GPIO_PORTS_A,2},
    {GPIO_PORTS_A,3},
    {GPIO_PORTS_A,4},
#if defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_A,5},
    {GPIO_PORTS_A,12},
    {GPIO_PORTS_A,13},
#endif
#if defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_A,14},
    {GPIO_PORTS_A,15},
    {GPIO_PORTS_A,16},
    {GPIO_PORTS_A,17},
#endif
    {GPIO_PORTS_A,18},
    {GPIO_PORTS_A,19},
    {GPIO_PORTS_A,20},

    {GPIO_PORTS_B,0},
    {GPIO_PORTS_B,1},
#if defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_B,2},
    {GPIO_PORTS_B,3},
#endif
#if defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_B,8},
    {GPIO_PORTS_B,9},
    {GPIO_PORTS_B,10},
    {GPIO_PORTS_B,11},
#endif
#if defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_B,16},
    {GPIO_PORTS_B,17},
#endif
#if defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_B,18},
    {GPIO_PORTS_B,19},
#endif

#if defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_C,0},
#endif
    {GPIO_PORTS_C,1},
    {GPIO_PORTS_C,2},
    {GPIO_PORTS_C,3},
    {GPIO_PORTS_C,4},
    {GPIO_PORTS_C,5},
    {GPIO_PORTS_C,6},
    {GPIO_PORTS_C,7},
#if defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_C,8},
    {GPIO_PORTS_C,9},
    {GPIO_PORTS_C,10},
    {GPIO_PORTS_C,11},
#endif
#if defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_C,12},
    {GPIO_PORTS_C,13},
    {GPIO_PORTS_C,16},
    {GPIO_PORTS_C,17},
#endif

#if defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_D,0},
    {GPIO_PORTS_D,1},
    {GPIO_PORTS_D,2},
    {GPIO_PORTS_D,3},
#endif
    {GPIO_PORTS_D,4},
    {GPIO_PORTS_D,5},
    {GPIO_PORTS_D,6},
    {GPIO_PORTS_D,7},

#if defined (LIBOHIBOARD_MKL25ZxFM) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_E,0},
#endif
#if defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_E,1},
#endif
#if defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_E,2},
    {GPIO_PORTS_E,3},
    {GPIO_PORTS_E,4},
    {GPIO_PORTS_E,5},
#endif
#if defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_E,20},
    {GPIO_PORTS_E,21},
#endif
#if defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_E,22},
    {GPIO_PORTS_E,23},
#endif
#if defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_E,24},
    {GPIO_PORTS_E,25},
    {GPIO_PORTS_E,29},
#endif
    {GPIO_PORTS_E,30},
#if defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
    {GPIO_PORTS_E,31},
#endif

// ENDIF LIBOHIBOARD_MKL15
#endif
};

static const uint8_t GPIO_AVAILABLE_PINS_COUNT = UTILITY_DIMOF(GPIO_AVAILABLE_PINS);

/**
 * This function return the pointer to the registers
 * of the selected GPIO port.
 */
static GPIO_Type* Gpio_getGpioRegister (Gpio_Ports port)
{
    switch (port)
    {
    case GPIO_PORTS_A:
        return GPIOA;
        break;
    case GPIO_PORTS_B:
        return GPIOB;
        break;
    case GPIO_PORTS_C:
        return GPIOC;
        break;
    case GPIO_PORTS_D:
        return GPIOD;
        break;
    case GPIO_PORTS_E:
        return GPIOE;
        break;
    default:
        ohiassert(0);
        return 0;
    }
}

/**
 * This function return the pointer to the port registers
 * of the selected GPIO port.
 */
static PORT_Type* Gpio_getPortRegister (Gpio_Ports port)
{
    switch (port)
    {
    case GPIO_PORTS_A:
        return PORTA;
        break;
    case GPIO_PORTS_B:
        return PORTB;
        break;
    case GPIO_PORTS_C:
        return PORTC;
        break;
    case GPIO_PORTS_D:
        return PORTD;
        break;
    case GPIO_PORTS_E:
        return PORTE;
        break;
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
        SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
        break;

    case GPIO_PORTS_B:
        SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
        break;

    case GPIO_PORTS_C:
        SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
        break;

    case GPIO_PORTS_D:
        SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
        break;

    case GPIO_PORTS_E:
        SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
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
        SIM->SCGC5 &= ~SIM_SCGC5_PORTA_MASK;
        break;

    case GPIO_PORTS_B:
        SIM->SCGC5 &= ~SIM_SCGC5_PORTB_MASK;
        break;

    case GPIO_PORTS_C:
        SIM->SCGC5 &= ~SIM_SCGC5_PORTC_MASK;
        break;

    case GPIO_PORTS_D:
        SIM->SCGC5 &= ~SIM_SCGC5_PORTD_MASK;
        break;

    case GPIO_PORTS_E:
        SIM->SCGC5 &= ~SIM_SCGC5_PORTE_MASK;
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
        ohiassert(0);
        return; //ERRORS_GPIO_NULL_PIN;
    }

    GPIO_Type* gpio;
    PORT_Type* port;
    // Pin number into the current port
    uint8_t number = 0;

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

    // Check the alternate value: it have 16 possibility
    ohiassert(alternate < 16);

    // Enable clock and save current port
    Gpio_enablePortClock(GPIO_AVAILABLE_PINS[pin].port);
    //gpio = Gpio_getGpioRegister(GPIO_AVAILABLE_PINS[pin].port);
    port = Gpio_getPortRegister(GPIO_AVAILABLE_PINS[pin].port);

    number = GPIO_AVAILABLE_PINS[pin].pinNumber;

    if (alternate >= 0)
    {
        port->PCR[number] = PORT_PCR_MUX(alternate) | PORT_PCR_IRQC(0);
    }
    else
    {
        ohiassert(0);
    }
}

System_Errors Gpio_config (Gpio_Pins pin, uint16_t options)
{
    if (pin == GPIO_PINS_NONE)
    {
        return ERRORS_GPIO_NULL_PIN;
    }

    GPIO_Type* gpio;
    PORT_Type* port;

    // Only one type of configuration is possible
    ohiassert(((options & GPIO_PINS_OUTPUT) == GPIO_PINS_OUTPUT) ^
              ((options & GPIO_PINS_INPUT) == GPIO_PINS_INPUT));

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

    uint8_t number = GPIO_AVAILABLE_PINS[pin].pinNumber;

    // Enable clock and save current port
    Gpio_enablePortClock(GPIO_AVAILABLE_PINS[pin].port);
    gpio = Gpio_getGpioRegister(GPIO_AVAILABLE_PINS[pin].port);
    port = Gpio_getPortRegister(GPIO_AVAILABLE_PINS[pin].port);

    uint32_t controlBits = 0 | PORT_PCR_MUX(1);
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
    port->PCR[number] = controlBits;

    if (options & GPIO_PINS_OUTPUT)
    {
        gpio->PDDR |= GPIO_PIN(number);
    }
    else
    {
        gpio->PDDR &= ~GPIO_PIN(number);
    }

    return ERRORS_NO_ERROR;
}

void Gpio_set (Gpio_Pins pin)
{
    if (pin != GPIO_PINS_NONE)
    {
        //Check if pin definition exist
        ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

        GPIO_Type* port = Gpio_getGpioRegister(GPIO_AVAILABLE_PINS[pin].port);
        port->PSOR = GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber);
    }
}

void Gpio_clear (Gpio_Pins pin)
{
    if (pin != GPIO_PINS_NONE)
    {
        //Check if pin definition exist
        ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

        GPIO_Type* port = Gpio_getGpioRegister(GPIO_AVAILABLE_PINS[pin].port);
        port->PCOR = GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber);
    }
}

void Gpio_toggle (Gpio_Pins pin)
{
    if (pin != GPIO_PINS_NONE)
    {
        //Check if pin definition exist
        ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

        GPIO_Type* port = Gpio_getGpioRegister(GPIO_AVAILABLE_PINS[pin].port);
        port->PTOR = GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber);
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

    GPIO_Type* port = Gpio_getGpioRegister(GPIO_AVAILABLE_PINS[pin].port);
    return ((port->PDIR & GPIO_PIN(GPIO_AVAILABLE_PINS[pin].pinNumber)) > 0) ? GPIO_HIGH : GPIO_LOW;
}

System_Errors Gpio_configInterrupt (Gpio_Pins pin, Interrupt_Priority priority, void* callback)
{
    (void)priority; //FIXME: manage interrupt priority

    if (pin == GPIO_PINS_NONE)
    {
        return ERRORS_GPIO_NULL_PIN;
    }

    // Check callback value
    ohiassert(callback != 0);

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);
    if (pin >= GPIO_AVAILABLE_PINS_COUNT)
        return ERRORS_GPIO_WRONG_PIN;

    switch (GPIO_AVAILABLE_PINS[pin].port)
    {
    case GPIO_PORTS_A:
        Gpio_isrPortARequestVector[GPIO_AVAILABLE_PINS[pin].pinNumber].pin      = pin;
        Gpio_isrPortARequestVector[GPIO_AVAILABLE_PINS[pin].pinNumber].callback = callback;
        Gpio_isrRegisterPortA |= 1 << GPIO_AVAILABLE_PINS[pin].pinNumber;
        break;
    case GPIO_PORTS_D:
        Gpio_isrPortDRequestVector[GPIO_AVAILABLE_PINS[pin].pinNumber].pin      = pin;
        Gpio_isrPortDRequestVector[GPIO_AVAILABLE_PINS[pin].pinNumber].callback = callback;
        Gpio_isrRegisterPortD |= 1 << GPIO_AVAILABLE_PINS[pin].pinNumber;
        break;
    default:
        ohiassert(0);
        return ERRORS_GPIO_WRONG_PORT;
    }
    return ERRORS_NO_ERROR;
}

System_Errors Gpio_enableInterrupt (Gpio_Pins pin, Gpio_EventType event)
{
    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);
    if (pin >= GPIO_AVAILABLE_PINS_COUNT)
        return ERRORS_GPIO_WRONG_PIN;

    ohiassert(GPIO_VALID_ISR_EVENT(event));

    PORT_Type* port = Gpio_getPortRegister(GPIO_AVAILABLE_PINS[pin].port);

    switch(GPIO_AVAILABLE_PINS[pin].port)
    {
    case GPIO_PORTS_A:
        port->PCR[GPIO_AVAILABLE_PINS[pin].pinNumber] &= ~PORT_PCR_IRQC_MASK;
        port->PCR[GPIO_AVAILABLE_PINS[pin].pinNumber] |= PORT_PCR_IRQC(event)|PORT_PCR_MUX(0x1);
        Gpio_isrPortARequestVector[GPIO_AVAILABLE_PINS[pin].pinNumber].enabled = true;
        //Gpio_isrRegisterPortA |= 1 << GPIO_AVAILABLE_PINS[pin].pinNumber;
        Interrupt_enable (INTERRUPT_PORTA);
        break;
    case GPIO_PORTS_D:
        port->PCR[GPIO_AVAILABLE_PINS[pin].pinNumber] &= ~PORT_PCR_IRQC_MASK;
        port->PCR[GPIO_AVAILABLE_PINS[pin].pinNumber] |= PORT_PCR_IRQC(event)|PORT_PCR_MUX(0x1);
        Gpio_isrPortDRequestVector[GPIO_AVAILABLE_PINS[pin].pinNumber].enabled = true;
        //Gpio_isrRegisterPortD |= 1 << GPIO_AVAILABLE_PINS[pin].pinNumber;
        Interrupt_enable (INTERRUPT_PORTD);
        break;
    default:
        ohiassert(0);
        return ERRORS_GPIO_WRONG_PORT;
    }
    return ERRORS_NO_ERROR;
}

System_Errors Gpio_disableInterrupt (Gpio_Pins pin)
{
    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);
    if (pin >= GPIO_AVAILABLE_PINS_COUNT)
        return ERRORS_GPIO_WRONG_PIN;

    PORT_Type* port = Gpio_getPortRegister(GPIO_AVAILABLE_PINS[pin].port);

    switch(GPIO_AVAILABLE_PINS[pin].port)
    {
    case GPIO_PORTS_A:
        port->PCR[GPIO_AVAILABLE_PINS[pin].pinNumber] &= ~PORT_PCR_IRQC_MASK;
        Gpio_isrRegisterPortA &= ~(1 << GPIO_AVAILABLE_PINS[pin].pinNumber);
        if (!Gpio_isrRegisterPortA) Interrupt_disable(INTERRUPT_PORTA);
        break;
    case GPIO_PORTS_D:
        port->PCR[GPIO_AVAILABLE_PINS[pin].pinNumber] &= ~PORT_PCR_IRQC_MASK;
        Gpio_isrRegisterPortD &= ~(1 << GPIO_AVAILABLE_PINS[pin].pinNumber);
        if (!Gpio_isrRegisterPortD) Interrupt_disable(INTERRUPT_PORTD);
        break;
    default:
        ohiassert(0);
        return ERRORS_GPIO_WRONG_PORT;
    }
    return ERRORS_NO_ERROR;
}

_weak void PORTA_IRQHandler (void)
{
    uint8_t i=0;

    while (i < PORT_MAX_PIN)
    {
        if(Gpio_isrRegisterPortA & (1 << i))
        {
            if (PORTA->PCR[i] & PORT_PCR_ISF_MASK)
            {
                Gpio_isrPortARequestVector[i].callback(Gpio_isrPortARequestVector[i].pin);
                //reset interrupt
                PORTA->PCR[i] |= PORT_PCR_ISF_MASK;
            }
        }
        i++;
    }
}

_weak void PORTD_IRQHandler (void)
{
    uint8_t i=0;

    while (i < PORT_MAX_PIN)
    {
        if(Gpio_isrRegisterPortD & (1 << i))
        {
            if (PORTD->PCR[i] & PORT_PCR_ISF_MASK)
            {
                Gpio_isrPortDRequestVector[i].callback(Gpio_isrPortDRequestVector[i].pin);
                //reset interrupt
                PORTD->PCR[i] |= PORT_PCR_ISF_MASK;
            }
        }
        i++;
    }
}

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif
