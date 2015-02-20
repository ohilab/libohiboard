/******************************************************************************
 * Copyright (C) 2014-2015 A. C. Open Hardware Ideas Lab
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
 ******************************************************************************/

/**
 * @file libohiboard/source/gpio_KL15Z4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO implementations for KL15Z4.
 */

#if defined (LIBOHIBOARD_KL15Z4)

#include "gpio.h"
#include "platforms.h"

typedef enum
{
    GPIO_PORTS_A,
    GPIO_PORTS_B,
    GPIO_PORTS_C,
    GPIO_PORTS_D,
    GPIO_PORTS_E,

} Gpio_Ports;

typedef struct _Gpio_PinDevice
{
    Gpio_Ports port;                      /**< The port of the selected pins */
    uint8_t pinNumber;       /**< The number of the pin of the relative port */
} Gpio_PinDevice;

static Gpio_PinDevice Gpio_availablePins[] =
{
    {GPIO_PORTS_A,0},
    {GPIO_PORTS_A,1},
    {GPIO_PORTS_A,2},
    {GPIO_PORTS_A,3},
    {GPIO_PORTS_A,4},
    {GPIO_PORTS_A,5},
    {GPIO_PORTS_A,12},
    {GPIO_PORTS_A,13},
    {GPIO_PORTS_A,14},
    {GPIO_PORTS_A,15},
    {GPIO_PORTS_A,16},
    {GPIO_PORTS_A,17},
    {GPIO_PORTS_A,18},
    {GPIO_PORTS_A,19},
    {GPIO_PORTS_A,20},

    {GPIO_PORTS_B,0},
    {GPIO_PORTS_B,1},
    {GPIO_PORTS_B,2},
    {GPIO_PORTS_B,3},
    {GPIO_PORTS_B,8},
    {GPIO_PORTS_B,9},
    {GPIO_PORTS_B,10},
    {GPIO_PORTS_B,11},
    {GPIO_PORTS_B,16},
    {GPIO_PORTS_B,17},
    {GPIO_PORTS_B,18},
    {GPIO_PORTS_B,19},

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
    {GPIO_PORTS_C,16},
    {GPIO_PORTS_C,17},

    {GPIO_PORTS_D,0},
    {GPIO_PORTS_D,1},
    {GPIO_PORTS_D,2},
    {GPIO_PORTS_D,3},
    {GPIO_PORTS_D,4},
    {GPIO_PORTS_D,5},
    {GPIO_PORTS_D,6},
    {GPIO_PORTS_D,7},

    {GPIO_PORTS_E,0},
    {GPIO_PORTS_E,1},
    {GPIO_PORTS_E,2},
    {GPIO_PORTS_E,3},
    {GPIO_PORTS_E,4},
    {GPIO_PORTS_E,5},
    {GPIO_PORTS_E,16},
    {GPIO_PORTS_E,17},
    {GPIO_PORTS_E,18},
    {GPIO_PORTS_E,19},
    {GPIO_PORTS_E,20},
    {GPIO_PORTS_E,21},
    {GPIO_PORTS_E,22},
    {GPIO_PORTS_E,23},
    {GPIO_PORTS_E,24},
    {GPIO_PORTS_E,25},
    {GPIO_PORTS_E,29},
    {GPIO_PORTS_E,30},
    {GPIO_PORTS_E,31},
};

static void Gpio_getPort (Gpio_Pins pin, GPIO_MemMapPtr* port)
{
    switch (Gpio_availablePins[pin].port)
    {
    case GPIO_PORTS_A:
        *port = PTA_BASE_PTR;
        break;
    case GPIO_PORTS_B:
        *port = PTB_BASE_PTR;
        break;
    case GPIO_PORTS_C:
        *port = PTC_BASE_PTR;
        break;
    case GPIO_PORTS_D:
        *port = PTD_BASE_PTR;
        break;
    case GPIO_PORTS_E:
        *port = PTE_BASE_PTR;
        break;
    default:
        assert(0);
    }
}

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

#endif /* LIBOHIBOARD_KL15Z4 */
