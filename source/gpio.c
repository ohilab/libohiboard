/******************************************************************************
 * Copyright (C) 2014 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/gpio.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO implementations.
 */

#include "gpio.h"

#if defined (MK60DZ10)

#elif defined (OHIBOARD_R1)

#elif defined (FRDMKL25Z)

typedef enum
{
    GPIO_PORTS_A,
    GPIO_PORTS_B,
    GPIO_PORTS_C,
    GPIO_PORTS_D,
    GPIO_PORTS_E,
} Gpio_Ports;

#endif

typedef struct _Gpio_PinDevice
{
    Gpio_Ports port;                      /**< The port of the selected pins */
    uint8_t pinNumber;       /**< The number of the pin of the relative port */
} Gpio_PinDevice;

#if defined (MK60DZ10)

#elif defined (OHIBOARD_R1)

#elif defined (FRDMKL25Z)

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
};

#endif

//static Gpio_Ports Gpio_getPort ()
//{
//    
//}

void Gpio_config (Gpio_Pins pin, uint8_t options)
{
    /* Enable clock */
    switch (Gpio_availablePins[pin].port)
    {
    case GPIO_PORTS_A:
        SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
        break;
    case GPIO_PORTS_B:
        SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
        break;
    case GPIO_PORTS_C:
        SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
        break;
    case GPIO_PORTS_D:
        SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
        break;
    case GPIO_PORTS_E:
        SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
        break;
    }
}

void Gpio_set (Gpio_Pins pin)
{
    
}

void Gpio_clear (Gpio_Pins pin)
{
    
}

void Gpio_toggle (Gpio_Pins pin)
{
    
}

uint8_t Gpio_get (Gpio_Pins pin)
{
    return 0;
}
