/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/include/hardware/PIC24FJ/spi_PIC24FJ.h
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief GPIO pins and device definitions for PIC24FJ series
 */

#include "platforms.h"

#if defined (LIBOHIBOARD_GPIO)

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"

#include "utility.h"
#include "gpio.h"
#include "clock.h"
#include "interrupt.h"

#if defined (LIBOHIBOARD_PIC24FJ)

#define  PORT_MAX_PIN_NUM  16
#define  PORT_NUM  7

typedef void (*IocCallback_t)(void);
IocCallback_t IocCallback[PORT_NUM][PORT_MAX_PIN_NUM] = { };

typedef struct _Gpio_Device
{
    GPIO_TypeDef* regmap[PORT_NUM];
    IO_TypeDef* regMapIo;
    INTERRUPT_TypeDef* regmapInt;
} Gpio_Device;

Gpio_Device gpioDevice = 
{
    .regmap = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG},
    .regMapIo = IO,
    .regmapInt = INTERRUPT,
};

typedef struct _Gpio_PinDevice
{
    Gpio_Ports port;                      /**< The port of the selected pins */
    uint8_t pinNumber;       /**< The number of the pin of the relative port */
    uint8_t portIndex;       /**< Current port number for interrupt managing */
} Gpio_PinDevice;

static const Gpio_PinDevice GPIO_AVAILABLE_PINS[] =
{
    {0xFF,0xFF,0},

    //PORT A
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_A,0,0},
    {GPIO_PORTS_A,1,0},
    {GPIO_PORTS_A,2,0},
    {GPIO_PORTS_A,3,0},
    {GPIO_PORTS_A,4,0},
    {GPIO_PORTS_A,5,0},
    {GPIO_PORTS_A,6,0},
    {GPIO_PORTS_A,7,0},
    {GPIO_PORTS_A,9,0},
    {GPIO_PORTS_A,10,0},
    {GPIO_PORTS_A,14,0},
    {GPIO_PORTS_A,15,0},
#endif

    //PORT B
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
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
#endif

    //PORT C
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_C,1,2},
    {GPIO_PORTS_C,2,2},
    {GPIO_PORTS_C,3,2},
    {GPIO_PORTS_C,4,2},
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_C,12,2},
    {GPIO_PORTS_C,13,2},
    {GPIO_PORTS_C,14,2},
    {GPIO_PORTS_C,15,2},
#endif

    //PORT D
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_D,0,3},
    {GPIO_PORTS_D,1,3},
    {GPIO_PORTS_D,2,3},
    {GPIO_PORTS_D,3,3},
    {GPIO_PORTS_D,4,3},
    {GPIO_PORTS_D,5,3},
    {GPIO_PORTS_D,6,3},
    {GPIO_PORTS_D,7,3},
    {GPIO_PORTS_D,8,3},
    {GPIO_PORTS_D,9,3},
    {GPIO_PORTS_D,10,3},
    {GPIO_PORTS_D,11,3},
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_D,12,3},
    {GPIO_PORTS_D,13,3},
    {GPIO_PORTS_D,14,3},
    {GPIO_PORTS_D,15,3},
#endif

    //PORT E
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_E,0,4},
    {GPIO_PORTS_E,1,4},
    {GPIO_PORTS_E,2,4},
    {GPIO_PORTS_E,3,4},
    {GPIO_PORTS_E,4,4},
    {GPIO_PORTS_E,5,4},
    {GPIO_PORTS_E,6,4},
    {GPIO_PORTS_E,7,4},
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_E,8,4},
    {GPIO_PORTS_E,9,4},
#endif

    //PORT F
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_F,0,5},
    {GPIO_PORTS_F,1,5},
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_F,2,5},
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_F,3,5},
    {GPIO_PORTS_F,4,5},
    {GPIO_PORTS_F,5,5},
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610)
    {GPIO_PORTS_F,6,5},
#endif
#if defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_F,7,5},
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_F,8,5},
    {GPIO_PORTS_F,12,5},
    {GPIO_PORTS_F,13,5},
#endif

    //PORT G
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_G,0,6},
    {GPIO_PORTS_G,1,6},
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_G,2,6},
    {GPIO_PORTS_G,3,6},
    {GPIO_PORTS_G,6,6},
    {GPIO_PORTS_G,7,6},
    {GPIO_PORTS_G,8,6},
    {GPIO_PORTS_G,9,6},
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    {GPIO_PORTS_G,12,6},
    {GPIO_PORTS_G,13,6},
    {GPIO_PORTS_G,14,6},
    {GPIO_PORTS_G,15,6},
#endif
};

static const uint8_t GPIO_AVAILABLE_PINS_COUNT = UTILITY_DIMOF(GPIO_AVAILABLE_PINS);

static GPIO_TypeDef* Gpio_getPort (Gpio_Ports port)
{
    switch (port)
    {
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    case GPIO_PORTS_A:
        return GPIOA;
#endif
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

    default:
        ohiassert(0);
        return 0;
    }
}

System_Errors Gpio_config (Gpio_Pins pin, uint16_t options)
{
    if(pin == GPIO_PINS_NONE)
    {
        return ERRORS_GPIO_NULL_PIN;
    }

    GPIO_TypeDef* port = 0;
    uint8_t pinNumber = 0;

    // Only one type of configuration is possible
    ohiassert(((options & GPIO_PINS_OUTPUT) == GPIO_PINS_OUTPUT) ^
              ((options & GPIO_PINS_INPUT) == GPIO_PINS_INPUT));

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

    pinNumber = GPIO_AVAILABLE_PINS[pin].pinNumber;
    port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);

    //Disable Analog
    uint16_t tAns = port->ANS;
    tAns &= (~(0x0001 << pinNumber));
    port->ANS = tAns;

    //Read the current direction configuation
    uint16_t tTris = port->TRIS;
    //Set new configuration
    if(options & GPIO_PINS_INPUT)
    {
        //Set Input
        tTris |= (0x0001 << pinNumber);
    }
    else
    {
        //Set Output
        tTris &= (~(0x0001 << pinNumber));
    }
    //Save new configuration
    port->TRIS = tTris;

    if(options & GPIO_PINS_INPUT)
    {
        // Check configuration pullup or pulldown configuration
        ohiassert(~(((options & GPIO_PINS_ENABLE_PULLUP) == GPIO_PINS_ENABLE_PULLUP) &&
                    ((options & GPIO_PINS_ENABLE_PULLDOWN) == GPIO_PINS_ENABLE_PULLDOWN)));

        uint16_t tPullUp = port->IOCPU;
        if(options & GPIO_PINS_ENABLE_PULLUP)
        {
            tPullUp |= (0x0001 << pinNumber);
        }
        else
        {
            tPullUp &= (~(0x0001 << pinNumber));
        }
        port->IOCPU = tPullUp;

        uint16_t tPullDown = port->IOCPD;
        if(options & GPIO_PINS_ENABLE_PULLDOWN)
        {
            tPullDown |= (0x0001 << pinNumber);
        }
        else
        {
            tPullDown &= (~(0x0001 << pinNumber));
        }
        port->IOCPD = tPullDown;
    }
    else
    {
        // Check configuration for output mode
        ohiassert(~(((options & GPIO_PINS_ENABLE_OUTPUT_PUSHPULL) == GPIO_PINS_ENABLE_OUTPUT_PUSHPULL) &&
                    ((options & GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN) == GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN)));

        //Pull-ups and pull-downs on pins should always be disabled whenever
        // the pin is configured as a digital output.
        uint16_t tPullUp = port->IOCPU;
        tPullUp &= (~(0x0001 << pinNumber));
        port->IOCPU = tPullUp;
        uint16_t tPullDown = port->IOCPD;
        tPullDown &= (~(0x0001 << pinNumber));
        port->IOCPD = tPullDown;

        uint16_t tOdc = port->ODC;
        if(options & GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN)
        {
            tOdc |= (0x0001 << pinNumber);
        }
        else
        {
            tOdc &= (~(0x0001 << pinNumber));
        }
        port->ODC = tOdc;
    }

    return ERRORS_NO_ERROR;
}

void Gpio_configAlternate (Gpio_Pins pin, Gpio_Alternate alternate, uint16_t options)
{
    if (pin == GPIO_PINS_NONE)
    {
        return; //ERRORS_GPIO_NULL_PIN;
    }

    GPIO_TypeDef* port = 0;
    uint8_t pinNumber = 0;

    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

    ohiassert((alternate == GPIO_ALTERNATE_DIGITAL) ||
              (alternate == GPIO_ALTERNATE_ANALOG));

    pinNumber = GPIO_AVAILABLE_PINS[pin].pinNumber;
    port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);

    uint16_t tAns = port->ANS;
    if (alternate == GPIO_ALTERNATE_ANALOG)
    {
        //Input
        uint16_t tTris = port->TRIS;
        tTris |= (0x0001 << pinNumber);
        port->TRIS = tTris;

        //Analog
        tAns |= (0x0001 << pinNumber);
    }
    else
    {
        //Digital
        tAns &= (~(0x0001 << pinNumber));
    }
    port->ANS = tAns;
}


void Gpio_set (Gpio_Pins pin)
{
    if (pin != GPIO_PINS_NONE)
    {
        GPIO_TypeDef* port = 0;
        uint8_t pinNumber = 0;

        ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

        pinNumber = GPIO_AVAILABLE_PINS[pin].pinNumber;
        port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);

        uint16_t tLat = port->LAT;
        tLat |= (0x0001 << pinNumber);
        port->LAT = tLat;
    }
}

void Gpio_clear (Gpio_Pins pin)
{
    if(pin != GPIO_PINS_NONE)
    {
        GPIO_TypeDef* port = 0;
        uint8_t pinNumber = 0;

        ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

        pinNumber = GPIO_AVAILABLE_PINS[pin].pinNumber;
        port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);

        uint16_t tLat = port->LAT;
        tLat &= (~(0x0001 << pinNumber));
        port->LAT = tLat;
    }
}

void Gpio_toggle (Gpio_Pins pin)
{
    if(pin != GPIO_PINS_NONE)
    {
        GPIO_TypeDef* port = 0;
        uint8_t pinNumber = 0;

        ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

        pinNumber = GPIO_AVAILABLE_PINS[pin].pinNumber;
        port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);

        uint16_t tLat = port->LAT;
        tLat ^= (0x0001 << pinNumber);
        port->LAT = tLat;
    }
}

Gpio_Level Gpio_get (Gpio_Pins pin)
{
    if(pin == GPIO_PINS_NONE)
    {
        return GPIO_TOGGLE;
    }

    GPIO_TypeDef* port = 0;
    uint8_t pinNumber = 0;

    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

    pinNumber = GPIO_AVAILABLE_PINS[pin].pinNumber;
    port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);

    uint16_t tPort = port->PORT;
    tPort &= (0x0001 << pinNumber);
    return (tPort > 0) ? GPIO_HIGH : GPIO_LOW;
}

System_Errors Gpio_configInterrupt (Gpio_Pins pin, void* callback)
{
    if(pin == GPIO_PINS_NONE)
    {
        return ERRORS_GPIO_NULL_PIN;
    }

    uint16_t portNum = 0, pinNum = 0;

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);
    ohiassert(callback != nullptr);

    pinNum = (uint16_t)GPIO_AVAILABLE_PINS[pin].pinNumber;
    portNum = (uint16_t)GPIO_AVAILABLE_PINS[pin].portIndex;

    IocCallback[portNum][pinNum] = callback;

    return ERRORS_NO_ERROR;
}

System_Errors Gpio_enableInterrupt (Gpio_Pins pin, Gpio_EventType event)
{
    if(pin == GPIO_PINS_NONE)
    {
        return ERRORS_GPIO_NULL_PIN;
    }

    GPIO_TypeDef* port = 0;
    uint8_t pinNumber = 0;

    // Check configuration for direction
    ohiassert( (event == GPIO_EVENT_NONE) ||
              ((event & GPIO_EVENT_ON_RISING) == GPIO_EVENT_ON_RISING)   ||
              ((event & GPIO_EVENT_ON_FALLING) == GPIO_EVENT_ON_FALLING));

    pinNumber = GPIO_AVAILABLE_PINS[pin].pinNumber;
    port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);
 
    if(event == GPIO_EVENT_NONE)
    {
        uint16_t tIocp = port->IOCP;
        tIocp &= (~(0x0001 << pinNumber));
        port->IOCP = tIocp;

        uint16_t tIocn = port->IOCN;
        tIocn &= (~(0x0001 << pinNumber));
        port->IOCN = tIocn;
    }
    else
    {
        if(event & GPIO_EVENT_ON_RISING)
        {
            uint16_t tIocp = port->IOCP;
            tIocp |= (0x0001 << pinNumber);
            port->IOCP = tIocp;
        }

        if(event & GPIO_EVENT_ON_FALLING)
        {
            uint16_t tIocn = port->IOCN;
            tIocn |= (0x0001 << pinNumber);
            port->IOCN = tIocn;
        }
    }

    // Clear Pin Flag
    uint16_t tIocf = port->IOCF;
    tIocf &= (~(0x0001 << pinNumber));
    port->IOCF = tIocf;

    // Enable Interrupt-on-Change functionality
    UTILITY_SET_REGISTER_BIT(gpioDevice.regMapIo->PADCON, _PADCON_IOCON_MASK);

    // Clear Global IOC
    UTILITY_CLEAR_REGISTER_BIT(gpioDevice.regmapInt->IFS[1], _IFS1_IOCIF_MASK);

    // Enable Global IOC
    UTILITY_SET_REGISTER_BIT(gpioDevice.regmapInt->IEC[1], _IEC1_IOCIE_MASK);

    return ERRORS_NO_ERROR;
}

System_Errors Gpio_disableInterrupt (Gpio_Pins pin)
{
    if(pin == GPIO_PINS_NONE)
    {
        return ERRORS_GPIO_NULL_PIN;
    }

    GPIO_TypeDef* port = 0;
    uint8_t pinNumber = 0;

    //Check if pin definition exist
    ohiassert(pin < GPIO_AVAILABLE_PINS_COUNT);

    pinNumber = GPIO_AVAILABLE_PINS[pin].pinNumber;
    port = Gpio_getPort(GPIO_AVAILABLE_PINS[pin].port);

    // Disable IOC for pin
    uint16_t tIocp = port->IOCP;
    tIocp &=  (~(0x0001 << pinNumber));
    port->IOCP = tIocp;

    uint16_t tIocn = port->IOCN;
    tIocn &=  (~(0x0001 << pinNumber));
    port->IOCN = tIocn;

    // Clear Pin Flag
    uint16_t tIocf = port->IOCF;
    tIocf &= (~(0x0001 << pinNumber));
    port->IOCF = tIocf;

    return ERRORS_NO_ERROR;
}

void __attribute__ (( interrupt, no_auto_psv )) _IOCInterrupt ( void )
{
    uint16_t flags = 0xFFFF;

    if (UTILITY_READ_REGISTER_BIT(gpioDevice.regmapInt->IFS[1], _IFS1_IOCIF_MASK) != 0)
    {
        while (flags != 0x0000)
        {
            flags = 0;
            // Check for port
            for (uint16_t portField = 1, portNum = 0; portField < 0x0080; portField <<= 1, portNum++)
            {
                if (UTILITY_READ_REGISTER_BIT(gpioDevice.regMapIo->IOCSTAT, portField) != 0)
                {
                    flags++;
                     // Check for pin
                    for (uint32_t pinField = 1, pinNum = 0; pinField < 0x10000; pinField <<= 1, pinNum++)
                    {
                        if (UTILITY_READ_REGISTER_BIT(gpioDevice.regmap[portNum]->IOCF, pinField) != 0)
                        {
                            // CallBack
                            if (IocCallback[portNum][pinNum] != 0)
                            {
                                IocCallback[portNum][pinNum]();
                            }

                            // Clear Flag Pin
                            UTILITY_CLEAR_REGISTER_BIT(gpioDevice.regmap[portNum]->IOCF, pinField);
                        }
                    }
                }
            }
        }

        // Clear Global IOC
        UTILITY_CLEAR_REGISTER_BIT(gpioDevice.regmapInt->IFS[1], _IFS1_IOCIF_MASK);
    }
}

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_GPIO
