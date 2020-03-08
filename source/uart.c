/*
 * Copyright (C) 2012-2018 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Edoardo Bezzeccheri <coolman3@gmail.com>
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
 * @file libohiboard/source/uart.c
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART implementations.
 */

#ifdef LIBOHIBOARD_UART

#ifdef __cplusplus
extern "C" {
#endif

#include "uart.h"
#include "interrupt.h"
#include "clock.h"

static const char Uart_hexDigits[] = "0123456789ABCDEF";
static const uint8_t Uart_lf = 0x0A;
static const uint8_t Uart_cr = 0x0D;

void Uart_sendString (Uart_DeviceHandle dev, const char* text)
{
    if (text)
    {
        while (*text)
        {
#if (LIBOHIBOARD_VERSION >= 0x20000)
            Uart_write(dev,(const uint8_t*)text++,300);
#else
            Uart_putChar(dev, *text++);
#endif

        }
    }
}

void Uart_sendStringln (Uart_DeviceHandle dev, const char* text)
{
    if (text)
    {
        while (*text)
        {
#if (LIBOHIBOARD_VERSION >= 0x20000)
            Uart_write(dev,(const uint8_t*)text++,300);
#else
            Uart_putChar(dev, *text++);
#endif
        }
#if (LIBOHIBOARD_VERSION >= 0x20000)
            Uart_write(dev,&Uart_cr,300);
            Uart_write(dev,&Uart_lf,300);
#else
            Uart_putChar(dev, Uart_cr);
            Uart_putChar(dev, Uart_lf);
#endif
    }
}

void Uart_sendData (Uart_DeviceHandle dev, const char* data, uint8_t length)
{
    if (data)
    {
        while (length--)
        {
#if (LIBOHIBOARD_VERSION >= 0x20000)
            Uart_write(dev,(const uint8_t*)data++,300);
#else
            Uart_putChar(dev, *data++);
#endif
        }
    }
}

void Uart_sendHex (Uart_DeviceHandle dev, const char* data, uint8_t length)
{
    if (data)
    {
        while (length--)
        {
            uint8_t value = *data++;
#if (LIBOHIBOARD_VERSION >= 0x20000)
            Uart_write(dev,(const uint8_t*)&Uart_hexDigits[(value >> 4) & 0x0F],300);
            Uart_write(dev,(const uint8_t*)&Uart_hexDigits[(value >> 0) & 0x0F],300);
#else
            Uart_putChar(dev, Uart_hexDigits[(value >> 4) & 0x0F]);
            Uart_putChar(dev, Uart_hexDigits[(value >> 0) & 0x0F]);
#endif
        }
    }
}

#ifdef __cplusplus
}
#endif

#endif


