/******************************************************************************
 * Copyright (C) 2012-2014 A. C. Open Hardware Ideas Lab
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
 ******************************************************************************/

/**
 * @file libohiboard/source/uart.c
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART implementations.
 */

#ifdef LIBOHIBOARD_UART

#include "uart.h"
#include "interrupt.h"
#include "clock.h"

static const char Uart_hexDigits[] = "0123456789ABCDEF";

void Uart_sendString (Uart_DeviceHandle dev, const char* text)
{
    if (text)
    {
        while (*text)
        {
            Uart_putChar(dev, *text++);
        }
    }
}

void Uart_sendData (Uart_DeviceHandle dev, const char* data, uint8_t length)
{
    if (data)
    {
        while (length--)
        {
            Uart_putChar(dev, *data++);
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
            Uart_putChar(dev, Uart_hexDigits[(value >> 4) & 0x0F]);
            Uart_putChar(dev, Uart_hexDigits[(value >> 0) & 0x0F]);
        }
    }
}

#endif


