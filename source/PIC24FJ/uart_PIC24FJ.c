/*
 * This file is part of the libohiboard project.
 *
 * Authors:
 *  Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/source/PIC24FJ/uart_PIC24FJ.c
 * @author Leonardo Morichelli
 * @brief UART implementations for PIC24FJ Series.
 */

#include "platforms.h"

#if defined(LIBOHIBOARD_UART)

#ifdef __cplusplus
extern "C" {
#endif

#include "uart.h"

#include "platforms.h"
#include "utility.h"
#include "gpio.h"
#include "clock.h"
#include "interrupt.h"

#if defined (LIBOHIBOARD_PIC24FJ)
    
System_Errors Uart_init (Uart_DeviceHandle dev, Uart_Config *config)
{
    System_Errors error = ERRORS_NO_ERROR;
    if (config == NULL)
    {
        return ERRORS_UART_NO_DEVICE;
    }
    return error;
}

System_Errors Uart_deInit (Uart_DeviceHandle dev)
{
    System_Errors error = ERRORS_NO_ERROR;
    return error;
}

System_Errors Uart_setRxPin (Uart_DeviceHandle dev, Uart_RxPins rxPin)
{
    System_Errors error = ERRORS_NO_ERROR;
    return error;
}

System_Errors Uart_setTxPin (Uart_DeviceHandle dev, Uart_TxPins txPin)
{
    System_Errors error = ERRORS_NO_ERROR;
    return error;
}

void Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate)
{
    
}

uint8_t Uart_isTransmissionComplete (Uart_DeviceHandle dev)
{
    return true;
}

System_Errors Uart_read (Uart_DeviceHandle dev, uint8_t *data, uint32_t timeout)
{
    System_Errors error = ERRORS_NO_ERROR;
    return error;
}

System_Errors Uart_write (Uart_DeviceHandle dev, const uint8_t* data, uint32_t timeout)
{
    System_Errors error = ERRORS_NO_ERROR;
    return error;
}

bool Uart_isPresent (Uart_DeviceHandle dev)
{
    return false;
}

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_UART
