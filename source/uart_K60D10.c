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
 * @file libohiboard/source/uart_K60D10.c
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART implementations.
 */

#ifdef LIBOHIBOARD_UART

#include "uart.h"

#include "interrupt.h"
#include "clock.h"

#if defined (LIBOHIBOARD_K60DZ10) || \
    defined (LIBOHIBOARD_OHIBOARD_R1)

#define UART_MAX_PINS                     10

typedef struct Uart_Device
{
    UART_MemMapPtr regMap;                         /**< Device memory pointer */

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Uart_RxPins rxPins[UART_MAX_PINS];
    Uart_TxPins txPins[UART_MAX_PINS];

    volatile uint32_t* rxPinsPtr[UART_MAX_PINS];
    volatile uint32_t* txPinsPtr[UART_MAX_PINS];
    uint8_t rxPinsMux[UART_MAX_PINS];
    uint8_t txPinsMux[UART_MAX_PINS];

    Uart_ClockSource clockSource;

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Uart_Device;


/* TODO: ADD DEVICE! */


static void Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate)
{
    /* TODO: Copy from K10! Pay attention to get clock source! */
}

/**
 * @brief Wait for a character to be received on the specified uart
 * @param dev Serial port device to receive byte from
 * @param *out Buffer where to store the received character
 * @return Error signal
 */
System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out)
{
    /* Wait until character has been received */
    while (!(UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK));

    /* Save the 8-bit data from the receiver to the output param */
    *out = UART_D_REG(dev->regMap);
    return ERRORS_NO_ERROR;
}

/**
 * @brief Wait for space in the UART Tx FIFO and then send a character
 * @param dev uart Device to send to
 * @param c Character to send
 */
void Uart_putChar (Uart_DeviceHandle dev, char c)
{
    /* Wait until space is available in the FIFO */
    while(!(UART_S1_REG(dev->regMap) & UART_S1_TDRE_MASK));

    /* Send the character */
    UART_D_REG(dev->regMap) = (uint8_t)c;
}

/**
 * @brief Check to see if a character has been received
 * @param dev uart Device to check for a character
 * @return uint8_t
 *  0 No character received
 *  1 Character has been received
 */
uint8_t Uart_isCharPresent (Uart_DeviceHandle dev)
{
    return (UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK);
}

System_Errors Uart_open (Uart_DeviceHandle dev, void *callback, Uart_Config *config)
{
    if (dev->devInitialized) return ERRORS_UART_DEVICE_JUST_INIT;

    /* Enable the clock to the selected UART */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* Make sure that the transmitter and receiver are disabled while we change settings. */
    UART_C2_REG(dev->regMap) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

    /* Clear out receive and transmit buffer */
    UART_CFIFO_REG(dev->regMap) |= UART_CFIFO_RXFLUSH_MASK | UART_CFIFO_TXFLUSH_MASK;

    /* FIXME: Configure the UART just for 8-bit mode */
    switch (config->dataBits)
    {
    case UART_DATABITS_EIGHT:
        UART_C1_REG(dev->regMap) &= ~(UART_C1_M_MASK);
        break;
    case UART_DATABITS_NINE:
        /* FIXME: UART_C1_REG(dev->regMap) |= UART_C1_M_MASK; */
        break;
    }

    /* Set parity type */
    switch (config->parity)
    {
    case UART_PARITY_NONE:
        UART_C1_REG(dev->regMap) &= ~(UART_C1_PE_MASK | UART_C1_PT_MASK);
        break;
    case UART_PARITY_ODD:
        UART_C1_REG(dev->regMap) |= UART_C1_PE_MASK | UART_C1_PT_MASK | 0;
        break;
    case UART_PARITY_EVEN:
        UART_C1_REG(dev->regMap) |= UART_C1_PE_MASK | 0;
        break;
    }

    dev->clockSource = config->clockSource;
    Uart_setBaudrate(dev,config->baudrate);

    /* Enable receiver and transmitter */
    UART_C2_REG(dev->regMap) |= (UART_C2_TE_MASK | UART_C2_RE_MASK );

    dev->devInitialized = 1;

    /* Config the port controller */
    if (config->rxPin != UART_PINS_RXNONE)
        Uart_setRxPin(dev, config->rxPin);

    if (config->txPin != UART_PINS_TXNONE)
        Uart_setTxPin(dev, config->txPin);

    return ERRORS_NO_ERROR;
}

System_Errors Uart_close (Uart_DeviceHandle dev)
{
    if (!dev->devInitialized) return ERRORS_UART_DEVICE_NOT_INIT;

    /* Disable transmitter and receiver. */
    UART_C2_REG(dev->regMap) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );


    /* Disable the clock to the selected UART */
    *dev->simScgcPtr &= ~dev->simScgcBitEnable;

    dev->devInitialized = 0;
    return ERRORS_NO_ERROR;
}

System_Errors Uart_setRxPin (Uart_DeviceHandle dev, Uart_RxPins rxPin)
{
    uint8_t devPinIndex;

    if (dev->devInitialized == 0)
        return ERRORS_UART_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < UART_MAX_PINS; ++devPinIndex)
    {
        if (dev->rxPins[devPinIndex] == rxPin)
        {
            *(dev->rxPinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->rxPinsMux[devPinIndex]);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_UART_NO_PIN_FOUND;
}

System_Errors Uart_setTxPin (Uart_DeviceHandle dev, Uart_TxPins txPin)
{
    uint8_t devPinIndex;

    if (dev->devInitialized == 0)
        return ERRORS_UART_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < UART_MAX_PINS; ++devPinIndex)
    {
        if (dev->txPins[devPinIndex] == txPin)
        {
            *(dev->txPinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->txPinsMux[devPinIndex]);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_UART_NO_PIN_FOUND;
}

#endif /* LIBOHIBOARD_K60DZ10 || LIBOHIBOARD_OHIBOARD_R1 */

#endif /* LIBOHIBOARD_UART */
