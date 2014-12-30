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
 * @file libohiboard/source/uart_KL03Z4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART implementations.
 */

#ifdef LIBOHIBOARD_UART

#include "uart.h"

#include "interrupt.h"
#include "clock.h"

#if defined (FRDMKL03Z) || defined (MKL03Z4)

#define UART_MAX_PINS                     4

typedef struct Uart_Device
{
    LPUART_MemMapPtr regMap;             /**< Device memory pointer for UART0 */

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


static Uart_Device uart0 = {
        .regMap           = LPUART0_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC5,
        .simScgcBitEnable = SIM_SCGC5_LPUART0_MASK,

        .rxPins           = {UART_PINS_PTA4,
        		             UART_PINS_PTB1R,
        		             UART_PINS_PTB2R,
        		             UART_PINS_PTB4,
        },
        .rxPinsPtr        = {&PORTA_PCR4,
                             &PORTB_PCR1,
                             &PORTB_PCR2,
                             &PORTB_PCR4,
        },
        .rxPinsMux        = {4,
        		             3,
                             2,
                             3,
        },

        .txPins           = {UART_PINS_PTA3,
                             UART_PINS_PTB1T,
                             UART_PINS_PTB2T,
                             UART_PINS_PTB3,
        },
        .txPinsPtr        = {&PORTA_PCR3,
                             &PORTB_PCR1,
                             &PORTB_PCR2,
                             &PORTB_PCR3,
        },
        .txPinsMux        = {4,
                             2,
                             3,
                             3,
        },

        .devInitialized = 0,
};
Uart_DeviceHandle UART0 = &uart0;

static void Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate)
{

}

System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out)
{
    /* Wait until character has been received */
    while (!(LPUART_STAT_REG(dev->regMap) & LPUART_STAT_RDRF_MASK));

    /* Save the 8-bit data from the receiver to the output param */
    *out = LPUART_DATA_REG(dev->regMap);
}

void Uart_putChar (Uart_DeviceHandle dev, char c)
{
    /* Wait until space is available in the FIFO */
    while(!(LPUART_STAT_REG(dev->regMap) & LPUART_STAT_TDRE_MASK));

    /* Send the character */
    LPUART_DATA_REG(dev->regMap) = (uint8_t)c;
}

uint8_t Uart_isCharPresent (Uart_DeviceHandle dev)
{
    if (LPUART_STAT_REG(dev->regMap) & LPUART_STAT_OR_MASK)
    	LPUART_STAT_REG(dev->regMap) |= LPUART_STAT_OR_MASK;
    return (LPUART_STAT_REG(dev->regMap) & LPUART_STAT_RDRF_MASK);
}

uint8_t Uart_isTransmissionComplete (Uart_DeviceHandle dev)
{
	return (LPUART_STAT_REG(dev->regMap) & LPUART_STAT_TC_MASK);
}

System_Errors Uart_open (Uart_DeviceHandle dev, void *callback, Uart_Config *config)
{
    if (dev->devInitialized) return ERRORS_UART_DEVICE_JUST_INIT;

    /* Enable the clock to the selected UART */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* Make sure that the transmitter and receiver are disabled while we change settings. */
	LPUART_CTRL_REG(dev->regMap) &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);

    switch (config->dataBits)
    {
    case UART_DATABITS_EIGHT:
    	LPUART_CTRL_REG(dev->regMap) &= ~(LPUART_CTRL_M_MASK);
    	LPUART_BAUD_REG(dev->regMap) &= ~(LPUART_BAUD_M10_MASK);
    	break;
    case UART_DATABITS_NINE:
    	LPUART_CTRL_REG(dev->regMap) |= LPUART_CTRL_M_MASK;
    	LPUART_BAUD_REG(dev->regMap) &= ~(LPUART_BAUD_M10_MASK);
    	break;
    case UART_DATABITS_TEN:
    	LPUART_BAUD_REG(dev->regMap) |= LPUART_BAUD_M10_MASK;
        break;
    }

    switch (config->parity)
    {
    case UART_PARITY_NONE:
    	LPUART_CTRL_REG(dev->regMap) &= ~(LPUART_CTRL_PE_MASK);
        break;
    case UART_PARITY_ODD:
    	LPUART_CTRL_REG(dev->regMap) |= LPUART_CTRL_PE_MASK;
    	LPUART_CTRL_REG(dev->regMap) |= LPUART_CTRL_PT_MASK;
        break;
    case UART_PARITY_EVEN:
    	LPUART_CTRL_REG(dev->regMap) |= LPUART_CTRL_PE_MASK;
    	LPUART_CTRL_REG(dev->regMap) &= ~(LPUART_CTRL_PT_MASK);
        break;
    }

    // TODO!!
//    dev->clockSource = config->clockSource;
    Uart_setBaudrate(dev,config->baudrate);

    /* Enable receiver and transmitter */
    LPUART_CTRL_REG(dev->regMap) |= (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK | 0);

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

    /* Wait until data is sended! */
    while (!Uart_isTransmissionComplete(dev)) {}

    /* Disable transmitter and receiver. */
	LPUART_CTRL_REG(dev->regMap) &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);

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

#endif /* defined (FRDMKL03Z) || defined (MKL03Z4) */

#endif /* LIBOHIBOARD_UART */
