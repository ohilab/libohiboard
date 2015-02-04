/******************************************************************************
 * Copyright (C) 2012-2015 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/uart_KL15Z4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART implementations for KL15Z4.
 */

#ifdef LIBOHIBOARD_UART

#if defined (LIBOHIBOARD_KL15Z4)

#include "uart.h"

#include "interrupt.h"
#include "clock.h"

#define UART_MAX_PINS                     10

typedef struct Uart_Device
{
    UART_MemMapPtr regMap;                         /**< Device memory pointer */
    UART0_MemMapPtr regMap0;             /**< Device memory pointer for UART0 */

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
        .regMap0          = UART0_BASE_PTR,
        .regMap           = 0,

        .simScgcPtr       = &SIM_SCGC4,
        .simScgcBitEnable = SIM_SCGC4_UART0_MASK,

        .rxPins           = {UART_PINS_PTA1,
                             UART_PINS_PTA15,
                             UART_PINS_PTB16,
                             UART_PINS_PTD6,
                             UART_PINS_PTE21,
        },
        .rxPinsPtr        = {&PORTA_PCR1,
                             &PORTA_PCR15,
                             &PORTB_PCR16,
                             &PORTD_PCR6,
                             &PORTE_PCR21,
        },
        .rxPinsMux        = {2,
                             3,
                             3,
                             3,
                             4,
        },

        .txPins           = {UART_PINS_PTA2,
                             UART_PINS_PTA14,
                             UART_PINS_PTB17,
                             UART_PINS_PTD7,
                             UART_PINS_PTE20,
        },
        .txPinsPtr        = {&PORTA_PCR2,
                             &PORTA_PCR14,
                             &PORTB_PCR17,
                             &PORTD_PCR7,
                             &PORTE_PCR20,
        },
        .txPinsMux        = {2,
                             3,
                             3,
                             3,
                             4,
        },

        .devInitialized = 0,
};
Uart_DeviceHandle UART0 = &uart0;

static Uart_Device uart1 = {
        .regMap0          = 0,
        .regMap           = UART1_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC4,
        .simScgcBitEnable = SIM_SCGC4_UART1_MASK,

        .rxPins           = {UART_PINS_PTA18,
                             UART_PINS_PTC3,
                             UART_PINS_PTE1,
        },
        .rxPinsPtr        = {&PORTA_PCR18,
                             &PORTC_PCR3,
                             &PORTE_PCR1,
        },
        .rxPinsMux        = {3,
                             3,
                             3,
        },

        .txPins           = {UART_PINS_PTA19,
                             UART_PINS_PTC4,
                             UART_PINS_PTE0,
        },
        .txPinsPtr        = {&PORTA_PCR19,
                             &PORTC_PCR4,
                             &PORTE_PCR0,
        },
        .txPinsMux        = {3,
                             3,
                             3,
        },

        .devInitialized = 0,
};
Uart_DeviceHandle UART1 = &uart1;

static Uart_Device uart2 = {
        .regMap0          = 0,
        .regMap           = UART2_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC4,
        .simScgcBitEnable = SIM_SCGC4_UART2_MASK,

        .rxPins           = {UART_PINS_PTD2,
                             UART_PINS_PTD4,
                             UART_PINS_PTE17,
                             UART_PINS_PTE23,
        },
        .rxPinsPtr        = {&PORTD_PCR2,
                             &PORTD_PCR4,
                             &PORTE_PCR17,
                             &PORTE_PCR23,
        },
        .rxPinsMux        = {3,
                             3,
                             3,
                             4,
        },

        .txPins           = {UART_PINS_PTD3,
                             UART_PINS_PTD5,
                             UART_PINS_PTE16,
                             UART_PINS_PTE22,
        },
        .txPinsPtr        = {&PORTD_PCR3,
                             &PORTD_PCR5,
                             &PORTE_PCR16,
                             &PORTE_PCR22,
        },
        .txPinsMux        = {3,
                             3,
                             3,
                             4,
        },

        .devInitialized = 0,
};
Uart_DeviceHandle UART2 = &uart2;

static void Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate)
{
    register uint16_t sbr;
    uint8_t temp;
    uint32_t clockHz;

    switch (dev->clockSource)
    {
    case UART_CLOCKSOURCE_BUS:
        clockHz = Clock_getFrequency(CLOCK_BUS);
        break;
    case UART_CLOCKSOURCE_SYSTEM:
        clockHz = Clock_getFrequency(CLOCK_SYSTEM);
        break;
    }

    if (dev == UART0)
    {
        clockHz = Clock_getFrequency(CLOCK_SYSTEM);
    }

    /* Calculate baud settings */
    sbr = (uint16_t)((clockHz)/(baudrate * 16));

    /* Save off the current value of the UARTx_BDH except for the SBR field */
    if (dev == UART0)
    {
        temp = UART0_BDH_REG(dev->regMap0) & ~(UART0_BDH_SBR(0x1F));

        UART0_BDH_REG(dev->regMap0) = temp |  UART0_BDH_SBR(((sbr & 0x1F00) >> 8));
        UART0_BDL_REG(dev->regMap0) = (uint8_t)(sbr & UART0_BDL_SBR_MASK);
    }
    else
    {

        temp = UART_BDH_REG(dev->regMap) & ~(UART_BDH_SBR(0x1F));

        UART_BDH_REG(dev->regMap) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
        UART_BDL_REG(dev->regMap) = (uint8_t)(sbr & UART_BDL_SBR_MASK);
    }
}

/**
 * @brief Wait for a character to be received on the specified uart
 * @param dev Serial port device to receive byte from
 * @param *out Buffer where to store the received character
 * @return Error signal
 */
System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out)
{
    if (dev == UART0)
    {
        /* Wait until character has been received */
        while (!(UART0_S1_REG(dev->regMap0) & UART0_S1_RDRF_MASK));

        /* Save the 8-bit data from the receiver to the output param */
        *out = UART0_D_REG(dev->regMap0);
    }
    else
    {
        /* Wait until character has been received */
        while (!(UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK));

        /* Save the 8-bit data from the receiver to the output param */
        *out = UART_D_REG(dev->regMap);
    }
    return ERRORS_NO_ERROR;
}

/**
 * @brief Wait for space in the UART Tx FIFO and then send a character
 * @param dev uart Device to send to
 * @param c Character to send
 */
void Uart_putChar (Uart_DeviceHandle dev, char c)
{
    if (dev == UART0)
    {
        /* Wait until space is available in the FIFO */
        while(!(UART0_S1_REG(dev->regMap0) & UART0_S1_TDRE_MASK));

        /* Send the character */
        UART0_D_REG(dev->regMap0) = (uint8_t)c;
    }
    else
    {
        /* Wait until space is available in the FIFO */
        while(!(UART_S1_REG(dev->regMap) & UART_S1_TDRE_MASK));

        /* Send the character */
        UART_D_REG(dev->regMap) = (uint8_t)c;
    }
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
    if (dev == UART0)
    {
        if (UART0_S1_REG(dev->regMap0) & UART0_S1_OR_MASK)
            UART0_S1_REG(dev->regMap0) |= UART0_S1_OR_MASK;
        return (UART0_S1_REG(dev->regMap0) & UART0_S1_RDRF_MASK);
    }
    else
    {
        return (UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK);
    }
}

System_Errors Uart_open (Uart_DeviceHandle dev, void *callback, Uart_Config *config)
{
    if (dev->devInitialized) return ERRORS_UART_DEVICE_JUST_INIT;

    /* Enable the clock to the selected UART */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* Make sure that the transmitter and receiver are disabled while we change settings. */
    if (dev == UART0)
        UART0_C2_REG(dev->regMap0) &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK );
    else
        UART_C2_REG(dev->regMap) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

    /* FIXME: Configure the UART just for 8-bit mode */
    if (dev == UART0)
    {
        switch (config->dataBits)
        {
        case UART_DATABITS_EIGHT:
            UART_C1_REG(dev->regMap0) &= ~(UART_C1_M_MASK);
            break;
        case UART_DATABITS_NINE:
            /* FIXME: UART_C1_REG(dev->regMap0) |= UART_C1_M_MASK; */
            break;
        }
    }
    else
    {
        switch (config->dataBits)
        {
        case UART_DATABITS_EIGHT:
            UART_C1_REG(dev->regMap) &= ~(UART_C1_M_MASK);
            break;
        case UART_DATABITS_NINE:
            /* FIXME: UART_C1_REG(dev->regMap) |= UART_C1_M_MASK; */
            break;
        }
    }

    /* Set parity type */
    if (dev == UART0)
    {
        switch (config->parity)
        {
        case UART_PARITY_NONE:
            UART0_C1_REG(dev->regMap0) &= ~(UART0_C1_PE_MASK | UART0_C1_PT_MASK);
            break;
        case UART_PARITY_ODD:
            UART0_C1_REG(dev->regMap0) |= UART0_C1_PE_MASK | UART0_C1_PT_MASK | 0;
            break;
        case UART_PARITY_EVEN:
            UART0_C1_REG(dev->regMap0) |= UART0_C1_PE_MASK | 0;
            break;
        }
    }
    else
    {
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
    }

    dev->clockSource = config->clockSource;
    Uart_setBaudrate(dev,config->baudrate);

    /* Enable receiver and transmitter */
    if (dev == UART0)
        UART0_C2_REG(dev->regMap0) |= (UART0_C2_TE_MASK | UART0_C2_RE_MASK );
    else
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
    if (dev == UART0)
        UART0_C2_REG(dev->regMap0) &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK );
    else
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

#endif /* LIBOHIBOARD_KL15Z4 */

#endif /* LIBOHIBOARD_UART */
