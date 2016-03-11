/* Copyright (C) 2014-2016 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Alessio Paolucci <a.paolucci89@gmail.com>
 *  Matteo Civale <matteo.civale@gmail.com>
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
 * @file libohiboard/source/uart_K64F12.c
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief UART implementations for K64F12 and FRDMK64.
 */

#ifdef LIBOHIBOARD_UART

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

#include "uart.h"

#include "interrupt.h"
#include "clock.h"

#define UART_MAX_PINS                     12

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
    
    void (*isr)(void);                     /**< The function pointer for ISR. */
    void (*callbackRx)(void); /**< The function pointer for user Rx callback. */
    void (*callbackTx)(void); /**< The function pointer for user Tx callback. */
    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Uart_ClockSource clockSource;
    
    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Uart_Device;

static Uart_Device uart0 = {
        .regMap           = UART0_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC4,
        .simScgcBitEnable = SIM_SCGC4_UART0_MASK,

        .rxPins           = {UART_PINS_PTA1,
                             UART_PINS_PTA15,
                             UART_PINS_PTB16,
                             UART_PINS_PTD6,
        },
        .rxPinsPtr        = {&PORTA_PCR1,
                             &PORTA_PCR15,
                             &PORTB_PCR16,
                             &PORTD_PCR6,
        },
        .rxPinsMux        = {2,
                             3,
                             3,
                             3,
        },

        .txPins           = {UART_PINS_PTA2,
                             UART_PINS_PTA14,
                             UART_PINS_PTB17,
                             UART_PINS_PTD7,
        },
        .txPinsPtr        = {&PORTA_PCR2,
                             &PORTA_PCR14,
                             &PORTB_PCR17,
                             &PORTD_PCR7,
        },
        .txPinsMux        = {2,
                             3,
                             3,
                             3,
        },

        .isr              = UART0_RX_TX_IRQHandler,
        .isrNumber        = INTERRUPT_UART0,
        .callbackRx       = 0,
        .callbackTx       = 0,

        .devInitialized = 0,
};
Uart_DeviceHandle OB_UART0 = &uart0;

static Uart_Device uart1 = {
        .regMap           = UART1_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC4,
        .simScgcBitEnable = SIM_SCGC4_UART1_MASK,

        .rxPins           = {UART_PINS_PTE1,
                             UART_PINS_PTC3,
        },
        .rxPinsPtr        = {&PORTE_PCR1,
                             &PORTC_PCR3,
        },
        .rxPinsMux        = {3,
                             3,
        },

        .txPins           = {UART_PINS_PTE0,
                             UART_PINS_PTC4,
        },
        .txPinsPtr        = {&PORTE_PCR0,
                             &PORTC_PCR4,
        },
        .txPinsMux        = {3,
                             3,
        },

        .isr              = UART1_RX_TX_IRQHandler,
        .isrNumber        = INTERRUPT_UART1,
        .callbackRx       = 0,
        .callbackTx       = 0,

        .devInitialized = 0,
};
Uart_DeviceHandle OB_UART1 = &uart1;

static Uart_Device uart2 = {
        .regMap           = UART2_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC4,
        .simScgcBitEnable = SIM_SCGC4_UART2_MASK,

        .rxPins           = {UART_PINS_PTD2,
        },
        .rxPinsPtr        = {&PORTD_PCR2,                             
        },
        .rxPinsMux        = {3,
        },

        .txPins           = {UART_PINS_PTD3,
        },
        .txPinsPtr        = {&PORTD_PCR3,
        },
        .txPinsMux        = {3,
        },

        .isr              = UART2_RX_TX_IRQHandler,
        .isrNumber        = INTERRUPT_UART2,
        .callbackRx       = 0,
        .callbackTx       = 0,

        .devInitialized = 0,
};
Uart_DeviceHandle OB_UART2 = &uart2;

static Uart_Device uart3 = {
        .regMap           = UART3_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC4,
        .simScgcBitEnable = SIM_SCGC4_UART3_MASK,

        .rxPins           = {UART_PINS_PTE5,
                             UART_PINS_PTB10,
                             UART_PINS_PTC16,
        },
        .rxPinsPtr        = {&PORTE_PCR5,
                             &PORTB_PCR10,
                             &PORTC_PCR16,
        },
        .rxPinsMux        = {3,
                             3,
                             3,
        },

        .txPins           = {UART_PINS_PTE4,
                             UART_PINS_PTB11,
                             UART_PINS_PTC17,
        },
        .txPinsPtr        = {&PORTE_PCR4,
                             &PORTB_PCR11,
                             &PORTC_PCR17,
        },
        .txPinsMux        = {3,
                             3,
                             3,
        },

        .isr              = UART3_RX_TX_IRQHandler,
        .isrNumber        = INTERRUPT_UART3,
        .callbackRx       = 0,
        .callbackTx       = 0,

        .devInitialized = 0,
};
Uart_DeviceHandle OB_UART3 = &uart3;

static Uart_Device uart4 = {
        .regMap           = UART4_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC1,
        .simScgcBitEnable = SIM_SCGC1_UART4_MASK,

        .rxPins           = {UART_PINS_PTE25,
                             UART_PINS_PTC14,
        },
        .rxPinsPtr        = {&PORTE_PCR25,
                             &PORTC_PCR14,
        },
        .rxPinsMux        = {3,
                             3,
        },

        .txPins           = {UART_PINS_PTE24,
                             UART_PINS_PTC15,
        },
        .txPinsPtr        = {&PORTE_PCR24,
                             &PORTC_PCR15,
        },
        .txPinsMux        = {3,
                             3,
        },

        .isr              = UART4_RX_TX_IRQHandler,
        .isrNumber        = INTERRUPT_UART4,
        .callbackRx       = 0,
        .callbackTx       = 0,

        .devInitialized = 0,
};
Uart_DeviceHandle OB_UART4 = &uart4;

static Uart_Device uart5 = {
        .regMap           = UART5_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC1,
        .simScgcBitEnable = SIM_SCGC1_UART5_MASK,

        .rxPins           = {
        					 UART_PINS_PTE9,
        					 UART_PINS_PTD8,
        },
        .rxPinsPtr        = {&PORTE_PCR9,
        		             &PORTD_PCR8,
        },
        .rxPinsMux        = {3,
        		             3,
        },

        .txPins           = {UART_PINS_PTE8,
        		             UART_PINS_PTD9,
        },
        .txPinsPtr        = {&PORTE_PCR8,
        		             &PORTD_PCR9,
        },
        .txPinsMux        = {3,
        		             3,
        },

        .isr              = UART5_RX_TX_IRQHandler,
        .isrNumber        = INTERRUPT_UART5,
        .callbackRx       = 0,
        .callbackTx       = 0,

        .devInitialized = 0,
};
Uart_DeviceHandle OB_UART5 = &uart5;

void UART0_RX_TX_IRQHandler (void)
{
    if (UART_S1_REG(OB_UART0->regMap) & UART_S1_RDRF_MASK)
    {
        OB_UART0->callbackRx();
        (void)UART_S1_REG(OB_UART0->regMap);
        (void)UART_D_REG(OB_UART0->regMap);
    }
    else if (UART_S1_REG(OB_UART0->regMap) & UART_S1_TDRE_MASK)
    {
        OB_UART0->callbackTx();
        (void)UART_S1_REG(OB_UART0->regMap);
        (void)UART_D_REG(OB_UART0->regMap);
    }
}

void UART1_RX_TX_IRQHandler (void)
{
    if (UART_S1_REG(OB_UART1->regMap) & UART_S1_RDRF_MASK)
    {
        OB_UART1->callbackRx();
        (void)UART_S1_REG(OB_UART1->regMap);
        (void)UART_D_REG(OB_UART1->regMap);
    }
    else if (UART_S1_REG(OB_UART1->regMap) & UART_S1_TDRE_MASK)
    {
        OB_UART1->callbackTx();
        (void)UART_S1_REG(OB_UART1->regMap);
        (void)UART_D_REG(OB_UART1->regMap);
    }
}

void UART2_RX_TX_IRQHandler (void)
{
    if (UART_S1_REG(OB_UART2->regMap) & UART_S1_RDRF_MASK)
    {
        OB_UART2->callbackRx();
        (void)UART_S1_REG(OB_UART2->regMap);
        (void)UART_D_REG(OB_UART2->regMap);
    }
    else if (UART_S1_REG(OB_UART2->regMap) & UART_S1_TDRE_MASK)
    {
        OB_UART2->callbackTx();
        (void)UART_S1_REG(OB_UART2->regMap);
        (void)UART_D_REG(OB_UART2->regMap);
    }
}

void UART3_RX_TX_IRQHandler (void)
{
    if (UART_S1_REG(OB_UART3->regMap) & UART_S1_RDRF_MASK)
    {
        OB_UART3->callbackRx();
        (void)UART_S1_REG(OB_UART3->regMap);
        (void)UART_D_REG(OB_UART3->regMap);
    }
    else if (UART_S1_REG(OB_UART3->regMap) & UART_S1_TDRE_MASK)
    {
        OB_UART3->callbackTx();
        (void)UART_S1_REG(OB_UART3->regMap);
        (void)UART_D_REG(OB_UART3->regMap);
    }
}

void UART4_RX_TX_IRQHandler (void)
{
    if (UART_S1_REG(OB_UART4->regMap) & UART_S1_RDRF_MASK)
    {
        OB_UART4->callbackRx();
        (void)UART_S1_REG(OB_UART4->regMap);
        (void)UART_D_REG(OB_UART4->regMap);
    }
    else if (UART_S1_REG(OB_UART4->regMap) & UART_S1_TDRE_MASK)
    {
        OB_UART4->callbackTx();
        (void)UART_S1_REG(OB_UART4->regMap);
        (void)UART_D_REG(OB_UART4->regMap);
    }
}

void UART5_RX_TX_IRQHandler (void)
{
    if (UART_S1_REG(OB_UART5->regMap) & UART_S1_RDRF_MASK)
    {
        OB_UART5->callbackRx();
        (void)UART_S1_REG(OB_UART5->regMap);
        (void)UART_D_REG(OB_UART5->regMap);
    }
    else if (UART_S1_REG(OB_UART5->regMap) & UART_S1_TDRE_MASK)
    {
        OB_UART5->callbackTx();
        (void)UART_S1_REG(OB_UART5->regMap);
        (void)UART_D_REG(OB_UART5->regMap);
    }
}

static void Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate)
{
    register uint16_t sbr, brfa;    //BaudRateFineAdjust

    uint8_t temp;
    uint32_t clockHz;
    
    if ((dev == OB_UART0) || (dev == OB_UART1))
    {
        clockHz = Clock_getFrequency(CLOCK_SYSTEM);
    }
    else
    {
        clockHz = Clock_getFrequency(CLOCK_BUS);
    }
    
    /* Calculate baud settings */
    sbr = (uint16_t)((clockHz)/(baudrate * 16));
    
    /* Save off the current value of the UARTx_BDH except for the SBR field */

    temp = UART_BDH_REG(dev->regMap) & ~(UART_BDH_SBR(0x1F));
        
    UART_BDH_REG(dev->regMap) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
    UART_BDL_REG(dev->regMap) = (uint8_t)(sbr & UART_BDL_SBR_MASK);
        
    /* Determine if a fractional divider is needed to get closer to the baud rate */
    brfa = (((clockHz * 32)/(baudrate * 16)) - (sbr * 32));
    
    /* Save off the current value of the UARTx_C4 register except for the BRFA field */
    temp = UART_C4_REG(dev->regMap) & ~(UART_C4_BRFA(0x1F));
    
    UART_C4_REG(dev->regMap) = temp | UART_C4_BRFA(brfa);
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

System_Errors Uart_open (Uart_DeviceHandle dev, Uart_Config *config)
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

    /* If call back exist save it */
    if (config->callbackRx)
    {
        dev->callbackRx = config->callbackRx;
        /* Enable interrupt */
        Interrupt_enable(dev->isrNumber);
        /* Enable RX interrupt */
        UART_C2_REG(dev->regMap) |= UART_C2_RIE_MASK;
    }
    if (config->callbackTx)
    {
        dev->callbackTx = config->callbackTx;
        /* Enable interrupt */
        Interrupt_enable(dev->isrNumber);
        /* Enable TX interrupt */
        UART_C2_REG(dev->regMap) |= UART_C2_TIE_MASK;
    }

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

uint8_t Uart_isTransmissionComplete (Uart_DeviceHandle dev)
{
    return (UART_S1_REG(dev->regMap) & UART_S1_TC_MASK);
}

#endif /* LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F */

#endif /* LIBOHIBOARD_UART */
