/* Copyright (C) 2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Simone Giacomucci <simone.giacomucci@gmail.com>
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
 * @file libohiboard/source/ethernet_K64F12.c
 * @author Simone Giacomucci <simone.giacomucci@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Ethernet HAL implementations for K64F12 and FRDMK64F.
 */

#ifdef LIBOHIBOARD_ETHERNET

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

/*!
 * @addtogroup Ethernet_K64F12 Ethernet Device of K64F12
 * @{
 */

#include "platforms.h"

#include "ethernet.h"
#include "interrupt.h"
#include "clock.h"

#define ETHERNET_MAX_PINS                    10//14

/* Ethernet general constants */
#define ETHERNET_MAX_TIMEOUT                 0xFFFF /**< Ethernet Timeout. */

/* MAC Constants */
#define ETHERNET_MIN_VALID_TX_IPG            8 /**< Minimum valid transmit IPG */
#define ETHERNET_MAX_VALID_TX_IPG            27 /**< Maximum valid transmit IPG */

/* FIFO Constants */
#define ETHERNET_FIFO_MIN_TX_ALMOST_FULL     6 /**< Ethernet minimum transmit FIFO almost full value */
#define ETHERNET_FIFO_MIN_ALMOST_EMPTY       4 /**< Ethernet minimum FIFO almost empty value */
#define ETHERNET_FIFO_DEFAULT_TX_ALMOST_FULL 8 /**< ENET default transmit FIFO almost full value */


#define ETHERNET_MDC_FREQUENCY               2500000  /**< MDC Frequency*/
#define ETHERNET_MII_FREQUENCY               25000000 /**< Internal Frequency of MII for PHY */
#define ETHERNET_RMII_FREQUENCY              50000000 /**< Internal Frequency of RMII for PHY */

typedef struct Ethernet_Device
{
    ENET_MemMapPtr regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Ethernet_Pins pins[ETHERNET_MAX_PINS];
    volatile uint32_t* pinsPtr[ETHERNET_MAX_PINS];
    uint8_t pinMux[ETHERNET_MAX_PINS];

    void (*isrRx)(void);                   /**< The function pointer for ISR. */
    void (*callbackRx)(void);    /**< The function pointer for user callback. */
    Interrupt_Vector isrRxNumber;                     /**< ISR vector number. */

    void (*isrTx)(void);                   /**< The function pointer for ISR. */
    void (*callbackTx)(void);    /**< The function pointer for user callback. */
    Interrupt_Vector isrTxNumber;                     /**< ISR vector number. */

    void (*isrTs)(void);      /**< The function pointer for ISR of timestamp. */
    void (*callbackTs)(void);/**< The pointer for user callback of timestamp. */
    Interrupt_Vector isrTsNumber;        /**< ISR vector number of timestamp. */

    Ethernet_BufferDescriptors* bd;

    /* Status values */
    uint16_t maxFrameLength; /**< TODO */
    uint8_t statusFlag; /**< TODO */

    bool isTxStoreForwardEnable;        /**< Transmit FIFO store and forward. */

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Ethernet_Device;

static Ethernet_Device enet0 = {
    .regMap           = ENET_BASE_PTR,

    .simScgcPtr       = &SIM_SCGC2,
    .simScgcBitEnable = SIM_SCGC2_ENET_MASK,

    .pins             = {ETHERNET_PINS_PTA5,
                         ETHERNET_PINS_PTA12,
                         ETHERNET_PINS_PTA13,
                         ETHERNET_PINS_PTA14,
                         ETHERNET_PINS_PTA15,
                         ETHERNET_PINS_PTA16,
                         ETHERNET_PINS_PTA17,
                         ETHERNET_PINS_PTA28,
                         ETHERNET_PINS_PTB0,
                         ETHERNET_PINS_PTB1,
//                         ETHERNET_PINS_PTC16,
//                         ETHERNET_PINS_PTC17,
//                         ETHERNET_PINS_PTC18,
//                         ETHERNET_PINS_PTC19,
    },
    .pinsPtr          = {&PORTA_PCR5,
                         &PORTA_PCR12,
                         &PORTA_PCR13,
                         &PORTA_PCR14,
                         &PORTA_PCR15,
                         &PORTA_PCR16,
                         &PORTA_PCR17,
                         &PORTA_PCR28,
                         &PORTB_PCR0,
                         &PORTB_PCR1,
//                         &PORTC_PCR16,
//                         &PORTC_PCR17,
//                         &PORTC_PCR18,
//                         &PORTC_PCR19,
    },
    .pinMux           = {4,
                         4,
                         4,
                         4,
                         4,
                         4,
                         4,
                         4,
                         4,
                         4,
//                         4,
//                         4,
//                         4,
//                         4,
    },

    .isrRx            = Ethernet_isrEnet0Rx,
    .isrRxNumber      = INTERRUPT_ETHERNET_RX,

    .isrTx            = Ethernet_isrEnet0Tx,
    .isrTxNumber      = INTERRUPT_ETHERNET_TX,

    .isrTs            = Ethernet_isrEnet0Ts,
    .isrTsNumber      = INTERRUPT_ETHERNET_1588,
};
Ethernet_DeviceHandle ENET0 = &enet0;

/**
 *  Initializes the correct multiplexing of microcontroller ethernet pins.
 *
 *  @param[in] dev The ethernet device
 */
static void Ethernet_initPins (Ethernet_DeviceHandle dev)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < ETHERNET_MAX_PINS; ++devPinIndex)
    {
        *(dev->pinsPtr[devPinIndex]) =
        PORT_PCR_MUX(dev->pinMux[devPinIndex]) | PORT_PCR_IRQC(0);
    }

    /* Setting pull-up mode for PTB0 alias MDIO. MDIO is a signal of MII
     * interface that is bidirectional.*/
    PORTB_PCR0 |= (PORT_PCR_ODE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK);
}

/**
 *  The function called on receiving packet interrupt occurrence. It cleans
 *  the related interrupt flags and calls the upper level callback function.
 */
void Ethernet_isrEnet0Rx (void)
{
    while (((ENET_EIR_REG(ENET0->regMap) & ENET_EIR_RXB_MASK) != 0) ||
           ((ENET_EIR_REG(ENET0->regMap) & ENET_EIR_RXF_MASK) != 0))
    {
        ENET_EIR_REG(ENET0->regMap) |= ENET_EIR_RXB_MASK;
        ENET_EIR_REG(ENET0->regMap) |= ENET_EIR_RXF_MASK;
        ENET0->callbackRx();
    }
}

/**
 *  The function called on transmitting packet interrupt occurrence. It cleans
 *  the related interrupt flags and calls the upper level callback function.
 */
void Ethernet_isrEnet0Tx (void)
{
    while (((ENET_EIR_REG(ENET0->regMap) & ENET_EIR_TXB_MASK) != 0) ||
           ((ENET_EIR_REG(ENET0->regMap) & ENET_EIR_TXF_MASK) != 0))
    {
        ENET_EIR_REG(ENET0->regMap) |= ENET_EIR_TXB_MASK;
        ENET_EIR_REG(ENET0->regMap) |=
                (ENET_EIR_REG(ENET0->regMap) & ENET_EIR_TXF_MASK);
        ENET0->callbackTx();
    }

    ENET_TDAR_REG(ENET0->regMap) |= ENET_TDAR_TDAR_MASK;
}

/**
 *  The function called on time-stamp timer timeout interrupt occurrence.
 *  It cleans the related interrupt flags and calls the upper level callback
 *  function.
 */
void Ethernet_isrEnet0Ts (void)
{
    if ((ENET_EIR_REG(ENET0->regMap) & ENET_EIR_TS_TIMER_MASK) != 0)
    {
        ENET_EIR_REG(ENET0->regMap) |= ENET_EIR_TS_TIMER_MASK;

        /* TODO: Increase Ptp timer counter */
        if(ENET0->callbackTs)
        {
            ENET0->callbackTs();
        }
    }
}

/**
 *  Initializes the buffer decriptors.
 *
 *  @param[in] dev The ethernet device
 */
static void Ethernet_initBufferDescriptors (Ethernet_DeviceHandle dev)
{
    uint32_t rxBdIndex;
    uint32_t txBdIndex;

    volatile Ethernet_BufferDescriptor* rxBdTmp;
    volatile Ethernet_BufferDescriptor* txBdTmp;

    ENET_RDSR_REG(dev->regMap) = (uint32_t) dev->bd->rx;
    ENET_MRBR_REG(dev->regMap) = (dev->bd->rxBufferSizeAlign & ENET_MRBR_R_BUF_SIZE_MASK);

    ENET_TDSR_REG(dev->regMap) = (uint32_t) (dev->bd->tx);

    rxBdTmp = dev->bd->rx;
    for(rxBdIndex = 0; rxBdIndex < dev->bd->rxBdNumber; rxBdIndex++)
    {
        rxBdTmp->buffer = (uint8_t *)LONGSWAP((uint32_t)&dev->bd->rxBuffer[rxBdIndex * dev->bd->rxBufferSizeAlign]);
        rxBdTmp->length = 0; /* Initialize data length */

        /* The last buffer descriptor should be set with the wrap flag */
        if (rxBdIndex == (dev->bd->rxBdNumber - 1))
        {
            rxBdTmp->control |= ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_WRAP;
        }

        rxBdTmp->control |= ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_EMPTY; /* Initialize Buffer Descriptor with empty bit*/
        rxBdTmp->controlExtend1 |= ETHERNET_BUFFERDESCRIPTOR_CTRLEXT1_RX_INTERRUPT; /* Enable receive interrupt*/
        rxBdTmp++;
    }

    txBdTmp = dev->bd->tx;
    for(txBdIndex = 0; txBdIndex < dev->bd->txBdNumber; txBdIndex++)
    {
        txBdTmp->buffer = (uint8_t *)LONGSWAP((uint32_t)&dev->bd->txBuffer[txBdIndex * dev->bd->txBufferSizeAlign]);
        txBdTmp->length = 0; /* Set data length*/

        /* The last buffer descriptor should be set with the wrap flag */
        if (txBdIndex == dev->bd->txBdNumber - 1)
        {
            txBdTmp->control |= ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_WRAP;
        }

        txBdTmp++;
    }
}

/**
 *  Intializes the MAC controller.
 *
 *  @param[in] dev    The ethernet device
 *  @param[in] config The MAC controller configuration structure
 */
static void Ethernet_initMac (Ethernet_DeviceHandle dev, Ethernet_MacConfig* config)
{
    uint32_t macAddress;

    switch (config->mode)
    {
    case ETHERNET_MACOPERATINGMODE_SLEEP:
        ENET_ECR_REG(dev->regMap) |= ENET_ECR_SLEEP_MASK;
        ENET_ECR_REG(dev->regMap) |= ENET_ECR_MAGICEN_MASK;
        break;
    case ETHERNET_MACOPERATINGMODE_NORMAL:
        ENET_ECR_REG(dev->regMap) &=~ ENET_ECR_SLEEP_MASK;
        ENET_ECR_REG(dev->regMap) &=~ ENET_ECR_MAGICEN_MASK;
        break;
    }

    if (config->control & ETHERNET_MACCONFIG_STOP_MODE_ENABLE)
        ENET_ECR_REG(dev->regMap) |= ENET_ECR_STOPEN_MASK;

    if (config->control & ETHERNET_MACCONFIG_DEBUG_MODE_ENABLE)
        ENET_ECR_REG(dev->regMap) |= ENET_ECR_DBGEN_MASK;

    if (config->control & ETHERNET_MACCONFIG_PAYLOAD_CHECK_ENABLE)
        ENET_RCR_REG(dev->regMap) |= ENET_RCR_NLC_MASK;

    if (config->control & ETHERNET_MACCONFIG_RX_FLOW_CONTROL_ENABLE)
    {
        ENET_RCR_REG(dev->regMap) |= ENET_RCR_CFEN_MASK;
        ENET_RCR_REG(dev->regMap) |= ENET_RCR_FCE_MASK;
        ENET_OPD_REG(dev->regMap) |= config->pauseDuration;
    }

    if (config->control & ETHERNET_MACCONFIG_RX_CRC_FWD_ENABLE)
        ENET_RCR_REG(dev->regMap) &=~ ENET_RCR_CRCFWD_MASK;

    if (config->control & ETHERNET_MACCONFIG_RX_PAUSE_FWD_ENABLE)
        ENET_RCR_REG(dev->regMap) |= ENET_RCR_PAUFWD_MASK;

    if (config->control & ETHERNET_MACCONFIG_RX_PAD_MOVE_ENABLE)
        ENET_RCR_REG(dev->regMap) |= ENET_RCR_PADEN_MASK;

    if (config->control & ETHERNET_MACCONFIG_RX_BC_REJECT_ENABLE)
        ENET_RCR_REG(dev->regMap) |= ENET_RCR_BC_REJ_MASK;

    if (config->control & ETHERNET_MACCONFIG_RX_PROMISCUOUS_ENABLE)
        ENET_RCR_REG(dev->regMap) |= ENET_RCR_PROM_MASK;

    if (config->control & ETHERNET_MACCONFIG_TX_CRC_FWD_ENABLE)
        ENET_TCR_REG(dev->regMap) |= ENET_TCR_CRCFWD_MASK;

    if (config->control & ETHERNET_MACCONFIG_MAC_ADDR_INSERT)
        ENET_TCR_REG(dev->regMap) |= ENET_TCR_ADDINS_MASK;

    if (config->control & ETHERNET_MACCONFIG_TX_ACCEL_ENABLE)
        ENET_TACC_REG(dev->regMap) |= config->txAccelerConfig;

    if (config->control & ETHERNET_MACCONFIG_RX_ACCEL_ENABLE)
        ENET_RACC_REG(dev->regMap) |= config->rxAccelerConfig;

    if (config->control & ETHERNET_MACCONFIG_MAC_MIB_ENABLE)
        ENET_MIBC_REG(dev->regMap) &=~ ENET_MIBC_MIB_CLEAR_MASK;

    if (config->control & ETHERNET_MACCONFIG_SMI_PREAMBLE_DISABLE)
        ENET_MSCR_REG(dev->regMap) |= ENET_MSCR_DIS_PRE_MASK;

    if (config->control & ETHERNET_MACCONFIG_MAC_ENHANCED_ENABLE)
        ENET_ECR_REG(dev->regMap) |= ENET_ECR_EN1588_MASK;

    /* Control if store and forward is requested */
    if (((config->control & ETHERNET_MACCONFIG_TX_ACCEL_ENABLE) &&
         (config->txAccelerConfig & (ETHERNET_TXACCEL_CONFIG_IP_HEADER_CHECKSUM_ENABLED
         | ETHERNET_TXACCEL_CONFIG_PROTO_CHECKSUM_ENABLED))) ||
         (config->control & ETHERNET_MACCONFIG_STORE_AND_FWD_ENABLE))
    {
        dev->isTxStoreForwardEnable = TRUE;
    }
    if (((config->control & ETHERNET_MACCONFIG_RX_ACCEL_ENABLE) &&
         (config->rxAccelerConfig & (ETHERNET_RXACCEL_CONFIG_WRONG_IPV4_FRAME_DISCARD
         | ETHERNET_RXACCEL_CONFIG_WRONG_PROTO_FRAME_DISCARD))) ||
         (config->control & ETHERNET_MACCONFIG_STORE_AND_FWD_ENABLE))
    {
        ENET_RSFL_REG(dev->regMap) = 0;
    }

    /* Verify and set MAC Special Configurations */
    if (config->rxMaxFrameLength != 0)
        ENET_RCR_REG(dev->regMap) |=
                (ENET_RCR_MAX_FL_MASK & (config->rxMaxFrameLength<<ENET_RCR_MAX_FL_SHIFT));

    if (config->rxTruncLength != 0)
      ENET_FTRL_REG(dev->regMap) |= (config->rxTruncLength & ENET_FTRL_TRUNC_FL_MASK);

    if (config->txInterPacketGap != 0)
    {
        if (config->txInterPacketGap < ETHERNET_MIN_VALID_TX_IPG)
            ENET_TIPG_REG(dev->regMap) |= ETHERNET_MIN_VALID_TX_IPG;
        else if (config->txInterPacketGap > ETHERNET_MAX_VALID_TX_IPG)
            ENET_TIPG_REG(dev->regMap) |= ETHERNET_MAX_VALID_TX_IPG;
        else
            ENET_TIPG_REG(dev->regMap) |= config->txInterPacketGap;
    }

    /* Set MAC Address */
    macAddress = (config->macAddress.addrByte.addr[0] << 24) |
                 (config->macAddress.addrByte.addr[1] << 16) |
                 (config->macAddress.addrByte.addr[2] << 8)  |
                 (config->macAddress.addrByte.addr[3]);
    ENET_PALR_REG(dev->regMap) = macAddress;
    macAddress = 0;
    macAddress = (config->macAddress.addrByte.addr[4] << 24) |
                 (config->macAddress.addrByte.addr[5] << 16);
    ENET_PAUR_REG(dev->regMap) |= macAddress;
}

/**
 *  Initializes the Reduced Media Independent Interface (RMII).
 *
 *  @param[in] dev    The ethernet device
 *  @param[in] config The RMII configuration structure
 */
static void Ethernet_initRmii (Ethernet_DeviceHandle dev, Ethernet_RmiiConfig* config)
{
    ENET_RCR_REG(dev->regMap) |= ENET_RCR_MII_MODE_MASK; /* This bit has always to be set */
    ENET_RCR_REG(dev->regMap) &=~ ENET_RCR_LOOP_MASK;

    if (config)
    {
        if (config->isRmiiMode)
        {
            ENET_RCR_REG(dev->regMap) |= ENET_RCR_RMII_MODE_MASK;
        }
        else
        {
            ENET_RCR_REG(dev->regMap) &=~ ENET_RCR_RMII_MODE_MASK;

            if (config->isLoopEnabled)
                ENET_RCR_REG(dev->regMap) |= ENET_RCR_LOOP_MASK;
        }

        if (config->isSpeed10T)
            ENET_RCR_REG(dev->regMap) |= ENET_RCR_RMII_10T_MASK;
        else
            ENET_RCR_REG(dev->regMap) &=~ ENET_RCR_RMII_10T_MASK;

        if (config->isFullDuplex)
        {
            ENET_TCR_REG(dev->regMap) |= ENET_TCR_FDEN_MASK;
        }
        else
        {
            ENET_TCR_REG(dev->regMap) &=~ ENET_TCR_FDEN_MASK;

            if (config->isRxOnTxDisabled)
                ENET_RCR_REG(dev->regMap) |= ENET_RCR_DRT_MASK;
            else
                ENET_RCR_REG(dev->regMap) &=~ ENET_RCR_DRT_MASK;
        }
    }
    else
    {
        ENET_RCR_REG(dev->regMap) |= ENET_RCR_RMII_MODE_MASK;
        ENET_TCR_REG(dev->regMap) |= ENET_TCR_FDEN_MASK;
    }
}

/**
 *  Initializes the FIFO.
 *
 *  @param[in] dev    The ethernet device
 *  @param[in] config The FIFO configuration structure
 */
static void Ethernet_initFifo (Ethernet_DeviceHandle dev, Ethernet_FifoConfig* config)
{
    if (config)
    {
        if(!dev->isTxStoreForwardEnable)
        {
            ENET_TFWR_REG(dev->regMap) |= ENET_TFWR_STRFWD_MASK;
            ENET_TFWR_REG(dev->regMap) |= (config->txFifoWrite & ENET_TFWR_TFWR_MASK);
        }
        else
        {
            ENET_TFWR_REG(dev->regMap) &=~ ENET_TFWR_STRFWD_MASK;
        }

        ENET_TSEM_REG(dev->regMap) |= config->txFifoEmpty;
        ENET_TAEM_REG(dev->regMap) |= config->txFifoAlmostEmpty;
        ENET_TAFL_REG(dev->regMap) |= config->txFifoAlmostFull;

        ENET_RSFL_REG(dev->regMap) |= config->rxFifoFull;
        ENET_RSEM_REG(dev->regMap) |= config->rxFifoEmpty;
        ENET_RAEM_REG(dev->regMap) |= config->rxFifoAlmostEmpty;
        ENET_RAFL_REG(dev->regMap) |= config->rxFifoAlmostFull;
    }
    else
    {
        ENET_TAEM_REG(dev->regMap) |= ETHERNET_FIFO_MIN_ALMOST_EMPTY;
        ENET_TAFL_REG(dev->regMap) |= ETHERNET_FIFO_DEFAULT_TX_ALMOST_FULL;

        ENET_RAEM_REG(dev->regMap) |= ETHERNET_FIFO_MIN_ALMOST_EMPTY;
        ENET_RAFL_REG(dev->regMap) |= ETHERNET_FIFO_MIN_ALMOST_EMPTY;
    }
}

/**
 *  Initialize the Precision Time Protocol (PTP) timer.
 *
 *  @param[in] dev    The ethernet device
 *  @param[in] config The PTP configuration structure
 */
static void Ethernet_initPtpTimer (Ethernet_DeviceHandle dev, Ethernet_PtpConfig* config)
{
    /* Restart 1588 timer */
    ENET_ATCR_REG(dev->regMap) |= ENET_ATCR_RESTART_MASK;

    /* Init 1588 timer */
    /* Set increase value for PTP timer */
    ENET_ATINC_REG(dev->regMap) |= ENET_ATINC_INC(config->clockIncease);
    /* Set wrap time for PTP timer */
    ENET_ATPER_REG(dev->regMap) = config->period;

    /* Set periodical event and the event signal output assertion */
    ENET_ATCR_REG(dev->regMap) |= ENET_ATCR_PEREN_MASK;
    ENET_ATCR_REG(dev->regMap) |= ENET_ATCR_PINPER_MASK;

    /* Set PTP timer slave/master mode */
    if(config->isSlaveEnabled)
        ENET_ATCR_REG(dev->regMap) |= ENET_ATCR_SLAVE_MASK;
    else
        ENET_ATCR_REG(dev->regMap) &=~ ENET_ATCR_SLAVE_MASK;

    /* Active 1588 timer */
    ENET_ATCR_REG(dev->regMap) |= ENET_ATCR_EN_MASK;
}

System_Errors Ethernet_init (Ethernet_DeviceHandle dev, Ethernet_Config *config)
{
    uint8_t clockCycleMdioNo; /* Number of Clock Cycles for MDIO Holding time */
    uint32_t timeout = 0;

    /* Enable the clock to the selected ENET */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* Init all pins */
    Ethernet_initPins(dev);

    /* Reset Ethernet */
    ENET_ECR_REG(dev->regMap) = ENET_ECR_RESET_MASK;
    /* Check for reset complete */
    while ((ENET_ECR_REG(dev->regMap) & ENET_ECR_RESET_MASK) && (timeout < ETHERNET_MAX_TIMEOUT))
        timeout++;

    /* Check for timeout*/
    if(timeout >= ETHERNET_MAX_TIMEOUT)
        return ERRORS_ETHERNET_TIMEOUT;

    /* Disable Ethernet interrupt and Clear interrupt events */
    ENET_EIMR_REG(dev->regMap) = 0;
    ENET_EIR_REG(dev->regMap) |= 0x7FFFFFFF;

    /* Clear multicast group and individual hash register */
    ENET_GALR_REG(dev->regMap) = ENET_GAUR_REG(dev->regMap) = 0;
    ENET_IALR_REG(dev->regMap) = ENET_IAUR_REG(dev->regMap) = 0;

    /* init MAC */
    Ethernet_initMac(dev,&(config->mac));

    /* init RMII */
    Ethernet_initRmii(dev,&(config->rmii));

    /* Set hold time for MDIO */
    clockCycleMdioNo = (uint8_t) 10/(1000000000/Clock_getFrequency(CLOCK_BUS));
    /*TODO: Find the exact frequency source for the calculation of Clock cycle (System Clock or Bus Clock?)*/
    ENET_MSCR_REG(dev->regMap) |= ENET_MSCR_HOLDTIME(clockCycleMdioNo);

    /* Set register for MDC frequency */
    /*TODO: Find the exact frequency source for the calculation of MII_SPEED (System Clock, Bus Clock or Local Transceiver Clock?)*/
    ENET_MSCR_REG(dev->regMap) |=
            ENET_MSCR_MII_SPEED((config->internalSpeed/(2*ETHERNET_MDC_FREQUENCY))-1);

    /* init FIFO */
    Ethernet_initFifo(dev,&(config->fifo));

    /* init BUFFER DESCRIPTOR */
    dev->bd = config->bd;
    Ethernet_initBufferDescriptors(dev);

    /* init PTP timer */
    /* TODO: reach a stable implementation of ptp timer */
    if (config->mode == ETHERNET_MODE_1588)
    {
        SIM_SOPT2 |= SIM_SOPT2_TIMESRC(2); /* OSCERCLK selected */
        Ethernet_initPtpTimer(dev,&(config->ptp));
    }

    /* Interrupt Enabling Section */
    if (!(config->ptp.isSlaveEnabled))
    {
        /* Enable Timestamp Ptp Interrupt. */
        ENET_EIMR_REG(dev->regMap) |= ENET_EIMR_TS_TIMER_MASK;
        dev->callbackTs = config->callbackTsIsr;
        Interrupt_enable(dev->isrTsNumber);
    }

    if (config->callbackRxIsr)
    {
        dev->callbackRx = config->callbackRxIsr;
        /* Enable RX interrupt after PHY initialization */
    }

    if (config->callbackTxIsr)
    {
        dev->callbackTx = config->callbackTxIsr;
        /* Enable interrupt TX */
        Interrupt_enable(dev->isrTxNumber);
        ENET_EIMR_REG(dev->regMap) |= ENET_EIMR_TXB_MASK;
    }

    /* Enable Ethernet Module */
    ENET_ECR_REG(dev->regMap) |= ENET_ECR_ETHEREN_MASK;

    /* Buffer descriptor byte swapping for little-endian system and endianness configurable IP */
#ifndef ETHERNET_BIGENDIAN
    ENET_ECR_REG(dev->regMap) |= ENET_ECR_DBSWP_MASK;
#endif

    /* Enable Rx Buffer Descriptor. Must be done after enabling Ethernet Module */
    ENET_RDAR_REG(dev->regMap) |= ENET_RDAR_RDAR_MASK;

    dev->devInitialized = 1;

    return ERRORS_NO_ERROR;
}

void Ethernet_disableInterrupt (Ethernet_DeviceHandle dev, Ethernet_Interrupt source)
{
    switch (source)
    {
    case ETHERNET_INTERRUPT_RX:
        Interrupt_disable(dev->isrRxNumber);
        ENET_EIMR_REG(dev->regMap) &= ~ENET_EIMR_RXF_MASK;
        break;
    case ETHERNET_INTERRUPT_TX:
        Interrupt_disable(dev->isrTxNumber);
        ENET_EIMR_REG(dev->regMap) &= ~ENET_EIMR_TXB_MASK;
        break;
        //TODO: Handle Timestamp Inerrupt case.
    }
}

void Ethernet_enableInterrupt (Ethernet_DeviceHandle dev, Ethernet_Interrupt source)
{
    switch (source)
    {
    case ETHERNET_INTERRUPT_RX:
        Interrupt_enable(dev->isrRxNumber);
        ENET_EIMR_REG(dev->regMap) |= ENET_EIMR_RXF_MASK;
        break;
    case ETHERNET_INTERRUPT_TX:
        Interrupt_enable(dev->isrTxNumber);
        ENET_EIMR_REG(dev->regMap) |= ENET_EIMR_TXB_MASK;
        break;
        //TODO: Handle Timestamp Interrupt case.
    }
}

void Ethernet_updateStatus (Ethernet_DeviceHandle dev)
{
    /* Clear all */
    dev->statusFlag = 0;
    dev->maxFrameLength = 0;

    if (ENET_MSCR_REG(dev->regMap) & ENET_MSCR_MII_SPEED_MASK)
    {
        dev->statusFlag |= ETHERNET_STATUS_MII_CONFIG_FLAG;
    }
    dev->maxFrameLength = (uint16_t) ((ENET_RCR_REG(dev->regMap) & ENET_RCR_MAX_FL_MASK)>>ENET_RCR_MAX_FL_SHIFT);
}

System_Errors Ethernet_smiOperation (Ethernet_DeviceHandle dev,
                                     uint32_t phyAddress,
                                     uint32_t phyRegister,
                                     Ethernet_Smi smi,
                                     uint32_t* data)
{
    uint32_t count;
    ENET_EIR_REG(dev->regMap) |= ENET_EIR_MII_MASK;

    if ((smi == ETHERNET_SMI_WRITE_FRAME_COMPLIANT) || (smi == ETHERNET_SMI_WRITE_FRAME_NOT_COMPLIANT))
    {
        ENET_MMFR_REG(dev->regMap) = (ENET_MMFR_ST(1)          |
                                      ENET_MMFR_OP(smi)        |
                                      ENET_MMFR_PA(phyAddress) |
                                      ENET_MMFR_RA(phyRegister)|
                                      ENET_MMFR_TA(2)          |
                                      ((*data)&0xFFFF));
    }
    else if ((smi == ETHERNET_SMI_READ_FRAME_COMPLIANT) || (smi == ETHERNET_SMI_READ_FRAME_NOT_COMPLIANT))
    {
        ENET_MMFR_REG(dev->regMap) = (ENET_MMFR_ST(1)          |
                                      ENET_MMFR_OP(smi)        |
                                      ENET_MMFR_PA(phyAddress) |
                                      ENET_MMFR_RA(phyRegister)|
                                      ENET_MMFR_TA(2));
    }

    for (count = 0; count < ETHERNET_MAX_TIMEOUT; count++)
    {
        if ((ENET_EIR_REG(dev->regMap) & ENET_EIR_MII_MASK) != 0)
            break;

        if (count == ETHERNET_MAX_TIMEOUT-1)
            return ERRORS_ETHERNET_SMI_TIMEOUT; /* SMI timeout */
    }

    if ((smi == ETHERNET_SMI_READ_FRAME_COMPLIANT) || (smi == ETHERNET_SMI_READ_FRAME_NOT_COMPLIANT))
        *data = ENET_MMFR_REG(dev->regMap) & ENET_MMFR_DATA_MASK;

    ENET_EIR_REG(dev->regMap) |= ENET_EIR_MII_MASK;
    return ERRORS_ETHERNET_OK;
}

/*!
 * @}
 */ /* end of group Ethernet_K64F12 */

#endif /* LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F */

#endif /* LIBOHIBOARD_ETHERNET */
