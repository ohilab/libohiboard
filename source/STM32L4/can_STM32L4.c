/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2021 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/source/STM32L4/can_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief CAN implementations for STM32L4 series.
 */

#ifdef LIBOHIBOARD_CAN

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L4)

#include "can.h"
#include "interrupt.h"
#include "utility.h"
#include "system.h"

#if !defined (CAN_TIMEOUT_VALUE)
#define CAN_TIMEOUT_VALUE                        10u
#endif

#define CAN_IS_STANDARD_ID(ID)                   ((ID) <= 0x7FFU)
#define CAN_IS_EXTENDED_ID(ID)                   ((ID) <= 0x1FFFFFFFU)
#define CAN_IS_VALID_DLC(DLC)                    ((DLC) <= 8U)
#define CAN_IS_VALID_IDTYPE(IDTYPE)              (((IDTYPE) == CAN_MESSAGEIDTYPE_STANDARD) || \
                                                  ((IDTYPE) == CAN_MESSAGEIDTYPE_EXTENDED))

#define CAN_IS_VALID_RTR(RTR)                    (((RTR) == CAN_REMOTETXREQ_DATA) || \
                                                  ((RTR) == CAN_REMOTETXREQ_REMOTE))

#define CAN_IS_VALID_TRANSMIT_GLOBAL_TIME(ENABLE) (((ENABLE) == UTILITY_STATE_DISABLE) || \
                                                   ((ENABLE) == UTILITY_STATE_ENABLE))

#define CAN_IS_VALID_TX_MAILBOX(MAILBOX)         (((MAILBOX) == CAN_TXMAILBOX_0) || \
                                                  ((MAILBOX) == CAN_TXMAILBOX_1) || \
                                                  ((MAILBOX) == CAN_TXMAILBOX_2))

#define CAN_IS_VALID_RX_FIFO(FIFO)               (((FIFO) == CAN_RXFIFO_0) || \
                                                  ((FIFO) == CAN_RXFIFO_1))

#define CAN_IS_VALID_FILTER_BANK_NUMBER(BANK)    ((BANK) <= 14)

typedef struct _Can_Device
{
    CAN_TypeDef* regmap;                           /**< Device memory pointer */

//    volatile uint32_t* rccRegisterPtr;       /**< Register for clock enabling */
//    uint32_t rccRegisterEnable;         /**< Register mask for current device */
//
//    volatile uint32_t* rccTypeRegisterPtr;   /**< Register for clock enabling */
//    uint32_t rccTypeRegisterMask;       /**< Register mask for user selection */
//    uint32_t rccTypeRegisterPos;        /**< Mask position for user selection */

//    Adc_Pins pins[ADC_MAX_PINS];      /**< List of the pin for the peripheral */
//    Adc_Channels pinsChannel[ADC_MAX_PINS];
//    Gpio_Pins pinsGpio[ADC_MAX_PINS];

//    void (* eocCallback)(struct _Adc_Device *dev);
//    void (* eosCallback)(struct _Adc_Device *dev);
//    void (* overrunCallback)(struct _Adc_Device *dev);

    Interrupt_Vector isrTxNumber;                   /**< ISR TX vector number */
    Interrupt_Vector isrRx0Number;                 /**< ISR RX0 vector number */
    Interrupt_Vector isrRx1Number;                 /**< ISR RX1 vector number */
    Interrupt_Vector isrSceNumber;                 /**< ISR SCE vector number */

    Can_DeviceState state;                      /**< Current peripheral state */

    Can_Config config;                                /**< User configuration */

} Can_Device;

#define CAN_IS_DEVICE(DEVICE) (((DEVICE) == OB_CAN1))

static Can_Device can1 =
{
    .regmap              = CAN1,

//    .rccRegisterPtr      = &RCC->AHB2ENR,
//    .rccRegisterEnable   = RCC_AHB2ENR_ADCEN,
//
//    .rccTypeRegisterPtr  = &RCC->CCIPR,
//    .rccTypeRegisterMask = RCC_CCIPR_ADCSEL,
//    .rccTypeRegisterPos  = RCC_CCIPR_ADCSEL_Pos,

    .isrTxNumber         = INTERRUPT_CAN1_TX,
    .isrRx0Number        = INTERRUPT_CAN1_RX0,
    .isrRx1Number        = INTERRUPT_CAN1_RX1,
    .isrSceNumber        = INTERRUPT_CAN1_SCE,

    .state               = CAN_DEVICESTATE_RESET,
};
Can_DeviceHandle OB_CAN1 = &can1;


System_Errors Can_init (Can_DeviceHandle dev, Can_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }

    // TODO: config check!
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_PARAM;
    }
    // Save configuration
    dev->config = *config;

    // Enable peripheral clock if needed
    if (dev->state == CAN_DEVICESTATE_RESET)
    {

    }

    // TODO: config!

    dev->state = CAN_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Can_deInit (Can_DeviceHandle dev)
{
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
}

System_Errors Can_start (Can_DeviceHandle dev)
{
    uint32_t tickstart = 0;

    System_Errors err = ERRORS_NO_ERROR;
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }

    if (dev->state == CAN_DEVICESTATE_READY)
    {
        // Change current state to listening
        dev->state = CAN_DEVICESTATE_LISTENING;

        // Enable device...
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_INRQ);

        tickstart = System_currentTick();
        // Wait the acknowledge
        while ((dev->regmap->MSR & CAN_MSR_INAK) != 0U)
        {
            // Check for the Timeout
            if ((System_currentTick() - tickstart) > CAN_TIMEOUT_VALUE)
            {
                // Change device state to ERROR
                dev->state = CAN_DEVICESTATE_ERROR;

                return ERRORS_CAN_TIMEOUT;
            }
        }

        return ERRORS_NO_ERROR;
    }
    else
    {
        return ERRORS_CAN_DEVICE_NOT_READY;
    }
}

System_Errors Can_stop (Can_DeviceHandle dev)
{
    uint32_t tickstart = 0;

    System_Errors err = ERRORS_NO_ERROR;
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }

    if (dev->state == CAN_DEVICESTATE_LISTENING)
    {
        UTILITY_SET_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_INRQ);

        tickstart = System_currentTick();
        // Wait the acknowledge
        while ((dev->regmap->MSR & CAN_MSR_INAK) == 0U)
        {
            // Check for the Timeout
            if ((System_currentTick() - tickstart) > CAN_TIMEOUT_VALUE)
            {
                // Change device state to ERROR
                dev->state = CAN_DEVICESTATE_ERROR;

                return ERRORS_CAN_TIMEOUT;
            }
        }

        // Exit from sleep mode...
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_SLEEP);

        // Change current state to ready
        dev->state = CAN_DEVICESTATE_READY;

        return ERRORS_NO_ERROR;
    }
    else
    {
        return ERRORS_CAN_DEVICE_NOT_STARTED;
    }
}

System_Errors Can_configFilter (Can_DeviceHandle dev,
                                Can_FilterHandle filter)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Check device
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }

    // CAN1 is single instance with 14 dedicated filters banks
    err = ohiassert(CAN_IS_VALID_FILTER_BANK_NUMBER(filter->bank));

    if ((dev->state == CAN_DEVICESTATE_LISTENING) ||
        (dev->state == CAN_DEVICESTATE_READY))
    {
        // Start with initialization mode for the filter
        UTILITY_SET_REGISTER_BIT(dev->regmap->FMR,CAN_FMR_FINIT);

        // Convert filter number in bit position
        uint32_t filterPos = (uint32_t)(1 << (filter->bank & 0x1Fu));

        // Deactivation of selected filter
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->FA1R,filterPos);

        if (filter->scale == CAN_FILTERSCALE_16_BIT)
        {
            // CAN filter scale register
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->FS1R,filterPos);

            dev->regmap->sFilterRegister[filter->bank].FR1 =
                    ((0x0000FFFFu & (uint32_t)filter->filterMaskId) << 16u) |
                    ((0x0000FFFFu & (uint32_t)filter->filterId));

            dev->regmap->sFilterRegister[filter->bank].FR2 =
                    ((0xFFFF0000u & (uint32_t)filter->filterMaskId)) |
                    ((0xFFFF0000u & (uint32_t)filter->filterId) >> 16u);

        }
        else // CAN_FILTERSCALE_32_BIT
        {
            // CAN filter scale register
            UTILITY_SET_REGISTER_BIT(dev->regmap->FS1R,filterPos);

            // 32-bit identifier or First 32-bit identifier
            dev->regmap->sFilterRegister[filter->bank].FR1 = filter->filterId;
            // 32-bit mask or Second 32-bit identifier
            dev->regmap->sFilterRegister[filter->bank].FR2 = filter->filterMaskId;
        }

        // TODOx
    }
    else
    {
        return ERRORS_CAN_DEVICE_NOT_READY;
    }
}

System_Errors Can_addTxMessage (Can_DeviceHandle dev,
                                Can_TxHeaderHandle header,
                                uint8_t data[],
                                Can_TxMailbox* mailbox)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Check device
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }

    // Check id type
    err = ohiassert(CAN_IS_VALID_IDTYPE(header->idType));
    // Check RTR
    err = ohiassert(CAN_IS_VALID_RTR(header->frameType));
    // Check DLC
    err = ohiassert(CAN_IS_VALID_DLC(header->frameLength));
    // Check id validity
    if (header->idType == CAN_MESSAGEIDTYPE_STANDARD)
    {
        err = ohiassert(CAN_IS_STANDARD_ID(header->idStandard));
    }
    else
    {
        err = ohiassert(CAN_IS_EXTENDED_ID(header->idExtended));
    }
    err = ohiassert(CAN_IS_VALID_TRANSMIT_GLOBAL_TIME(header->globalTime));

    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_PARAM;
    }

    if ((dev->state == CAN_DEVICESTATE_LISTENING) ||
        (dev->state == CAN_DEVICESTATE_READY))
    {
        uint32_t tsr = dev->regmap->TSR;
        uint32_t txMailbox = 0;

        // Search an empty mailbox
        if (((tsr & CAN_TSR_TME0) != 0u) ||
            ((tsr & CAN_TSR_TME1) != 0u) ||
            ((tsr & CAN_TSR_TME2) != 0u))
        {
            // Select an empty transmit mailbox:
            // In case at least one transmit mailbox is free, the code value is equal
            // to the number of the next transmit mailbox free.
            // In case all transmit mailboxes are pending, the code value is equal to
            // the number of the transmit mailbox with the lowest priority.
            txMailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
            if (txMailbox > 2)
            {
                return ERRORS_CAN_INTERNAL;
            }
            // store the mailbox
            *mailbox = (Can_TxMailbox)(1 << txMailbox);

            // Set up the ID
            if (header->idType == CAN_MESSAGEIDTYPE_STANDARD)
            {
                dev->regmap->sTxMailBox[txMailbox].TIR = ((header->idStandard << CAN_TI0R_STID_Pos) |
                                                           header->frameType);
            }
            else
            {
                dev->regmap->sTxMailBox[txMailbox].TIR = ((header->idExtended << CAN_TI0R_EXID_Pos) |
                                                           header->idType |
                                                           header->frameType);
            }

            // Set up the DLC (length)
            dev->regmap->sTxMailBox[txMailbox].TDTR = header->frameLength;

            // Set up the Transmit Global Time mode
            if (header->globalTime == UTILITY_STATE_ENABLE)
            {
                UTILITY_SET_REGISTER_BIT(dev->regmap->sTxMailBox[txMailbox].TDTR,CAN_TDT0R_TGT);
            }

            // Set up the data...
            dev->regmap->sTxMailBox[txMailbox].TDHR = (uint32_t)(((uint32_t)data[7] << CAN_TDH0R_DATA7_Pos) |
                                                                 ((uint32_t)data[6] << CAN_TDH0R_DATA6_Pos) |
                                                                 ((uint32_t)data[5] << CAN_TDH0R_DATA5_Pos) |
                                                                 ((uint32_t)data[4] << CAN_TDH0R_DATA4_Pos));

            dev->regmap->sTxMailBox[txMailbox].TDLR = (uint32_t)(((uint32_t)data[3] << CAN_TDL0R_DATA3_Pos) |
                                                                 ((uint32_t)data[2] << CAN_TDL0R_DATA2_Pos) |
                                                                 ((uint32_t)data[1] << CAN_TDL0R_DATA1_Pos) |
                                                                 ((uint32_t)data[0] << CAN_TDL0R_DATA0_Pos));

            // Request transmission...
            UTILITY_SET_REGISTER_BIT(dev->regmap->sTxMailBox[txMailbox].TIR,CAN_TI0R_TXRQ);

            return ERRORS_NO_ERROR;
        }
        else
        {
            return ERRORS_CAN_WRONG_PARAM;
        }
    }
    else
    {
        return ERRORS_CAN_DEVICE_NOT_READY;
    }
}

System_Errors Can_abortTxMessage (Can_DeviceHandle dev,
                                  Can_TxMailbox mailbox)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Check device
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }

    err = ohiassert(CAN_IS_VALID_TX_MAILBOX(mailbox));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_PARAM;
    }

    if ((dev->state == CAN_DEVICESTATE_LISTENING) ||
        (dev->state == CAN_DEVICESTATE_READY))
    {
        switch (mailbox)
        {
        case CAN_TXMAILBOX_0:
            // Add cancellation request for Tx Mailbox 0
            UTILITY_SET_REGISTER_BIT(dev->regmap->TSR,CAN_TSR_ABRQ0);
            break;
        case CAN_TXMAILBOX_1:
            // Add cancellation request for Tx Mailbox 1
            UTILITY_SET_REGISTER_BIT(dev->regmap->TSR,CAN_TSR_ABRQ1);
            break;
        case CAN_TXMAILBOX_2:
            // Add cancellation request for Tx Mailbox 2
            UTILITY_SET_REGISTER_BIT(dev->regmap->TSR,CAN_TSR_ABRQ2);
            break;
        }
    }
    else
    {
        return ERRORS_CAN_DEVICE_NOT_READY;
    }
}

System_Errors Can_isTxMessagePending (Can_DeviceHandle dev,
                                      Can_TxMailbox mailbox,
                                      bool* isPending)
{
    System_Errors err = ERRORS_NO_ERROR;
    *isPending = FALSE;

    // Check device
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }

    err = ohiassert(CAN_IS_VALID_TX_MAILBOX(mailbox));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_PARAM;
    }

    if ((dev->state == CAN_DEVICESTATE_LISTENING) ||
        (dev->state == CAN_DEVICESTATE_READY))
    {
        if ((dev->regmap->TSR & (mailbox << CAN_TSR_TME0_Pos)) !=
            (mailbox << CAN_TSR_TME0_Pos))
        {
            *isPending = TRUE;
        }
        return ERRORS_NO_ERROR;
    }
    else
    {
        return ERRORS_CAN_DEVICE_NOT_READY;
    }
}

System_Errors Can_getTxTimestamp (Can_DeviceHandle dev,
                                  Can_TxMailbox mailbox,
                                  uint32_t* timestamp)
{
    System_Errors err = ERRORS_NO_ERROR;
    *timestamp = 0;

    // Check device
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }

    err = ohiassert(CAN_IS_VALID_TX_MAILBOX(mailbox));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_PARAM;
    }

    if ((dev->state == CAN_DEVICESTATE_LISTENING) ||
        (dev->state == CAN_DEVICESTATE_READY))
    {
        uint32_t mailboxNumber = __CLZ(__RBIT(mailbox));
        *timestamp = (dev->regmap->sTxMailBox[mailboxNumber].TDTR & CAN_TDT0R_TIME) >> CAN_TDT0R_TIME_Pos;
        return ERRORS_NO_ERROR;
    }
    else
    {
        return ERRORS_CAN_DEVICE_NOT_READY;
    }
}

System_Errors Can_getRxMessage (Can_DeviceHandle dev,
                                Can_RxHeaderHandle header,
                                uint8_t data[],
                                Can_RxFIFO fifo)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Check device
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }

    err = ohiassert(CAN_IS_VALID_RX_FIFO(fifo));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_PARAM;
    }

    if ((dev->state == CAN_DEVICESTATE_LISTENING) ||
        (dev->state == CAN_DEVICESTATE_READY))
    {
        // Check the FIFO status...
        switch (fifo)
        {
        case CAN_RXFIFO_0:
            // Check than FIFO 0 is not empty
            if ((dev->regmap->RF0R & CAN_RF0R_FMP0) == 0)
            {
                return ERRORS_CAN_WRONG_PARAM;
            }
            break;

        case CAN_RXFIFO_1:
            // Check than FIFO 1 is not empty
            if ((dev->regmap->RF1R & CAN_RF1R_FMP1) == 0)
            {
                return ERRORS_CAN_WRONG_PARAM;
            }
            break;
        }

        // Get header
        header->idType = (Can_MessageIdType)(dev->regmap->sFIFOMailBox[fifo].RIR & CAN_RI0R_RTR);
        if (header->idType == CAN_MESSAGEIDTYPE_STANDARD)
        {
            header->idStandard = (CAN_RI0R_STID & dev->regmap->sFIFOMailBox[fifo].RIR) >> CAN_TI0R_STID_Pos;
        }
        else
        {
            header->idExtended = ((CAN_RI0R_EXID | CAN_RI0R_STID) & dev->regmap->sFIFOMailBox[fifo].RIR) >> CAN_RI0R_EXID_Pos;
        }

        header->frameType        = (Can_RemoteTransmissionRequest)(dev->regmap->sFIFOMailBox[fifo].RIR & CAN_RI0R_RTR);
        header->frameLength      = (dev->regmap->sFIFOMailBox[fifo].RDTR & CAN_RDT0R_DLC) >> CAN_RDT0R_DLC_Pos;
        header->filterMatchIndex = (dev->regmap->sFIFOMailBox[fifo].RDTR & CAN_RDT0R_FMI) >> CAN_RDT0R_FMI_Pos;
        header->timestamp        = (dev->regmap->sFIFOMailBox[fifo].RDTR & CAN_RDT0R_TIME) >> CAN_RDT0R_TIME_Pos;

        // Get data...
        data[0] = (uint8_t)((dev->regmap->sFIFOMailBox[fifo].RDLR & CAN_RDL0R_DATA0) >> CAN_RDL0R_DATA0_Pos);
        data[1] = (uint8_t)((dev->regmap->sFIFOMailBox[fifo].RDLR & CAN_RDL0R_DATA1) >> CAN_RDL0R_DATA1_Pos);
        data[2] = (uint8_t)((dev->regmap->sFIFOMailBox[fifo].RDLR & CAN_RDL0R_DATA2) >> CAN_RDL0R_DATA2_Pos);
        data[3] = (uint8_t)((dev->regmap->sFIFOMailBox[fifo].RDLR & CAN_RDL0R_DATA3) >> CAN_RDL0R_DATA3_Pos);
        data[4] = (uint8_t)((dev->regmap->sFIFOMailBox[fifo].RDHR & CAN_RDH0R_DATA4) >> CAN_RDH0R_DATA4_Pos);
        data[5] = (uint8_t)((dev->regmap->sFIFOMailBox[fifo].RDHR & CAN_RDH0R_DATA5) >> CAN_RDH0R_DATA5_Pos);
        data[6] = (uint8_t)((dev->regmap->sFIFOMailBox[fifo].RDHR & CAN_RDH0R_DATA6) >> CAN_RDH0R_DATA6_Pos);
        data[7] = (uint8_t)((dev->regmap->sFIFOMailBox[fifo].RDHR & CAN_RDH0R_DATA7) >> CAN_RDH0R_DATA7_Pos);

        // Release the selected FIFO
        switch (fifo)
        {
        case CAN_RXFIFO_0:
            UTILITY_SET_REGISTER_BIT(dev->regmap->RF0R,CAN_RF0R_RFOM0);
            break;

        case CAN_RXFIFO_1:
            UTILITY_SET_REGISTER_BIT(dev->regmap->RF1R,CAN_RF1R_RFOM1);
            break;
        }

        return ERRORS_NO_ERROR;
    }
    else
    {
        return ERRORS_CAN_DEVICE_NOT_READY;
    }
}

System_Errors Can_getRxFifoLevel (Can_DeviceHandle dev,
                                  Can_RxFIFO fifo,
                                  uint32_t* level)
{
    System_Errors err = ERRORS_NO_ERROR;
    *level = 0;

    // Check device
    // Check the CAN device
    if (dev == NULL)
    {
        return ERRORS_CAN_NO_DEVICE;
    }
    // Check the CAN instance
    err = ohiassert(CAN_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_DEVICE;
    }

    err = ohiassert(CAN_IS_VALID_RX_FIFO(fifo));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_PARAM;
    }

    if ((dev->state == CAN_DEVICESTATE_LISTENING) ||
        (dev->state == CAN_DEVICESTATE_READY))
    {
        switch (fifo)
        {
        case CAN_RXFIFO_0:
            *level = dev->regmap->RF0R & CAN_RF0R_FMP0;
            break;

        case CAN_RXFIFO_1:
            *level = dev->regmap->RF1R & CAN_RF1R_FMP1;
            break;
        }

        return ERRORS_NO_ERROR;
    }
    else
    {
        return ERRORS_CAN_DEVICE_NOT_READY;
    }
}

static inline void __attribute__((always_inline)) Can_callbackInterrupt (Can_DeviceHandle dev)
{
    (void)dev;
}

#if defined (LIBOHIBOARD_STM32L4x6)

void CAN1_TX_IRQHandler (void)
{
    Can_callbackInterrupt(OB_CAN1);
}

void CAN1_RX0_IRQHandler (void)
{
    Can_callbackInterrupt(OB_CAN1);
}

void CAN1_RX1_IRQHandler (void)
{
    Can_callbackInterrupt(OB_CAN1);
}

void CAN1_SCE_IRQHandler (void)
{
    Can_callbackInterrupt(OB_CAN1);
}
#endif


#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_CAN
