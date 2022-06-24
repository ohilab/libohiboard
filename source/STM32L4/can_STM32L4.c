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
#include "clock.h"
#include "gpio.h"
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

#define CAN_IS_VALID_FILTER_MODE(MODE)           (((MODE) == CAN_FILTERMODE_ID_MASK) || \
                                                  ((MODE) == CAN_FILTERMODE_ID_LIST))

#define CAN_IS_VALID_FILTER_SCALE(SCALE)         (((SCALE) == CAN_FILTERSCALE_16_BIT) || \
                                                  ((SCALE) == CAN_FILTERSCALE_32_BIT))

#define CAN_IS_VALID_FILTER_FIFO(FIFO)           (((FIFO) == CAN_FILTERFIFO_0) || \
                                                  ((FIFO) == CAN_FILTERFIFO_1))

#define CAN_IS_VALID_FILTER_ACTIVE(ENABLE)       (((ENABLE) == UTILITY_STATE_DISABLE) || \
                                                  ((ENABLE) == UTILITY_STATE_ENABLE))

#define CAN_CLOCK_ENABLE(REG,MASK)               do { \
                                                    UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                                    asm("nop"); \
                                                    (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                                  } while (0)

#define CAN_CLOCK_DISABLE(REG,MASK)              do { \
                                                    UTILITY_CLEAR_REGISTER_BIT(REG,MASK); \
                                                    asm("nop"); \
                                                    (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                                  } while (0)

#define CAN_IS_VALID_TIMING_SEG_1(VALUE)         ((VALUE) <= 15u)
#define CAN_IS_VALID_TIMING_SEG_2(VALUE)         ((VALUE) <= 7u)
#define CAN_IS_VALID_TIMING_BRP(VALUE)           ((VALUE) <= 1023u)
#define CAN_IS_VALID_TIMING_SJW(VALUE)           ((VALUE) <= 3)

#define CAN_MAX_PINS                             5

typedef struct _Can_Device
{
    CAN_TypeDef* regmap;                           /**< Device memory pointer */

    volatile uint32_t* rccRegisterPtr;       /**< Register for clock enabling */
    uint32_t rccRegisterEnable;         /**< Register mask for current device */

    Can_RxPins rxPins[CAN_MAX_PINS];
    Can_TxPins txPins[CAN_MAX_PINS];

    Gpio_Pins rxPinsGpio[CAN_MAX_PINS];
    Gpio_Pins txPinsGpio[CAN_MAX_PINS];
    Gpio_Alternate rxPinsMux[CAN_MAX_PINS];
    Gpio_Alternate txPinsMux[CAN_MAX_PINS];

    Interrupt_Vector isrTxNumber;                   /**< ISR TX vector number */
    Interrupt_Vector isrRx0Number;                 /**< ISR RX0 vector number */
    Interrupt_Vector isrRx1Number;                 /**< ISR RX1 vector number */
    Interrupt_Vector isrSceNumber;                 /**< ISR SCE vector number */

    Can_DeviceState state;                      /**< Current peripheral state */

    Can_Config config;                                /**< User configuration */

    /** The callback function pointers for TX Mailbox0 complete event. */
    void (*callbackTxMailbox0Complete)(struct _Can_Device* dev, void* obj);
    /** The callback function pointers for TX Mailbox1 complete event. */
    void (*callbackTxMailbox1Complete)(struct _Can_Device* dev, void* obj);
    /** The callback function pointers for TX Mailbox2 complete event. */
    void (*callbackTxMailbox2Complete)(struct _Can_Device* dev, void* obj);
    /** The callback function pointers for RX FIFO 0 full event. */
    void (*callbackRxFifo0Full)(struct _Can_Device* dev, void* obj);
    /** The callback function pointers for RX FIFO 0 message pending event. */
    void (*callbackRxFifo0MessagePending)(struct _Can_Device* dev, void* obj);
    /** The callback function pointers for RX FIFO 1 full event. */
    void (*callbackRxFifo1Full)(struct _Can_Device* dev, void* obj);
    /** The callback function pointers for RX FIFO 1 message pending event. */
    void (*callbackRxFifo1MessagePending)(struct _Can_Device* dev, void* obj);

    /** Useful object added to callback when interrupt triggered. */
    void* callbackObj;

} Can_Device;

#define CAN_IS_DEVICE(DEVICE) (((DEVICE) == OB_CAN1))

static Can_Device can1 =
{
    .regmap              = CAN1,

    .rccRegisterPtr      = &RCC->APB1ENR1,
    .rccRegisterEnable   = RCC_APB1ENR1_CAN1EN,

    .rxPins              =
    {
                           CAN_PINS_PA11,
                           CAN_PINS_PB8,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                           CAN_PINS_PD0,
#endif
    },
    .rxPinsGpio          =
    {
                           GPIO_PINS_PA11,
                           GPIO_PINS_PB8,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                           GPIO_PINS_PD0,
#endif
    },
    .rxPinsMux           =
    {
                           GPIO_ALTERNATE_9,
                           GPIO_ALTERNATE_9,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                           GPIO_ALTERNATE_9,
#endif
    },


    .txPins              =
    {
                           CAN_PINS_PA12,
                           CAN_PINS_PB9,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                           CAN_PINS_PD1,
#endif
    },
    .txPinsGpio          =
    {
                           GPIO_PINS_PA12,
                           GPIO_PINS_PB9,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                           GPIO_PINS_PD1,
#endif
    },
    .txPinsMux           =
    {
                           GPIO_ALTERNATE_9,
                           GPIO_ALTERNATE_9,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                           GPIO_ALTERNATE_9,
#endif
    },

    .isrTxNumber         = INTERRUPT_CAN1_TX,
    .isrRx0Number        = INTERRUPT_CAN1_RX0,
    .isrRx1Number        = INTERRUPT_CAN1_RX1,
    .isrSceNumber        = INTERRUPT_CAN1_SCE,

    .state               = CAN_DEVICESTATE_RESET,

    .callbackTxMailbox0Complete    = NULL,
    .callbackTxMailbox1Complete    = NULL,
    .callbackTxMailbox2Complete    = NULL,
    .callbackRxFifo0Full           = NULL,
    .callbackRxFifo0MessagePending = NULL,
    .callbackRxFifo1Full           = NULL,
    .callbackRxFifo1MessagePending = NULL,
    .callbackObj         = NULL,
};
Can_DeviceHandle OB_CAN1 = &can1;

System_Errors Can_setRxPin (Can_DeviceHandle dev, Can_RxPins rxPin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < CAN_MAX_PINS; ++devPinIndex)
    {
        if (dev->rxPins[devPinIndex] == rxPin)
        {
            Gpio_configAlternate(dev->rxPinsGpio[devPinIndex],
                                 dev->rxPinsMux[devPinIndex],
                                 0);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_CAN_NO_PIN_FOUND;
}

System_Errors Can_setTxPin (Can_DeviceHandle dev, Can_TxPins txPin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < CAN_MAX_PINS; ++devPinIndex)
    {
        if (dev->txPins[devPinIndex] == txPin)
        {
            Gpio_configAlternate(dev->txPinsGpio[devPinIndex],
                                 dev->txPinsMux[devPinIndex],
                                 0);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_CAN_NO_PIN_FOUND;
}

void Can_setBaudrate (Can_DeviceHandle dev,
                      uint32_t syncJumpWidth,
                      uint32_t timeSeg1,
                      uint32_t timeSeg2,
                      uint32_t prescaler)
{
    // Clear flags
    dev->regmap->BTR &= ~(CAN_BTR_BRP_Msk | CAN_BTR_TS1_Msk | CAN_BTR_TS2_Msk | CAN_BTR_BRP_Msk);

    // Write values
    dev->regmap->BTR |= (syncJumpWidth | timeSeg1 | timeSeg2 | (prescaler - 1u));
}

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

    err  = ohiassert(CAN_IS_VALID_TIMING_SEG_1(config->timeSeg1));
    err |= ohiassert(CAN_IS_VALID_TIMING_SEG_2(config->timeSeg2));
    err |= ohiassert(CAN_IS_VALID_TIMING_BRP(config->prescaler));
    err |= ohiassert(CAN_IS_VALID_TIMING_SJW(config->syncJumpWidth));
    err |= ohiassert(UTILITY_VALID_STATE(config->timeTriggeredMode));
    err |= ohiassert(UTILITY_VALID_STATE(config->autoBusOff));
    err |= ohiassert(UTILITY_VALID_STATE(config->autoRetransmission));
    err |= ohiassert(UTILITY_VALID_STATE(config->fifoLocked));
    err |= ohiassert(UTILITY_VALID_STATE(config->txPriority));
    err |= ohiassert(UTILITY_VALID_STATE(config->autoWakeUp));

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
        // Enable peripheral clock
        CAN_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

        // Enable pins
        if (config->rxPin != CAN_PINS_RXNONE)
            Can_setRxPin(dev,config->rxPin);

        if (config->txPin != CAN_PINS_TXNONE)
            Can_setTxPin(dev,config->txPin);
    }
    dev->state = CAN_DEVICESTATE_BUSY;

    // Exit from sleep mode
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_SLEEP);

    uint32_t tickstart = System_currentTick();
    // Wait the acknowledge
    while ((dev->regmap->MSR & CAN_MSR_SLAK) != 0U)
    {
        // Check for the Timeout
        if ((System_currentTick() - tickstart) > CAN_TIMEOUT_VALUE)
        {
            // Change device state to ERROR
            dev->state = CAN_DEVICESTATE_ERROR;

            return ERRORS_CAN_TIMEOUT;
        }
    }

    // Request initialisation
    UTILITY_SET_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_INRQ);

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

    // Set the time triggered communication mode
    if (dev->config.timeTriggeredMode == UTILITY_STATE_ENABLE)
    {
        UTILITY_SET_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_TTCM);
    }
    else
    {
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_TTCM);
    }

    // Set the automatic bus-off management
    if (dev->config.autoBusOff == UTILITY_STATE_ENABLE)
    {
        UTILITY_SET_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_ABOM);
    }
    else
    {
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_ABOM);
    }

    // Set the automatic wake-up mode
    if (dev->config.autoWakeUp == UTILITY_STATE_ENABLE)
    {
        UTILITY_SET_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_AWUM);
    }
    else
    {
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_AWUM);
    }

    // Set the automatic retransmission
    if (dev->config.autoRetransmission == UTILITY_STATE_ENABLE)
    {
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_NART);
    }
    else
    {
        UTILITY_SET_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_NART);
    }

    // Set the receive FIFO locked mode
    if (dev->config.fifoLocked == UTILITY_STATE_ENABLE)
    {
        UTILITY_SET_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_RFLM);
    }
    else
    {
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_RFLM);
    }

    // Set the transmit FIFO priority
    if (dev->config.txPriority == UTILITY_STATE_ENABLE)
    {
        UTILITY_SET_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_TXFP);
    }
    else
    {
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_TXFP);
    }

    // Write operative mode
    dev->regmap->BTR = dev->config.mode;

    // Set the bit timing register
    Can_setBaudrate(dev,
                    dev->config.syncJumpWidth,
                    dev->config.timeSeg1,
                    dev->config.timeSeg2,
                    dev->config.prescaler);

    // ---------------------------------------
    // Enable interrupts...
    // ---------------------------------------
    if (config->callbackRxFifo0Full != NULL)
    {
        // Copy callback
        dev->callbackRxFifo0Full = config->callbackRxFifo0Full;

        // Enable interrupt
        UTILITY_SET_REGISTER_BIT(dev->regmap->IER,CAN_IER_FFIE0_Msk);

        // Enable NVIC interrupt
        Interrupt_enable(dev->isrRx0Number);
    }

    if (config->callbackRxFifo0MessagePending != NULL)
    {
        // Copy callback
        dev->callbackRxFifo0MessagePending = config->callbackRxFifo0MessagePending;

        // Enable interrupt
        UTILITY_SET_REGISTER_BIT(dev->regmap->IER,CAN_IER_FMPIE0_Msk);

        // Enable NVIC interrupt
        Interrupt_enable(dev->isrRx0Number);
    }

    if (config->callbackRxFifo1Full != NULL)
    {
        // Copy callback
        dev->callbackRxFifo1Full = config->callbackRxFifo1Full;

        // Enable interrupt
        UTILITY_SET_REGISTER_BIT(dev->regmap->IER,CAN_IER_FFIE1_Msk);

        // Enable NVIC interrupt
        Interrupt_enable(dev->isrRx0Number);
    }

    if (config->callbackRxFifo1MessagePending != NULL)
    {
        // Copy callback
        dev->callbackRxFifo1MessagePending = config->callbackRxFifo1MessagePending;

        // Enable interrupt
        UTILITY_SET_REGISTER_BIT(dev->regmap->IER,CAN_IER_FMPIE1_Msk);

        // Enable NVIC interrupt
        Interrupt_enable(dev->isrRx1Number);
    }

    if ((config->callbackTxMailbox0Complete != NULL) ||
        (config->callbackTxMailbox1Complete != NULL) ||
        (config->callbackTxMailbox2Complete != NULL))
    {
        // Copy callback
        dev->callbackTxMailbox0Complete = config->callbackTxMailbox0Complete;
        dev->callbackTxMailbox1Complete = config->callbackTxMailbox1Complete;
        dev->callbackTxMailbox2Complete = config->callbackTxMailbox2Complete;

        // Enable interrupt
        UTILITY_SET_REGISTER_BIT(dev->regmap->IER,CAN_IER_TMEIE_Msk);

        // Enable NVIC interrupt
        Interrupt_enable(dev->isrTxNumber);
    }

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

    // Stop the device
    Can_stop(dev);

    // TODO: deInit pin

    // Reset the peripheral
    UTILITY_SET_REGISTER_BIT(dev->regmap->MCR,CAN_MCR_RESET);

    dev->state = CAN_DEVICESTATE_RESET;

    return ERRORS_NO_ERROR;
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
    err  = ohiassert(CAN_IS_VALID_FILTER_BANK_NUMBER(filter->bank));
    err |= ohiassert(CAN_IS_VALID_FILTER_ACTIVE(filter->activation));
    err |= ohiassert(CAN_IS_VALID_FILTER_FIFO(filter->fifo));
    err |= ohiassert(CAN_IS_VALID_FILTER_MODE(filter->mode));
    err |= ohiassert(CAN_IS_VALID_FILTER_SCALE(filter->scale));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_CAN_WRONG_PARAM;
    }

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

        // Filter Mode
        if (filter->mode == CAN_FILTERMODE_ID_MASK)
        {
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->FM1R,filterPos);
        }
        else // CAN_FILTERMODE_ID_LIST
        {
            UTILITY_SET_REGISTER_BIT(dev->regmap->FM1R,filterPos);
        }

        // Filter FIFO assignment
        if (filter->fifo == CAN_FILTERFIFO_0)
        {
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->FFA1R,filterPos);
        }
        else
        {
            UTILITY_SET_REGISTER_BIT(dev->regmap->FFA1R,filterPos);
        }

        // Activate filter
        if (filter->activation == UTILITY_STATE_ENABLE)
        {
            UTILITY_SET_REGISTER_BIT(dev->regmap->FA1R,filterPos);
        }

        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->FMR,CAN_FMR_FINIT);

        return ERRORS_NO_ERROR;
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
    err |= ohiassert(CAN_IS_VALID_IDTYPE(header->idType));
    // Check RTR
    err |= ohiassert(CAN_IS_VALID_RTR(header->frameType));
    // Check DLC
    err |= ohiassert(CAN_IS_VALID_DLC(header->frameLength));
    // Check id validity
    if (header->idType == CAN_MESSAGEIDTYPE_STANDARD)
    {
        err |= ohiassert(CAN_IS_STANDARD_ID(header->idStandard));
    }
    else
    {
        err |= ohiassert(CAN_IS_EXTENDED_ID(header->idExtended));
    }
    err |= ohiassert(CAN_IS_VALID_TRANSMIT_GLOBAL_TIME(header->globalTime));

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
        return ERRORS_NO_ERROR;
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

void Can_setCallbackObject (Can_DeviceHandle dev, void* obj)
{
    ohiassert(obj != NULL);

    if (obj != NULL)
    {
        dev->callbackObj = obj;
    }
}

static inline void __attribute__((always_inline)) Can_callbackInterrupt (Can_DeviceHandle dev)
{
    uint32_t tsrflags = dev->regmap->TSR;
    uint32_t interrupts = dev->regmap->IER;
    uint32_t rf0rflags = dev->regmap->RF0R;
    uint32_t rf1rflags = dev->regmap->RF1R;

    // Transmit Mailbox empty interrupt management
    if ((interrupts & CAN_IER_TMEIE_Msk) != 0u)
    {
        // Mailbox0
        if ((tsrflags & CAN_TSR_RQCP0) != 0u)
        {
            // Clear flag
            UTILITY_SET_REGISTER_BIT(dev->regmap->sTxMailBox[0].TIR,CAN_TI0R_TXRQ_Msk);

            if ((tsrflags & CAN_TSR_TXOK0_Msk) != 0)
            {
                if (dev->callbackTxMailbox0Complete != NULL)
                {
                    dev->callbackTxMailbox0Complete(dev,dev->callbackObj);
                }
            }
            else
            {
                // TODO ERROR!
            }
        }

        // Mailbox1
        if ((tsrflags & CAN_TSR_RQCP1) != 0u)
        {
            // Clear flag
            UTILITY_SET_REGISTER_BIT(dev->regmap->sTxMailBox[1].TIR,CAN_TI1R_TXRQ_Msk);

            if ((tsrflags & CAN_TSR_TXOK1_Msk) != 0)
            {
                if (dev->callbackTxMailbox1Complete != NULL)
                {
                    dev->callbackTxMailbox1Complete(dev,dev->callbackObj);
                }
            }
            else
            {
                // TODO ERROR!
            }
        }

        // Mailbox2
        if ((tsrflags & CAN_TSR_RQCP2) != 0u)
        {
            // Clear flag
            UTILITY_SET_REGISTER_BIT(dev->regmap->sTxMailBox[2].TIR,CAN_TI2R_TXRQ_Msk);

            if ((tsrflags & CAN_TSR_TXOK2_Msk) != 0)
            {
                if (dev->callbackTxMailbox2Complete != NULL)
                {
                    dev->callbackTxMailbox2Complete(dev,dev->callbackObj);
                }
            }
            else
            {
                // TODO ERROR!
            }
        }
    }

    // RX FIFO 0 overrun
    if ((interrupts & CAN_IER_FOVIE0_Msk) != 0)
    {
        if ((rf0rflags & CAN_RF0R_FOVR0_Msk) != 0)
        {
            // Clear flag
            UTILITY_SET_REGISTER_BIT(dev->regmap->RF0R,CAN_RF0R_FOVR0_Msk);
        }
    }

    // RX FIFO 0 full
    if ((interrupts & CAN_IER_FFIE0_Msk) != 0)
    {
        if ((rf0rflags & CAN_RF0R_FULL0_Msk) != 0)
        {
            // Clear flag
            UTILITY_SET_REGISTER_BIT(dev->regmap->RF0R,CAN_RF0R_FULL0_Msk);

            if (dev->callbackRxFifo0Full != NULL)
            {
                dev->callbackRxFifo0Full(dev,dev->callbackObj);
            }
        }
    }

    // RX FIFO 0 message pending
    if ((interrupts & CAN_IER_FMPIE0_Msk) != 0)
    {
        if ((rf0rflags & CAN_RF0R_FMP0_Msk) != 0)
        {
            // Clear flag
            UTILITY_SET_REGISTER_BIT(dev->regmap->RF0R,CAN_RF0R_FMP0_Msk);

            if (dev->callbackRxFifo0MessagePending != NULL)
            {
                dev->callbackRxFifo0MessagePending(dev,dev->callbackObj);
            }
        }
    }

    // RX FIFO 1 overrun
    if ((interrupts & CAN_IER_FOVIE0_Msk) != 0)
    {
        if ((rf1rflags & CAN_RF1R_FOVR1_Msk) != 0)
        {
            // Clear flag
            UTILITY_SET_REGISTER_BIT(dev->regmap->RF1R,CAN_RF1R_FOVR1_Msk);
        }
    }

    // RX FIFO 1 full
    if ((interrupts & CAN_IER_FFIE1_Msk) != 0)
    {
        if ((rf1rflags & CAN_RF1R_FULL1_Msk) != 0)
        {
            // Clear flag
            UTILITY_SET_REGISTER_BIT(dev->regmap->RF1R,CAN_RF1R_FULL1_Msk);

            if (dev->callbackRxFifo1Full != NULL)
            {
                dev->callbackRxFifo1Full(dev,dev->callbackObj);
            }
        }
    }

    // RX FIFO 1 message pending
    if ((interrupts & CAN_IER_FMPIE1_Msk) != 0)
    {
        if ((rf1rflags & CAN_RF1R_FMP1_Msk) != 0)
        {
            // Clear flag
            UTILITY_SET_REGISTER_BIT(dev->regmap->RF1R,CAN_RF1R_FMP1_Msk);

            if (dev->callbackRxFifo1MessagePending != NULL)
            {
                dev->callbackRxFifo1MessagePending(dev,dev->callbackObj);
            }
        }
    }
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
