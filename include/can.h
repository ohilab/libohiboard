/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2021 A. C. Open Hardware Ideas Lab
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
 */

/**
 * @file libohiboard/include/can.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief CAN definitions and prototypes.
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_CAN

/**
 * @defgroup CAN CAN
 * @brief CAN HAL driver
 * @{
 */

#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "utility.h"

/**
 * @defgroup CAN_Configuration_Functions CAN configuration functions and types
 * @brief Functions and types to open, close and configure a CAN peripheral.
 * @{
 */

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Can_DeviceState
{
    CAN_DEVICESTATE_RESET,
    CAN_DEVICESTATE_READY,
    CAN_DEVICESTATE_BUSY,
    CAN_DEVICESTATE_LISTENING,
    CAN_DEVICESTATE_ERROR,

} Can_DeviceState;

#if (LIBOHIBOARD_VERSION >= 0x20000u)
typedef struct _Can_Device* Can_DeviceHandle;

//typedef void (*Can_callback)(Can_DeviceHandle dev, void* obj);

#else
#error "[LIBOHIBOARD ERROR] Not implemented in the previous version!"
#endif

#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/STM32L4/can_STM32L4.h"

#else
#error "[LIBOHIBOARD ERROR] Not implemented for the current platform!"
#endif

typedef struct _Can_Config
{
//    Uart_RxPins rxPin;
//    Uart_TxPins txPin;

    uint32_t baudrate;

} Can_Config;

/**
 * This function initialize the selected peripheral.
 *
 * @param[in] dev Can device handle
 * @param[in] config Configuration parameters for the Uart
 */
System_Errors Can_init (Can_DeviceHandle dev, Can_Config *config);

/**
 * This function de-initialize the selected peripheral.
 *
 * @param[in] dev Can device handle
 */
System_Errors Can_deInit (Can_DeviceHandle dev);

/**
 * Enable and start CAN peripheral.
 *
 * @param[in] dev CAN device handle
 * @return
 */
System_Errors Can_start (Can_DeviceHandle dev);

/**
 * Stop CAN peripheral.
 *
 * @param[in] dev CAN device handle
 * @return
 */
System_Errors Can_stop (Can_DeviceHandle dev);

/**
 * CAN Filter Mode
 */
typedef enum _Can_FilterMode
{
    CAN_FILTERMODE_ID_MASK = 0x00000000u,
    CAN_FILTERMODE_ID_LIST = 0x00000001u,

} Can_FilterMode;

/**
 * CAN Filter Scale
 */
typedef enum _Can_FilterScale
{
    CAN_FILTERSCALE_16_BIT = 0x00000000u,
    CAN_FILTERSCALE_32_BIT = 0x00000001u,

} Can_FilterScale;

typedef struct _Can_Filter
{
    /**
     * Specifies the filter identification number.
     */
    uint32_t filterId;

    /**
     * Specifies the filter mask number or identification number.
     */
    uint32_t filterMaskId;

    /**
     * Specifies the filter mode to be initialized.
     */
    Can_FilterMode mode;

    /**
     * Specifies the filter scale.
     */
    Can_FilterScale scale;

    /**
     * Specifies the filter bank which will be initialized.
     */
    uint32_t bank;

} Can_Filter, *Can_FilterHandle;

/**
 *
 */
System_Errors Can_configFilter (Can_DeviceHandle dev,
                                Can_FilterHandle filter);

/**
 * @}
 */

/**
 * @defgroup CAN_Message_Functions CAN message functions and types
 * @brief Functions and types to manage messages.
 * @{
 */

/**
 * CAN Identifier Type.
 */
typedef enum _Can_MessageIdType
{
    CAN_MESSAGEIDTYPE_STANDARD = 0x00000000u,
    CAN_MESSAGEIDTYPE_EXTENDED = 0x00000004u,

} Can_MessageIdType;

/**
 * CAN Remote Transmission Request.
 */
typedef enum _Can_RemoteTransmissionRequest
{
    CAN_REMOTETXREQ_DATA   = 0x00000000u,
    CAN_REMOTETXREQ_REMOTE = 0x00000002u,

} Can_RemoteTransmissionRequest;

/**
 * CAN Tx message header definition.
 */
typedef struct _Can_TxHeader
{
    /**
     * Specifies the standard identifier (11 bit).
     * This value must be a number between 0 and 0x7FF.
     */
    uint32_t idStandard;

    /**
     * Specifies the extended identifier (29 bit).
     * This value must be a number between 0 and 0x1FFFFFFF.
     */
    uint32_t idExtended;

    /**
     * Specifies the type of identifier for the message that will be transmitted.
     */
    Can_MessageIdType idType;

    /**
     * Specifies the type of frame for the message that will be transmitted.
     */
    Can_RemoteTransmissionRequest frameType;

    /**
     * Specifies the length of the frame that will be transmitted.
     * This value must be a number between 0 and 8.
     */
    uint32_t frameLength;

    /**
     * Specifies whether the timestamp counter value captured on start
     * of frame transmission is sent in DATA6 and DATA7.
     */
    Utility_State globalTime;

} Can_TxHeader, *Can_TxHeaderHandle;

/**
 * CAN Rx message header definition.
 */
typedef struct _Can_RxHeader
{
    /**
     * Specifies the standard identifier (11 bit).
     * This value must be a number between 0 and 0x7FF.
     */
    uint32_t idStandard;

    /**
     * Specifies the extended identifier (29 bit).
     * This value must be a number between 0 and 0x1FFFFFFF.
     */
    uint32_t idExtended;

    /**
     * Specifies the type of identifier for the message that was transmitted.
     */
    Can_MessageIdType idType;

    /**
     * Specifies the type of frame for the message that was transmitted.
     */
    Can_RemoteTransmissionRequest frameType;

    /**
     * Specifies the length of the frame that was transmitted.
     * This value must be a number between 0 and 8.
     */
    uint32_t frameLength;

    /**
     * Specifies the timestamp counter value captured on start of frame reception.
     */
    uint32_t timestamp;

    /**
     * Specifies the index of matching acceptance filter element.
     */
    uint32_t filterMatchIndex;

} Can_RxHeader, *Can_RxHeaderHandle;

/**
 * CAN Tx Mailboxes
 */
typedef enum _Can_TxMailbox
{
    CAN_TXMAILBOX_0 = 0x00000001u,
    CAN_TXMAILBOX_1 = 0x00000002u,
    CAN_TXMAILBOX_2 = 0x00000004u,

} Can_TxMailbox;

/**
 * CAN Rx FIFO
 */
typedef enum _Can_RxFIFO
{
    CAN_RXFIFO_0 = 0x00000000u,
    CAN_RXFIFO_1 = 0x00000001u,

} Can_RxFIFO;

/**
 *
 */
System_Errors Can_addTxMessage (Can_DeviceHandle dev,
                                Can_TxHeaderHandle header,
                                uint8_t data[],
                                Can_TxMailbox* mailbox);

/**
 *
 */
System_Errors Can_abortTxMessage (Can_DeviceHandle dev,
                                  Can_TxMailbox mailbox);

/**
 *
 */
System_Errors Can_isTxMessagePending (Can_DeviceHandle dev,
                                      Can_TxMailbox mailbox,
                                      bool* isPending);

/**
 *
 */
System_Errors Can_getTxTimestamp (Can_DeviceHandle dev,
                                  Can_TxMailbox mailbox,
                                  uint32_t* timestamp);

/**
 *
 */
System_Errors Can_getRxMessage (Can_DeviceHandle dev,
                                Can_RxHeaderHandle header,
                                uint8_t data[],
                                Can_RxFIFO fifo);

/**
 *
 */
System_Errors Can_getRxFifoLevel (Can_DeviceHandle dev,
                                  Can_RxFIFO fifo,
                                  uint32_t* level);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __CAN_H

/**
 * @}
 */

#endif // LIBOHIBOARD_CAN

/**
 * @}
 */

