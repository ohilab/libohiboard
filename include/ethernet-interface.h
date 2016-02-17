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
 * @file libohiboard/include/ethernet-interface.h
 * @author Simone Giacomucci <simone.giacomucci@gmail.com>
 * @brief Ethernet mid-low level Interface declarations and constants.
 */

#ifdef LIBOHIBOARD_ETHERNET

#ifndef __ETHERNET_INTERFACE_H
#define __ETHERNET_INTERFACE_H

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "ethernet.h"

typedef struct EthernetInterface_Device* EthernetInterface_DeviceHandle;

#if defined (LIBOHIBOARD_K60DZ10) || \
    defined (LIBOHIBOARD_OHIBOARD_R1)

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

extern EthernetInterface_DeviceHandle ENETIF0;

#endif

/**
 *  Defines the interface to pass to upper level for the handling of data.
 */
typedef struct _EthernetInterface_BufferData
{
    uint8_t *data;
    uint16_t size;

    struct _EthernetInterface_BufferData *next;
} EthernetInterface_BufferData;

/**
 *  Defines the structure used to handle multiple buffer descriptors.
 */
typedef struct _EthernetInterface_BufferDescriptorHandler
{
    volatile Ethernet_BufferDescriptor *rxBdBasePtr;     /**< Receive buffer descriptor base address pointer.*/
    volatile Ethernet_BufferDescriptor *rxBdCurrPtr;     /**< Current receive buffer descriptor pointer.*/
    volatile Ethernet_BufferDescriptor *rxBdToCleanPtr;  /**< Receive buffer descriptor to be cleaned.*/
    volatile Ethernet_BufferDescriptor *txBdBasePtr;     /**< Transmit buffer descriptor base address pointer.*/
    volatile Ethernet_BufferDescriptor *txBdCurrPtr;     /**< Current transmit buffer descriptor pointer.*/
    volatile Ethernet_BufferDescriptor *txBdToCleanPtr;  /**< Last cleaned transmit buffer descriptor pointer.*/

    uint32_t rxBuffSizeAlign;                            /**< Receive buffer size alignment.*/
    uint32_t txBuffSizeAlign;                            /**< Transmit buffer size alignment.*/

    bool isRxBufferDescriptorFull;                       /**< Indicates if Rx buffer descriptor is full.*/
    bool isTxBufferDescriptorFull;                       /**< Indicates if Tx buffer descriptor is full.*/

} EthernetInterface_BufferDescriptorHandler;

/**
 *  Initialize the mid-low level ethernet interface.
 *
 *  @param[in] dev The main mid-low level ethernet interface handle
 *
 *  @return An error if the initialization went wrong.
 */
System_Errors EthernetInterface_init(EthernetInterface_DeviceHandle dev);

/**
 *  Takes data to send from upper level, handles transmitting buffer
 *  descriptors and pass the data to the lower level for transmission.
 *
 *  @param[in] dev          The main mid-low level ethernet interface
 *  @param[in] dataLength   The transmitting data length in byte
 *  @param[in] bdUsedNumber Number of used transmitting buffer descriptors to send the data
 */
System_Errors EthernetInterface_sendData(EthernetInterface_DeviceHandle dev,
                                         uint32_t dataLength,
                                         uint32_t bdUsedNumber);

/**
 *  Disables ethernet receiving interrupt.
 *
 *  @param[in] dev The main mid-low level ethernet interface handle
 */
void EthernetInterface_disableRxInterrupt (EthernetInterface_DeviceHandle dev);
/**
 *  Enables ethernet receiving interrupt.
 *
 *  @param[in] dev The main mid-low level ethernet interface handle
 */
void EthernetInterface_enableRxInterrupt (EthernetInterface_DeviceHandle dev);

/**
 *  Disables ethernet transmitting interrupt.
 *
 *  @param[in] dev The main mid-low level ethernet interface handle
 */
void EthernetInterface_disableTxInterrupt (EthernetInterface_DeviceHandle dev);
/**
 *  Enables ethernet transmitting interrupt.
 *
 *  @param[in] dev The main mid-low level ethernet interface handle
 */
void EthernetInterface_enableTxInterrupt (EthernetInterface_DeviceHandle dev);


#endif /* __ETHERNET_INTERFACE_H */

#endif /* LIBOHIBOARD_ETHERNET */
