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
 * @file libohiboard/source/ethernet-interface_K64F12.c
 * @author Simone Giacomucci <simone.giacomucci@gmail.com>
 * @brief Ethernet HAL implementations for K64F12 and FRDMK64F.
 */

#ifdef LIBOHIBOARD_ETHERNET

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

#include <stdlib.h>

#include "platforms.h"

#include "ethernet.h"
#include "ethernet-interface.h"

/* Buffer Alignment Macros */
#define ETHERNET_DRIVER_ALIGN(x,align) ((unsigned int)((x) + ((align)-1)) & (unsigned int)(~(unsigned int)((align)- 1)))
#define ETHERNET_DRIVER_BD_ALIGNMENT   (16)

#define ETHERNET_MAX_TIMEOUT                        0xFFFF /**< Ethernet Timeout. */
#define ETHERNET_MAX_FRAME_SIZE                     1518   /**< Ethernet Max Frame Size. */
//#define ETHERNET_VLAN_SIZE                          1522   /**< Maximum VLAN frame size*/
//#define ETHERNET_MAX_FRAME_DATA_SIZE                1500   /**< Maximum frame data size*/
//#define ETHERNET_DEFAULT_TRUNC_LEN                  2047   /**< Default Truncate length*/
//#define ETHERNET_DEFAULT_IPG                        12     /**< ENET default transmit inter packet gap*/
//#define ETHERNET_MAX_VALID_TX_IPG                   27     /**< Maximum valid transmit IPG*/
//#define ETHERNET_MIN_VALID_IPG                      8      /**< Minimum valid transmit IPG*/
//#define ETHERNET_MAX_MDIO_HOLD_CYCLE                7      /**< Maximum hold time clock cycle on MDIO Output*/
#define ETHERNET_MAX_FRAME_BUFFERDESCRIPTOR_NUMBER  6      /**< Maximum buffer descriptor numbers of a frame */
#define ETHERNET_FRAME_FCS_LEN                      4      /**< FCS length */
//#define ETHERNET_HEAD_LEN                           14     /**< Ethernet Frame header length*/
//#define ETHERNET_VLAN_HEAD_LEN                      18     /**< Ethernet VLAN frame header length*/

void EthernetInterface_txInterruptHandler ();
void EthernetInterface_rxInterruptHandler ();

/**
 *  Defines the main mid-low level ethernet interface structure.
 */
typedef struct EthernetInterface_Device
{
    uint8_t devNumber;
    Ethernet_DeviceHandle dev;
    Ethernet_Config config;            /**< Ethernet Configuration Structure. */

    /** Buffer Descriptors Handling Structure. */
    EthernetInterface_BufferDescriptorHandler bdHandler;
    Ethernet_BufferDescriptors bd;

    /** Function used to handle receiving data to upper levels. */
    void (*receiveDataCallback) (
            EthernetInterface_DeviceHandle  dev,
            EthernetInterface_BufferData *buffer);

    /** Indicates if the CRC is forwarded to the upper levels */
    bool isRxCrcForwardEnable;
    /** Indicates if the buffer descriptor has to transmit CRC */
    bool isTxCrcEnable;

} EthernetInterface_Device;

EthernetInterface_Device enetif0 = {
        .dev = 0,
        .devNumber = 0, /* DEV ENET0 */

        .config = {
                .mode = ETHERNET_MODE_1588,

                .mac = {
                        .control = ETHERNET_MACCONFIG_RX_CRC_FWD_ENABLE |
                                   ETHERNET_MACCONFIG_MAC_ENHANCED_ENABLE,
                        .mode = ETHERNET_MACOPERATINGMODE_NORMAL,
                },

                .rmii = {
                        .isRmiiMode = TRUE,
                        .isSpeed10T = FALSE,
                        .isFullDuplex = TRUE,
                        .isRxOnTxDisabled = TRUE,
                        .isLoopEnabled = FALSE,
                },

                .callbackRxIsr = EthernetInterface_rxInterruptHandler,
                .callbackTxIsr = EthernetInterface_txInterruptHandler,
        },

        .bd = {
                .rxBdNumber = 8,
                .txBdNumber = 4,
        },

        .isTxCrcEnable = TRUE,
        .isRxCrcForwardEnable = FALSE,
};
EthernetInterface_DeviceHandle ENETIF0 = &enetif0;

/**
 *  Initialize and allocates the memory necessary to use transmitting and receiving buffer descriptors.
 *
 *  @param[in] dev The main mid-low level ethernet interface
 */
static void EthernetInterface_initBufferDescriptor(EthernetInterface_DeviceHandle dev)
{
    uint32_t rxBufferSizeAlign, txBufferSizeAlign;
    uint8_t  *txBufferAlign, *rxBufferAlign, *txBdPtr, *rxBdPtr, *txBuffer, *rxBuffer;
    volatile Ethernet_BufferDescriptor *txBdPtrAlign, *rxBdPtrAlign;

    txBdPtr = (uint8_t *)calloc(1,dev->bd.txBdNumber * sizeof(Ethernet_BufferDescriptor) + ETHERNET_DRIVER_BD_ALIGNMENT);
    txBdPtrAlign = (volatile Ethernet_BufferDescriptor *)ETHERNET_DRIVER_ALIGN((uint32_t)txBdPtr, ETHERNET_DRIVER_BD_ALIGNMENT);

    rxBdPtr = (uint8_t *)calloc(1,dev->bd.rxBdNumber * sizeof(Ethernet_BufferDescriptor) + ETHERNET_DRIVER_BD_ALIGNMENT);
    rxBdPtrAlign = (volatile Ethernet_BufferDescriptor *)ETHERNET_DRIVER_ALIGN((uint32_t)rxBdPtr, ETHERNET_DRIVER_BD_ALIGNMENT);

    rxBufferSizeAlign = ETHERNET_DRIVER_ALIGN(ETHERNET_MAX_FRAME_SIZE , ETHERNET_DRIVER_BD_ALIGNMENT);
    rxBuffer = (uint8_t *)calloc(1,dev->bd.rxBdNumber * rxBufferSizeAlign  + ETHERNET_DRIVER_BD_ALIGNMENT);
    rxBufferAlign = (uint8_t *)ETHERNET_DRIVER_ALIGN((uint32_t)rxBuffer, ETHERNET_DRIVER_BD_ALIGNMENT);

    txBufferSizeAlign = ETHERNET_DRIVER_ALIGN(ETHERNET_MAX_FRAME_SIZE , ETHERNET_DRIVER_BD_ALIGNMENT);
    txBuffer = calloc(1,dev->bd.txBdNumber * txBufferSizeAlign + ETHERNET_DRIVER_BD_ALIGNMENT);
    txBufferAlign = (uint8_t *)ETHERNET_DRIVER_ALIGN((uint32_t)txBuffer, ETHERNET_DRIVER_BD_ALIGNMENT);

    dev->bd.rx= rxBdPtrAlign;
    dev->bd.rxBuffer = rxBufferAlign;
    dev->bd.rxBufferSizeAlign = rxBufferSizeAlign;
    dev->bd.tx = txBdPtrAlign;
    dev->bd.txBuffer = txBufferAlign;
    dev->bd.txBufferSizeAlign = txBufferSizeAlign;
}

/**
 *  Cleans all receiving buffer descriptors after receiving data.
 *
 *  @param[in] dev The main mid-low level ethernet interface
 */
static void EthernetInterface_clearRxBufferDescriptor(EthernetInterface_DeviceHandle dev)
{
    while ((dev->bdHandler.rxBdCurrPtr != dev->bdHandler.rxBdToCleanPtr) || dev->bdHandler.isRxBufferDescriptorFull)
    {
        Ethernet_clearRxBufferDescriptorAfterHandled(dev->bdHandler.rxBdToCleanPtr);

        /* Increment current Buffer Descriptor index in Ethernet Interface Buffer Descriptor Handler*/
        if (!(Ethernet_hasNextRxBufferDescriptor(dev->bdHandler.rxBdToCleanPtr)))
            dev->bdHandler.rxBdToCleanPtr = dev->bdHandler.rxBdBasePtr;
        else
            dev->bdHandler.rxBdToCleanPtr++;

        dev->bdHandler.isRxBufferDescriptorFull = FALSE;

        /* Active Rx Buffer Descriptor */
        Ethernet_activeRxBufferDescriptor();
    }
}

/**
 *  Cleans all transmitting descriptors after sent data.
 *
 *  @param[in] dev The main mid-low level ethernet interface
 */
static void EthernetInterface_clearTxBufferDescriptor(EthernetInterface_DeviceHandle dev)
{
    volatile Ethernet_BufferDescriptor *currBd;

    while ((dev->bdHandler.txBdToCleanPtr != dev->bdHandler.txBdCurrPtr))
    {
        currBd = dev->bdHandler.txBdToCleanPtr;
        /* Get the control status data, If the bd has not been processed break out */
        if(Ethernet_isTxBufferDescriptorReady(currBd))
            break;

        /* Clear the buffer descriptor buffer address */
        Ethernet_clearTxBufferDescriptorAfterSend(currBd);

        /* Update the buffer address */
        if (!(Ethernet_hasNextTxBufferDescriptor(dev->bdHandler.txBdToCleanPtr)))
            dev->bdHandler.txBdToCleanPtr = dev->bdHandler.txBdBasePtr;
        else
            dev->bdHandler.txBdToCleanPtr++;
    }
}

System_Errors EthernetInterface_init (EthernetInterface_DeviceHandle dev)
{
    /* Choose right device */
    switch (dev->devNumber)
    {
    case  0:
        dev->dev = ENET0;
        break;
    default:
        return ERRORS_ETHERNETIF_WRONG_DEVICE;
    }

    /* Init buffer descriptor */
    EthernetInterface_initBufferDescriptor(dev);
    dev->config.bd = &dev->bd;

    Ethernet_init(dev->dev,&dev->config);

    if (dev->config.mac.control && ETHERNET_MACCONFIG_RX_CRC_FWD_ENABLE)
        dev->isRxCrcForwardEnable = TRUE;
    else
        dev->isRxCrcForwardEnable = FALSE;

    /* Save buffer descriptor handler */
    dev->bdHandler.rxBdBasePtr = dev->bd.rx;
    dev->bdHandler.rxBdCurrPtr = dev->bd.rx;
    dev->bdHandler.rxBdToCleanPtr = dev->bd.rx;

    dev->bdHandler.txBdBasePtr = dev->bd.tx;
    dev->bdHandler.txBdCurrPtr = dev->bd.tx;
    dev->bdHandler.txBdToCleanPtr = dev->bd.tx;

    dev->bdHandler.rxBuffSizeAlign = dev->bd.rxBufferSizeAlign;
    dev->bdHandler.txBuffSizeAlign = dev->bd.txBufferSizeAlign;

    return ERRORS_NO_ERROR;
}

System_Errors EthernetInterface_ReceiveData (EthernetInterface_DeviceHandle dev)
{
    volatile Ethernet_BufferDescriptor *currBd;
    EthernetInterface_BufferData buffer[dev->bd.rxBdNumber];
    uint16_t size = 0;
    uint16_t dataIndex = 0;

    currBd = dev->bdHandler.rxBdCurrPtr;

    while (!(Ethernet_isRxBufferDescriptorEmpty(currBd)))
    {
        if (dev->bdHandler.isRxBufferDescriptorFull)
        {
            EthernetInterface_clearRxBufferDescriptor(dev);
            return ERRORS_ETHERNETIF_RX_BUFFERDESCRIPTOR_FULL;
        }

        /* Increment current Buffer Descriptor index in Ethernet Interface Buffer Descriptor Handler */
        if (!(Ethernet_hasNextRxBufferDescriptor(dev->bdHandler.rxBdCurrPtr)))
            dev->bdHandler.rxBdCurrPtr = dev->bdHandler.rxBdBasePtr;
        else
            dev->bdHandler.rxBdCurrPtr++;

        if (dev->bdHandler.rxBdCurrPtr == dev->bdHandler.rxBdToCleanPtr)
            dev->bdHandler.isRxBufferDescriptorFull = TRUE;

        if (Ethernet_isRxFrameTruncated(currBd))
        {
            EthernetInterface_clearRxBufferDescriptor(dev);
            return ERRORS_ETHERNETIF_RX_FRAME_TRUNCATED;
        }

        if (Ethernet_isLastRxBufferDescriptor(currBd))
        {
            if (Ethernet_hasRxBufferDescriptorErrors(currBd))
            {
                EthernetInterface_clearRxBufferDescriptor(dev);
                return ERRORS_ETHERNETIF_RX_GENERIC_ERROR;
            }
            else
            {
                buffer[dataIndex].data =
                        Ethernet_getBufferDescriptorData(currBd);
                buffer[dataIndex].size = currBd->length;
                if (dev->isRxCrcForwardEnable)
                    buffer[dataIndex].size -= ETHERNET_FRAME_FCS_LEN;

                buffer[dataIndex].next = NULL;

                if (dev->receiveDataCallback)
                {
                    dev->receiveDataCallback(dev,buffer);
                }
                /* Clean all receiving Buffer Descriptors after get all data */
                EthernetInterface_clearRxBufferDescriptor(dev);
            }
        }
        else
        {
            buffer[dataIndex].data = Ethernet_getBufferDescriptorData(currBd);
            buffer[dataIndex].size = currBd->length;
            buffer[dataIndex].next = &buffer[dataIndex+1];
            dataIndex++;

            if (dataIndex == ETHERNET_MAX_FRAME_BUFFERDESCRIPTOR_NUMBER)
            {
                EthernetInterface_clearRxBufferDescriptor(dev);
                return ERRORS_ETHERNETIF_RX_SMALL_BUFFERDESCRIPTOR_NUMBER;
            }
        }

        currBd = dev->bdHandler.rxBdCurrPtr;
    }
}

/*TODO: Implement Buffer Descriptor errors.*/
System_Errors EthernetInterface_sendData(EthernetInterface_DeviceHandle dev,
                                         uint32_t dataLength,
                                         uint32_t bdUsedNumber)
{
    volatile Ethernet_BufferDescriptor *currBd;
    uint8_t buffIndex;
    uint32_t size = 0;

    currBd = dev->bdHandler.txBdCurrPtr;
    /*TODO: Implement Ptp*/

    for (buffIndex = 0; buffIndex < bdUsedNumber; buffIndex++)
    {
        if (dev->bdHandler.isTxBufferDescriptorFull)
            return ERRORS_ETHERNETIF_RX_BUFFERDESCRIPTOR_FULL;

        /* Last Buffer Descriptor */
        if (buffIndex == bdUsedNumber - 1)
        {
            /*
             * Set the transmit flag in the buffer descriptor before the frame
             * is sent and indicate it is the last one.
             */
            Ethernet_setTxBufferDescriptorBeforeSend(
                    currBd, dataLength - size, 0, dev->isTxCrcEnable, TRUE);
        }
        else
        {
            /*
             * Set the transmit flag in the buffer descriptor before the frame
             * is sent and indicate it is not the last one.
             */
            Ethernet_setTxBufferDescriptorBeforeSend(
                    currBd, dev->bdHandler.txBuffSizeAlign, 0, dev->isTxCrcEnable, FALSE);
            size += dev->bdHandler.txBuffSizeAlign;
        }

        /* Increase the buffer descriptor address */
        if (!(Ethernet_hasNextTxBufferDescriptor(dev->bdHandler.txBdCurrPtr)))
            dev->bdHandler.txBdCurrPtr = dev->bdHandler.txBdBasePtr;
        else
            dev->bdHandler.txBdCurrPtr++;

        if (dev->bdHandler.txBdCurrPtr == dev->bdHandler.txBdToCleanPtr)
            dev->bdHandler.isTxBufferDescriptorFull = TRUE;
        else
            dev->bdHandler.isTxBufferDescriptorFull = FALSE;

        /* Get the current buffer descriptor address */
        currBd = dev->bdHandler.txBdCurrPtr;

        /* Active the transmit Buffer Descriptor. */
        Ethernet_activeTxBufferDescriptor();
    }
    return ERRORS_NO_ERROR;
}

void EthernetInterface_rxInterruptHandler ()
{
    EthernetInterface_ReceiveData(ENETIF0);
}

void EthernetInterface_txInterruptHandler ()
{
    EthernetInterface_clearTxBufferDescriptor(ENETIF0);
}

void EthernetInterface_disableRxInterrupt (EthernetInterface_DeviceHandle dev)
{
    Ethernet_disableInterrupt(dev->dev, ETHERNET_INTERRUPT_RX);
}

void EthernetInterface_enableRxInterrupt (EthernetInterface_DeviceHandle dev)
{
    Ethernet_enableInterrupt(dev->dev, ETHERNET_INTERRUPT_RX);
}

void EthernetInterface_disableTxInterrupt (EthernetInterface_DeviceHandle dev)
{
    Ethernet_disableInterrupt(dev->dev, ETHERNET_INTERRUPT_RX);
}

void EthernetInterface_enableTxInterrupt (EthernetInterface_DeviceHandle dev)
{
    Ethernet_enableInterrupt(dev->dev, ETHERNET_INTERRUPT_RX);
}

#endif /* LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F */

#endif /* LIBOHIBOARD_ETHERNET */
