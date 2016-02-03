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
 * @file libohiboard/source/ethernet.c
 * @author Simone Giacomucci <simone.giacomucci@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Ethernet HAL implementations for K64F12 and FRDMK64F.
 */

#ifdef LIBOHIBOARD_ETHERNET

#include "platforms.h"

#include "ethernet.h"
#include "interrupt.h"
#include "clock.h"

uint8_t* Ethernet_getBufferDescriptorData (volatile Ethernet_BufferDescriptor* bd)
{
  return bd->buffer;
}

void Ethernet_clearRxBufferDescriptorAfterHandled (volatile Ethernet_BufferDescriptor* bd)
{
    bd->control &= ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_WRAP;               /* Clear status*/
    bd->control |= ETHERNET_BUFFERDESCRIPTOR_CONTROL_RX_EMPTY;              /* Set rx Buffer Descriptor empty*/
    bd->controlExtend1 |= ETHERNET_BUFFERDESCRIPTOR_CTRLEXT1_RX_INTERRUPT;  /* Enable interrupt*/
}

void Ethernet_setTxBufferDescriptorBeforeSend (
        volatile Ethernet_BufferDescriptor* bd,
        uint16_t dataLength,
        bool isTxTsConfigured,
        bool isTxCrcEnable,
        bool isLast)
{
    bd->length = dataLength; /* Set data length*/
    if(isLast)
    {
        bd->control |= ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_LAST;
        if(isTxCrcEnable)
        {
            bd->control |= ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_TRANSMIT_CRC;  /* set control */
        }
    }

    bd->controlExtend1 |= ETHERNET_BUFFERDESCRIPTOR_CTRLEXT1_TX_INTERRUPT;
    bd->controlExtend2 = 0;
    bd->control |= ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_READY;

    /* TODO: PTP Timestamp implementations */
}

void Ethernet_clearTxBufferDescriptorAfterSend (volatile Ethernet_BufferDescriptor* bd)
{
    bd->length = 0;                                            /* Set data length.*/
    bd->control &= ETHERNET_BUFFERDESCRIPTOR_CONTROL_TX_WRAP; /* Set control.*/
    bd->controlExtend1 = 0;
}

#endif /* LIBOHIBOARD_ETHERNET */
