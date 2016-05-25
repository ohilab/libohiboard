/******************************************************************************
 * Copyright (C) 2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
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
 * @file libohiboard/include/dma.h
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief DMA configuration and parameter.
 */

#ifdef LIBOHIBOARD_DMA

#ifndef __DMA_H
#define __DMA_H

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)   || \
    defined (LIBOHIBOARD_K12D5)      || \
    defined (LIBOHIBOARD_KL25Z4)     || \
    defined (LIBOHIBOARD_KV46F)      || \
    defined (LIBOHIBOARD_TRWKV46F)

#include "platforms.h"
#include "errors.h"
#include "types.h"


typedef enum
{
    DMA_REQ_SURCE_NOT_USED     = 0x00,

	DMA_REQ_UART_RECEIVE       = 0x01,
	DMA_REQ_UART_TRANSMIT      = 0x02,

	DMA_REQ_DAC_UPPER_POINTER  = 0x03,
	DMA_REQ_DAC_BOTTOM_POINTER = 0x04,
	DMA_REQ_DAC_BOOTH_POINTER  = 0x05,
	DMA_REQ_DAC_BUFFER         = 0x06,

	DMA_REQ_ADC_CONV_COMPLETE  = 0x07,

} Dma_RequestSource;


typedef enum
{
	DMA_CHANNEL_0  = 0,
	DMA_CHANNEL_1  = 1,
	DMA_CHANNEL_2  = 2,
	DMA_CHANNEL_3  = 3,

#if	defined (LIBOHIBOARD_K64F12)   || \
	defined (LIBOHIBOARD_FRDMK64F) || \
	defined (LIBOHIBOARD_K12D5)    || \
	defined (LIBOHIBOARD_KV46F)    || \
	defined (LIBOHIBOARD_TRWKV46F)

	DMA_CHANNEL_4  = 4,
	DMA_CHANNEL_5  = 5,
	DMA_CHANNEL_6  = 6,
	DMA_CHANNEL_7  = 7,
	DMA_CHANNEL_8  = 8,
	DMA_CHANNEL_9  = 9,
	DMA_CHANNEL_10 = 10,
	DMA_CHANNEL_11 = 11,
	DMA_CHANNEL_12 = 12,
	DMA_CHANNEL_13 = 13,
	DMA_CHANNEL_14 = 14,
	DMA_CHANNEL_15 = 15,
#endif

} Dma_Channel;

typedef enum
{
    DMA_DATASIZE_8BIT    = 0x00,
    DMA_DATASIZE_16BIT   = 0x01,
    DMA_DATASIZE_32BIT   = 0x02,
    DMA_DATASIZE_16BYTE  = 0x04,
    DMA_DATASIZE_32BYTE  = 0x05,

} Dma_DataSize;

typedef enum
{
	DMA_TRANSFERMODE_STEAL_CYCLE = 0x1,
	DMA_TRANSFERMODE_CONTINUOS   = 0x0,

} Dma_TransferMode;

typedef enum
{
	DMA_LINKCHANNEL_0 = 0x0,
	DMA_LINKCHANNEL_1 = 0x1,
	DMA_LINKCHANNEL_2 = 0x2,
	DMA_LINKCHANNEL_3 = 0x3

} Dma_LinkCannel;

typedef struct Dma_Config
{
    Dma_Channel channel;

	uint32_t sourceAddress;
	uint32_t destinationAddress;

#if defined(LIBOHIBOARD_K64F12)    || \
	defined(LIBOHIBOARD_FRDMK64F)  || \
	defined(LIBOHIBOARD_K12D5)     || \
	defined (LIBOHIBOARD_KV46F)    || \
	defined (LIBOHIBOARD_TRWKV46F)

	/* Source and destination for minor cycle */
	uint8_t  sourceOff;
	uint8_t  destinationOff;

	/* Last source adjustment */
	int32_t  lsAdjust;

	/* Last destination adjustment */
	int32_t  ldAdjust;

	/* This is for trigger by pit */
	uint8_t enableTimerTrig;

#elif defined(LIBOHIBOARD_KL25Z4)

	/* Apply destination or source offset */

	bool incrementSource;
	bool incrementDestination;
	bool enableAA;

	Dma_LinkCannel linkCh1;
	Dma_LinkCannel linkCh2;

#endif

	uint8_t sourceModulo;
	uint8_t destinationModulo;

	/* Source and destination data size */
    Dma_DataSize sSize;
    Dma_DataSize dSize;

    /* Number of byte for request */
    uint32_t nByteforReq;

    /* Counter of major iteration count */
    uint32_t nOfCycle;

    /* Transfer mode */
    Dma_TransferMode transferMode;

    /* Disable channel after transfer complete */
    bool disableAfterComplete;

    /*Handler of the peripheral that trigger the DMA */
    void *pHandler;

} Dma_Config;

typedef struct Dma_Device* Dma_DeviceHandle;

extern Dma_DeviceHandle OB_DMA0;

/**
 * This function initialize a DMA Channel
 *
 * @param[in] dev
 * @param[in] pHandler
 * @param[in] request
 * @param[in] channel
 * @param[in] callback
 */
System_Errors  Dma_init(Dma_DeviceHandle dev,
                        void* pHandler,
                        Dma_RequestSource request,
                        Dma_Channel channel,
                        void *callback);

/**
 * This function disable a DMA Channel
 *
 * @param[in] dev
 * @param[in] channel
 */
void Dma_disableChannel(Dma_DeviceHandle dev, Dma_Channel channel);

/**
 * This function start a pre-configured DMA Channel
 *
 * @param[in] dev
 * @param[in] config
 */
void Dma_startChannel(Dma_DeviceHandle dev, Dma_Config* config);


#endif /* FIXME: Only for some microcontrollers... */

#endif /* __DMA_H */

#endif/* LIBOHIBOARD_DMA */
