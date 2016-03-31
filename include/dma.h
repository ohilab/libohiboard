/******************************************************************************
 * Copyright (C) 2014-2016 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/dma.h
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief DMA configuration and parameter.
 */

#ifdef LIBOHIBOARD_DMA

#ifndef __DMA_H
#define __DMA_H

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)   ||\
    defined (LIBOHIBOARD_K12D5)      ||\
    defined(LIBOHIBOARD_KL25Z4)

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

	DMA_REQ_ADC_CONV_COMPLETE  = 0x06,

}Dma_RequestSourceType;


typedef enum
{
	DMA_CHANNEL_0     =0,
	DMA_CHANNEL_1     =1,
	DMA_CHANNEL_2     =2,
	DMA_CHANNEL_3     =3,

#if	defined (LIBOHIBOARD_K64F12)   ||\
	defined (LIBOHIBOARD_FRDMK64F) ||\
	defined (LIBOHIBOARD_K12D5)

	DMA_CHANNEL_4     =4,
	DMA_CHANNEL_5     =5,
	DMA_CHANNEL_6     =6,
	DMA_CHANNEL_7     =7,
	DMA_CHANNEL_8     =8,
	DMA_CHANNEL_9     =9,
	DMA_CHANNEL_10    =10,
	DMA_CHANNEL_11    =11,
	DMA_CHANNEL_12    =12,
	DMA_CHANNEL_13    =13,
	DMA_CHANNEL_14    =14,
	DMA_CHANNEL_15    =15,
#endif
}Dma_ChannelType;

typedef enum
{
    DMA_8BIT    =0x00,
    DMA_16BIT   =0x01,
    DMA_32BIT   =0x02,
    DMA_16BYTE  =0x04,
    DMA_32BYTE  =0x05,

}Dma_DataSizeType;

typedef enum
{
	MODE_STEAL_CYCLE = 0x1,
	MODE_CONTINUOS   = 0x0,

} Dma_TransferModeType;

typedef enum
{
	DMA_LINK_CH_0 = 0x0,
	DMA_LINK_CH_1 = 0x1,
	DMA_LINK_CH_2 = 0x2,
	DMA_LINK_CH_3 = 0x3

}Dma_LinkCannel;




typedef struct dma_ConfigType
{

    Dma_ChannelType channel;

//    Dma_RequestSourceType requestSource;
	uint32_t sourceAddress;
	uint32_t destinationAddress;

#if defined(LIBOHIBOARD_K64F12)||\
	defined(LIBOHIBOARD_FRDMK64F)||\
	defined(LIBOHIBOARD_K12D5)

	/* Source and destination for minor cycle */
	uint8_t  sourceOff;
	uint8_t  destinationOff;

	/* Last source adjustment */
	long int  lsAdjust;

	/* Last destination adjustment */
	long int  ldAdjust;

	/* This is for trigger by pit */
	uint8_t enableTimerTrig;

#elif defined(LIBOHIBOARD_KL25Z4)

	/* Apply destination or source offset */

	bool incrementSource;
	bool incrementDestination;
	bool enableAA;
	uint8_t sourceModulo;
	uint8_t destinationModulo;
	Dma_LinkCannel linkCh1;
	Dma_LinkCannel linkCh2;

#endif

	/* Source and destination data size */
    Dma_DataSizeType sSize;
    Dma_DataSizeType dSize;

    /* Number of byte for request */
    uint32_t nByteforReq;

    /* Counter of major iteration count */
    uint32_t nOfCycle;

    /* Transfer mode */
    Dma_TransferModeType transferMode;



    /* Disable channel after transfer complete */
    bool disableAfterComplete;




    /*Handler of the peripheral that trigger the DMA */
    void *pHandler;

}dma_ConfigType;




typedef struct Dma_Device* Dma_DeviceHandle;

extern Dma_DeviceHandle OB_DMA0;

/*************************************************************************************************************
 *                                                                                                           *
 *                                 This function initialize a DMA Channel                                     *
 *                                                                                                           *
 *************************************************************************************************************/

System_Errors  Dma_init(Dma_DeviceHandle dev, void* pHandler, Dma_RequestSourceType request, Dma_ChannelType channel, void *callback);

/*************************************************************************************************************
 *                                                                                                           *
 *                                 This function disable a DMA Channel                                     *
 *                                                                                                           *
 *************************************************************************************************************/

void Dma_disableChannel(Dma_DeviceHandle dev, Dma_ChannelType channel);

/*************************************************************************************************************
 *                                                                                                           *
 *                            This function start pre-configured a DMA Channel                                     *
 *                                                                                                           *
 *************************************************************************************************************/

void Dma_startChannel(Dma_DeviceHandle dev, dma_ConfigType* config);


#endif

#endif /* define __DMA_H */

#endif/* LIBOHIBOARD_DMA */
