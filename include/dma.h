/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2016-2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Matteo Civale <matteo.civale@gmail.com>
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
 * @file libohiboard/include/dma.h
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief DMA configuration and parameter.
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_DMA

/**
 * @defgroup DMA DMA
 * @brief DMA HAL driver
 * @{
 */

#ifndef __DMA_H
#define __DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "utility.h"

/**
 * @defgroup DMA_Configuration_Params DMA configuration types
 * @{
 */

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Dma_DeviceState
{
    DMA_DEVICESTATE_RESET,
    DMA_DEVICESTATE_READY,
    DMA_DEVICESTATE_BUSY,
    DMA_DEVICESTATE_ERROR,

} Dma_DeviceState;

/**
 * Device handle for DMA peripheral.
 */
typedef struct _Dma_Device* Dma_DeviceHandle;

// Include the correct hardware definitions
#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/dma_STM32L4.h"

#endif

/**
 * The list of possible data direction into DMA channel.
 */
typedef enum _Dma_Direction
{
    DMA_DIRECTION_PERIPHERAL_TO_MEMORY = (0x00000000u),
    DMA_DIRECTION_MEMORY_TO_PERIPHERAL = (DMA_CCR_DIR),
    DMA_DIRECTION_MEMORY_TO_MEMORY     = (DMA_CCR_MEM2MEM),

} Dma_Direction;

typedef enum _Dma_Mode
{
#if defined (LIBOHIBOARD_ST_STM32)
    DMA_MODE_NORMAL   = (0x00000000u),
    DMA_MODE_CIRCULAR = (DMA_CCR_CIRC),
#endif
#if defined (LIBOHIBOARD_NXP_KINETIS)

#endif
} Dma_Mode;

/**
 * List of possible channel priority.
 */
typedef enum _Dma_Priority
{
    DMA_PRIORITY_LOW       = (0x00000000u),
    DMA_PRIORITY_MEDIUM    = (DMA_CCR_PL_0),
    DMA_PRIORITY_HIGH      = (DMA_CCR_PL_1),
    DMA_PRIORITY_VERY_HIGH = (DMA_CCR_PL),
} Dma_Priority;

/**
 * List of possible peripheral data size.
 */
typedef enum _Dma_PeripheralDataSize
{
    DMA_PERIPHERALDATASIZE_BYTE     = (0x00000000u),
    DMA_PERIPHERALDATASIZE_HALFWORD = (DMA_CCR_PSIZE_0),
    DMA_PERIPHERALDATASIZE_WORD     = (DMA_CCR_PSIZE_1),
} Dma_PeripheralDataSize;

/**
 * List of possible memory data size.
 */
typedef enum _Dma_MemoryDataSize
{
    DMA_MEMORYDATASIZE_BYTE     = (0x00000000u),
    DMA_MEMORYDATASIZE_HALFWORD = (DMA_CCR_MSIZE_0),
    DMA_MEMORYDATASIZE_WORD     = (DMA_CCR_MSIZE_1),
} Dma_MemoryDataSize;

typedef struct _Dma_Config
{
    /**
     * Specifies the transfer mode.
     */
    Dma_Mode mode;

    /**
     * Specifies the direction that will be used to transfer data.
     */
    Dma_Direction direction;

    /**
     * Specify the software priority for the channel.
     */
    Dma_Priority priority;

    /**
     * Specifies the memory data width.
     */
    Dma_MemoryDataSize mSize;

    /**
     * Specifies the peripheral data size.
     */
    Dma_PeripheralDataSize pSize;

    /**
     * Specifies whether the memory address register should be incremented or not.
     * Use @ref UTILITY_STATE_ENABLE to enable the automatic increment mode.
     */
    Utility_State mIncrement;

    /**
     * Specifies whether the peripheral address register should be incremented or not.
     * Use @ref UTILITY_STATE_ENABLE to enable the automatic increment mode.
     */
    Utility_State pIncrement;

    /**
     * Specifies the request selected for the specified channel.
     */
    Dma_RequestChannels request;

    /** Dma transfer complete callback */
    void (* transferCompleteCallback)(struct _Dma_Device *dev, Dma_Channels channel);

    /** Dma transfer abort callback */
    void (* transferAbortCallback)(struct _Dma_Device *dev, Dma_Channels channel);

    /** Dma transfer error callback */
    void (* transferErrorCallback)(struct _Dma_Device *dev, Dma_Channels channel);
} Dma_Config;

/**
 * @}
 */

/**
 * @defgroup DMA_Configuration_Functions DMA configuration functions
 * @brief Functions to initialize and de-initialize a DMA peripheral.
 * @{
 */

/**
 * This function initialize the selected Dma Channel.
 *
 * @param[in] dev Dma device handle
 * @param[in] channel The Dma channel to initialize
 * @param[in] config A pointer to configuration object
 * @return ERRORS_NO_ERROR whether the function exit without error.
 */
System_Errors Dma_init (Dma_DeviceHandle dev,
                        Dma_Channels channel,
                        Dma_Config* config);

/**
 * This function de-initialize the selected Dma channel.
 *
 * @param[in] dev Dma device handle
 * @param[in] channel The Dma channel to de-initialize
 * @return ERRORS_NO_ERROR whether the function exit without error.
 */
System_Errors Dma_deInit (Dma_DeviceHandle dev,
                          Dma_Channels channel);

/**
 * @}
 */

/**
 * @defgroup DMA_IO_Functions DMA transfer management functions
 * @brief Functions to manage transfer operations into DMA channel.
 * @{
 */

/**
 * Start the Dma transfer.
 *
 * @param[in] dev Dma device handle
 * @param[in] channel The Dma channel
 * @param[in] sourceAddress The source memory buffer address
 * @param[in] destinationAddress The destination memory buffer address
 * @param[in] length The length of data to be transferred
 * @return ERRORS_NO_ERROR whether the transfer starts without errors.
 */
System_Errors Dma_start (Dma_DeviceHandle dev,
                         Dma_Channels channel,
                         uint32_t sourceAddress,
                         uint32_t destinationAddress,
                         uint32_t length);

/**
 * Stop the Dma transfer.
 *
 * @param[in] dev Dma device handle
 * @param[in] channel The Dma channel
 * @return ERRORS_NO_ERROR whether the transfer abort without errors.
 */
System_Errors Dma_stop (Dma_DeviceHandle dev,
                        Dma_Channels channel);

/**
 * @}
 */

#if 0 // FIXME
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
 * @param[in] callbackError
 */
System_Errors  Dma_init(Dma_DeviceHandle dev,
                        void* pHandler,
                        Dma_RequestSource request,
                        Dma_Channel channel,
                        void *callback,
                        void *callbackError);

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

#endif // FIXME

#ifdef __cplusplus
}
#endif

#endif // __DMA_H

/**
 * @}
 */

#endif // LIBOHIBOARD_DMA

/**
 * @}
 */
