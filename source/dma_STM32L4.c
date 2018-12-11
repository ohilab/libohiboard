/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/dma_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief DMA implementations for STM32L4 series.
 */

#ifdef LIBOHIBOARD_DMA

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L4)

#include "dma.h"

#include "utility.h"
#include "gpio.h"
#include "clock.h"
#include "interrupt.h"

#define DMA_CHANNEL_NUMBERS     (7u)

/**
 * Check whether the transfer direction is valid or not.
 */
#define DMA_VALID_DIRECTION(DIRECTION) (((DIRECTION) == DMA_DIRECTION_PERIPHERAL_TO_MEMORY) || \
                                        ((DIRECTION) == DMA_DIRECTION_MEMORY_TO_PERIPHERAL) || \
                                        ((DIRECTION) == DMA_DIRECTION_MEMORY_TO_MEMORY))

/**
 * Check whether the memory address increment type is valid or not.
 */
#define DMA_VALID_MEMORY_INCREMENT(INCREMENT) (((INCREMENT) == UTILITY_STATE_DISABLE) || \
                                               ((INCREMENT) == UTILITY_STATE_ENABLE))

/**
 * Check whether the peripheral address increment type is valid or not.
 */
#define DMA_VALID_PERIPHERAL_INCREMENT(INCREMENT) (((INCREMENT) == UTILITY_STATE_DISABLE) || \
                                                   ((INCREMENT) == UTILITY_STATE_ENABLE))

/**
 * Check whether the channel request is valid or not.
 */
#define DMA_VALID_CHANNEL_REQUEST(REQUEST) (((REQUEST) == DMA_REQUESTCHANNELS_CH0) || \
                                            ((REQUEST) == DMA_REQUESTCHANNELS_CH1) || \
                                            ((REQUEST) == DMA_REQUESTCHANNELS_CH2) || \
                                            ((REQUEST) == DMA_REQUESTCHANNELS_CH3) || \
                                            ((REQUEST) == DMA_REQUESTCHANNELS_CH4) || \
                                            ((REQUEST) == DMA_REQUESTCHANNELS_CH5) || \
                                            ((REQUEST) == DMA_REQUESTCHANNELS_CH6) || \
                                            ((REQUEST) == DMA_REQUESTCHANNELS_CH7))
/**
 * Check whether the mmeory data size is valid or not.
 */
#define DMA_VALID_MEMORY_DATA_SIZE(SIZE) (((SIZE) == DMA_MEMORYDATASIZE_BYTE)     || \
                                          ((SIZE) == DMA_MEMORYDATASIZE_HALFWORD) || \
                                          ((SIZE) == DMA_MEMORYDATASIZE_WORD ))

/**
 * Check whether the peripheral data size is valid or not.
 */
#define DMA_VALID_PERIPHERAL_DATA_SIZE(SIZE) (((SIZE) == DMA_PERIPHERALDATASIZE_BYTE)     || \
                                              ((SIZE) == DMA_PERIPHERALDATASIZE_HALFWORD) || \
                                              ((SIZE) == DMA_PERIPHERALDATASIZE_WORD ))

/**
 * DMA peripheral representation.
 */
typedef struct _Dma_Device
{
    DMA_TypeDef* regmap;                           /**< Device memory pointer */

    volatile uint32_t* rccRegisterPtr;       /**< Register for clock enabling */
    uint32_t rccRegisterEnable;         /**< Register mask for current device */

    DMA_Channel_TypeDef* channelRegisterPtr[DMA_CHANNEL_NUMBERS];

    Dma_Config channelConfig[DMA_CHANNEL_NUMBERS];

    Interrupt_Vector isrNumber[DMA_CHANNEL_NUMBERS];   /**< ISR vector number */

} Dma_Device;

#define DMA_IS_DEVICE(DEVICE) (((DEVICE) == OB_DMA1) || \
                               ((DEVICE) == OB_DMA2))

static Dma_Device dma1 =
{
        .regmap              = DMA1,

        .rccRegisterPtr      = &RCC->AHB1ENR,
        .rccRegisterEnable   = RCC_AHB1ENR_DMA1EN,

        .channelRegisterPtr  =
        {
                               DMA1_Channel1,
                               DMA1_Channel2,
                               DMA1_Channel3,
                               DMA1_Channel4,
                               DMA1_Channel5,
                               DMA1_Channel6,
                               DMA1_Channel7,
        },

        .isrNumber  =
        {
                               INTERRUPT_DMA1_CH1,
                               INTERRUPT_DMA1_CH2,
                               INTERRUPT_DMA1_CH3,
                               INTERRUPT_DMA1_CH4,
                               INTERRUPT_DMA1_CH5,
                               INTERRUPT_DMA1_CH6,
                               INTERRUPT_DMA1_CH7,
        },
};
Dma_DeviceHandle OB_DMA1 = &dma1;

static Dma_Device dma2 =
{
        .regmap              = DMA2,

        .rccRegisterPtr      = &RCC->AHB1ENR,
        .rccRegisterEnable   = RCC_AHB1ENR_DMA2EN,

        .channelRegisterPtr  =
        {
                               DMA2_Channel1,
                               DMA2_Channel2,
                               DMA2_Channel3,
                               DMA2_Channel4,
                               DMA2_Channel5,
                               DMA2_Channel6,
                               DMA2_Channel7,
        },

        .isrNumber  =
        {
                               INTERRUPT_DMA2_CH1,
                               INTERRUPT_DMA2_CH2,
                               INTERRUPT_DMA2_CH3,
                               INTERRUPT_DMA2_CH4,
                               INTERRUPT_DMA2_CH5,
                               INTERRUPT_DMA2_CH6,
                               INTERRUPT_DMA2_CH7,
        },
};
Dma_DeviceHandle OB_DMA2 = &dma2;

/**
 * This function set the transfer parameters for the selected channel.
 *
 *
 */
static System_Errors Dma_config (Dma_DeviceHandle dev,
                                 Dma_Channels channel,
                                 uint32_t sourceAddress,
                                 uint32_t destinationAddress,
                                 uint32_t length)
{
    // Clear global interrupt flag of selected channel
    dev->regmap->IFCR = (DMA_ISR_GIF1 << (channel << 2));

    // Save data length
    dev->channelRegisterPtr[channel]->CNDTR = length;

    // Check direction
    if (dev->channelConfig[channel].direction == DMA_DIRECTION_MEMORY_TO_PERIPHERAL)
    {
        dev->channelRegisterPtr[channel]->CMAR = sourceAddress;
        dev->channelRegisterPtr[channel]->CPAR = destinationAddress;
    }
    else
    {
        dev->channelRegisterPtr[channel]->CPAR = sourceAddress;
        dev->channelRegisterPtr[channel]->CMAR = destinationAddress;
    }
}

System_Errors Dma_init (Dma_DeviceHandle dev,
                        Dma_Channels channel,
                        Dma_Config config)
{
    // Check the DMA device
    if (dev == NULL)
    {
        return ERRORS_DMA_NO_DEVICE;
    }
    // Check the DMA instance
    if (ohiassert(DMA_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_DMA_WRONG_DEVICE;
    }

    ohiassert(DMA_VALID_CHANNEL_REQUEST(config->request));
    ohiassert(DMA_VALID_MEMORY_INCREMENT(config->mIncrement));
    ohiassert(DMA_VALID_PERIPHERAL_INCREMENT(config->pIncrement));

    dev->channelConfig[channel] = *config;
}

System_Errors Dma_deInit (Dma_DeviceHandle dev,
                          Dma_Channels channel)
{
    // Check the DMA device
    if (dev == NULL)
    {
        return ERRORS_DMA_NO_DEVICE;
    }
    // Check the DMA instance
    if (ohiassert(DMA_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_DMA_WRONG_DEVICE;
    }
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_DMA
