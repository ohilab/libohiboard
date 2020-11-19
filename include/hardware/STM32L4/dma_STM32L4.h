/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/STM32L4/dma_STM32L4.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief DMA useful definitions for STM32L4 series
 */

#ifndef __DMA_STM32L4_H
#define __DMA_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_DMA) & defined(LIBOHIBOARD_STM32L4)

/**
 * @addtogroup DMA
 * @{
 */

/**
 * @defgroup DMA_Hardware DMA specific hardware types
 * @{
 */

/**
 * List of possible DMA channel.
 */
typedef enum _Dma_Channels
{
    DMA_CHANNELS_CH1 = (0x00000000u),
    DMA_CHANNELS_CH2,
    DMA_CHANNELS_CH3,
    DMA_CHANNELS_CH4,
    DMA_CHANNELS_CH5,
    DMA_CHANNELS_CH6,
    DMA_CHANNELS_CH7,

} Dma_Channels;

/**
 * List of possible DMA request for specific channel.
 */
typedef enum _Dma_RequestChannels
{
    DMA_REQUESTCHANNELS_CH0 = (0x00000000u),
    DMA_REQUESTCHANNELS_CH1,
    DMA_REQUESTCHANNELS_CH2,
    DMA_REQUESTCHANNELS_CH3,
    DMA_REQUESTCHANNELS_CH4,
    DMA_REQUESTCHANNELS_CH5,
    DMA_REQUESTCHANNELS_CH6,
    DMA_REQUESTCHANNELS_CH7,

} Dma_RequestChannels;


/**
 * DMA Device Handle Number 1.
 */
extern Dma_DeviceHandle OB_DMA1;

/**
 * DMA Device Handle Number 2.
 */
extern Dma_DeviceHandle OB_DMA2;

void DMA1_Channel1_IRQHandler (void);
void DMA1_Channel2_IRQHandler (void);
void DMA1_Channel3_IRQHandler (void);
void DMA1_Channel4_IRQHandler (void);
void DMA1_Channel5_IRQHandler (void);
void DMA1_Channel6_IRQHandler (void);
void DMA1_Channel7_IRQHandler (void);
void DMA2_Channel1_IRQHandler (void);
void DMA2_Channel2_IRQHandler (void);
void DMA2_Channel3_IRQHandler (void);
void DMA2_Channel4_IRQHandler (void);
void DMA2_Channel5_IRQHandler (void);
void DMA2_Channel6_IRQHandler (void);
void DMA2_Channel7_IRQHandler (void);


/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_DAC & LIBOHIBOARD_STM32L4

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __DAC_STM32L4_H
