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

#define DMA_CLOCK_ENABLE(REG,MASK)  do { \
                                      UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                      asm("nop"); \
                                      (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                    } while (0)

#define DMA_CLOCK_DISABLE(REG,MASK)  do { \
                                       UTILITY_CLEAR_REGISTER_BIT(REG,MASK); \
                                       asm("nop"); \
                                       (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                     } while (0)

/**
 * Enable selected channel of peripheral
 */
#define DMA_DEVICE_ENABLE(CHANNEL) ((CHANNEL)->CCR |= DMA_CCR_EN)

/**
 * Disable selected channel of peripheral
 */
#define DMA_DEVICE_DISABLE(CHANNEL) ((CHANNEL)->CCR &= ~DMA_CCR_EN)

/**
 * Check whether the current channel is valid or not.
 */
#define DMA_VALID_CHANNEL(CHANNEL) (((CHANNEL) == DMA_CHANNELS_CH1) || \
                                    ((CHANNEL) == DMA_CHANNELS_CH2) || \
                                    ((CHANNEL) == DMA_CHANNELS_CH3) || \
                                    ((CHANNEL) == DMA_CHANNELS_CH4) || \
                                    ((CHANNEL) == DMA_CHANNELS_CH5) || \
                                    ((CHANNEL) == DMA_CHANNELS_CH6) || \
                                    ((CHANNEL) == DMA_CHANNELS_CH7))

/**
 * Check whether the transfer mode is valid or not.
 */
#define DMA_VALID_MODE(MODE) (((MODE) == DMA_MODE_NORMAL) || \
                              ((MODE) == DMA_MODE_CIRCULAR))

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
 * Check whether the priority is valid or not.
 */
#define DMA_VALID_PRIORITY(PRIORITY) (((PRIORITY) == DMA_PRIORITY_LOW)    || \
                                      ((PRIORITY) == DMA_PRIORITY_MEDIUM) || \
                                      ((PRIORITY) == DMA_PRIORITY_HIGH)   || \
                                      ((PRIORITY) == DMA_PRIORITY_VERY_HIGH ))

/**
 *
 */
#define DMA_VALID_BUFFER_SIZE(SIZE) (((SIZE) >= 0x1U) && ((SIZE) < 0x10000U))

/**
 * DMA peripheral representation.
 */
typedef struct _Dma_Device
{
    DMA_TypeDef* regmap;                           /**< Device memory pointer */
    DMA_Request_TypeDef* regmapCSELR;

    volatile uint32_t* rccRegisterPtr;       /**< Register for clock enabling */
    uint32_t rccRegisterEnable;         /**< Register mask for current device */

    DMA_Channel_TypeDef* channelRegisterPtr[DMA_CHANNEL_NUMBERS];

    Dma_Config channelConfig[DMA_CHANNEL_NUMBERS];

    Interrupt_Vector isrNumber[DMA_CHANNEL_NUMBERS];   /**< ISR vector number */

    Dma_DeviceState state;                      /**< Current peripheral state */

} Dma_Device;

#define DMA_IS_DEVICE(DEVICE) (((DEVICE) == OB_DMA1) || \
                               ((DEVICE) == OB_DMA2))

static Dma_Device dma1 =
{
        .regmap              = DMA1,
        .regmapCSELR         = DMA1_CSELR,

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

        .state               = DMA_DEVICESTATE_RESET,
};
Dma_DeviceHandle OB_DMA1 = &dma1;

static Dma_Device dma2 =
{
        .regmap              = DMA2,
        .regmapCSELR         = DMA2_CSELR,

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

        .state               = DMA_DEVICESTATE_RESET,
};
Dma_DeviceHandle OB_DMA2 = &dma2;

/**
 * This function set the transfer parameters for the selected channel.
 *
 *
 */
static void Dma_config (Dma_DeviceHandle dev,
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

static inline void __attribute__((always_inline)) Dma_callbackInterrupt (Dma_DeviceHandle dev,
                                                                         Dma_Channels channel)
{
    uint32_t channelIndex = ((channel << 2) & 0x0000001Cu);
    uint32_t isr = dev->regmap->ISR;
    uint32_t isrEnabled = dev->channelRegisterPtr[channel]->CCR;

    // Transfer complete interrupt
    if (((isr & (DMA_ISR_TCIF1_Msk << channelIndex)) != 0u) &&
         ((isrEnabled & DMA_CCR_TCIE) != 0u))
    {

    }
    // TODO
}

System_Errors Dma_init (Dma_DeviceHandle dev,
                        Dma_Channels channel,
                        Dma_Config* config)
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

    ohiassert(DMA_VALID_CHANNEL(channel));
    ohiassert(DMA_VALID_CHANNEL_REQUEST(config->request));
    ohiassert(DMA_VALID_MEMORY_INCREMENT(config->mIncrement));
    ohiassert(DMA_VALID_PERIPHERAL_INCREMENT(config->pIncrement));
    ohiassert(DMA_VALID_DIRECTION(config->direction));
    ohiassert(DMA_VALID_MEMORY_DATA_SIZE(config->mSize));
    ohiassert(DMA_VALID_PERIPHERAL_DATA_SIZE(config->pSize));
    ohiassert(DMA_VALID_PRIORITY(config->priority));
    ohiassert(DMA_VALID_MODE(config->mode));
    // Save channel configuration
    dev->channelConfig[channel] = *config;

    // Enable peripheral clock
    if (dev->state == DMA_DEVICESTATE_RESET)
    {
        DMA_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);
    }

    // Set channel configuration
    uint32_t tmpreg = dev->channelRegisterPtr[channel]->CCR;
    tmpreg &= (~(DMA_CCR_PL_Msk    | DMA_CCR_MSIZE_Msk  | DMA_CCR_PSIZE_Msk  |
                 DMA_CCR_MINC_Msk  | DMA_CCR_PINC_Msk   | DMA_CCR_CIRC_Msk   |
                 DMA_CCR_DIR_Msk   | DMA_CCR_MEM2MEM_Msk));
    tmpreg |= config->direction |
              config->priority  |
              config->mSize     |
              config->pSize     |
              config->mode      |
              ((config->mIncrement == UTILITY_STATE_ENABLE) ? DMA_CCR_MINC : 0u) |
              ((config->pIncrement == UTILITY_STATE_ENABLE) ? DMA_CCR_PINC : 0u);
    dev->channelRegisterPtr[channel]->CCR = tmpreg;

    // In case of peripheral-to-memory or memory-to-peripheral, request selection
    // must be set!
    if (config->direction != DMA_DIRECTION_MEMORY_TO_MEMORY)
    {
        dev->regmapCSELR->CSELR &= (~(DMA_CSELR_C1S << (channel << 2)));
        dev->regmapCSELR->CSELR |= (config->request << (channel << 2));
    }

    // Check callback and enable interrupts
    if ((config->transferCompleteCallback != 0) ||
        (config->transferAbortCallback != 0)    ||
        (config->transferErrorCallback != 0))
    {
        // Enable interrupt
        Interrupt_enable(dev->isrNumber[channel]);
    }

    dev->state = DMA_DEVICESTATE_READY;

    return ERRORS_NO_ERROR;
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

    dev->state = DMA_DEVICESTATE_RESET;
    return ERRORS_NO_ERROR;
}

System_Errors Dma_start (Dma_DeviceHandle dev,
                         Dma_Channels channel,
                         uint32_t sourceAddress,
                         uint32_t destinationAddress,
                         uint32_t length)
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

    ohiassert(DMA_VALID_CHANNEL(channel));

    if (dev->state == DMA_DEVICESTATE_READY)
    {
        dev->state = DMA_DEVICESTATE_BUSY;
        DMA_DEVICE_DISABLE(dev->channelRegisterPtr[channel]);

        // Configure the transfer...
        Dma_config(dev,channel,sourceAddress,destinationAddress,length);

        // Check callback and enable interrupts
        uint32_t tmpIsr = 0;
        if (dev->channelConfig[channel].transferCompleteCallback != 0)
        {
            tmpIsr |= DMA_CCR_TCIE;
        }
        if (dev->channelConfig[channel].transferErrorCallback != 0)
        {
            tmpIsr |= DMA_CCR_TEIE;
        }
        dev->channelRegisterPtr[channel]->CCR |= tmpIsr;

        // Enable the channel
        DMA_DEVICE_ENABLE(dev->channelRegisterPtr[channel]);
    }
    else
    {
        return ERRORS_DMA_BUSY;
    }
    return ERRORS_NO_ERROR;
}

System_Errors Dma_stop (Dma_DeviceHandle dev,
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

    ohiassert(DMA_VALID_CHANNEL(channel));

    // Disable interrupts (all)
    dev->channelRegisterPtr[channel]->CCR &= (~(DMA_CCR_TCIE  | DMA_CCR_HTIE | DMA_CCR_TEIE));

    // Disable channel
    DMA_DEVICE_DISABLE(dev->channelRegisterPtr[channel]);

    // Clear global interrupt flag
    dev->regmap->IFCR = (DMA_ISR_GIF1 << (channel << 2));

    dev->state = DMA_DEVICESTATE_READY;

    // In case callback was present, call it!
    if (dev->channelConfig[channel].transferAbortCallback != 0)
    {
        dev->channelConfig[channel].transferAbortCallback(dev,channel);
    }

    return ERRORS_NO_ERROR;
}

_weak void DMA1_Channel1_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA1,DMA_CHANNELS_CH1);
}

_weak void DMA1_Channel2_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA1,DMA_CHANNELS_CH2);
}

_weak void DMA1_Channel3_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA1,DMA_CHANNELS_CH3);
}

_weak void DMA1_Channel4_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA1,DMA_CHANNELS_CH4);
}

_weak void DMA1_Channel5_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA1,DMA_CHANNELS_CH5);
}

_weak void DMA1_Channel6_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA1,DMA_CHANNELS_CH6);
}

_weak void DMA1_Channel7_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA1,DMA_CHANNELS_CH7);
}

_weak void DMA2_Channel1_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA2,DMA_CHANNELS_CH1);
}

_weak void DMA2_Channel2_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA2,DMA_CHANNELS_CH2);
}

_weak void DMA2_Channel3_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA2,DMA_CHANNELS_CH3);
}

_weak void DMA2_Channel4_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA2,DMA_CHANNELS_CH4);
}

_weak void DMA2_Channel5_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA2,DMA_CHANNELS_CH5);
}

_weak void DMA2_Channel6_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA2,DMA_CHANNELS_CH6);
}

_weak void DMA2_Channel7_IRQHandler (void)
{
    Dma_callbackInterrupt(OB_DMA2,DMA_CHANNELS_CH7);
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_DMA
