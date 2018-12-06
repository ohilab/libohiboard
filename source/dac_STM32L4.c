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
 * @file libohiboard/source/dac_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief DAC implementations for STM32L4 series.
 */

#ifdef LIBOHIBOARD_DAC

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L4)

#include "dac.h"

#include "utility.h"
#include "gpio.h"
#include "clock.h"
#include "interrupt.h"

#define DAC_CLOCK_ENABLE(REG,MASK) do {                                         \
                                     UTILITY_SET_REGISTER_BIT(REG,MASK);        \
                                     asm("nop");                                \
                                     (void) UTILITY_READ_REGISTER_BIT(REG,MASK);\
                                   } while (0)

#define DAC_CLOCK_DISABLE(REG,MASK) do {                                         \
                                      UTILITY_CLEAR_REGISTER_BIT(REG,MASK);      \
                                      asm("nop");                                \
                                      (void) UTILITY_READ_REGISTER_BIT(REG,MASK);\
                                    } while (0)

/**
 * Enable selected channel of peripheral
 */
#define DAC_DEVICE_ENABLE(DEVICE,CHANNEL)     \
    UTILITY_SET_REGISTER_BIT(DEVICE->regmap->CR,(DAC_CR_EN1 << ((CHANNEL) & DAC_CHANNEL_SHIFT_MASK)))

/**
 * Disable selected channel of peripheral
 */
#define DAC_DEVICE_DISABLE(DEVICE,CHANNEL)     \
    UTILITY_CLEAR_REGISTER_BIT(DEVICE->regmap->CR,(DAC_CR_EN1 << ((CHANNEL) & DAC_CHANNEL_SHIFT_MASK)))


#define DAC_MAX_PINS                     2

/**
 * Check whether the sample-and-hold mode is valid or not.
 */
#define DAC_VALID_SAMPLE_AND_HOLD(SAH) (((SAH) == UTILITY_STATE_DISABLE) || \
                                        ((SAH) == UTILITY_STATE_ENABLE))

/**
 * Check whether the output buffer condition is valid or not.
 */
#define DAC_VALID_OUTPUT_BUFFER(OUTPUT) (((OUTPUT) == UTILITY_STATE_DISABLE) || \
                                         ((OUTPUT) == UTILITY_STATE_ENABLE))

/**
 * Check whether the internal connection to peripheral condition is valid or not.
 */
#define DAC_VALID_INTERNAL_CONNECTION(CONNECTION) (((CONNECTION) == UTILITY_STATE_DISABLE) || \
                                                   ((CONNECTION) == UTILITY_STATE_ENABLE))

/**
 * Check whether the data align type is valid or not.
 */
#define DAC_VALID_DATA_ALIGN(ALIGN) (((ALIGN) == DAC_DATAALIGN_12BIT_RIGHT) || \
                                     ((ALIGN) == DAC_DATAALIGN_12BIT_LEFT)  || \
                                     ((ALIGN) == DAC_DATAALIGN_8BIT_RIGHT))

/**
 *
 */
typedef struct _Dac_Device
{
    DAC_TypeDef* regmap;                           /**< Device memory pointer */

    volatile uint32_t* rccRegisterPtr;       /**< Register for clock enabling */
    uint32_t rccRegisterEnable;         /**< Register mask for current device */

    volatile uint32_t* rccTypeRegisterPtr;   /**< Register for clock enabling */
    uint32_t rccTypeRegisterMask;       /**< Register mask for user selection */
    uint32_t rccTypeRegisterPos;        /**< Mask position for user selection */

    Dac_Pins pins[DAC_MAX_PINS];      /**< List of the pin for the peripheral */
    Dac_Channels pinsChannel[DAC_MAX_PINS];
    Gpio_Pins pinsGpio[DAC_MAX_PINS];

    Interrupt_Vector isrNumber;                        /**< ISR vector number */

    Dac_DeviceState state;                      /**< Current peripheral state */

    Dac_Config config;                                /**< User configuration */

    /** Channel configurations */
    Dac_ChannelConfig channelConfig[DAC_MAX_PINS];

} Dac_Device;

#if defined (LIBOHIBOARD_STM32L476)

#define DAC_IS_DEVICE(DEVICE) ((DEVICE) == OB_DAC1)

static Dac_Device dac1 =
{
        .regmap              = DAC1,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_DAC1EN,

//        .rccTypeRegisterPtr  = &RCC->CCIPR,
//        .rccTypeRegisterMask = RCC_CCIPR_ADCSEL,
//        .rccTypeRegisterPos  = RCC_CCIPR_ADCSEL_Pos,

        .pins                =
        {
                               DAC_PINS_PA4,
                               DAC_PINS_PA5,
        },
        .pinsChannel         =
        {
                               DAC_CHANNELS_CH1,
                               DAC_CHANNELS_CH2,
        },
        .pinsGpio            =
        {
                               GPIO_PINS_PA4,
                               GPIO_PINS_PA5,
        },

//        .isrNumber           = INTERRUPT_ADC1_2,
//
//        .state               = ADC_DEVICESTATE_RESET,
};
Dac_DeviceHandle OB_DAC1 = &dac1;

#endif

/**
 *
 */
static inline void __attribute__((always_inline)) Dac_setDataAligned (Dac_DeviceHandle dev,
                                                                      Dac_Channels channel,
                                                                      Dac_DataAlign align,
                                                                      uint32_t data)
{
    // Save offset to get correct register
    // The startup position is DAC_DHR12R1
    uint32_t tmp = 0x00000008;
    switch (channel)
    {
    case DAC_CHANNELS_CH1:
        tmp += 0x00000000;
        break;
    case DAC_CHANNELS_CH2:
        tmp += 0x00000006;
        break;
    }
    // Add value of align parameter to found the correct register
    tmp += (uint32_t)align;

    // Get register
    volatile uint32_t* dhrReg = (volatile uint32_t*)((uint32_t)((uint32_t)(&dev->regmap->CR) + tmp));
    *dhrReg &= data;
}

System_Errors Dac_init (Dac_DeviceHandle dev, Dac_Config* config)
{
    // Check the DAC device
    if (dev == NULL)
    {
        return ERRORS_DAC_NO_DEVICE;
    }
    // Check the DAC instance
    if (ohiassert(DAC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_DAC_WRONG_DEVICE;
    }

    // Enable peripheral clock if needed
    if (dev->state == DAC_DEVICESTATE_RESET)
    {
        // Enable peripheral clock
        DAC_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);
    }

    dev->state = DAC_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Dac_deInit (Dac_DeviceHandle dev)
{
    // Check the DAC device
    if (dev == NULL)
    {
        return ERRORS_DAC_NO_DEVICE;
    }
    // Check the DAC instance
    if (ohiassert(DAC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_DAC_WRONG_DEVICE;
    }

    DAC_CLOCK_DISABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

    dev->state = DAC_DEVICESTATE_RESET;
    return ERRORS_NO_ERROR;
}

System_Errors Dac_configPin (Dac_DeviceHandle dev, Dac_ChannelConfig* config, Dac_Pins pin)
{
    // Check the DAC device
    if (dev == NULL)
    {
        return ERRORS_DAC_NO_DEVICE;
    }
    // Check the DAC instance
    if (ohiassert(DAC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_DAC_WRONG_DEVICE;
    }

    ohiassert(DAC_VALID_INTERNAL_CONNECTION(config->internalConnect));
    ohiassert(DAC_VALID_OUTPUT_BUFFER(config->outputBuffer));
    ohiassert(DAC_VALID_SAMPLE_AND_HOLD(config->sampleAndHold));
    ohiassert(DAC_VALID_DATA_ALIGN(config->align));

    // If sample-and-hold is active, check other params
    if (config->sampleAndHold == UTILITY_STATE_ENABLE)
    {
        // TODO
    }

    // Select relative channel
    Dac_Channels channel;
    for (uint16_t i = 0; i < DAC_MAX_PINS; ++i)
    {
        if (dev->pins[i] == pin)
        {
            Gpio_configAlternate(dev->pinsGpio[i],
                                 GPIO_ALTERNATE_ANALOG,
                                 0);
            channel = dev->pinsChannel[i];
            break;
        }
    }

    dev->state = DAC_DEVICESTATE_BUSY;

    // Setup configuration for sample-and-hold mode
    if (config->sampleAndHold == UTILITY_STATE_ENABLE)
    {

    }

    // Save configuration

    uint32_t tmpreg = 0;
    // Setup channel register
    tmpreg = dev->regmap->MCR;
    tmpreg &= (~(((uint32_t)(DAC_MCR_MODE1_Msk)) << (channel & DAC_CHANNEL_SHIFT_MASK)));
    tmpreg |= (((config->sampleAndHold   == UTILITY_STATE_ENABLE) ? DAC_MCR_MODE1_2 : 0x00000000u) |
               ((config->outputBuffer    == UTILITY_STATE_ENABLE) ? DAC_MCR_MODE1_1 : 0x00000000u) |
               ((config->internalConnect == UTILITY_STATE_ENABLE) ? DAC_MCR_MODE1_0 : 0x00000000u)) << (channel & DAC_CHANNEL_SHIFT_MASK);
    dev->regmap->MCR = tmpreg;

    // Put channel in normal mode
    dev->regmap->CR &= (~(DAC_CR_CEN1_Msk << (channel & DAC_CHANNEL_SHIFT_MASK)));

    // Disable wave generation and trigger
    tmpreg = dev->regmap->CR;
    tmpreg &= (~(((uint32_t)(DAC_CR_MAMP1_Msk | DAC_CR_WAVE1_Msk | DAC_CR_TSEL1_Msk | DAC_CR_TEN1_Msk)) << (channel & DAC_CHANNEL_SHIFT_MASK)));

    // Configure Trigger: TSELx and TENx bits
    tmpreg |= ((config->trigger) << (channel & DAC_CHANNEL_SHIFT_MASK));

    // Save CR configuration
    dev->regmap->CR = tmpreg;

    // Save channel configuration
    dev->channelConfig[DAC_CHANNEL_NUMBER(channel)] = *config;

    dev->state = DAC_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Dac_start (Dac_DeviceHandle dev, Dac_Channels channel)
{
    // Check the DAC device
    if (dev == NULL)
    {
        return ERRORS_DAC_NO_DEVICE;
    }
    // Check the DAC instance
    if (ohiassert(DAC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_DAC_WRONG_DEVICE;
    }
    dev->state = DAC_DEVICESTATE_BUSY;

    // Enable peripheral and selected channel
    DAC_DEVICE_ENABLE(dev,channel);

    // Send software trigger, if needed!
    switch (channel)
    {
    case DAC_CHANNELS_CH1:
        if ((dev->regmap->CR & DAC_TRIGGER_SOFTWARE) == DAC_TRIGGER_SOFTWARE)
        {
            dev->regmap->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
        }
        break;
    case DAC_CHANNELS_CH2:
        if ((dev->regmap->CR & (DAC_TRIGGER_SOFTWARE << DAC_CHANNELS_CH2)) == (DAC_TRIGGER_SOFTWARE << DAC_CHANNELS_CH2))
        {
            dev->regmap->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;
        }
        break;
    }

    dev->state = DAC_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Dac_stop (Dac_DeviceHandle dev, Dac_Channels channel)
{
    // Check the DAC device
    if (dev == NULL)
    {
        return ERRORS_DAC_NO_DEVICE;
    }
    // Check the DAC instance
    if (ohiassert(DAC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_DAC_WRONG_DEVICE;
    }
    dev->state = DAC_DEVICESTATE_BUSY;

    // Disable peripheral and selected channel
    DAC_DEVICE_DISABLE(dev,channel);

    dev->state = DAC_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Dac_write (Dac_DeviceHandle dev, Dac_Channels channel, uint16_t value)
{
    // Check the DAC device
    if (dev == NULL)
    {
        return ERRORS_DAC_NO_DEVICE;
    }
    // Check the DAC instance
    if (ohiassert(DAC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_DAC_WRONG_DEVICE;
    }

    Dac_setDataAligned(dev,channel,dev->channelConfig[DAC_CHANNEL_NUMBER(channel)].align,value);

    return ERRORS_NO_ERROR;
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_ADC
