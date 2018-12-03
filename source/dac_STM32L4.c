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

#define DAC_MAX_PINS                     2

///**
// * Useful mask to detect the current abilitation status of peripheral.
// */
//#define ADC_DEVICE_IS_ENABLE(DEVICE) (((DEVICE->regmap->CR & ADC_CR_ADEN_Msk) == ADC_CR_ADEN_Msk) &&   \
//		                              ((DEVICE->regmap->ISR & ADC_ISR_ADRDY_Msk) == ADC_ISR_ADRDY_Msk))
//
///**
// * Enable selected peripheral
// */
//#define ADC_DEVICE_ENABLE(DEVICE)     \
//    UTILITY_MODIFY_REGISTER(dev->regmap->CR,ADC_DEVICE_ENABLE_MASK,ADC_CR_ADEN)
//
///**
// * Disable selected peripheral
// */
//#define ADC_DEVICE_DISABLE(DEVICE)     \
//    UTILITY_MODIFY_REGISTER(dev->regmap->CR,ADC_DEVICE_ENABLE_MASK,ADC_CR_ADDIS)

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

    Dac_ChannelConfig chConfig[DAC_MAX_PINS];     /**< Channel configurations */

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

    // If sample-and-hold is active, check other params
    if (config->sampleAndHold == UTILITY_STATE_ENABLE)
    {
        // TODO
    }

    // Select relative channel
    Dac_Channels channel;
    for (uint16_t i = 0; i < ADC_MAX_PINS; ++i)
    {
        if (dev->pins[i] == pin)
        {
//            Gpio_configAlternate(dev->pinsGpio[i],
//                                 GPIO_ALTERNATE_ANALOG,
//                                 GPIO_PINS_ADC_CONNECTED);
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
    tmpreg &= (~(((uint32_t)(DAC_MCR_MODE1)) << channel));
    tmpreg |= (((config->sampleAndHold == UTILITY_STATE_ENABLE) ? DAC_MCR_MODE1_2 : 0x00000000u) << channel);
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_ADC
