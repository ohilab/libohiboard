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
 * @file libohiboard/source/adc_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief ADC implementations for STM32L4 series.
 */

#ifdef LIBOHIBOARD_ADC

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L4)

#include "adc.h"

#include "utility.h"
#include "gpio.h"
#include "clock.h"

#define ADC_CLOCK_ENABLE(REG,MASK) do {                                         \
                                     UTILITY_SET_REGISTER_BIT(REG,MASK);        \
                                     asm("nop");                                \
                                     (void) UTILITY_READ_REGISTER_BIT(REG,MASK);\
                                   } while (0)

///**
// * @brief Enable the ADC peripheral
// */
//#define IIC_DEVICE_ENABLE(REGMAP)        (REGMAP->CR1 |= I2C_CR1_PE)
///**
// * @brief Disable the ADC peripheral
// */
//#define IIC_DEVICE_DISABLE(REGMAP)       (REGMAP->CR1 &= ~I2C_CR1_PE)

#define ADC_DEVICE_IS_ENABLE(DEVICE) (((DEVICE->regmap->CR & ADC_CR_ADEN) == ADC_CR_ADEN) &&   \
		                              ((DEVICE->regmap->ISR & ADC_ISR_ADRDY) == ADC_ISR_ADRDY))

#define ADC_VALID_RESOLUTION(RESOLUTION) (((RESOLUTION) == ADC_RESOLUTION_12BIT) || \
                                          ((RESOLUTION) == ADC_RESOLUTION_10BIT) || \
                                          ((RESOLUTION) == ADC_RESOLUTION_8BIT)  || \
                                          ((RESOLUTION) == ADC_RESOLUTION_6BIT))

#define ADC_VALID_CLOCK_SOURCE(SOURCE) (((SOURCE) == ADC_CLOCKSOURCE_SYSCLK)     || \
                                        ((SOURCE) == ADC_CLOCKSOURCE_PLLADC1CLK) || \
                                        ((SOURCE) == ADC_CLOCKSOURCE_PLLADC2CLK) || \
                                        ((SOURCE) == ADC_CLOCKSOURCE_HCLK))

#define ADC_VALID_PRESCALER(PRESCALER) (((PRESCALER) == ADC_PRESCALER_SYNC_DIV1)    || \
                                        ((PRESCALER) == ADC_PRESCALER_SYNC_DIV2)    || \
                                        ((PRESCALER) == ADC_PRESCALER_SYNC_DIV4)    || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV1)   || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV2)   || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV4)   || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV6)   || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV8)   || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV10)  || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV12)  || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV16)  || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV32)  || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV64)  || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV128) || \
                                        ((PRESCALER) == ADC_PRESCALER_ASYNC_DIV256))

#define ADC_VALID_DATA_ALIGN(ALIGN) (((ALIGN) == ADC_DATAALIGN_RIGHT) || \
                                     ((ALIGN) == ADC_DATAALIGN_LEFT))

#define ADC_VALID_EOC(EOC) (((EOC) == ADC_ENDOFCONVERSION_SINGLE)   || \
                            ((EOC) == ADC_ENDOFCONVERSION_SEQUENCE))

#define ADC_VALID_EXTERNAL_TRIGGER(TRIGGER) (((TRIGGER) == ADC_TRIGGER_EXT0_TIM1_CH1)    || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT1_TIM1_CH2)    || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT2_TIM1_CH3)    || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT3_TIM2_CH2)    || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT4_TIM3_TRGO)   || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT5_TIM4_CH4)    || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT6_EXTI_11)     || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT7_TIM8_TRGO)   || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT8_TIM8_TRGO2)  || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT9_TIM1_TRGO)   || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT10_TIM1_TRGO2) || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT11_TIM2_TRGO)  || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT12_TIM4_TRGO)  || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT13_TIM6_TRGO)  || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT14_TIM15_TRGO) || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT15_TIM3_CH4))

#define ADC_VALID_EXTERNAL_TRIGGER_POLARITY(POLARITY) (((POLARITY) == ADC_TRIGGERPOLARITY_DISABLE) || \
                                                       ((POLARITY) == ADC_TRIGGERPOLARITY_RISING)  || \
                                                       ((POLARITY) == ADC_TRIGGERPOLARITY_FALLING) || \
                                                       ((POLARITY) == ADC_TRIGGERPOLARITY_BOTH))

typedef struct _Adc_Device
{
    ADC_TypeDef* regmap;                           /**< Device memory pointer */
    ADC_Common_TypeDef* rcommon;            /**< Common device memory pointer */


    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    volatile uint32_t* rccTypeRegisterPtr;  /**< Register for clock enabling. */
    uint32_t rccTypeRegisterMask;      /**< Register mask for user selection. */
    uint32_t rccTypeRegisterPos;       /**< Mask position for user selection. */

    Adc_DeviceState state;                     /**< Current peripheral state. */

    Adc_Resolution resolution;                           /**< ADC resolutions */
    Adc_DataAlign dataAlign;  /**< Data align position for conversion results */

    Adc_EndOfConversion eoc;                      /**< End-Of-Conversion type */

    /**
     * Specify whether the conversion is performed in single mode or
     * continuous mode.
     */
    Utility_State continuousConversion;

    /**
     * Specify whether the conversions is performed in complete sequence or
     * discontinuous sequence.
     * When both continuous and discontinuous are enabled, the ADC behaves
     * as if continuous mode was disabled.
     */
    Utility_State discontinuousConversion;
    /**
     * the number of regular channels to be converted in discontinuous mode,
     * after receiving an external trigger.
     * The number must be between 0 and 7, that is 1 channel to 8 channels.
     */
    uint8_t discontinuousConversionNumber;

    /**
     * Select the behavior in case of overrun: data overwritten (ENABLE) or preserved (DISABLE)
     * that is the default behavior.
     */
    Utility_State overrun;
    /**
     * Select the external event source used to trigger ADC conversion start.
     */
    Adc_Trigger externalTrigger;
    /**
     * External trigger enable and polarity selection for regular channels.
     */
    Adc_TriggerPolarity externalTriggerPolarity;

} Adc_Device;

#if defined (LIBOHIBOARD_STM32L476)

/**
 * The time required to stabilize the internal voltage regulator
 * after initialization. The value is in micro-seconds.
 */
#define ADC_TIME_VOLTAGE_REGULATOR_STARTUP 20

#define ADC_IS_DEVICE(DEVICE) (((DEVICE) == OB_ADC1)   || \
                               ((DEVICE) == OB_ADC2)   || \
                               ((DEVICE) == OB_ADC3))

static Adc_Device adc1 =
{
        .regmap              = ADC1,
        .rcommon             = ADC123_COMMON,

        .rccRegisterPtr      = &RCC->AHB2ENR,
        .rccRegisterEnable   = RCC_AHB2ENR_ADCEN,

//        .rccTypeRegisterPtr  = &RCC->CCIPR,
//        .rccTypeRegisterMask = RCC_CCIPR_I2C1SEL,
//        .rccTypeRegisterPos  = RCC_CCIPR_I2C1SEL_Pos,

        .state              = ADC_DEVICESTATE_RESET,
};
Adc_DeviceHandle OB_ADC1 = &adc1;

static Adc_Device adc2 =
{
        .regmap              = ADC2,
        .rcommon             = ADC123_COMMON,

        .rccRegisterPtr      = &RCC->AHB2ENR,
        .rccRegisterEnable   = RCC_AHB2ENR_ADCEN,

//        .rccTypeRegisterPtr  = &RCC->CCIPR,
//        .rccTypeRegisterMask = RCC_CCIPR_I2C1SEL,
//        .rccTypeRegisterPos  = RCC_CCIPR_I2C1SEL_Pos,

        .state              = ADC_DEVICESTATE_RESET,
};
Adc_DeviceHandle OB_ADC2 = &adc2;

static Adc_Device adc3 =
{
        .regmap              = ADC3,
        .rcommon             = ADC123_COMMON,

        .rccRegisterPtr      = &RCC->AHB2ENR,
        .rccRegisterEnable   = RCC_AHB2ENR_ADCEN,

//        .rccTypeRegisterPtr  = &RCC->CCIPR,
//        .rccTypeRegisterMask = RCC_CCIPR_I2C1SEL,
//        .rccTypeRegisterPos  = RCC_CCIPR_I2C1SEL_Pos,

        .state              = ADC_DEVICESTATE_RESET,
};
Adc_DeviceHandle OB_ADC3 = &adc3;

#define ADC_DEVICE_IS_OTHER_ENABLE(DEVICE)                                                        \
    ( (DEVICE == OB_ADC1) ? (ADC_DEVICE_IS_ENABLE(OB_ADC2) || ADC_DEVICE_IS_ENABLE(OB_ADC3)) : (  \
        (DEVICE == OB_ADC2) ? (ADC_DEVICE_IS_ENABLE(OB_ADC1) || ADC_DEVICE_IS_ENABLE(OB_ADC3)) : (\
            (ADC_DEVICE_IS_ENABLE(OB_ADC1) || ADC_DEVICE_IS_ENABLE(OB_ADC3))                      \
        ))                                                                                        \
    )

#endif

/**
 * Delay after voltage regulator was enabled.
 * The value is in milli-second and it is used with SysTick Delay function
 */
#define ADC_DELAY_VOLTAGE_REGULATOR_STARTUP \
    (ADC_TIME_VOLTAGE_REGULATOR_STARTUP <= 1000) ? 1 : (ADC_TIME_VOLTAGE_REGULATOR_STARTUP / 1000)

/**
 * Enable internal voltage regulator for the specific peripheral.
 *
 * @param[in] dev Adc device handle
 */
static inline void __attribute__((always_inline)) Adc_enableInternalRegulator (Adc_DeviceHandle dev)
{
    // All the operation must be disabled. See RM0351, page 589
    dev->regmap->CR &= (~(ADC_CR_ADCAL_Msk | ADC_CR_JADSTP_Msk | ADC_CR_ADSTP_Msk |
                          ADC_CR_JADSTART_Msk | ADC_CR_ADSTART_Msk | ADC_CR_ADDIS_Msk | ADC_CR_ADEN_Msk));

    dev->regmap->CR |= ADC_CR_ADVREGEN;
}

static inline void __attribute__((always_inline)) Adc_setClock (Adc_DeviceHandle dev, Adc_Config* config)
{
    dev->rcommon->CCR &= (~(ADC_CCR_CKMODE_Msk | ADC_CCR_PRESC_Msk));
    // Put the user choice
    dev->rcommon->CCR |= config->prescaler;
}

System_Errors Adc_init (Adc_DeviceHandle dev, Adc_Config* config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    err = ohiassert(ADC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    err  = ohiassert(ADC_VALID_RESOLUTION(config->resolution));
//    err |= ohiassert(ADC_VALID_CLOCKSOURCE(config->source));
    err |= ohiassert(ADC_VALID_PRESCALER(config->prescaler));
    err |= ohiassert(ADC_VALID_DATA_ALIGN(config->dataAlign));
    err |= ohiassert(ADC_VALID_EOC(config->eoc));
    err |= ohiassert(ADC_VALID_EXTERNAL_TRIGGER(config->externalTrigger));
    err |= ohiassert(ADC_VALID_EXTERNAL_TRIGGER_POLARITY(config->externalTriggerPolarity));
    err |= ohiassert(UTILITY_VALID_STATE(config->continuousConversion));
    err |= ohiassert(UTILITY_VALID_STATE(config->discontinuousConversion));
    err |= ohiassert(config->discontinuousConversionNumber < 8);
    err |= ohiassert(UTILITY_VALID_STATE(config->overrun));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_PARAM;
    }

    // Enable peripheral clock if needed
    if (dev->state == ADC_DEVICESTATE_RESET)
    {
        // Enable peripheral clock
        ADC_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);
    }

    // Check internal regulator status
    if ((dev->regmap->CR & ADC_CR_ADVREGEN) == 0u)
    {
        Adc_enableInternalRegulator(dev);

        // Wait for voltage stabilization
        System_delay(ADC_DELAY_VOLTAGE_REGULATOR_STARTUP);
        // Check again if the voltage regulator was enabled,
        // Otherwise the ADC initialization fail and return an error
        if ((dev->regmap->CR & ADC_CR_ADVREGEN) == 0u)
        {
            dev->state = ADC_DEVICESTATE_ERROR;
            return ERRORS_ADC_INIT_FAIL;
        }
    }

    // Now it is possible configure the peripheral
    // WARNING: no on-going conversion...
    if ((dev->regmap->CR & ADC_CR_ADSTART) == 0u)
    {
        // Configuration of common parameters
        // WARNING: all peripheral must be disabled
        if (!ADC_DEVICE_IS_ENABLE(dev) && !ADC_DEVICE_IS_OTHER_ENABLE(dev))
        {
            Adc_setClock(dev,config);
        }

        // Configure specific peripheral

        // Set continuous conversion, data align, resolution and discontinuous conversion mode
        dev->regmap->CFGR &= (~(ADC_CFGR_CONT_Msk    |
                                ADC_CFGR_DISCEN_Msk  |
                                ADC_CFGR_DISCNUM_Msk |
                                ADC_CFGR_RES_Msk     |
                                ADC_CFGR_ALIGN_Msk   |
                                ADC_CFGR_OVRMOD_Msk));
        // Save configuration paramenters
        dev->resolution = config->resolution;
        dev->overrun = config->overrun;
        dev->dataAlign = config->dataAlign;
        dev->discontinuousConversion = config->discontinuousConversion;
        dev->discontinuousConversionNumber = config->discontinuousConversionNumber;
        dev->continuousConversion = config->continuousConversion;
        // Write user config
        dev->regmap->CFGR |= config->dataAlign  |
                             config->resolution |
                             ((config->continuousConversion == UTILITY_STATE_ENABLE) ? ADC_CFGR_CONT : 0u) |
                             ((config->overrun == UTILITY_STATE_ENABLE) ? ADC_CFGR_OVRMOD : 0u)            |
                             ((config->discontinuousConversion == UTILITY_STATE_ENABLE) ? ADC_CFGR_DISCEN : 0u);

        if (config->discontinuousConversion == UTILITY_STATE_ENABLE)
        {
            dev->regmap->CFGR |= ((config->discontinuousConversionNumber & 0x07ul) << ADC_CFGR_DISCNUM_Pos);
        }

        // Check external trigger choice
        dev->externalTrigger = config->externalTrigger;
        dev->externalTriggerPolarity = config->externalTriggerPolarity;
        // In case of external trigger enabled, update CFGR register
        if (config->externalTriggerPolarity != 0u)
        {
            // Mask register
            dev->regmap->CFGR &= (~(ADC_CFGR_EXTSEL_Msk | ADC_CFGR_EXTEN_Msk));
            // Write user config
            dev->regmap->CFGR |= config->externalTriggerPolarity | config->externalTrigger;
        }

        // FIXME: Add Oversampling, LowPowerAutoWait and DMA

    }
    else
    {
        dev->state = ADC_DEVICESTATE_ERROR;
        return ERRORS_ADC_INIT_FAIL;
    }

    dev->state = ADC_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Adc_deInit (Adc_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
//    err = ohiassert(IIC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    // TODO

    return err;
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_ADC
