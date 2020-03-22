/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2020 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/STM32L4/adc_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief ADC implementations for STM32L4 series.
 */

#ifdef LIBOHIBOARD_ADC

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L0)

#include "adc.h"

#include "utility.h"
#include "gpio.h"
#include "clock.h"
#include "interrupt.h"

#define ADC_CLOCK_ENABLE(REG,MASK) do {                                         \
                                     UTILITY_SET_REGISTER_BIT(REG,MASK);        \
                                     asm("nop");                                \
                                     (void) UTILITY_READ_REGISTER_BIT(REG,MASK);\
                                   } while (0)

#define ADC_CLOCK_DISABLE(REG,MASK) do {                                         \
                                      UTILITY_CLEAR_REGISTER_BIT(REG,MASK);      \
                                      asm("nop");                                \
                                      (void) UTILITY_READ_REGISTER_BIT(REG,MASK);\
                                    } while (0)

#define ADC_MAX_PINS                     20


#define ADC_DEVICE_ENABLE_MASK           (ADC_CR_ADCAL_Msk    | \
                                          ADC_CR_ADSTP_Msk    | \
                                          ADC_CR_ADSTART_Msk  | \
                                          ADC_CR_ADDIS_Msk    | \
                                          ADC_CR_ADEN_Msk)
/**
 * Useful mask to detect the current abilitation status of peripheral.
 */
#define ADC_DEVICE_IS_ENABLE(DEVICE) (((DEVICE->regmap->CR & ADC_CR_ADEN_Msk) == ADC_CR_ADEN_Msk) &&   \
		                              ((DEVICE->regmap->ISR & ADC_ISR_ADRDY_Msk) == ADC_ISR_ADRDY_Msk))

/**
 * Verification of hardware constraints before ADC can be disabled.
 */
#define ADC_DISABLING_CONDITIONS(DEVICE) ((DEVICE->regmap->CR & (ADC_CR_ADSTART | ADC_CR_ADEN)) == ADC_CR_ADEN)

/**
 * Enable selected peripheral
 */
#define ADC_DEVICE_ENABLE(DEVICE)     \
    UTILITY_MODIFY_REGISTER(dev->regmap->CR,ADC_DEVICE_ENABLE_MASK,ADC_CR_ADEN)

/**
 * Disable selected peripheral
 */
#define ADC_DEVICE_DISABLE(DEVICE)     \
    UTILITY_MODIFY_REGISTER(dev->regmap->CR,ADC_DEVICE_ENABLE_MASK,ADC_CR_ADDIS)

#define ADC_VALID_RESOLUTION(RESOLUTION) (((RESOLUTION) == ADC_RESOLUTION_12BIT) || \
                                          ((RESOLUTION) == ADC_RESOLUTION_10BIT) || \
                                          ((RESOLUTION) == ADC_RESOLUTION_8BIT)  || \
                                          ((RESOLUTION) == ADC_RESOLUTION_6BIT))

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

//#define ADC_VALID_EOC(EOC) (((EOC) == ADC_ENDOFCONVERSION_SINGLE)   || \
//                            ((EOC) == ADC_ENDOFCONVERSION_SEQUENCE))

#define ADC_VALID_EXTERNAL_TRIGGER(TRIGGER) (((TRIGGER) == ADC_TRIGGER_EXT0_TIM6_TRGO)   || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT1_TIM21_CH2)   || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT2_TIM2_TRGO)   || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT3_TIM2_CH4)    || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT4_TIM22_TRGO)  || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT5_TIM2_CH3)    || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT6_TIM3_TRGO)   || \
                                             ((TRIGGER) == ADC_TRIGGER_EXT7_EXTI_11))

#define ADC_VALID_EXTERNAL_TRIGGER_POLARITY(POLARITY) (((POLARITY) == ADC_TRIGGERPOLARITY_DISABLE) || \
                                                       ((POLARITY) == ADC_TRIGGERPOLARITY_RISING)  || \
                                                       ((POLARITY) == ADC_TRIGGERPOLARITY_FALLING) || \
                                                       ((POLARITY) == ADC_TRIGGERPOLARITY_BOTH))

#define ADC_VALID_SAMPLING_TIME(SAMPLING) (((SAMPLING) == ADC_SAMPLINGTIME_1_ADCCLK_5)   || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_3_ADCCLK_5)   || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_7_ADCCLK_5)   || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_12_ADCCLK_5)  || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_19_ADCCLK_5)  || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_39_ADCCLK_5)  || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_79_ADCCLK_5)  || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_160_ADCCLK_5))

#define ADC_VALID_CONVERSION_TYPE(TYPE) (((TYPE) == ADC_INPUTTYPE_SINGLE_ENDED))

#define ADC_FLAG_RDY           ADC_ISR_ADRDY    /*!< ADC Ready flag */
#define ADC_FLAG_EOSMP         ADC_ISR_EOSMP    /*!< ADC End of Sampling flag */
#define ADC_FLAG_EOC           ADC_ISR_EOC      /*!< ADC End of Regular Conversion flag */
#define ADC_FLAG_EOS           ADC_ISR_EOSEQ    /*!< ADC End of Regular sequence of Conversions flag */
#define ADC_FLAG_OVR           ADC_ISR_OVR      /*!< ADC overrun flag */
#define ADC_FLAG_AWD           ADC_ISR_AWD      /*!< ADC Analog watchdog flag */
#define ADC_FLAG_EOCAL         ADC_ISR_EOCAL    /*!< ADC Enf Of Calibration flag */

typedef struct _Adc_Device
{
    ADC_TypeDef* regmap;                           /**< Device memory pointer */
    ADC_Common_TypeDef* rcommon;            /**< Common device memory pointer */


    volatile uint32_t* rccRegisterPtr;       /**< Register for clock enabling */
    uint32_t rccRegisterEnable;         /**< Register mask for current device */

    volatile uint32_t* rccTypeRegisterPtr;   /**< Register for clock enabling */
    uint32_t rccTypeRegisterMask;       /**< Register mask for user selection */
    uint32_t rccTypeRegisterPos;        /**< Mask position for user selection */

    Adc_Pins pins[ADC_MAX_PINS];      /**< List of the pin for the peripheral */
    Adc_Channels pinsChannel[ADC_MAX_PINS];
    Gpio_Pins pinsGpio[ADC_MAX_PINS];

    void (* eocCallback)(struct _Adc_Device *dev);
    void (* eosCallback)(struct _Adc_Device *dev);
    void (* overrunCallback)(struct _Adc_Device *dev);

    Interrupt_Vector isrNumber;                        /**< ISR vector number */

    Adc_DeviceState state;                      /**< Current peripheral state */

    Adc_Config config;                                /**< User configuration */

} Adc_Device;

#if defined (LIBOHIBOARD_STM32L0x3)

#if defined (LIBOHIBOARD_STM32L073)

/**
 * The time required to stabilize the internal voltage regulator
 * after initialization. The value is in micro-seconds.
 */
#define ADC_TIME_VOLTAGE_REGULATOR_STARTUP 20

#define ADC_TIMEOUT_ENABLE                 2

/**
 * Fixed timeout value for conversion ending.
 * It is computed by the sum of max sampling time and maximum conversion time,
 * divided by the minimum ADC clock frequency.
 * The value is expressed in milli-second.
 */
#define ADC_TIMEOUT_STOP_CONVERSION        5

#define ADC_IS_DEVICE(DEVICE) (((DEVICE) == OB_ADC1))

#define ADC_IS_ENABLE(DEVICE)                                                        \
       ((((((DEVICE)->regmap->CR) & (ADC_CR_ADEN | ADC_CR_ADDIS)) == ADC_CR_ADEN) && \
         ((((DEVICE)->regmap->ISR) & ADC_FLAG_RDY) == ADC_FLAG_RDY)))

static Adc_Device adc1 =
{
        .regmap              = ADC1,
        .rcommon             = ADC1_COMMON,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_ADCEN,

        .rccTypeRegisterPtr  = 0,
        .rccTypeRegisterMask = 0,
        .rccTypeRegisterPos  = 0,

        .pins                =
        {
                               ADC_PINS_PA0,
                               ADC_PINS_PA1,
                               ADC_PINS_PA2,
                               ADC_PINS_PA3,
                               ADC_PINS_PA4,
                               ADC_PINS_PA5,
                               ADC_PINS_PA6,
                               ADC_PINS_PA7,
                               ADC_PINS_PB0,
                               ADC_PINS_PB1,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               ADC_PINS_PC0,
                               ADC_PINS_PC1,
                               ADC_PINS_PC2,
                               ADC_PINS_PC3,
                               ADC_PINS_PC4,
                               ADC_PINS_PC5,
#endif
        },
        .pinsChannel         =
        {
                               ADC_CHANNELS_CH0,
                               ADC_CHANNELS_CH1,
                               ADC_CHANNELS_CH2,
                               ADC_CHANNELS_CH3,
                               ADC_CHANNELS_CH4,
                               ADC_CHANNELS_CH5,
                               ADC_CHANNELS_CH6,
                               ADC_CHANNELS_CH7,
                               ADC_CHANNELS_CH8,
                               ADC_CHANNELS_CH9,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               ADC_CHANNELS_CH10,
                               ADC_CHANNELS_CH11,
                               ADC_CHANNELS_CH12,
                               ADC_CHANNELS_CH13,
                               ADC_CHANNELS_CH14,
                               ADC_CHANNELS_CH15,
#endif
        },
        .pinsGpio            =
        {
                               GPIO_PINS_PA0,
                               GPIO_PINS_PA1,
                               GPIO_PINS_PA2,
                               GPIO_PINS_PA3,
                               GPIO_PINS_PA4,
                               GPIO_PINS_PA5,
                               GPIO_PINS_PA6,
                               GPIO_PINS_PA7,
                               GPIO_PINS_PB0,
                               GPIO_PINS_PB1,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PC0,
                               GPIO_PINS_PC1,
                               GPIO_PINS_PC2,
                               GPIO_PINS_PC3,
                               GPIO_PINS_PC4,
                               GPIO_PINS_PC5,
#endif
        },

        .isrNumber           = INTERRUPT_ADC_COMP,

        .state               = ADC_DEVICESTATE_RESET,
};
Adc_DeviceHandle OB_ADC1 = &adc1;

#define ADC_VALID_CHANNEL(DEV,CHANNEL) (((CHANNEL) == ADC_CHANNELS_VREFINT)    || \
                                        ((CHANNEL) == ADC_CHANNELS_CH0)        || \
                                        ((CHANNEL) == ADC_CHANNELS_CH1)        || \
                                        ((CHANNEL) == ADC_CHANNELS_CH2)        || \
                                        ((CHANNEL) == ADC_CHANNELS_CH3)        || \
                                        ((CHANNEL) == ADC_CHANNELS_CH4)        || \
                                        ((CHANNEL) == ADC_CHANNELS_CH5)        || \
                                        ((CHANNEL) == ADC_CHANNELS_CH6)        || \
                                        ((CHANNEL) == ADC_CHANNELS_CH7)        || \
                                        ((CHANNEL) == ADC_CHANNELS_CH8)        || \
                                        ((CHANNEL) == ADC_CHANNELS_CH9)        || \
                                        ((CHANNEL) == ADC_CHANNELS_CH10)       || \
                                        ((CHANNEL) == ADC_CHANNELS_CH11)       || \
                                        ((CHANNEL) == ADC_CHANNELS_CH12)       || \
                                        ((CHANNEL) == ADC_CHANNELS_CH13)       || \
                                        ((CHANNEL) == ADC_CHANNELS_CH14)       || \
                                        ((CHANNEL) == ADC_CHANNELS_CH15)       || \
                                        ((CHANNEL) == ADC_CHANNELS_TEMPERATURE)|| \
                                        ((CHANNEL) == ADC_CHANNELS_VLCD))

#define ADC_VALID_TEMPERATURE_CHANNEL(DEVICE) ((DEVICE) == OB_ADC1)

#define ADC_VALID_VREFINT_CHANNEL(DEVICE) ((DEVICE) == OB_ADC1)

#endif // LIBOHIBOARD_STM32L073

#endif // LIBOHIBOARD_STM32L0x3

/**
 * Delay after voltage regulator was enabled.
 * The value is in milli-second and it is used with SysTick Delay function
 */
#define ADC_DELAY_VOLTAGE_REGULATOR_STARTUP \
    (ADC_TIME_VOLTAGE_REGULATOR_STARTUP <= 1000) ? 1 : (ADC_TIME_VOLTAGE_REGULATOR_STARTUP / 1000)

static inline void __attribute__((always_inline)) Adc_callbackInterrupt (Adc_DeviceHandle dev)
{
    // TODO
}

/**
 * Enable internal voltage regulator for the specific peripheral.
 *
 * @param[in] dev Adc device handle
 */
static inline void __attribute__((always_inline)) Adc_enableInternalRegulator (Adc_DeviceHandle dev)
{
    // All the operation must be disabled.
    dev->regmap->CR &= (~(ADC_CR_ADCAL_Msk  | ADC_CR_ADSTP_Msk | ADC_CR_ADSTART_Msk | ADC_CR_ADDIS_Msk | ADC_CR_ADEN_Msk));

    dev->regmap->CR |= ADC_CR_ADVREGEN;
}

static inline void __attribute__((always_inline)) Adc_setClock (Adc_DeviceHandle dev, Adc_Config* config)
{
    dev->rcommon->CCR &= (~(ADC_CCR_PRESC_Msk));
    dev->regmap->CFGR2 &= (~(ADC_CFGR2_CKMODE_Msk));

    // Put the user choice
    dev->rcommon->CCR |= (config->prescaler & ADC_CCR_PRESC_Msk);
    dev->regmap->CFGR2 |= (config->prescaler & ADC_CFGR2_CKMODE_Msk);
}

static inline void __attribute__((always_inline)) Adc_setSamplingTime (Adc_DeviceHandle dev,
                                                                       Adc_Channels channel,
                                                                       Adc_SamplingTime sampling)
{
    // Get register
    volatile uint32_t* smprReg = (volatile uint32_t*)((uint32_t)((uint32_t)(&dev->regmap->SMPR)));
    *smprReg &= (~(ADC_SMPR_SMP_Msk));
    // Save sampling time for selected channel
    *smprReg |= (sampling);
}

System_Errors Adc_calibration (Adc_DeviceHandle dev)
{
    System_Errors error = ERRORS_NO_ERROR;
    uint32_t tickstart = 0U;
    uint32_t backup_setting_adc_dma_transfer = 0U;

    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    /* Calibration prerequisite: ADC must be disabled. */
    if (ADC_IS_ENABLE(dev) == FALSE)
    {
        /* Set ADC state */
        dev->state = ADC_DEVICESTATE_BUSY_INTERNAL;

        /* Disable ADC DMA transfer request during calibration */
        /* Note: Specificity of this STM32 serie: Calibration factor is           */
        /*       available in data register and also transfered by DMA.           */
        /*       To not insert ADC calibration factor among ADC conversion data   */
        /*       in array variable, DMA transfer must be disabled during          */
        /*       calibration.                                                     */
        backup_setting_adc_dma_transfer = UTILITY_READ_REGISTER_BIT(dev->regmap->CFGR1, ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG);
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->CFGR1, ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG);

        /* Start ADC calibration */
        dev->regmap->CR |= ADC_CR_ADCAL;

        tickstart = System_currentTick();

        /* Wait for calibration completion */
        while (UTILITY_READ_REGISTER_BIT(dev->regmap->CR, ADC_CR_ADCAL))
        {
            if ((System_currentTick() - tickstart) > ADC_CALIBRATION_TIMEOUT)
            {
                /* Update ADC state machine to error */
                dev->state = ADC_DEVICESTATE_ERROR_INTERNAL;

                return ERRORS_ADC_CALIBRATION;
            }
        }

        /* Restore ADC DMA transfer request after calibration */
        UTILITY_SET_REGISTER_BIT(dev->regmap->CFGR1, backup_setting_adc_dma_transfer);

        /* Set ADC state */
        dev->state = ADC_DEVICESTATE_READY;
    }
    else
    {
        dev->state = ADC_DEVICESTATE_ERROR;
        error = ERRORS_ADC_IS_BUSY;
    }

    return error;
}

/**
  * Get the calibration factor.
  * @param[in] dev Adc device handle
  * @retval Calibration value.
  */
uint32_t Adc_calibrationGetValue (Adc_DeviceHandle dev)
{
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    /* Return the ADC calibration value */
    return ((dev->regmap->CALFACT) & 0x0000007FU);
}

/**
 * Enable the selected ADC peripheral.
 *
 * @param[in] dev Adc device handle
 */
static System_Errors Adc_enable (Adc_DeviceHandle dev)
{
    uint32_t tickstart = 0;

    // Check if the peripheral is just enabled
    if (ADC_DEVICE_IS_ENABLE(dev) == 0)
    {
        // Before enable the peripheral, we must check all CR register
        if ((dev->regmap->CR & ADC_DEVICE_ENABLE_MASK) != 0)
        {
            dev->state = ADC_DEVICESTATE_ERROR;
            return ERRORS_ADC_IS_BUSY;
        }

        // Enable the peripheral
        ADC_DEVICE_ENABLE(dev);

        // Wait for stabilization
        System_delay(ADC_DELAY_VOLTAGE_REGULATOR_STARTUP);

        // Wait until ADC is enabled... otherwise timeout!
        tickstart = System_currentTick();
        while ((dev->regmap->ISR & ADC_ISR_ADRDY) == 0)
        {
            if ((System_currentTick() - tickstart) > ADC_TIMEOUT_ENABLE)
            {
                dev->state = ADC_DEVICESTATE_ERROR;
                return ERRORS_ADC_TIMEOUT;
            }
        }
    }

    return ERRORS_NO_ERROR;
}

/**
 * Disable the selected ADC peripheral.
 *
 * @param[in] dev Adc device handle
 */
static System_Errors Adc_disable (Adc_DeviceHandle dev)
{
    uint32_t tickstart = 0;

    // Check if the peripheral is enabled
    if (ADC_DEVICE_IS_ENABLE(dev) != 0)
    {
        // Before disable the peripheral, we must check CR register
        // If the peripheral is enabled and no on-going conversion is present,
        // Disable the peripheral
        if (ADC_DISABLING_CONDITIONS(dev) != 0)
        {
            ADC_DEVICE_DISABLE(dev);
        }
        else
        {
            dev->state = ADC_DEVICESTATE_ERROR;
            return ERRORS_ADC_IS_BUSY;
        }

        // Wait until ADC is disabled... otherwise timeout!
        tickstart = System_currentTick();
        while ((dev->regmap->CR & ADC_CR_ADEN) != 0)
        {
            if ((System_currentTick() - tickstart) > ADC_TIMEOUT_ENABLE)
            {
                dev->state = ADC_DEVICESTATE_ERROR;
                return ERRORS_ADC_TIMEOUT;
            }
        }
    }
    return ERRORS_NO_ERROR;
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
    err |= ohiassert(ADC_VALID_PRESCALER(config->prescaler));
    err |= ohiassert(ADC_VALID_DATA_ALIGN(config->dataAlign));
    err |= ohiassert(ADC_VALID_EXTERNAL_TRIGGER(config->externalTrigger));
    err |= ohiassert(ADC_VALID_EXTERNAL_TRIGGER_POLARITY(config->externalTriggerPolarity));
    err |= ohiassert(UTILITY_VALID_STATE(config->continuousConversion));
    err |= ohiassert(UTILITY_VALID_STATE(config->discontinuousConversion));
    err |= ohiassert(config->discontinuousConversionNumber < 0x08u);
    err |= ohiassert(UTILITY_VALID_STATE(config->sequence));
    err |= ohiassert(config->sequenceNumber < 0x10u);
    err |= ohiassert(UTILITY_VALID_STATE(config->overrun));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_PARAM;
    }
    // Save configuration
    dev->config = *config;

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
        if (!ADC_DEVICE_IS_ENABLE(dev))
        {
            Adc_setClock(dev,config);
        }

        // Configure specific peripheral

        // Set continuous conversion, data align, resolution and discontinuous conversion mode
        dev->regmap->CFGR1 &= (~(ADC_CFGR1_CONT_Msk    |
                                 ADC_CFGR1_DISCEN_Msk  |
                                 ADC_CFGR1_RES_Msk     |
                                 ADC_CFGR1_ALIGN_Msk   |
                                 ADC_CFGR1_OVRMOD_Msk));
        // Write user configuration
        dev->regmap->CFGR1 |= config->dataAlign  |
                             config->resolution |
                             ((config->continuousConversion == UTILITY_STATE_ENABLE) ? ADC_CFGR1_CONT : 0u) |
                             ((config->overrun == UTILITY_STATE_ENABLE) ? ADC_CFGR1_OVRMOD : 0u)            |
                             ((config->discontinuousConversion == UTILITY_STATE_ENABLE) ? ADC_CFGR1_DISCEN : 0u);

        if (config->discontinuousConversion == UTILITY_STATE_ENABLE)
        {
            dev->regmap->CFGR1 |= ((config->discontinuousConversionNumber & 0x07ul) << ADC_CFGR1_DISCEN_Pos);
        }

        // Check external trigger choice
        // In case of external trigger enabled, update CFGR register
        // Mask register (disable trigger)
        dev->regmap->CFGR1 &= (~(ADC_CFGR1_EXTSEL_Msk | ADC_CFGR1_EXTEN_Msk));
        if (config->externalTriggerPolarity != 0u)
        {
            // Write user config
            dev->regmap->CFGR1 |= config->externalTriggerPolarity | config->externalTrigger;
        }

        // FIXME: Add Oversampling, LowPowerAutoWait and DMA

        // Check callback and enable interrupts
        if ((config->eocCallback != 0) ||
            (config->eosCallback != 0) ||
            (config->overrunCallback != 0))
        {
            // Save callback
            dev->eocCallback = config->eocCallback;
            dev->eosCallback = config->eosCallback;
            dev->overrunCallback = config->overrunCallback;
            // Enable interrupt
            Interrupt_enable(dev->isrNumber);
        }
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
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    // Stop on-going conversion...
    Adc_stop(dev);

    // TODO: disable interrupt

    // TODO: Clear interrupt flags

    // Disable peripheral clock
    ADC_CLOCK_DISABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

    dev->state = ADC_DEVICESTATE_RESET;
    return ERRORS_NO_ERROR;
}

System_Errors Adc_configPin (Adc_DeviceHandle dev, Adc_ChannelConfig* config, Adc_Pins pin)
{
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    ohiassert(ADC_VALID_SAMPLING_TIME(config->samplingTime));
    ohiassert(ADC_VALID_CONVERSION_TYPE(config->type));

    // Configure Pin
    Adc_Channels channel;
    if ((pin == ADC_PINS_INTERNAL) && (config->isInternal == TRUE))
    {
        channel = config->channel;
    }
    else
    {
        // Search channel
        bool isPinFound = FALSE;
        for (uint16_t i = 0; i < ADC_MAX_PINS; ++i)
        {
            if (dev->pins[i] == pin)
            {
                Gpio_configAlternate(dev->pinsGpio[i],
                                     GPIO_ALTERNATE_ANALOG,
                                     0);
                channel = dev->pinsChannel[i];
                isPinFound = TRUE;
                break;
            }
        }
        if (isPinFound == FALSE)
        {
            dev->state = ADC_DEVICESTATE_ERROR;
            return ERRORS_ADC_NO_PIN_FOUND;
        }
    }


    // WARNING: no on-going regular conversion...
    if ((dev->regmap->CR & ADC_CR_ADSTART) == 0u)
    {
        // WARNING: no on-going regular conversion and injected conversion...
        if (((dev->regmap->CR & ADC_CR_ADSTART) == 0u) &&
            ((dev->regmap->CR & ADC_CR_ADCAL) == 0u))
        {
            // Set channel sampling time
            Adc_setSamplingTime(dev,channel,config->samplingTime);

            // FIXME: Set channel offset
        }

        // Manage internal channel
        // Check if the selected peripheral have this channel connected, and
        // if the selected channel was just enabled
        if ((config->channel == ADC_CHANNELS_TEMPERATURE) &&
           ((dev->rcommon->CCR & ADC_CCR_TSEN_Msk) == 0u) &&
            ADC_VALID_TEMPERATURE_CHANNEL(dev))
        {
            dev->rcommon->CCR |= ADC_CCR_TSEN;
            // FIXME: Is it too much?
            System_delay(1);
        }
        else if ((config->channel == ADC_CHANNELS_VREFINT)       &&
                ((dev->rcommon->CCR & ADC_CCR_VREFEN_Msk) == 0u) &&
                 ADC_VALID_VREFINT_CHANNEL(dev))
        {
            dev->rcommon->CCR |= ADC_CCR_VREFEN;
        }
        dev->regmap->CHSELR = ((uint32_t)channel & ADC_CHANNEL_MASK);
    }
    else
    {
        return ERRORS_ADC_CONVERSION_ONGOING;
    }

    return ERRORS_NO_ERROR;
}

System_Errors Adc_start (Adc_DeviceHandle dev)
{
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    // WARNING: no on-going regular conversion...
    if ((dev->regmap->CR & ADC_CR_ADSTART) == 0u)
    {
        // Enable the peripheral
        System_Errors err = Adc_enable(dev);
        if (err != ERRORS_NO_ERROR)
        {
            return err;
        }

        // Clear flag, no unknown state form previous conversion
        dev->regmap->ISR = (ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR);

        // Disable all interrupt
        dev->regmap->IER &= (~(ADC_IER_EOCIE | ADC_IER_EOSIE | ADC_IER_OVRIE));

        // Check interrupt
        if ((dev->eocCallback != 0) && (dev->config.eoc == ADC_ENDOFCONVERSION_SINGLE))
        {
            dev->regmap->IER |= ADC_IER_EOCIE;
        }
        else if ((dev->eosCallback != 0) && (dev->config.eoc == ADC_ENDOFCONVERSION_SEQUENCE))
        {
            dev->regmap->IER |= ADC_IER_EOSIE;
        }

        // Only when data must be preserved the interrupt was lanched!
        if ((dev->overrunCallback != 0) && (dev->config.overrun == UTILITY_STATE_DISABLE))
        {
            dev->regmap->IER |= ADC_IER_OVRIE;
        }

        // Start conversion
        dev->regmap->CR |= ADC_CR_ADSTART;
    }
    else
    {
        return ERRORS_ADC_IS_BUSY;
    }

    return ERRORS_NO_ERROR;
}

System_Errors Adc_stop (Adc_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    uint32_t tickstart = 0;

    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    // Stop on-going conversion
    // Check some conversion are on-going
    if ((dev->regmap->CR & (ADC_CR_ADSTART | ADC_CR_ADDIS)) != 0u)
    {
        // Software can set ADSTP only when ADSTART=1 and ADDIS=0
        if ((dev->regmap->CR & (ADC_CR_ADSTART | ADC_CR_ADDIS)) == (ADC_CR_ADSTART | ADC_CR_ADDIS))
        {
            UTILITY_MODIFY_REGISTER(dev->regmap->CR,ADC_DEVICE_ENABLE_MASK,ADC_CR_ADSTP);
        }

        // Wait until conversion effectively stopped
        tickstart = System_currentTick();
        while ((dev->regmap->CR & (ADC_CR_ADSTART | ADC_CR_ADDIS)) != 0u)
        {
            if ((System_currentTick() - tickstart) > ADC_TIMEOUT_STOP_CONVERSION)
            {
                return ERRORS_ADC_TIMEOUT;
            }
        }
    }

    // Disable peripheral
    err = Adc_disable(dev);
    if (err != ERRORS_NO_ERROR)
    {
        return err;
    }

    // Disable all interrupt
    dev->regmap->IER &= (~(ADC_IER_EOCIE | ADC_IER_EOSIE | ADC_IER_OVRIE));

    return ERRORS_NO_ERROR;
}

uint32_t Adc_read (Adc_DeviceHandle dev)
{
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    return (dev->regmap->DR);
}

System_Errors Adc_poll (Adc_DeviceHandle dev, uint32_t timeout)
{
    uint32_t flag = 0u;
    uint32_t tickstart = 0u;

    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    if (dev->config.eoc == ADC_ENDOFCONVERSION_SINGLE)
    {
        flag = ADC_ISR_EOC;
    }
    else
    {
        flag = ADC_ISR_EOS;
    }

    // check conversion status
    tickstart = System_currentTick();
    while ((dev->regmap->ISR & flag) == 0u)
    {
        if ((System_currentTick() - tickstart) > timeout)
        {
            return ERRORS_ADC_TIMEOUT;
        }
    }

    // Single conversion or sequence conversions flag is up!
    return ERRORS_ADC_CONVERSION_DONE;
}

static inline int32_t __attribute__((always_inline)) Adc_computeTemperature (uint32_t measure, uint32_t vref)
{
	int32_t temperature = 0;
	temperature = ((measure * (((float)vref)/ADC_VREFINT_CAL)) - (int32_t) *ADC_TEMPERATURE_CAL1_ADDR );
	temperature = temperature * (int32_t)(ADC_TEMPERATURE_CAL2 - ADC_TEMPERATURE_CAL1);
	temperature = temperature / (int32_t)(*ADC_TEMPERATURE_CAL2_ADDR - *ADC_TEMPERATURE_CAL1_ADDR);
	temperature = temperature + 30;
	return temperature;
}

int32_t Adc_getTemperature (Adc_DeviceHandle dev, uint32_t data, uint32_t vref)
{
    if (dev->config.resolution != ADC_RESOLUTION_12BIT)
    {
        switch (dev->config.resolution)
        {
        case ADC_RESOLUTION_6BIT:
            data <<= 6u;
            break;
        case ADC_RESOLUTION_8BIT:
            data <<= 4u;
            break;
        case ADC_RESOLUTION_10BIT:
            data <<= 2u;
            break;
        default:
            break;
        }
    }

    return Adc_computeTemperature(data,vref);
}

uint16_t Adc_getBandGap (Adc_DeviceHandle dev)
{
    // TODO: we must implements this function
    return 0;
}

void ADC_COMP_IRQHandler (void)
{
    Adc_callbackInterrupt(OB_ADC1);
}

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_ADC
