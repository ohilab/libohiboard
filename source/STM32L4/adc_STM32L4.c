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
 * @file libohiboard/source/STM32L4/adc_STM32L4.c
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
                                          ADC_CR_JADSTP_Msk   | \
                                          ADC_CR_ADSTP_Msk    | \
                                          ADC_CR_JADSTART_Msk | \
                                          ADC_CR_ADSTART_Msk  | \
                                          ADC_CR_ADDIS_Msk    | \
                                          ADC_CR_ADEN_Msk)
/**
 * Useful mask to detect the current abilitation status of peripheral.
 */
#define ADC_DEVICE_IS_ENABLE(DEVICE) (((DEVICE->regmap->CR & ADC_CR_ADEN_Msk) == ADC_CR_ADEN_Msk) &&   \
		                              ((DEVICE->regmap->ISR & ADC_ISR_ADRDY_Msk) == ADC_ISR_ADRDY_Msk))

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


#if defined(LIBOHIBOARD_STM32L471) || \
    defined(LIBOHIBOARD_STM32L475) || \
    defined(LIBOHIBOARD_STM32L476) || \
    defined(LIBOHIBOARD_STM32L485) || \
    defined(LIBOHIBOARD_STM32L486) || \
    defined(LIBOHIBOARD_STM32L496) || \
    defined(LIBOHIBOARD_STM32L4A6)

#define ADC_VALID_CLOCK_SOURCE(SOURCE) (((SOURCE) == ADC_CLOCKSOURCE_NONE)       || \
                                        ((SOURCE) == ADC_CLOCKSOURCE_PLLADC1CLK) || \
                                        ((SOURCE) == ADC_CLOCKSOURCE_PLLADC2CLK) || \
                                        ((SOURCE) == ADC_CLOCKSOURCE_SYSCLK))
#else

#define ADC_VALID_CLOCK_SOURCE(SOURCE) (((SOURCE) == ADC_CLOCKSOURCE_NONE)       || \
                                        ((SOURCE) == ADC_CLOCKSOURCE_PLLADC1CLK) || \
                                        ((SOURCE) == ADC_CLOCKSOURCE_SYSCLK))

#endif

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

#define ADC_VALID_SEQUENCE_POSITION(POSITION) (((POSITION) == ADC_SEQUENCEPOSITION_1)  || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_2)  || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_3)  || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_4)  || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_5)  || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_6)  || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_7)  || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_8)  || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_9)  || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_10) || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_11) || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_12) || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_13) || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_14) || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_15) || \
                                               ((POSITION) == ADC_SEQUENCEPOSITION_16))

#define ADC_VALID_SAMPLING_TIME(SAMPLING) (((SAMPLING) == ADC_SAMPLINGTIME_2_ADCCLK_5)   || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_6_ADCCLK_5)   || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_12_ADCCLK_5)  || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_24_ADCCLK_5)  || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_47_ADCCLK_5)  || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_92_ADCCLK_5)  || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_247_ADCCLK_5) || \
                                           ((SAMPLING) == ADC_SAMPLINGTIME_640_ADCCLK_5))

#define ADC_VALID_CONVERSION_TYPE(TYPE) (((TYPE) == ADC_INPUTTYPE_SINGLE_ENDED) || \
                                         ((TYPE) == ADC_INPUTTYPE_DIFFERENTIAL))

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

#if defined (LIBOHIBOARD_STM32L476)

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

#define ADC_IS_DEVICE(DEVICE) (((DEVICE) == OB_ADC1)   || \
                               ((DEVICE) == OB_ADC2)   || \
                               ((DEVICE) == OB_ADC3))

static Adc_Device adc1 =
{
        .regmap              = ADC1,
        .rcommon             = ADC123_COMMON,

        .rccRegisterPtr      = &RCC->AHB2ENR,
        .rccRegisterEnable   = RCC_AHB2ENR_ADCEN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_ADCSEL,
        .rccTypeRegisterPos  = RCC_CCIPR_ADCSEL_Pos,

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
                               ADC_PINS_PC0,
                               ADC_PINS_PC1,
                               ADC_PINS_PC2,
                               ADC_PINS_PC3,
                               ADC_PINS_PC4,
                               ADC_PINS_PC5,
        },
        .pinsChannel         =
        {
                               ADC_CHANNELS_CH5,
                               ADC_CHANNELS_CH6,
                               ADC_CHANNELS_CH7,
                               ADC_CHANNELS_CH8,
                               ADC_CHANNELS_CH9,
                               ADC_CHANNELS_CH10,
                               ADC_CHANNELS_CH11,
                               ADC_CHANNELS_CH12,
                               ADC_CHANNELS_CH15,
                               ADC_CHANNELS_CH16,
                               ADC_CHANNELS_CH1,
                               ADC_CHANNELS_CH2,
                               ADC_CHANNELS_CH3,
                               ADC_CHANNELS_CH4,
                               ADC_CHANNELS_CH13,
                               ADC_CHANNELS_CH14,
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
                               GPIO_PINS_PC0,
                               GPIO_PINS_PC1,
                               GPIO_PINS_PC2,
                               GPIO_PINS_PC3,
                               GPIO_PINS_PC4,
                               GPIO_PINS_PC5,
        },

        .isrNumber           = INTERRUPT_ADC1_2,

        .state               = ADC_DEVICESTATE_RESET,
};
Adc_DeviceHandle OB_ADC1 = &adc1;

static Adc_Device adc2 =
{
        .regmap              = ADC2,
        .rcommon             = ADC123_COMMON,

        .rccRegisterPtr      = &RCC->AHB2ENR,
        .rccRegisterEnable   = RCC_AHB2ENR_ADCEN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_ADCSEL,
        .rccTypeRegisterPos  = RCC_CCIPR_ADCSEL_Pos,

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
                               ADC_PINS_PC0,
                               ADC_PINS_PC1,
                               ADC_PINS_PC2,
                               ADC_PINS_PC3,
                               ADC_PINS_PC4,
                               ADC_PINS_PC5,
        },
        .pinsChannel         =
        {
                               ADC_CHANNELS_CH5,
                               ADC_CHANNELS_CH6,
                               ADC_CHANNELS_CH7,
                               ADC_CHANNELS_CH8,
                               ADC_CHANNELS_CH9,
                               ADC_CHANNELS_CH10,
                               ADC_CHANNELS_CH11,
                               ADC_CHANNELS_CH12,
                               ADC_CHANNELS_CH15,
                               ADC_CHANNELS_CH16,
                               ADC_CHANNELS_CH1,
                               ADC_CHANNELS_CH2,
                               ADC_CHANNELS_CH3,
                               ADC_CHANNELS_CH4,
                               ADC_CHANNELS_CH13,
                               ADC_CHANNELS_CH14,
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
                               GPIO_PINS_PC0,
                               GPIO_PINS_PC1,
                               GPIO_PINS_PC2,
                               GPIO_PINS_PC3,
                               GPIO_PINS_PC4,
                               GPIO_PINS_PC5,
        },

        .isrNumber           = INTERRUPT_ADC1_2,

        .state               = ADC_DEVICESTATE_RESET,
};
Adc_DeviceHandle OB_ADC2 = &adc2;

static Adc_Device adc3 =
{
        .regmap              = ADC3,
        .rcommon             = ADC123_COMMON,

        .rccRegisterPtr      = &RCC->AHB2ENR,
        .rccRegisterEnable   = RCC_AHB2ENR_ADCEN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_ADCSEL,
        .rccTypeRegisterPos  = RCC_CCIPR_ADCSEL_Pos,

        .pins                =
        {
                               ADC_PINS_PC0,
                               ADC_PINS_PC1,
                               ADC_PINS_PC2,
                               ADC_PINS_PC3,
        },
        .pinsChannel         =
        {
                               ADC_CHANNELS_CH1,
                               ADC_CHANNELS_CH2,
                               ADC_CHANNELS_CH3,
                               ADC_CHANNELS_CH4,
        },
        .pinsGpio            =
        {
                               GPIO_PINS_PC0,
                               GPIO_PINS_PC1,
                               GPIO_PINS_PC2,
                               GPIO_PINS_PC3,
        },

        .isrNumber           = INTERRUPT_ADC3,

        .state               = ADC_DEVICESTATE_RESET,
};
Adc_DeviceHandle OB_ADC3 = &adc3;

#define ADC_DEVICE_IS_OTHER_ENABLE(DEVICE)                                                        \
    ( (DEVICE == OB_ADC1) ? (ADC_DEVICE_IS_ENABLE(OB_ADC2) || ADC_DEVICE_IS_ENABLE(OB_ADC3)) : (  \
        (DEVICE == OB_ADC2) ? (ADC_DEVICE_IS_ENABLE(OB_ADC1) || ADC_DEVICE_IS_ENABLE(OB_ADC3)) : (\
            (ADC_DEVICE_IS_ENABLE(OB_ADC1) || ADC_DEVICE_IS_ENABLE(OB_ADC3))                      \
        ))                                                                                        \
    )

#endif

#if defined (LIBOHIBOARD_STM32L431) || \
    defined (LIBOHIBOARD_STM32L432) || \
    defined (LIBOHIBOARD_STM32L433) || \
    defined (LIBOHIBOARD_STM32L442) || \
    defined (LIBOHIBOARD_STM32L443) || \
    defined (LIBOHIBOARD_STM32L451) || \
    defined (LIBOHIBOARD_STM32L452) || \
    defined (LIBOHIBOARD_STM32L462) || \
    defined (LIBOHIBOARD_STM32L4R5) || \
    defined (LIBOHIBOARD_STM32L4R7) || \
    defined (LIBOHIBOARD_STM32L4R9) || \
    defined (LIBOHIBOARD_STM32L4S5) || \
    defined (LIBOHIBOARD_STM32L4S7) || \
    defined (LIBOHIBOARD_STM32L4S9)

#define ADC_VALID_DIFFERENTIAL_CHANNEL(DEV,CHANNEL) (((CHANNEL) == ADC_CHANNELS_CH1)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH2)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH3)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH4)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH5)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH6)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH7)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH8)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH9)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH10)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH11)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH12)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH13)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH14)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH15))

#define ADC_VALID_CHANNEL(DEV,CHANNEL) (((CHANNEL) == ADC_CHANNELS_VREFINT)    || \
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
                                        ((CHANNEL) == ADC_CHANNELS_CH16)       || \
                                        ((CHANNEL) == ADC_CHANNELS_TEMPERATURE)|| \
                                        ((CHANNEL) == ADC_CHANNELS_VBAT))

#define ADC_VALID_TEMPERATURE_CHANNEL(DEVICE) ((DEVICE) == OB_ADC1)

#define ADC_VALID_BATTERY_CHANNEL(DEVICE) ((DEVICE) == OB_ADC1)

#elif defined(LIBOHIBOARD_STM32L471) || \
      defined(LIBOHIBOARD_STM32L475) || \
      defined(LIBOHIBOARD_STM32L476) || \
      defined(LIBOHIBOARD_STM32L485) || \
      defined(LIBOHIBOARD_STM32L486) || \
      defined(LIBOHIBOARD_STM32L496) || \
      defined(LIBOHIBOARD_STM32L4A6)

#define ADC_VALID_DIFFERENTIAL_CHANNEL(DEV,CHANNEL)                                          \
                                                    (((DEV) == OB_ADC1)                   || \
                                                    (((DEV) == OB_ADC2) &&                   \
                                                    (((CHANNEL) == ADC_CHANNELS_CH1)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH2)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH3)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH4)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH5)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH6)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH7)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH8)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH9)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH10)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH11)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH12)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH13)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH14)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH15)))   || \
                                                    (((DEV) == OB_ADC3) &&                   \
                                                    (((CHANNEL) == ADC_CHANNELS_CH1)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH2)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH3)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH6)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH7)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH8)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH9)   ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH10)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH11)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH12))))

#define ADC_VALID_CHANNEL(DEV,CHANNEL)                                                            \
                                                   ((((DEV) == OB_ADC1) &&                        \
                                                    (((CHANNEL) == ADC_CHANNELS_VREFINT)    ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH1)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH2)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH3)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH4)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH5)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH6)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH7)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH8)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH9)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH10)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH11)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH12)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH13)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH14)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH15)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH16)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_TEMPERATURE)||    \
                                                     ((CHANNEL) == ADC_CHANNELS_VBAT)))        || \
                                                    (((DEV) == OB_ADC2) &&                        \
                                                    (((CHANNEL) == ADC_CHANNELS_CH1)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH2)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH3)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH4)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH5)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH6)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH7)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH8)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH9)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH10)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH11)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH12)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH13)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH14)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH15)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH16)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_DAC1_ADC2)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_DAC2_ADC2)))   || \
                                                    (((DEV) == OB_ADC3) &&                        \
                                                    (((CHANNEL) == ADC_CHANNELS_CH1)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH2)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH3)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH4)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH6)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH7)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH8)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH9)        ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH10)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH11)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH12)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_CH13)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_TEMPERATURE)||    \
                                                     ((CHANNEL) == ADC_CHANNELS_VBAT)       ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_DAC1_ADC3)  ||    \
                                                     ((CHANNEL) == ADC_CHANNELS_DAC2_ADC3))))

#define ADC_VALID_TEMPERATURE_CHANNEL(DEVICE) (((DEVICE) == OB_ADC1) || \
                                               ((DEVICE) == OB_ADC3))

#define ADC_VALID_BATTERY_CHANNEL(DEVICE) (((DEVICE) == OB_ADC1) || \
                                           ((DEVICE) == OB_ADC3))
#endif

#define ADC_VALID_VREFINT_CHANNEL(DEVICE) ((DEVICE) == OB_ADC1)

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
    // All the operation must be disabled. See RM0351, page 589
    dev->regmap->CR &= (~(ADC_CR_ADCAL_Msk | ADC_CR_JADSTP_Msk | ADC_CR_ADSTP_Msk |
                          ADC_CR_JADSTART_Msk | ADC_CR_ADSTART_Msk | ADC_CR_ADDIS_Msk | ADC_CR_ADEN_Msk));

    dev->regmap->CR |= ADC_CR_ADVREGEN;
}

/**
 * Enable Deep-Power-Down mode.
 *
 * @param[in] dev Adc device handle
 */
static inline void __attribute__((always_inline)) Adc_enableDeepPowerDown (Adc_DeviceHandle dev)
{
    // All the operation must be disabled. See RM0351, page 589
    dev->regmap->CR &= (~(ADC_DEVICE_ENABLE_MASK | ADC_CR_DEEPPWD));
    dev->regmap->CR |= ADC_CR_ADVREGEN;
}

/**
 * Disable Deep-Power-Down mode.
 *
 * @param[in] dev Adc device handle
 */
static inline void __attribute__((always_inline)) Adc_disableDeepPowerDown (Adc_DeviceHandle dev)
{
    // All the operation must be disabled. See RM0351, page 589
    dev->regmap->CR &= (~(ADC_DEVICE_ENABLE_MASK | ADC_CR_DEEPPWD));
}

/**
 * Check the current status of Deep-Power-Down mode.
 *
 * @param[in] dev Adc device handle
 */
static inline bool __attribute__((always_inline)) Adc_isDeepPowerDownEnable (Adc_DeviceHandle dev)
{
    // All the operation must be disabled. See RM0351, page 589
    return (((dev->regmap->CR & ADC_CR_DEEPPWD) != 0) ? TRUE : FALSE);
}

static inline void __attribute__((always_inline)) Adc_setClock (Adc_DeviceHandle dev, Adc_Config* config)
{
    dev->rcommon->CCR &= (~(ADC_CCR_CKMODE_Msk | ADC_CCR_PRESC_Msk));
    // Put the user choice
    dev->rcommon->CCR |= config->prescaler;
}

static inline void __attribute__((always_inline)) Adc_setSequencePosition (Adc_DeviceHandle dev,
                                                                           Adc_Channels channel,
                                                                           Adc_SequencePosition position)
{
    // Save offset to get correct register
    uint32_t tmp = ((position & ADC_SQR_REGOFFSET_MASK) >> ADC_SQR_REGOFFSET_POS);
    // Get register
    volatile uint32_t* sqrReg = (volatile uint32_t*)((uint32_t)((uint32_t)(&dev->regmap->SQR1) + (tmp << 2u)));
    tmp = ((position & ADC_SQR_SHIFT_MASK) >> ADC_SQR_SHIFT_POS);
    *sqrReg &= (~(0x0000001Fu << tmp));
    // Save selected channel into chosen ranking
    *sqrReg |= (((channel & ADC_CHANNEL_MASK) >> ADC_CHANNEL_POS) << tmp);
}

static inline void __attribute__((always_inline)) Adc_setSamplingTime (Adc_DeviceHandle dev,
                                                                           Adc_Channels channel,
                                                                           Adc_SamplingTime sampling)
{
    // Save offset to get correct register
    uint32_t tmp = ((channel & ADC_SMPR_REGOFFSET_MASK) >> ADC_SMPR_REGOFFSET_POS);
    // Get register
    volatile uint32_t* smprReg = (volatile uint32_t*)((uint32_t)((uint32_t)(&dev->regmap->SMPR1) + (tmp << 2u)));
    tmp = (channel & ADC_SMPR_SMPX_MASK);
    *smprReg &= (~(0x00000007u << tmp));
    // Save sampling time for selected channel
    *smprReg |= ((sampling) << tmp);
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
            if((System_currentTick() - tickstart) > ADC_TIMEOUT_ENABLE)
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
        if ((dev->regmap->CR & (ADC_CR_JADSTART | ADC_CR_ADSTART | ADC_CR_ADEN)) == ADC_CR_ADEN)
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
            if((System_currentTick() - tickstart) > ADC_TIMEOUT_ENABLE)
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
    err |= ohiassert(ADC_VALID_CLOCK_SOURCE(config->clockSource));
    err |= ohiassert(ADC_VALID_PRESCALER(config->prescaler));
    err |= ohiassert(ADC_VALID_DATA_ALIGN(config->dataAlign));
    err |= ohiassert(ADC_VALID_EOC(config->eoc));
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
        // Select clock source
        UTILITY_MODIFY_REGISTER(*dev->rccTypeRegisterPtr,dev->rccTypeRegisterMask,config->clockSource);

        // Enable peripheral clock
        ADC_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);
    }

    // Check if the deep-power-mode is enable
    if (Adc_isDeepPowerDownEnable(dev))
    {
        // Disable the deep-power-mode
        Adc_disableDeepPowerDown(dev);
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
        // Write user configuration
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
        // In case of external trigger enabled, update CFGR register
        // Mask register (disable trigger)
        dev->regmap->CFGR &= (~(ADC_CFGR_EXTSEL_Msk | ADC_CFGR_EXTEN_Msk));
        if (config->externalTriggerPolarity != 0u)
        {
            // Write user config
            dev->regmap->CFGR |= config->externalTriggerPolarity | config->externalTrigger;
        }

        // FIXME: Add Oversampling, LowPowerAutoWait and DMA

        // Configuration of sequencer. If not used SQR1_L at 0x00 (only one channel)
        if (config->sequence == UTILITY_STATE_ENABLE)
        {
            UTILITY_MODIFY_REGISTER(dev->regmap->SQR1,ADC_SQR1_L,config->sequenceNumber);
        }
        else
        {
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->SQR1, ADC_SQR1_L);
        }

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

    ohiassert(ADC_VALID_SEQUENCE_POSITION(config->position));
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
                                     GPIO_PINS_ADC_CONNECTED);
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

    // Check validity of selected channel (both single-ended and differential-ended)
    if (config->type == ADC_INPUTTYPE_DIFFERENTIAL)
    {
        ohiassert(ADC_VALID_DIFFERENTIAL_CHANNEL(dev,channel));
    }
    else
    {
        ohiassert(ADC_VALID_CHANNEL(dev,channel));
    }


    // WARNING: no on-going regular conversion...
    if ((dev->regmap->CR & ADC_CR_ADSTART) == 0u)
    {
        // Set sequence position
        Adc_setSequencePosition(dev,channel,config->position);

        // WARNING: no on-going regular conversion and injected conversion...
        if (((dev->regmap->CR & ADC_CR_ADSTART) == 0u) &&
            ((dev->regmap->CR & ADC_CR_JADSTART) == 0u))
        {
            // Set channel sampling time
            Adc_setSamplingTime(dev,channel,config->samplingTime);

            // FIXME: Set channel offset
        }

        // Warning: ADC must be disabled to manage SE/Differential mode
        if (ADC_DEVICE_IS_ENABLE(dev) == FALSE)
        {
            if (config->type == ADC_INPUTTYPE_DIFFERENTIAL)
            {
                dev->regmap->DIFSEL |= (1u << ((channel & ADC_CHANNEL_MASK) >> ADC_CHANNEL_POS));
                // Set the same sampling time for the next channel used for differential conversion
                Adc_setSamplingTime(dev,(channel + 0x00010000u),config->samplingTime);

                // FIXME: Enable as analog input the second pin
            }
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
        else if ((config->channel == ADC_CHANNELS_VBAT)          &&
                ((dev->rcommon->CCR & ADC_CCR_VBATEN_Msk) == 0u) &&
                 ADC_VALID_BATTERY_CHANNEL(dev))
        {
            dev->rcommon->CCR |= ADC_CCR_VBATEN;
        }
        else if ((config->channel == ADC_CHANNELS_VREFINT)       &&
                ((dev->rcommon->CCR & ADC_CCR_VREFEN_Msk) == 0u) &&
                 ADC_VALID_VREFINT_CHANNEL(dev))
        {
            dev->rcommon->CCR |= ADC_CCR_VREFEN;
        }
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
    if ((dev->regmap->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) != 0u)
    {
        // Software can set ADSTP only when ADSTART=1 and ADDIS=0
        if ((dev->regmap->CR & (ADC_CR_ADSTART | ADC_CR_ADDIS)) == (ADC_CR_ADSTART | ADC_CR_ADDIS))
        {
            UTILITY_MODIFY_REGISTER(dev->regmap->CR,ADC_DEVICE_ENABLE_MASK,ADC_CR_ADSTP);
        }

        // Software can set JADSTP only when JADSTART=1 and ADDIS=0
        if ((dev->regmap->CR & (ADC_CR_JADSTART | ADC_CR_ADDIS)) == (ADC_CR_JADSTART | ADC_CR_ADDIS))
        {
            UTILITY_MODIFY_REGISTER(dev->regmap->CR,ADC_DEVICE_ENABLE_MASK,ADC_CR_JADSTP);
        }

        // Wait until conversion effectively stopped
        tickstart = System_currentTick();
        while ((dev->regmap->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) != 0u)
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

    return (dev->regmap->DR & ADC_DR_RDATA);
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

    return ((((int32_t)(data * (((float)vref)/ADC_VREFINT_CAL)) - (int32_t)*ADC_TEMPERATURE_CAL1_ADDR) *
              (int32_t)(ADC_TEMPERATURE_CAL2 - ADC_TEMPERATURE_CAL1)) /
              (int32_t)((int32_t)*ADC_TEMPERATURE_CAL2_ADDR - (int32_t)*ADC_TEMPERATURE_CAL1_ADDR)) +
              ADC_TEMPERATURE_CAL1;
}

uint16_t Adc_getBandGap (Adc_DeviceHandle dev)
{
    // TODO: we must implements this function
    return 0;
}

_weak void ADC1_2_IRQHandler (void)
{
    Adc_callbackInterrupt(OB_ADC1);
    Adc_callbackInterrupt(OB_ADC2);
}

_weak void ADC3_IRQHandler (void)
{
    Adc_callbackInterrupt(OB_ADC3);
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_ADC
