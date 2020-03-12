/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2020 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Francesco Piunti <francesco.piunti89@gmail.com>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
 *  Matteo Civale <m.civale@gmail.com>
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
 * @file libohiboard/include/adc.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Francesco Piunti <francesco.piunti89@gmail.com>
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @author Matteo Civale    <matteo.civale@gmail.com>
 * @brief ADC definitions and prototypes.
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_ADC

/**
 * @defgroup ADC ADC
 * @brief ADC HAL driver
 * @{
 */

#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "utility.h"

#if !defined(ADC_SAMPLE_NUMBER)
#define ADC_SAMPLE_NUMBER                16
#endif 
    
/**
 * Device handle for ADC peripheral.
 */
typedef struct _Adc_Device* Adc_DeviceHandle;

#if defined (LIBOHIBOARD_STM32L0)

#include "hardware/STM32L0/adc_STM32L0.h"

#elif defined (LIBOHIBOARD_STM32L4)

#include "hardware/STM32L4/adc_STM32L4.h"

#elif defined (LIBOHIBOARD_PIC24FJ)

#include "hardware/PIC24FJ/adc_PIC24FJ.h"

#else

typedef enum 
{
    ADC_PINS_NONE,
} Adc_Pins;

typedef enum 
{
    ADC_CHANNELSMUX_NONE,
} Adc_ChannelsMux;

typedef enum 
{
    ADC_CHANNELS_NONE,
} Adc_Channels;

#endif

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Adc_DeviceState
{
    ADC_DEVICESTATE_RESET,
    ADC_DEVICESTATE_READY,
    ADC_DEVICESTATE_BUSY,
    ADC_DEVICESTATE_ERROR,

} Adc_DeviceState;

/**
 * The list of possible ADC resolution.
 */
typedef enum _Adc_Resolution
{
#if defined (LIBOHIBOARD_MKL)     || \
	defined (LIBOHIBOARD_MK)
	ADC_RESOLUTION_16BIT,
	ADC_RESOLUTION_12BIT,
	ADC_RESOLUTION_10BIT,
	ADC_RESOLUTION_8BIT,
#endif
#if defined (LIBOHIBOARD_STM32L0) || \
	defined (LIBOHIBOARD_STM32L4)
    ADC_RESOLUTION_12BIT = 0x00000000u,
    ADC_RESOLUTION_10BIT = ADC_CFGR_RES_0,
    ADC_RESOLUTION_8BIT  = ADC_CFGR_RES_1,
    ADC_RESOLUTION_6BIT  = ADC_CFGR_RES,
#endif
#if defined (LIBOHIBOARD_PIC24FJ)
    ADC_RESOLUTION_10BIT = 0x0000,
    ADC_RESOLUTION_12BIT = _AD1CON1_MODE12_MASK,
#endif
} Adc_Resolution;

typedef enum _Adc_InputType
{
    ADC_INPUTTYPE_SINGLE_ENDED = 0x0000,
#if !defined (LIBOHIBOARD_PIC24FJ)
    ADC_INPUTTYPE_DIFFERENTIAL = 1,
#endif
} Adc_InputType;

#if defined (LIBOHIBOARD_NXP_KINETIS)
typedef enum {
    ADC_AVERAGE_1_SAMPLES,
    ADC_AVERAGE_4_SAMPLES,
    ADC_AVERAGE_8_SAMPLES,
    ADC_AVERAGE_16_SAMPLES,
    ADC_AVERAGE_32_SAMPLES,
} Adc_Average;

typedef enum {
    ADC_SHORT_SAMPLE,
    ADC_LONG_SAMPLE_2,
    ADC_LONG_SAMPLE_6,
    ADC_LONG_SAMPLE_12,
    ADC_LONG_SAMPLE_20,
} Adc_SampleLength;

typedef enum {
    ADC_NORMAL_CONVERTION,
    ADC_HIGH_SPEED_CONVERTION,
}Adc_ConvertionSpeed;

typedef enum {
    ADC_SINGLE_CONVERTION,
    ADC_CONTINUOUS_CONVERTION,
}Adc_ContinuousConvertion;

typedef enum {
    ADC_VREF,
    ADC_VALT,
}Adc_VoltReference;

#endif // LIBOHIBOARD_NXP_KINETIS

/**
 * List of all possible ADC clock source
 *
 * @note STM32: The ADC clock configuration is common to all ADC instances.
 */
typedef enum _Adc_ClockSource
{
#if defined (LIBOHIBOARD_NXP_KINETIS)

    ADC_BUS_CLOCK,
    ADC_BUS_CLOCK_DIV2,
    ADC_ALTERNATE_CLOCK,
    ADC_ASYNCHRONOUS_CLOCK,

#elif defined (LIBOHIBOARD_ST_STM32)

    ADC_CLOCKSOURCE_NONE        = 0x00000000U,
    ADC_CLOCKSOURCE_PLLADC1CLK  = RCC_CCIPR_ADCSEL_0,

#if defined(LIBOHIBOARD_STM32L471) || \
    defined(LIBOHIBOARD_STM32L475) || \
    defined(LIBOHIBOARD_STM32L476) || \
    defined(LIBOHIBOARD_STM32L485) || \
    defined(LIBOHIBOARD_STM32L486) || \
    defined(LIBOHIBOARD_STM32L496) || \
    defined(LIBOHIBOARD_STM32L4A6)
    ADC_CLOCKSOURCE_PLLADC2CLK  = RCC_CCIPR_ADCSEL_1,
#endif
    ADC_CLOCKSOURCE_SYSCLK      = RCC_CCIPR_ADCSEL,
#elif defined (LIBOHIBOARD_MICROCHIP_PIC)

    ADC_CLOCKSOURCE_DEDICATED = _AD1CON3_ADRC_MASK, //!< Dedicated ADC RC clock generator (4 MHz nominal)
    ADC_CLOCKSOURCE_SYSCLOCK  = 0x0000,             //!< Clock derived from system clock

#endif
} Adc_ClockSource;

typedef enum _Adc_Prescaler
{
#if defined (LIBOHIBOARD_ST_STM32)

    ADC_PRESCALER_SYNC_DIV1    = (ADC_CCR_CKMODE_0),
    ADC_PRESCALER_SYNC_DIV2    = (ADC_CCR_CKMODE_1),
    ADC_PRESCALER_SYNC_DIV4    = (ADC_CCR_CKMODE_1 | ADC_CCR_CKMODE_0),
    ADC_PRESCALER_ASYNC_DIV1   = 0x00000000u,
    ADC_PRESCALER_ASYNC_DIV2   = (ADC_CCR_PRESC_0),
    ADC_PRESCALER_ASYNC_DIV4   = (ADC_CCR_PRESC_1),
    ADC_PRESCALER_ASYNC_DIV6   = (ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0),
    ADC_PRESCALER_ASYNC_DIV8   = (ADC_CCR_PRESC_2),
    ADC_PRESCALER_ASYNC_DIV10  = (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_0),
    ADC_PRESCALER_ASYNC_DIV12  = (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1),
    ADC_PRESCALER_ASYNC_DIV16  = (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0),
    ADC_PRESCALER_ASYNC_DIV32  = (ADC_CCR_PRESC_3),
    ADC_PRESCALER_ASYNC_DIV64  = (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_0),
    ADC_PRESCALER_ASYNC_DIV128 = (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1),
    ADC_PRESCALER_ASYNC_DIV256 = (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0),

#else
    
    ADC_PRESCALER_NONE, //!< Not used for this microcontroller
            
#endif
} Adc_Prescaler;

#if defined (LIBOHIBOARD_ST_STM32)

typedef enum _Adc_DataAlign
{
    ADC_DATAALIGN_RIGHT = 0x00000000u,
    ADC_DATAALIGN_LEFT  = ADC_CFGR_ALIGN,

} Adc_DataAlign;

typedef enum _Adc_EndOfConversion
{
    ADC_ENDOFCONVERSION_SINGLE   = ADC_ISR_EOC,
    ADC_ENDOFCONVERSION_SEQUENCE = ADC_ISR_EOS,

} Adc_EndOfConversion;

typedef enum _Adc_Trigger
{
    ADC_TRIGGER_EXT0_TIM1_CH1     = 0x00000000u,
    ADC_TRIGGER_EXT1_TIM1_CH2     = (ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT2_TIM1_CH3     = (ADC_CFGR_EXTSEL_1),
    ADC_TRIGGER_EXT3_TIM2_CH2     = (ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT4_TIM3_TRGO    = (ADC_CFGR_EXTSEL_2),
    ADC_TRIGGER_EXT5_TIM4_CH4     = (ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT6_EXTI_11      = (ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_1),
    ADC_TRIGGER_EXT7_TIM8_TRGO    = (ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT8_TIM8_TRGO2   = (ADC_CFGR_EXTSEL_3),
    ADC_TRIGGER_EXT9_TIM1_TRGO    = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT10_TIM1_TRGO2  = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_1),
    ADC_TRIGGER_EXT11_TIM2_TRGO   = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT12_TIM4_TRGO   = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_2),
    ADC_TRIGGER_EXT13_TIM6_TRGO   = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT14_TIM15_TRGO  = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_1),
    ADC_TRIGGER_EXT15_TIM3_CH4    = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0),

} Adc_Trigger;

typedef enum _Adc_TriggerPolarity
{
    ADC_TRIGGERPOLARITY_DISABLE = 0x00000000u,
    ADC_TRIGGERPOLARITY_RISING  = ADC_CFGR_EXTEN_0,
    ADC_TRIGGERPOLARITY_FALLING = ADC_CFGR_EXTEN_1,
    ADC_TRIGGERPOLARITY_BOTH    = ADC_CFGR_EXTEN,

} Adc_TriggerPolarity;

#endif

#if defined (LIBOHIBOARD_MICROCHIP_PIC)

typedef enum _Adc_VoltReference
{
#if defined LIBOHIBOARD_PIC24FJ
    ADC_VOLTREFERENCE_VREF = _AD1CON2_PCVFG0_MASK | _AD1CON2_NVCFG0_MASK,
    ADC_VOLTREFERENCE_AVDD = 0x0000,
#endif
} Adc_VoltReference;

#endif

typedef struct _Adc_Config
{
    Adc_Resolution resolution;                           /**< ADC resolutions */

    Adc_ClockSource clockSource;               /**< The clock source selected */
    
#if !defined (LIBOHIBOARD_MICROCHIP_PIC)
    Adc_Prescaler prescaler;                       /**< input clock prescaler */
#else
    /**
     * Input clock prescaler (T_AD).
     * This value is used to compute T_AD value by: T_AD = T_CY * (prescaler + 1).
     * The T_AD value is used as reference for compute sampling time into Clocked 
     * Conversion Trigger mode (see DS39739A-page 51-23). In the meantime the 
     * conversion time is 14 T_AD for 10bit and 16 T_AD for 12bit conversions.
     * \note The value must be between 0 and 255.
     */
    uint16_t prescaler;
#endif

#if !defined (LIBOHIBOARD_MICROCHIP_PIC)
    /**
     * Specify whether the conversion is performed in single mode or
     * continuous mode.
     */
    Utility_State continuousConversion;
#endif
    
#if defined (LIBOHIBOARD_NXP_KINETIS)
    uint8_t                  clkDiv;
    Adc_SampleLength         sampleLength;
    Adc_ConvertionSpeed      covertionSpeed;


    Adc_Average              average;
    Adc_ContinuousConvertion contConv;
    Adc_VoltReference        voltRef;

    bool                     doCalibration;

    bool                     enableHwTrigger;
#endif

#if defined (LIBOHIBOARD_ST_STM32)

    Adc_DataAlign dataAlign;  /**< Data align position for conversion results */

    Adc_EndOfConversion eoc;                      /**< End-Of-Conversion type */

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
     * Configure the sequencer of ADC groups regular conversion.
     */
    Utility_State sequence;
    /**
     * Specify the number of channel that will be converted within the group sequencer.
     * To use the sequencer and convert several channels, parameter 'sequence' must be enabled.
     * The number must be between 0x00 and 0x0F, that is 1 channel to 16 channels.
     */
    uint8_t sequenceNumber;

    /**
     * Select the external event source used to trigger ADC conversion start.
     */
    Adc_Trigger externalTrigger;
    /**
     * External trigger enable and polarity selection for regular channels.
     */
    Adc_TriggerPolarity externalTriggerPolarity;

    /**
     * Callback for End-of-Conversion
     */
    void (* eocCallback)(struct _Adc_Device *dev);
    /**
     * Callback for End-of-Sequence
     */
    void (* eosCallback)(struct _Adc_Device *dev);
    /**
     * Callback for Overrun
     */
    void (* overrunCallback)(struct _Adc_Device *dev);

#endif

#if defined (LIBOHIBOARD_MICROCHIP_PIC)

    /**
     * Select the voltage reference for the ADC device.
     */
    Adc_VoltReference voltReference;
    
#endif    
    
} Adc_Config;

/**
 * @defgroup ADC_Configuration_Functions ADC configuration functions
 * @brief Functions to initialize and de-initialize a ADC peripheral.
 * @{
 */

/**
 * This function initialize the ADC device and setup operational mode according
 * to the specified parameters in the @ref Adc_Config
 *
 * @param[in] dev Adc device handle
 * @param[in] config A pointer to configuration object
 * @return A System_Errors elements that indicate the status of initialization.
 */
System_Errors Adc_init (Adc_DeviceHandle dev, Adc_Config *config);

/**
 * This function de-initialize the selected peripheral.
 *
 * @param[in] dev Adc device handle
 */
System_Errors Adc_deInit (Adc_DeviceHandle dev);

/**
 * @}
 */

/**
 * @defgroup ADC_Read_Functions ADC read functions
 * @brief Functions to configure and read data from pins or internal channel.
 * @{
 */

typedef struct _Adc_ChannelConfig
{
    /**
     * Specify if the channel must be converted in single ended or differential mode.
     */
    Adc_InputType type;

    /**
     * Specify if the configuration is for internal channel. In this case, the configuration
     * use the channel parameter, otherwise search the correct channel using the pin name.
     */
    bool isInternal;
    /**
     * Specify the internal channel to configure.
     * \warning This field can be used only for internal channel.
     */
    Adc_Channels channel;

#if !defined (LIBOHIBOARD_MICROCHIP_PIC)
    Adc_SequencePosition position;

    Adc_SamplingTime samplingTime;
#endif
    
#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    
    /**
     * Sampling time for a single conversion.
     * 
     * \note The time is in milli-seconds.
     */
    uint32_t samplingTime;
    
#endif

} Adc_ChannelConfig;

/**
 * Configure pin or internal channel to be used as Adc input.
 *
 * @param[in] dev Adc device handle
 * @param[in] config Configuration list for selected pin
 * @param[in] pin Selected microcontroller pin or ADC_PIN_INTERNAL for internal channel
 * @return ERRORS_NO_ERROR for configuration without problems, otherwise a specific error.
 */
System_Errors Adc_configPin (Adc_DeviceHandle dev, Adc_ChannelConfig* config, Adc_Pins pin);

/**
 * Enable Adc and start conversion of regular group.
 *
 * @param[in] dev Adc device handle
 * @return ERRORS_NO_ERROR for start conversion without problems, otherwise a specific error.
 */
System_Errors Adc_start (Adc_DeviceHandle dev);

/**
 * Stop Adc conversion of regular group and disable peripheral.
 *
 * @param[in] dev Adc device handle
 * @return ERRORS_NO_ERROR for stop conversion without problems, otherwise a specific error.
 */
System_Errors Adc_stop (Adc_DeviceHandle dev);

/**
 * Get Adc conversion result.
 *
 * @param[in] dev Adc device handle
 * @return The converted values
 */
uint32_t Adc_read (Adc_DeviceHandle dev);

/**
 * Wait until conversion ends.
 * Check EOC or EOS based on what the user has selected during configuration.
 *
 * @param[in] dev Adc device handle
 * @param[in] timeout Maximum wait time in milli-second
 * @return
 */
System_Errors Adc_poll (Adc_DeviceHandle dev, uint32_t timeout);

/**
 * @}
 */

/**
 * @defgroup ADC_Utility_Functions ADC utility functions
 * @brief Useful functions to manage read data form Adc.
 * @{
 */

/**
 * Convert raw data read from channel 17 to temperature, in Celsius.
 *
 * @param[in] dev Adc device handle
 * @param[in] data Raw data from Adc
 * @param[in] vref Analog reference voltage connected to microcontroller in milli-volt
 */
int32_t Adc_getTemperature (Adc_DeviceHandle dev, uint32_t data, uint32_t vref);

/**
 * Read and avarage the raw value of band gap.
 * The number of averages is defined by @ref ADC_BADGAP_SAMPLE_NUMBER, the default 
 * value is 16.
 *
 * @param[in] dev Adc device handle
 * @return The raw value (based on bit configuration of ADC) of the band gap.
 */
uint16_t Adc_getBandGap (Adc_DeviceHandle dev);

/**
 * This function configure read a configured number of times the selected ADC channel and
 * return the avarage results.
 *
 * @param[in]    dev: Adc device handle
 * @param[in] config: Configuration list for selected pin
 * @param[in]    pin: Selected microcontroller pin
 * @param[in]  count: Number of times of channel reads
 * @return The avarage results of channel reads.
 */
uint16_t Adc_getAvarageRead (Adc_DeviceHandle dev, Adc_ChannelConfig* config, Adc_Pins pin, uint8_t count);

/**
 * This function returns the full-scale value based on the resoluction chosen.
 *
 *  @param[in] res: The resolution chosen
 */
static inline uint16_t Adc_getMaxValue (Adc_Resolution res)
{
    switch (res)
    {
#if defined (LIBOHIBOARD_MKL)  || \
	defined (LIBOHIBOARD_MK)
    case ADC_RESOLUTION_16BIT:
        return 0xFFFF;
	case ADC_RESOLUTION_12BIT:
        return 0x0FFF;
    case ADC_RESOLUTION_10BIT:
        return 0x03FF;
	ADC_RESOLUTION_8BIT:
        return 0x00FF;
#endif
#if defined (LIBOHIBOARD_STM32L4)
	case ADC_RESOLUTION_12BIT:
        return 0x0FFF;
    case ADC_RESOLUTION_10BIT:
        return 0x03FF;
	ADC_RESOLUTION_8BIT:
        return 0x00FF;
    ADC_RESOLUTION_6BIT:
        return 0x003F;
#endif
#if defined (LIBOHIBOARD_PIC24FJ)
	case ADC_RESOLUTION_12BIT:
        return 0x0FFF;
    case ADC_RESOLUTION_10BIT:
        return 0x03FF;
#endif
    default:
        return 0;
    }
}

/**
 * @}
 */

//System_Errors Adc_readValue (Adc_DeviceHandle dev,
//                             Adc_ChannelNumber channel,
//                             uint16_t *value,
//                             Adc_InputType type);
//System_Errors Adc_readValueFromInterrupt (Adc_DeviceHandle dev, uint16_t *value);
//
//System_Errors Adc_setHwChannelTrigger (Adc_DeviceHandle dev,
//                                       Adc_ChannelConfig* config,
//                                       uint8_t numChannel);
//
//System_Errors Adc_enableDmaTrigger (Adc_DeviceHandle dev);
//
///**
// * See AN4662 for info in calibration process.
// */
//System_Errors Adc_calibration (Adc_DeviceHandle dev);

#if defined (LIBOHIBOARD_FRDMKL02Z) || \
	defined (LIBOHIBOARD_KL02Z4)    || \
	defined (LIBOHIBOARD_FRDMKL03Z) || \
	defined (LIBOHIBOARD_KL03Z4)    || \
    defined (LIBOHIBOARD_KL15Z4)

extern Adc_DeviceHandle ADC0;

#elif defined (LIBOHIBOARD_KL25Z4)    || \
	  defined (LIBOHIBOARD_FRDMKL25Z)

/* Bandgap value */
#define ADC_BGR_mV               1000

void ADC0_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;

#elif defined (LIBOHIBOARD_K12D5)

void ADC0_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;

#elif defined (LIBOHIBOARD_K10DZ10)    || \
	  defined (LIBOHIBOARD_K10D10)

extern Adc_DeviceHandle OB_ADC0;
extern Adc_DeviceHandle OB_ADC1;

#elif defined (LIBOHIBOARD_K60DZ10)

extern Adc_DeviceHandle ADC0;
extern Adc_DeviceHandle ADC1;

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

void ADC0_IRQHandler();
void ADC1_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;
extern Adc_DeviceHandle OB_ADC1;

#elif defined (LIBOHIBOARD_KV31F12)

void ADC0_IRQHandler();
void ADC1_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;
extern Adc_DeviceHandle OB_ADC1;

#endif

#ifdef __cplusplus
}
#endif

#endif // __ADC_H

/**
 * @}
 */

#endif // LIBOHIBOARD_ADC

/**
 * @}
 */
