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
 * @file libohiboard/include/hardware/STM32L0/adc_STM32L0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief ADC pins and device definitions for STM32L0 series
 */

#ifndef __ADC_STM32L0_H
#define __ADC_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_ADC) & defined(LIBOHIBOARD_STM32L0)

#define ADC_CHANNEL_MASK                 (0x0007FFFFu)

typedef enum _Adc_Pins
{
#if defined (LIBOHIBOARD_STM32L0x1)

    ADC_PINS_PA0, //IN0
    ADC_PINS_PA1, //IN1
    ADC_PINS_PA2, //IN2
    ADC_PINS_PA3, //IN3
    ADC_PINS_PA4, //IN4
    ADC_PINS_PA5, //IN5
    ADC_PINS_PA6, //IN6
    ADC_PINS_PA7, //IN7

    ADC_PINS_PB0, //IN8
    ADC_PINS_PB1, //IN9

#elif defined (LIBOHIBOARD_STM32L0x3)

#if defined (LIBOHIBOARD_STM32L073)

    ADC_PINS_PA0, //IN0
    ADC_PINS_PA1, //IN1
    ADC_PINS_PA2, //IN2
    ADC_PINS_PA3, //IN3
    ADC_PINS_PA4, //IN4
    ADC_PINS_PA5, //IN5
    ADC_PINS_PA6, //IN6
    ADC_PINS_PA7, //IN7

    ADC_PINS_PB0, //IN8
    ADC_PINS_PB1, //IN9

#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    ADC_PINS_PC0, //IN10
    ADC_PINS_PC1, //IN11
    ADC_PINS_PC2, //IN12
    ADC_PINS_PC3, //IN13
    ADC_PINS_PC4, //IN14
    ADC_PINS_PC5, //IN15
#endif

#endif

#endif
    ADC_PINS_INTERNAL,

} Adc_Pins;

typedef enum _Adc_Channels
{
    ADC_CHANNELS_CH0         = ((uint32_t)(ADC_CHSELR_CHSEL0)),
    ADC_CHANNELS_CH1         = ((uint32_t)(ADC_CHSELR_CHSEL1) | ADC_CFGR1_AWDCH_0),
    ADC_CHANNELS_CH2         = ((uint32_t)(ADC_CHSELR_CHSEL2) | ADC_CFGR1_AWDCH_1),
    ADC_CHANNELS_CH3         = ((uint32_t)(ADC_CHSELR_CHSEL3) | ADC_CFGR1_AWDCH_1 | ADC_CFGR1_AWDCH_0),
    ADC_CHANNELS_CH4         = ((uint32_t)(ADC_CHSELR_CHSEL4) | ADC_CFGR1_AWDCH_2),
    ADC_CHANNELS_CH5         = ((uint32_t)(ADC_CHSELR_CHSEL5) | ADC_CFGR1_AWDCH_2 | ADC_CFGR1_AWDCH_0),
    ADC_CHANNELS_CH6         = ((uint32_t)(ADC_CHSELR_CHSEL6) | ADC_CFGR1_AWDCH_2 | ADC_CFGR1_AWDCH_1),
    ADC_CHANNELS_CH7         = ((uint32_t)(ADC_CHSELR_CHSEL7) | ADC_CFGR1_AWDCH_2 | ADC_CFGR1_AWDCH_1 | ADC_CFGR1_AWDCH_0),
    ADC_CHANNELS_CH8         = ((uint32_t)(ADC_CHSELR_CHSEL8) | ADC_CFGR1_AWDCH_3),
    ADC_CHANNELS_CH9         = ((uint32_t)(ADC_CHSELR_CHSEL9) | ADC_CFGR1_AWDCH_3 | ADC_CFGR1_AWDCH_0),
#if defined (LIBOHIBOARD_STM32L0x3)
    ADC_CHANNELS_CH10        = ((uint32_t)(ADC_CHSELR_CHSEL10)| ADC_CFGR1_AWDCH_3 | ADC_CFGR1_AWDCH_1),
    ADC_CHANNELS_CH11        = ((uint32_t)(ADC_CHSELR_CHSEL11)| ADC_CFGR1_AWDCH_3 | ADC_CFGR1_AWDCH_1 | ADC_CFGR1_AWDCH_0),
    ADC_CHANNELS_CH12        = ((uint32_t)(ADC_CHSELR_CHSEL12)| ADC_CFGR1_AWDCH_3 | ADC_CFGR1_AWDCH_2),
    ADC_CHANNELS_CH13        = ((uint32_t)(ADC_CHSELR_CHSEL13)| ADC_CFGR1_AWDCH_3 | ADC_CFGR1_AWDCH_2 | ADC_CFGR1_AWDCH_0),
    ADC_CHANNELS_CH14        = ((uint32_t)(ADC_CHSELR_CHSEL14)| ADC_CFGR1_AWDCH_3 | ADC_CFGR1_AWDCH_2 | ADC_CFGR1_AWDCH_1),
    ADC_CHANNELS_CH15        = ((uint32_t)(ADC_CHSELR_CHSEL15)| ADC_CFGR1_AWDCH_3 | ADC_CFGR1_AWDCH_2 | ADC_CFGR1_AWDCH_1| ADC_CFGR1_AWDCH_0),
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    ADC_CHANNELS_CH16        = ((uint32_t)(ADC_CHSELR_CHSEL16)| ADC_CFGR1_AWDCH_4),
#endif
#endif
    ADC_CHANNELS_CH17        = ((uint32_t)(ADC_CHSELR_CHSEL17)| ADC_CFGR1_AWDCH_4| ADC_CFGR1_AWDCH_0),
    ADC_CHANNELS_CH18        = ((uint32_t)(ADC_CHSELR_CHSEL18)| ADC_CFGR1_AWDCH_4| ADC_CFGR1_AWDCH_1),

/* Internal channels */
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    ADC_CHANNELS_VLCD        = ADC_CHANNELS_CH16,
#endif
    ADC_CHANNELS_VREFINT     = ADC_CHANNELS_CH17,
#if defined(ADC_CCR_TSEN)
    ADC_CHANNELS_TEMPERATURE = ADC_CHANNELS_CH18,
#endif

} Adc_Channels;

/**
 * Channel sampling time.
 * This value must be set for each channel.
 */
typedef enum _Adc_SamplingTime
{
    ADC_SAMPLINGTIME_1_ADCCLK_5   =  ((uint32_t)0x00000000U),
    ADC_SAMPLINGTIME_3_ADCCLK_5   =  ((uint32_t)ADC_SMPR_SMPR_0),
    ADC_SAMPLINGTIME_7_ADCCLK_5   =  ((uint32_t)ADC_SMPR_SMPR_1),
    ADC_SAMPLINGTIME_12_ADCCLK_5  =  ((uint32_t)(ADC_SMPR_SMPR_1 | ADC_SMPR_SMPR_0)),
    ADC_SAMPLINGTIME_19_ADCCLK_5  =  ((uint32_t)ADC_SMPR_SMPR_2),
    ADC_SAMPLINGTIME_39_ADCCLK_5  =  ((uint32_t)(ADC_SMPR_SMPR_2 | ADC_SMPR_SMPR_0)),
    ADC_SAMPLINGTIME_79_ADCCLK_5  =  ((uint32_t)(ADC_SMPR_SMPR_2 | ADC_SMPR_SMPR_1)),
    ADC_SAMPLINGTIME_160_ADCCLK_5 =  ((uint32_t)ADC_SMPR_SMPR),

} Adc_SamplingTime;

#if defined LIBOHIBOARD_STM32L0x1

extern Adc_DeviceHandle OB_ADC1;

void ADC1_COMP_IRQHandler (void);

//#define ADC_VREFINT_CAL_ADDR               ((uint16_t*)(0x1FFF0078ul))
#define ADC_VREFINT_CAL                    (3000ul)
#define ADC_BANDGAP_VALUE                  (1.224f) //from 1.202 Volt to 1.242 Volt

#define ADC_CALIBRATION_TIMEOUT            10U

#define ADC_TEMPERATURE_CAL1_ADDR          ((uint16_t*)(0x1FF8007Aul))
#define ADC_TEMPERATURE_CAL2_ADDR          ((uint16_t*)(0x1FF8007Eul))
#define ADC_TEMPERATURE_CAL1               (30ul)
#define ADC_TEMPERATURE_CAL2               (130ul)

#elif defined LIBOHIBOARD_STM32L0x3

extern Adc_DeviceHandle OB_ADC1;

void ADC_COMP_IRQHandler (void);

//#define ADC_VREFINT_CAL_ADDR               ((uint16_t*)(0x1FFF75AAul))
#define ADC_VREFINT_CAL                    (3000ul)
#define ADC_BANDGAP_VALUE                  (1.224f) //from 1.202 Volt to 1.242 Volt

#define ADC_CALIBRATION_TIMEOUT            10U

#define ADC_TEMPERATURE_CAL1_ADDR          ((uint16_t*)(0x1FF8007Aul))
#define ADC_TEMPERATURE_CAL2_ADDR          ((uint16_t*)(0x1FF8007Eul))
#define ADC_TEMPERATURE_CAL1               (30ul)
#define ADC_TEMPERATURE_CAL2               (130ul)

#endif

#endif // LIBOHIBOARD_ADC & LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // __ADC_STM32L0_H
