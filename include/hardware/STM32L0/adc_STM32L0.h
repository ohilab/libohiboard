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

#define ADC_CHANNEL_MASK                 (0x00FF0000u)
#define ADC_CHANNEL_POS                  (16u)

typedef enum _Adc_Pins
{
#if defined (LIBOHIBOARD_STM32L0x3)

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
    ADC_CHANNELS_CH0         = (0x00000000u),
    ADC_CHANNELS_CH1         = (0x00010000u),
    ADC_CHANNELS_CH2         = (0x00020000u),
    ADC_CHANNELS_CH3         = (0x00030000u),
    ADC_CHANNELS_CH4         = (0x00040000u),
    ADC_CHANNELS_CH5         = (0x00050000u),
    ADC_CHANNELS_CH6         = (0x00060000u),
    ADC_CHANNELS_CH7         = (0x00070000u),
    ADC_CHANNELS_CH8         = (0x00080000u),
    ADC_CHANNELS_CH9         = (0x00090000u),
    ADC_CHANNELS_CH10        = (0x000A0000u),
    ADC_CHANNELS_CH11        = (0x000B0000u),
    ADC_CHANNELS_CH12        = (0x000C0000u),
    ADC_CHANNELS_CH13        = (0x000D0000u),
    ADC_CHANNELS_CH14        = (0x000E0000u),
    ADC_CHANNELS_CH15        = (0x000F0000u),
    ADC_CHANNELS_CH16        = (0x00100000u),
    ADC_CHANNELS_CH17        = (0x00110000u),
    ADC_CHANNELS_CH18        = (0x00120000u),

//    ADC_CHANNELS_VREFINT     = (0x00000000u),
    ADC_CHANNELS_TEMPERATURE = (0x00110000u),
    ADC_CHANNELS_VLCD        = (0x00120000u),

} Adc_Channels;

/**
 * Channel sampling time.
 * This value must be set for each channel.
 */
typedef enum _Adc_SamplingTime
{
    ADC_SAMPLINGTIME_1_ADCCLK_5   = (0x00000000u),
    ADC_SAMPLINGTIME_3_ADCCLK_5   = (ADC_SMPR_SMP_0),
    ADC_SAMPLINGTIME_7_ADCCLK_5   = (ADC_SMPR_SMP_1),
    ADC_SAMPLINGTIME_12_ADCCLK_5  = (ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0),
    ADC_SAMPLINGTIME_19_ADCCLK_5  = (ADC_SMPR_SMP_2),
    ADC_SAMPLINGTIME_39_ADCCLK_5  = (ADC_SMPR_SMP_2 | ADC_SMPR_SMP_0),
    ADC_SAMPLINGTIME_79_ADCCLK_5  = (ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1),
    ADC_SAMPLINGTIME_160_ADCCLK_5 = (ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0),

} Adc_SamplingTime;

#if defined LIBOHIBOARD_STM32L0x3

extern Adc_DeviceHandle OB_ADC1;

void ADC_COMP_IRQHandler (void);

#endif


#endif // LIBOHIBOARD_ADC & LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // __ADC_STM32L0_H
