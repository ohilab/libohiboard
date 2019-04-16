/******************************************************************************
 * Copyright (C) 2012-2017 A. C. Open Hardware Ideas Lab
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
 ******************************************************************************/

/**
 * @file libohiboard/include/adc.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Francesco Piunti <francesco.piunti89@gmail.com>
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @author Matteo Civale    <matteo.civale@gmail.com>
 * @brief ADC definitions and prototypes.
 */

#ifdef LIBOHIBOARD_ADC

#ifndef __ADC_H
#define __ADC_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

/* Define interrupt mask */
#define ADC_ENABLE_INT_ZC         0x01
#define ADC_ENABLE_INT_EOSIA      0x02
#define ADC_ENABLE_INT_LLMT       0x04
#define ADC_ENABLE_INT_HLMT       0x08
#define ADC_ENABLE_INT_EOSIB      0x0C

#define ADC_ENABLE_INT_ZC_SHIFT     0x00
#define ADC_ENABLE_INT_EOSI_SHIFT   0x01
#define ADC_ENABLE_INT_LLMT_SHIFT   0x02
#define ADC_ENABLE_INT_HLMT_SHIFT   0x03
#define ADC_ENABLE_INT_EOSIB_SHIFT  0x04




typedef enum {
#if defined (LIBOHIBOARD_FRDMKL02Z) || \
    defined (LIBOHIBOARD_KL02Z4)

    ADC_PINS_INTERNAL,

#elif defined (LIBOHIBOARD_FRDMKL03Z) || \
      defined (LIBOHIBOARD_KL03Z4)

    ADC_PINS_PTA0,
    ADC_PINS_PTA8,
    ADC_PINS_PTA9,
    ADC_PINS_PTA12,

    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB5,

    ADC_PINS_INTERNAL,

#elif defined(LIBOHIBOARD_KL15Z4)

    ADC_PINS_PTE16,
    ADC_PINS_PTE17,
    ADC_PINS_PTE18,
    ADC_PINS_PTE19,
    ADC_PINS_PTE20,
    ADC_PINS_PTE21,
    ADC_PINS_PTE22,
    ADC_PINS_PTE23,
    ADC_PINS_PTE29,
    ADC_PINS_PTE30,
    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,
    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,
    
    ADC_PINS_INTERNAL,

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    ADC_PINS_PTE20,
    ADC_PINS_PTE21,
    ADC_PINS_PTE22,
    ADC_PINS_PTE23,
    ADC_PINS_PTE29,
    ADC_PINS_PTE30,
    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,
    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,
    
    ADC_PINS_INTERNAL,

#elif defined(LIBOHIBOARD_K12D5)

    ADC_PINS_DP0,
    ADC_PINS_DP1,
    ADC_PINS_DP3,

    ADC_PINS_DM0,
    ADC_PINS_DM1,
    ADC_PINS_DM3,

    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,

    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,

    ADC_PINS_PTD1,
    ADC_PINS_PTD4,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,
    ADC_PINS_PTD7,

    ADC_PINS_PTE0,
    ADC_PINS_PTE1,
    ADC_PINS_PTE16,
    ADC_PINS_PTE17,
    ADC_PINS_PTE18,
    ADC_PINS_PTE19,

    ADC_PINS_SE23,

    ADC_PINS_INTERNAL,
    
#elif defined(LIBOHIBOARD_K10DZ10)

    ADC_PINS_INTERNAL,

#elif defined(LIBOHIBOARD_K10D10)
    
    ADC_PINS_ADC0_DP0,
    ADC_PINS_ADC0_DP1,
    ADC_PINS_PGA0_DP,
    ADC_PINS_ADC0_DP3,
    ADC_PINS_PTE16,
    ADC_PINS_PTE17,
    ADC_PINS_PTE18,
    ADC_PINS_PTE19,
    ADC_PINS_PTC2,
    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,
    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTA7,
    ADC_PINS_PTA8,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTE24,
    ADC_PINS_PTE25,
    ADC_PINS_ADC0_DM0,
    ADC_PINS_ADC0_DM1,
    ADC_PINS_ADC0_SE21,
    ADC_PINS_ADC0_SE22,
    ADC_PINS_ADC0_SE23,
    
    ADC_PINS_ADC1_DP0,
    ADC_PINS_ADC1_DP1,
    ADC_PINS_PGA_DP,
    ADC_PINS_ADC1_DP3,
    ADC_PINS_PTE0,
    ADC_PINS_PTE1,
    ADC_PINS_PTE2,
    ADC_PINS_PTE3,
    ADC_PINS_PTC7,
    ADC_PINS_PTC8,
    ADC_PINS_PTC9,
    ADC_PINS_PTC10,
    
    ADC_PINS_PTB4,
    ADC_PINS_PTB5,
    ADC_PINS_PTB6,
    ADC_PINS_PTB7,
    ADC_PINS_PTB10,
    ADC_PINS_PTB11,
    ADC_PINS_ADC1_SE16,
    ADC_PINS_PTA17,
    ADC_PINS_ADC1_SE18,
    ADC_PINS_ADC1_DM0,
    ADC_PINS_ADC1_DM1,
    ADC_PINS_ADC1_SE23,
    
    ADC_PINS_INTERNAL,
    
#elif defined(LIBOHIBOARD_K60DZ10)

    ADC_PINS_INTERNAL,
    
#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    ADC_PINS_PTE0,
    ADC_PINS_PTE1,
    ADC_PINS_PTE2,
    ADC_PINS_PTE3,
    ADC_PINS_ADC0_DP1,
    ADC_PINS_ADC0_DM1,
    ADC_PINS_ADC1_DP1,
    ADC_PINS_ADC1_DM1,
    ADC_PINS_ADC0_DP0,
    ADC_PINS_ADC1_DP3,
    ADC_PINS_ADC0_DM0,
    ADC_PINS_ADC1_DM3,
    ADC_PINS_ADC1_DP0,
    ADC_PINS_ADC0_DP3,
    ADC_PINS_ADC1_DM0,
    ADC_PINS_ADC0_DM3,
    ADC_PINS_ADC1_SE16,
    ADC_PINS_ADC0_SE22,
    ADC_PINS_ADC0_SE16,
    ADC_PINS_ADC0_SE21,
    ADC_PINS_ADC1_SE18,
    ADC_PINS_ADC0_SE23,
    ADC_PINS_ADC1_SE23,
    ADC_PINS_PTE24,
    ADC_PINS_PTE25,
    ADC_PINS_PTA7,
    ADC_PINS_PTA8,
    ADC_PINS_PTA17,
    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTB4,
    ADC_PINS_PTB5,
    ADC_PINS_PTB6,
    ADC_PINS_PTB7,
    ADC_PINS_PTB10,
    ADC_PINS_PTB11,
    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,
    ADC_PINS_PTC8,
    ADC_PINS_PTC9,
    ADC_PINS_PTC10,
    ADC_PINS_PTC11,
    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,

    ADC_PINS_INTERNAL,

#elif defined (LIBOHIBOARD_KV31F12)

    ADC_PINS_PTE0,
    ADC_PINS_PTE1,
    ADC_PINS_PTE2,
    ADC_PINS_PTE3,
    ADC_PINS_PTE16,
    ADC_PINS_PTE17,
    ADC_PINS_PTE18,
    ADC_PINS_PTE19,
    ADC_PINS_PTE24,
    ADC_PINS_PTE25,

    ADC_PINS_ADC0_DP1,
    ADC_PINS_ADC0_DM1,
    ADC_PINS_ADC1_DP1,
    ADC_PINS_ADC1_DM1,
    ADC_PINS_ADC0_DP0,
    ADC_PINS_ADC1_DP3,
    ADC_PINS_ADC0_DM0,
    ADC_PINS_ADC1_DM3,
    ADC_PINS_ADC1_DP0,
    ADC_PINS_ADC0_DP3,
    ADC_PINS_ADC1_DM0,
    ADC_PINS_ADC0_DM3,

    ADC_PINS_ADC1_SE18,
    ADC_PINS_ADC0_SE23,
    ADC_PINS_ADC1_SE23,

    ADC_PINS_PTA17,

    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTB4,
    ADC_PINS_PTB10,
    ADC_PINS_PTB11,

    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,
    ADC_PINS_PTC8,
    ADC_PINS_PTC9,
    ADC_PINS_PTC10,
    ADC_PINS_PTC11,

    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,

    ADC_PINS_INTERNAL,

#elif defined (LIBOHIBOARD_KV46F) || \
      defined (LIBOHIBOARD_TRWKV46F)

    ADC_PINS_PTE0,
    ADC_PINS_PTE1,
    ADC_PINS_PTE2,
    ADC_PINS_PTE3,
    ADC_PINS_PTE16,
    ADC_PINS_PTE17,
    ADC_PINS_PTE18,
    ADC_PINS_PTE19,
    ADC_PINS_PTE20,
    ADC_PINS_PTE21,
    ADC_PINS_PTE29,
    ADC_PINS_PTE30,
    ADC_PINS_PTE24,
    ADC_PINS_PTE25,

    ADC_PINS_PTA17,

    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTB10,
    ADC_PINS_PTB11,

    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,
    ADC_PINS_PTC10,
    ADC_PINS_PTC11,

    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,


    ADC_PINS_ACH6a,
    ADC_PINS_ACH7a,
    ADC_PINS_ACH2,
    ADC_PINS_ACH3,
    ADC_PINS_ACH6c,
    ADC_PINS_ACH7c,
    ADC_PINS_ACH6d,


#endif

    ADC_PINS_NONE,
} Adc_Pins;

typedef enum {
#if defined (LIBOHIBOARD_FRDMKL03Z) || \
    defined (LIBOHIBOARD_KL03Z4)

    ADC_CHL_A = 0x00,

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)  || \
      defined (LIBOHIBOARD_KL15Z4)

    ADC_CHL_A = 0x00,
    ADC_CHL_B = 0x01,

#elif defined (LIBOHIBOARD_K10DZ10)    || \
      defined (LIBOHIBOARD_K10D10)     || \
      defined (LIBOHIBOARD_K12D5)      || \
      defined (LIBOHIBOARD_K60DZ10)    || \
      defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)   || \
      defined (LIBOHIBOARD_KV31F12)

    ADC_CHL_A = 0x00,
    ADC_CHL_B = 0x01,

#elif defined (LIBOHIBOARD_FRDMKL02Z) || \
      defined (LIBOHIBOARD_KL02Z4)    || \
      defined (LIBOHIBOARD_FRDMKL03Z) || \
      defined (LIBOHIBOARD_KL03Z4)

    ADC_CHL_A = 0x00,

#elif defined (LIBOHIBOARD_KV46F) || \
      defined (LIBOHIBOARD_TWRKV46F)

    ADC_CHL_A        = 0x0,
    ADC_CHL_B        = 0x1,
    ADC_CHL_C        = 0x2,
    ADC_CHL_D        = 0x3,
    ADC_CHL_E        = 0x4,
    ADC_CHL_F        = 0x5,
    ADC_CHL_G        = 0x6,
    ADC_CHL_RESERVED = 0x7,

#endif
} Adc_ChannelMux;


/**
 *Section for cyclic ADC converter
 */

#if defined (LIBOHIBOARD_KV46F) || \
    defined (LIBOHIBOARD_TRWKV46F)

typedef enum{
    ADC_A_CH0,
    ADC_A_CH1,
    ADC_A_CH2,
    ADC_A_CH3,
    ADC_A_CH4,
    ADC_A_CH5,

    ADC_A_CH6,
    ADC_A_CH6a,
    ADC_A_CH6b,
    ADC_A_CH6c,
    ADC_A_CH6d,
    ADC_A_CH6e,
    ADC_A_CH6f,
    ADC_A_CH6g,

    ADC_A_CH7,
    ADC_A_CH7a,
    ADC_A_CH7b,
    ADC_A_CH7c,
    ADC_A_CH7d,
    ADC_A_CH7e,
    ADC_A_CH7f,
    ADC_A_CH7g,


    ADC_B_CH0,
    ADC_B_CH1,
    ADC_B_CH2,
    ADC_B_CH3,
    ADC_B_CH4,
    ADC_B_CH5,

    ADC_B_CH6,
    ADC_B_CH6a,
    ADC_B_CH6b,
    ADC_B_CH6c,
    ADC_B_CH6d,
    ADC_B_CH6e,
    ADC_B_CH6f,
    ADC_B_CH6g,

    ADC_B_CH7,
    ADC_B_CH7a,
    ADC_B_CH7b,
    ADC_B_CH7c,
    ADC_B_CH7d,
    ADC_B_CH7e,
    ADC_B_CH7f,
    ADC_B_CH7g,
}Adc_Channel;

typedef enum {
    ADC_VREFH_PAD  = 0x0,
    ADC_VREFH_ANB2 = 0x1,
    ADC_VREFH_ANA2 = 0x1,

    ADC_VREFL_PAD  = 0x0,
    ADC_VREFL_ANA3 = 0x1,
    ADC_VREFL_ANB3 = 0x01
}Adc_VoltReference;

typedef enum{
    ADC_SCANMODE_ONCE_SEQUENCIAL      = 0x0,
    ADC_SCANMODE_ONCE_PARALLEL        = 0x1,
    ADC_SCANMODE_LOOP_SEQUENCIAL      = 0x2,
    ADC_SCANMODE_LOOP_PARALLEL        = 0x3,
    ADC_SCANMODE_TRIGGERED_SEQUENTIAL = 0x4,
    ADC_SCANMODE_TRIGGERED_PARALLEL   = 0x5,
}Adc_ScanMode;

typedef struct{
    Adc_Pins pin;
    bool diffEn;
    bool zeroCrossEn;
    uint16_t highLim;
    uint16_t lowLim;
}Adc_ChannelSetting;

typedef enum{
    ADC_SPEED_UPTO_6_25MHZ  = 0x0,
    ADC_SPEED_UPTO_12_5MHZ  = 0x1,
    ADC_SPEED_UPTO_18_75MHZ = 0x2,
    ADC_SPEED_UPTO_25MHZ    = 0x3,
}Adc_ConvertionSpeed;

typedef struct{
    uint8_t             clkDiv;

    Adc_VoltReference   vrefH;

    Adc_VoltReference   vrefL;

    Adc_ConvertionSpeed speed;
}Adc_Config;

typedef struct{
    bool                autoPwrDownEn;
    bool                autoStbEn;

    uint8_t             pwrUpDelay;
}Adc_powerConfig;

typedef enum{
    ADC_SYNC_MANUAL_ONLY,
    ADC_SYNC_HARDWERE,
}Adc_Sync;

typedef struct{
    Adc_ScanMode scanmode;
    struct
    {
      uint8_t zc   :1;
      uint8_t eos0 :1;
      uint8_t llmt :1;
      uint8_t hlmt :1;
      uint8_t eos1 :1;
    }interrToEnable;

    void (*isrADCA)(void);
    void (*isrADCB)(void);

    Adc_Sync sync0;
    Adc_Sync sync1;

    bool simultEn;
}Adc_acqConfig;

typedef enum{
    ADC_GAIN_X1 = 0x0,
    ADC_GAIN_X2 = 0x1,
    ADC_GAIN_X4 = 0x2,
} Adc_gain;

typedef enum{
    ADC_ZC_DISABLE          = 0x0,
    ADC_ZC_PLUS_TO_MINUS    = 0x1,
    ADC_ZC_MINUS_TO_PLUS    = 0x2,
    ADC_ZC_BOOTH            = 0x3,
}Adc_zeroCross;

typedef struct{
    bool slotEn;
    Adc_Channel channelP;
    Adc_Channel channelM;

    bool differencialEn;

    uint16_t Hlimit;
    uint16_t Llimit;

    Adc_zeroCross scMode;
    Adc_gain gain;
    bool scanIntEn;
    uint16_t offset;
    bool sampleOnSync;
}Adc_ChannelConfig;

typedef enum{
    ADC_CHANNEL_CHA0 = 0x00,
    ADC_CHANNEL_CHA1 = 0x01,
    ADC_CHANNEL_CHA2 = 0x02,
    ADC_CHANNEL_CHA3 = 0x03,
    ADC_CHANNEL_CHA4 = 0x04,
    ADC_CHANNEL_CHA5 = 0x05,
    ADC_CHANNEL_CHA6 = 0x06,
    ADC_CHANNEL_CHA7 = 0x07,

    ADC_CHANNEL_CHB0 = 0x08,
    ADC_CHANNEL_CHB1 = 0x09,
    ADC_CHANNEL_CHB2 = 0x0A,
    ADC_CHANNEL_CHB3 = 0x0B,
    ADC_CHANNEL_CHB4 = 0x0C,
    ADC_CHANNEL_CHB5 = 0x0D,
    ADC_CHANNEL_CHB6 = 0x0E,
    ADC_CHANNEL_CHB7 = 0x0F,
}Adc_ChannelNumber;

typedef enum
{
    ADC_CONVERTER_A = 0x0,
    ADC_CONVERTER_B = 0x1,
}Adc_converter;


typedef struct Adc_Device* Adc_DeviceHandle;
extern Adc_DeviceHandle OB_ADC0;

void ADCA_IRQHandler(void);

void ADCB_IRQHandler(void);

/**
 * This function initialize the ADC device and setup operational mode.
 *
 * @param[in] dev Adc device handle to be synchronize.
 * @param[in] config A pointer to configuration object
 * @return A System_Errors elements that indicate the status of initialization.
 */

 System_Errors Adc_init (Adc_DeviceHandle dev);

 /**
  * This function set the ADCs A and B acquisition parameter as indicate
  * by acquisitin config var
  *
  * @param[in] dev Adc device handle to be set.
  */
 System_Errors Adc_setPowerMode(Adc_DeviceHandle dev, Adc_powerConfig *config);

 System_Errors Adc_configureADCx(Adc_DeviceHandle dev, Adc_converter converter, Adc_Config*config );

 System_Errors Adc_powerUpADCx(Adc_DeviceHandle dev, Adc_converter converter);

 System_Errors Adc_acquireConfig (Adc_DeviceHandle dev,  Adc_acqConfig *config);

 System_Errors Adc_setChannel (Adc_DeviceHandle dev, uint8_t channelIndex, Adc_ChannelConfig *config);

 System_Errors Adc_acquireStart (Adc_DeviceHandle dev, bool adcAstart, bool adcBstart);

 System_Errors Adc_readValue(Adc_DeviceHandle dev, uint8_t slotIndex, uint16_t* value );


/**
 *Standard ADC section
 */

#else

typedef enum {
#if defined (LIBOHIBOARD_KL15Z4)     || \
	defined (LIBOHIBOARD_KL25Z4)     || \
    defined (LIBOHIBOARD_FRDMKL25Z)  || \
    defined (LIBOHIBOARD_FRDMKL25Z)  || \
	defined (LIBOHIBOARD_K12D5)      || \
    defined (LIBOHIBOARD_K10DZ10)    || \
	defined (LIBOHIBOARD_K10D10)     || \
	defined (LIBOHIBOARD_K60DZ10)    || \
    defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)   || \
    defined (LIBOHIBOARD_KV31F12)

	ADC_RESOLUTION_16BIT,

#endif
	ADC_RESOLUTION_12BIT,
	ADC_RESOLUTION_10BIT,
	ADC_RESOLUTION_8BIT,
} Adc_Resolution;

typedef enum {
    ADC_INPUTTYPE_SINGLE_ENDED = 0,
    ADC_INPUTTYPE_DIFFERENTIAL = 1,
} Adc_InputType;

typedef enum {
    ADC_AVERAGE_1_SAMPLES,
    ADC_AVERAGE_4_SAMPLES,
    ADC_AVERAGE_8_SAMPLES,
    ADC_AVERAGE_16_SAMPLES,
    ADC_AVERAGE_32_SAMPLES,
} Adc_Average;

typedef enum {
	ADC_BUS_CLOCK,
	ADC_BUS_CLOCK_DIV2,
	ADC_ALTERNATE_CLOCK,
	ADC_ASYNCHRONOUS_CLOCK,
} Adc_ClockSource;

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


#define ADC_MAX_CHANNEL_NUMBER 2

typedef enum {

#if defined (LIBOHIBOARD_FRDMKL02Z) || \
	defined (LIBOHIBOARD_KL02Z4)

	ADC_CH_TEMP          = 0x1A,
	ADC_CH_BANDGAP       = 0x1B,
	ADC_CH_VREFH         = 0x1D,
	ADC_CH_VREFL         = 0x1E,
	ADC_CH_DISABLE       = 0x1F,

#elif defined (LIBOHIBOARD_FRDMKL03Z) || \
	  defined (LIBOHIBOARD_KL03Z4)

    ADC_CH_SE0     = 0x00,
    ADC_CH_SE1     = 0x01,
    ADC_CH_SE2     = 0x02,
    ADC_CH_SE3     = 0x03,
    ADC_CH_SE8     = 0x08,
    ADC_CH_SE9     = 0x09,
    ADC_CH_SE15    = 0x0F,

	ADC_CH_TEMP    = 0x1A,
	ADC_CH_BANDGAP = 0x1B,
	ADC_CH_VREFH   = 0x1D,
	ADC_CH_VREFL   = 0x1E,
	ADC_CH_DISABLE = 0x1F,

#elif defined(LIBOHIBOARD_KL15Z4)

	ADC_CH_SE0     = 0x00,
	ADC_CH_SE1     = 0x01,
	ADC_CH_SE2     = 0x02,
	ADC_CH_SE3     = 0x03,
	ADC_CH_SE4a    = 0x04,
	ADC_CH_SE4b    = 0x24,
	ADC_CH_SE5a    = 0x05,
	ADC_CH_SE5b    = 0x25,
	ADC_CH_SE6a    = 0x06,
	ADC_CH_SE6b    = 0x26,
	ADC_CH_SE7a    = 0x07,
	ADC_CH_SE7b    = 0x27,
	ADC_CH_SE8     = 0x08,
	ADC_CH_SE9     = 0x09,
	ADC_CH_SE11    = 0x0B,
	ADC_CH_SE12    = 0x0C,
	ADC_CH_SE13    = 0x0D,
	ADC_CH_SE14    = 0x0E,
	ADC_CH_SE15    = 0x0F,
	ADC_CH_SE23    = 0x17,

	ADC_CH_TEMP    = 0x1A,
	ADC_CH_BANDGAP = 0x1B,
	ADC_CH_VREFH   = 0x1D,
	ADC_CH_VREFL   = 0x1E,
	ADC_CH_DISABLE = 0x1F

#elif defined (LIBOHIBOARD_KL25Z4)     || \
	  defined (LIBOHIBOARD_FRDMKL25Z)

	ADC_CH_SE0     = 0x00,
	ADC_CH_SE3     = 0x03,
	ADC_CH_SE4a    = 0x04,
	ADC_CH_SE4b    = 0x24,
	ADC_CH_SE5b    = 0x25,
	ADC_CH_SE6b    = 0x26,
	ADC_CH_SE7a    = 0x07,
	ADC_CH_SE7b    = 0x27,
	ADC_CH_SE8     = 0x08,
	ADC_CH_SE9     = 0x09,
	ADC_CH_SE11    = 0x0B,
	ADC_CH_SE12    = 0x0C,
	ADC_CH_SE13    = 0x0D,
	ADC_CH_SE14    = 0x0E,
	ADC_CH_SE15    = 0x0F,
	ADC_CH_SE23    = 0x17,

	ADC_CH_TEMP    = 0x1A,
	ADC_CH_BANDGAP = 0x1B,
	ADC_CH_VREFH   = 0x1D,
	ADC_CH_VREFL   = 0x1E,
	ADC_CH_DISABLE = 0x1F

#elif defined (LIBOHIBOARD_K10DZ10)

	ADC_CH_ADC0_DP0      = 0x00,
	ADC_CH_ADC0_PGA0_DP  = 0x02,
	ADC_CH_ADC0_DP3      = 0x03,
	ADC_CH_ADC0_SE4      = 0x04,
	ADC_CH_ADC0_SE5      = 0x05,
	ADC_CH_ADC0_SE6      = 0x06,
	ADC_CH_ADC0_SE7      = 0x07,
	ADC_CH_ADC0_SE8      = 0x08,
	ADC_CH_ADC0_SE9      = 0x09,
	ADC_CH_ADC0_SE12     = 0x0C,
	ADC_CH_ADC0_SE13     = 0x0D,
	ADC_CH_ADC0_SE14     = 0x0E,
	ADC_CH_ADC0_SE15     = 0x0F,
	ADC_CH_ADC0_DM0      = 0x13,
	ADC_CH_ADC0_SE23     = 0x17,

	ADC_CH_ADC1_DP0      = 0x00,
	ADC_CH_ADC1_PGA1_DP  = 0x02,
	ADC_CH_ADC1_DP3      = 0x03,
	ADC_CH_ADC1_SE4      = 0x04,
	ADC_CH_ADC1_SE5      = 0x05,
	ADC_CH_ADC1_SE6      = 0x06,
	ADC_CH_ADC1_SE7      = 0x07,
	ADC_CH_ADC1_SE8      = 0x08,
	ADC_CH_ADC1_SE9      = 0x09,
	ADC_CH_ADC1_SE14     = 0x0E,
	ADC_CH_ADC1_SE15     = 0x0F,
	ADC_CH_ADC1_SE17     = 0x11,
	ADC_CH_ADC1_VREF_OUT = 0x12,
	ADC_CH_ADC1_DM0      = 0x13,

	ADC_CH_TEMP          = 0x1A,
	ADC_CH_BANDGAP       = 0x1B,
	ADC_CH_VREFH         = 0x1D,
	ADC_CH_VREFL         = 0x1E,
	ADC_CH_DISABLE       = 0x1F

#elif defined (LIBOHIBOARD_K10D10)

    ADC_CH_DP0          = 0x00,
    ADC_CH_DP1          = 0x01,
    ADC_CH_PGA_DP       = 0x02,
    ADC_CH_DP3          = 0x03,
    ADC_CH_SE4a         = 0x04,
    ADC_CH_SE5a	        = 0x05,
    ADC_CH_SE6a	        = 0x06,
    ADC_CH_SE7a	        = 0x07,
    ADC_CH_SE4b	        = 0x24,
    ADC_CH_SE5b	        = 0x25,
    ADC_CH_SE6b	        = 0x26,
    ADC_CH_SE7b	        = 0x27,
    ADC_CH_SE8          = 0x08,
    ADC_CH_SE9          = 0x09,
    ADC_CH_SE10	        = 0x0A,
    ADC_CH_SE11	        = 0x0B,
    ADC_CH_SE12	        = 0x0C,
    ADC_CH_SE13	        = 0x0D,
    ADC_CH_SE14	        = 0x0E,
    ADC_CH_SE15	        = 0x0F,
    ADC_CH_SE16	        = 0x10,
    ADC_CH_SE17         = 0x11,
    ADC_CH_SE18         = 0x12,
    ADC_CH_DM0          = 0x13,
    ADC_CH_DM1          = 0x14,
    ADC_CH_SE21         = 0x15,
    ADC_CH_SE22         = 0x16,
    ADC_CH_SE23         = 0x17,
	
	ADC_CH_TEMP          = 0x1A,
	ADC_CH_BANDGAP       = 0x1B,
	ADC_CH_VREFH         = 0x1D,
	ADC_CH_VREFL         = 0x1E,
	ADC_CH_DISABLE       = 0x1F,

#elif defined (LIBOHIBOARD_K12D5)

    ADC_CH_DP0      = 0x00,
    ADC_CH_DP1      = 0x01,
    ADC_CH_DP3      = 0x03,
    ADC_CH_SE4a     = 0x04,
    ADC_CH_SE4b     = 0x24,
    ADC_CH_SE5a     = 0x05,
    ADC_CH_SE5b     = 0x25,
    ADC_CH_SE6a     = 0x06,
    ADC_CH_SE6b     = 0x26,
    ADC_CH_SE7a     = 0x07,
    ADC_CH_SE7b     = 0x27,
    ADC_CH_SE8      = 0x08,
    ADC_CH_SE9      = 0x09,
    ADC_CH_SE10     = 0x0A,
    ADC_CH_SE11     = 0x0B,
    ADC_CH_SE12     = 0x0C,
    ADC_CH_SE13     = 0x0D,
    ADC_CH_SE14     = 0x0E,
    ADC_CH_SE15     = 0x0F,
    ADC_CH_SE21     = 0x15,
    ADC_CH_SE22     = 0x16,
    ADC_CH_SE23     = 0x17,

    ADC_CH_TEMP     = 0x1A,
    ADC_CH_BANDGAP  = 0x1B,
    ADC_CH_VREFH    = 0x1D,
    ADC_CH_VREFL    = 0x1E,
    ADC_CH_DISABLE  = 0x1F

#elif defined(LIBOHIBOARD_K60DZ10)

	ADC_CH_TEMP    = 0x1A,
	ADC_CH_BANDGAP = 0x1B,
	ADC_CH_VREFH   = 0x1D,
	ADC_CH_VREFL   = 0x1E,
	ADC_CH_DISABLE = 0x1F

#elif defined (LIBOHIBOARD_K64F12)     || \
	  defined (LIBOHIBOARD_FRDMK64F)

    ADC_CH_DP0          = 0x00,
    ADC_CH_DP1          = 0x01,
    ADC_CH_DP2          = 0X02,
    ADC_CH_DM2,
    ADC_CH_DP3          = 0x03,
    ADC_CH_DM3          = 0x03,
    ADC_CH_SE4a         = 0x04,
    ADC_CH_SE5a	        = 0x05,
    ADC_CH_SE6a	        = 0x06,
    ADC_CH_SE7a	        = 0x07,
    ADC_CH_SE4b	        = 0x24,
    ADC_CH_SE5b	        = 0x25,
    ADC_CH_SE6b	        = 0x26,
    ADC_CH_SE7b	        = 0x27,
    ADC_CH_SE8          = 0x08,
    ADC_CH_SE9          = 0x09,
    ADC_CH_SE10	        = 0x0A,
    ADC_CH_SE11	        = 0x0B,
    ADC_CH_SE12	        = 0x0C,
    ADC_CH_SE13	        = 0x0D,
    ADC_CH_SE14	        = 0x0E,
    ADC_CH_SE15	        = 0x0F,
    ADC_CH_SE16	        = 0x10,
    ADC_CH_SE17         = 0x11,
    ADC_CH_SE18         = 0x12,
    ADC_CH_DM0          = 0x13,
    ADC_CH_DM1          = 0x14,
    ADC_CH_SE21         = 0x15,
    ADC_CH_SE22         = 0x16,
    ADC_CH_SE23         = 0x17,

	ADC_CH_TEMP          = 0x1A,
	ADC_CH_BANDGAP       = 0x1B,
	ADC_CH_VREFH         = 0x1D,
	ADC_CH_VREFL         = 0x1E,
	ADC_CH_DISABLE       = 0x1F,

#elif defined (LIBOHIBOARD_KV31F12)

    ADC_CH_DP0          = 0x00,
    ADC_CH_DP1          = 0x01,
    ADC_CH_DP2          = 0X02,
    ADC_CH_DP3          = 0x03,
    ADC_CH_SE4a         = 0x04,
    ADC_CH_SE5a         = 0x05,
    ADC_CH_SE6a         = 0x06,
    ADC_CH_SE7a         = 0x07,
    ADC_CH_SE4b         = 0x24,
    ADC_CH_SE5b         = 0x25,
    ADC_CH_SE6b         = 0x26,
    ADC_CH_SE7b         = 0x27,
    ADC_CH_SE8          = 0x08,
    ADC_CH_SE9          = 0x09,
    ADC_CH_SE12         = 0x0C,
    ADC_CH_SE13         = 0x0D,
    ADC_CH_SE14         = 0x0E,
    ADC_CH_SE15         = 0x0F,
    ADC_CH_SE17         = 0x11,
    ADC_CH_SE18         = 0x12,
    ADC_CH_SE23         = 0x17,

    ADC_CH_TEMP          = 0x1A,
    ADC_CH_BANDGAP       = 0x1B,
    ADC_CH_VREFH         = 0x1D,
    ADC_CH_VREFL         = 0x1E,
    ADC_CH_DISABLE       = 0x1F,

#endif
} Adc_ChannelNumber;



typedef struct Adc_Device* Adc_DeviceHandle;

typedef struct _Adc_Config
{
    uint8_t                  clkDiv;
    Adc_ClockSource          clkSource;
    Adc_SampleLength         sampleLength;
    Adc_ConvertionSpeed      covertionSpeed;

    Adc_Resolution           resolution;
    Adc_Average              average;
    Adc_ContinuousConvertion contConv;
    Adc_VoltReference        voltRef;

    bool                     doCalibration;

    bool                     enableHwTrigger;
} Adc_Config;

typedef struct _Adc_ChannelConfig
{
    Adc_InputType            inputType;
    Adc_ChannelNumber        channel;

} Adc_ChannelConfig;



/**
 * This function initialize the ADC device and setup operational mode.
 *
 * @param[in] dev Adc device handle to be synchronize.
 * @param[in] callback callback for interrupt function
 * @param[in] config A pointer to configuration object
 * @return A System_Errors elements that indicate the status of initialization.
 */
System_Errors Adc_init (Adc_DeviceHandle dev, void* callback, Adc_Config *config);

void Adc_enablePin (Adc_DeviceHandle dev, Adc_Pins pin);
System_Errors Adc_readValue (Adc_DeviceHandle dev,
                             Adc_ChannelNumber channel,
                             uint16_t *value,
                             Adc_InputType type);
System_Errors Adc_readValueFromInterrupt (Adc_DeviceHandle dev, uint16_t *value);

System_Errors Adc_setHwChannelTrigger (Adc_DeviceHandle dev,
                                       Adc_ChannelConfig* config,
                                       uint8_t numChannel);

System_Errors Adc_enableDmaTrigger (Adc_DeviceHandle dev);

/**
 * See AN4662 for info in calibration process.
 */
System_Errors Adc_calibration (Adc_DeviceHandle dev);

#if defined (LIBOHIBOARD_FRDMKL02Z) || \
	defined (LIBOHIBOARD_KL02Z4)    || \
	defined (LIBOHIBOARD_FRDMKL03Z) || \
	defined (LIBOHIBOARD_KL03Z4)    || \
    defined (LIBOHIBOARD_KL15Z4)

/* Bandgap value */
#define ADC_BGR_mV               1000

void ADC0_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;

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

#endif /* standard ADC section */

#endif /* __ADC_H */

#endif /* LIBOHIBOARD_ADC */

