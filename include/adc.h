/******************************************************************************
 * Copyright (C) 2012-2016 A. C. Open Hardware Ideas Lab
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
 * @author Matteo Civale <m.civale@gmail.com>
 * @brief ADC definitions and prototypes.
 */

#ifdef LIBOHIBOARD_ADC

#ifndef __ADC_H
#define __ADC_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

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
    defined (LIBOHIBOARD_FRDMK64F)
    
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

#endif

    ADC_PINS_NONE,

} Adc_Pins;

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

#endif
} Adc_ChannelNumber;

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
      defined (LIBOHIBOARD_FRDMK64F)

	ADC_CHL_A = 0x00,
	ADC_CHL_B = 0x01,

#elif defined (LIBOHIBOARD_FRDMKL02Z) || \
	  defined (LIBOHIBOARD_KL02Z4)    || \
      defined (LIBOHIBOARD_FRDMKL03Z) || \
      defined (LIBOHIBOARD_KL03Z4)

	ADC_CHL_A = 0x00,

#endif

} Adc_ChannelMux;

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

System_Errors Adc_setHwChannelTrigger (Adc_DeviceHandle dev,
                                       Adc_ChannelConfig* config,
                                       uint8_t numChannel);

#if defined (LIBOHIBOARD_FRDMKL02Z) || \
	defined (LIBOHIBOARD_KL02Z4)    || \
	defined (LIBOHIBOARD_FRDMKL03Z) || \
	defined (LIBOHIBOARD_KL03Z4)    || \
    defined (LIBOHIBOARD_KL15Z4)    || \
    defined (LIBOHIBOARD_KL25Z4)    || \
	defined (LIBOHIBOARD_FRDMKL25Z)

extern Adc_DeviceHandle ADC0;

#elif defined (LIBOHIBOARD_K12D5)

void ADC0_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;

#elif defined (LIBOHIBOARD_K10DZ10)    || \
	  defined (LIBOHIBOARD_K10D10)     || \
	  defined (LIBOHIBOARD_K60DZ10)

extern Adc_DeviceHandle ADC0;
extern Adc_DeviceHandle ADC1;

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

void ADC0_IRQHandler();
void ADC1_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;
extern Adc_DeviceHandle OB_ADC1;

#endif

#endif /* __ADC_H */

#endif // LIBOHIBOARD_ADC


