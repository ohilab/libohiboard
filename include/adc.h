/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Marco Giammarini <m.giammarini@warcomeb.it>
 *	
 * Project: libohiboard
 * Package: ADC
 * Version: 0.0
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>
 ******************************************************************************/

/**
 * @file libohiboard/include/adc.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief ADC definitions and prototypes.
 */

#ifndef __ADC_H
#define __ADC_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum {
	ADC_RESOLUTION_16BIT,
	ADC_RESOLUTION_12BIT,
	ADC_RESOLUTION_10BIT,
	ADC_RESOLUTION_8BIT,	
} Adc_Resolution;

typedef enum {
    ADC_AVERAGE_1_SAMPLES,
    ADC_AVERAGE_4_SAMPLES,
    ADC_AVERAGE_8_SAMPLES,
    ADC_AVERAGE_16_SAMPLES,
    ADC_AVERAGE_32_SAMPLES,    
} Adc_Average;

typedef enum {
#if defined (FRDMKL25Z)
	ADC_CH_SE0     = 0x00,
	ADC_CH_SE3     = 0x03,
	ADC_CH_SE4b    = 0x04,
	ADC_CH_SE5b    = 0x05,
	ADC_CH_SE6b    = 0x06,
	ADC_CH_SE7b    = 0x07,
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
#elif defined(MKL15Z4)
	ADC_CH_SE0     = 0x00,
	ADC_CH_SE1     = 0x01,
	ADC_CH_SE2     = 0x02,
	ADC_CH_SE3     = 0x03,
	ADC_CH_SE4     = 0x04,
	ADC_CH_SE5     = 0x05,
	ADC_CH_SE6     = 0x06,
	ADC_CH_SE7     = 0x07,
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
#elif defined(MK60DZ10)
	ADC_CH_TEMP    = 0x1A,
	ADC_CH_BANDGAP = 0x1B,
	ADC_CH_VREFH   = 0x1D,
	ADC_CH_VREFL   = 0x1E,
	ADC_CH_DISABLE = 0x1F
#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)
#elif defined(MK10DZ10)
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
#endif
} Adc_ChannelNumber;

typedef enum {
#if defined (FRDMKL25Z)
#elif defined (MKL15Z4)
    ADC_CHL_A = 0x00,
    ADC_CHL_B = 0x01,
#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)
#elif defined(MK10DZ10) || defined(MK60DZ10)
	ADC_CHL_A = 0x00,
	ADC_CHL_B = 0x01,
#endif
} Adc_ChannelMux;

typedef struct Adc_Device* Adc_DeviceHandle;

System_Errors Adc_init (Adc_DeviceHandle dev);

void Adc_setResolution (Adc_DeviceHandle dev, Adc_Resolution resolution);
void Adc_setAverage (Adc_DeviceHandle dev, Adc_Average average);

System_Errors Adc_readValue (Adc_DeviceHandle dev, Adc_ChannelNumber channel, 
		Adc_ChannelMux mux, uint16_t *value);

#if defined(MKL15Z4) || defined(FRDMKL25Z)

extern Adc_DeviceHandle ADC0;

#elif defined(MK60DZ10)

extern Adc_DeviceHandle ADC0;
extern Adc_DeviceHandle ADC1;

#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)

#elif defined(MK10DZ10)

extern Adc_DeviceHandle ADC0;
extern Adc_DeviceHandle ADC1;

#endif

#endif /* __ADC_H */
