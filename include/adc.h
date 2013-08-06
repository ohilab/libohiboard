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
    ADC_AVARAGE_1_SAMPLES,
    ADC_AVARAGE_4_SAMPLES,
    ADC_AVARAGE_8_SAMPLES,
    ADC_AVARAGE_16_SAMPLES,
    ADC_AVARAGE_32_SAMPLES,    
} Adc_Avarage;

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
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)
#endif
} Adc_ChannelNumber;

typedef struct Adc_Device* Adc_DeviceHandle;

System_Errors Adc_init (Adc_DeviceHandle dev);

void Adc_setResolution (Adc_DeviceHandle dev, Adc_Resolution resolution);


#if defined(MKL15Z4) || defined(FRDMKL25Z)
extern Adc_DeviceHandle ADC0;
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)
#endif

#endif /* __ADC_H */
