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
 * @file libohiboard/source/adc.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief ADC functions implementation.
 */


#include "platforms.h"
#include "system.h"

#include "adc.h"

#define ADC_PIN_ENABLED        1
#define ADC_PIN_DISABLED       0

typedef struct Adc_Device {
    ADC_MemMapPtr 		  regMap;
    
    uint8_t               pinEnabled;
} Adc_Device;

#if defined(MKL15Z4) || defined(FRDMKL25Z)
static Adc_Device adc0 = {
        .regMap           = ADC0_BASE_PTR,
};
Adc_DeviceHandle ADC0 = &adc0; 
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#endif
