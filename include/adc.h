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

typedef struct Adc_Device* Adc_DeviceHandle;

#if defined(MKL15Z4) || defined(FRDMKL25Z)
extern Adc_DeviceHandle ADC0;
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#endif

#endif /* __ADC_H */
