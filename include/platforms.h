/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  
 * Project: libohiboard
 * Package: -
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
 * @file libohiboard/include/platforms.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Platform definition.
 */

#ifndef __PLATFORMS_H
#define __PLATFORMS_H

/* microcontroller selection: */

/**
 * Define for use with this micro-controllers:
 * <ul>
 *   <li>MKL15Z128FM4</li>
 *   <li>MKL15Z128FT4</li>
 *   <li>MKL15Z128LH4</li>
 *   <li>MKL15Z128VLK4</li>
 * </ul>
 */
#undef MKL15Z4

/**
 * Define for use with this micro-controllers:
 * <ul>
 *   <li>MK60DN512ZVLL10</li>
 *   <li>MK60DX256ZVLL10</li>
 *   <li>MK60DN256ZVLL10</li>
 *   <li>MK60DN512ZVLQ10</li>
 *   <li>MK60DN256ZVLQ10</li>
 *   <li>MK60DX256ZVLQ10</li>
 *   <li>MK60DN512ZVMC10</li>
 *   <li>MK60DN256ZVMC10</li>
 *   <li>MK60DX256ZVMC10</li>
 *   <li>MK60DN512ZVMD10</li>
 *   <li>MK60DX256ZVMD10</li>
 *   <li>MK60DN256ZVMD10</li>
 * </ul>
 */
#undef  MK60DZ10
/**
 * Define for use with Freedom Platform FRDM-KL05Z. 
 */
#undef FRDMKL05Z
/**
 * Define for use with Freedom Platform FRDM-KL25Z. 
 */
#define FRDMKL25Z
/**
 * Define for use with Freedom Platform FRDM-K20D50M. 
 */
#undef FRDMK20D50M

#if defined(MK60DZ10)
#include "platforms/MK60DZ10.h"
#elif defined(MKL15Z4)
#include "platforms/MKL15Z4.h"
#elif defined(FRDMKL05Z)
#include "platforms/MKL05Z4.h"
#elif defined(FRDMKL25Z)
#include "platforms/MKL25Z4.h"
#elif defined(FRDMK20D50M)
#include "platforms/MK20D5.h"
#endif

#endif /* __PLATFORMS_H */
