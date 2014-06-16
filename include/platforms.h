/******************************************************************************
 * Copyright (C) 2012-2014 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * Define for use with this micro-controllers:
 * <ul>
 *   <li>MK10DN512ZVLK10</li>
 *   <li>MK10DN512ZVLL10</li>
 *   <li>MK10DN512ZVLQ10</li>
 *   <li>MK10DX128ZVLQ10</li>
 *   <li>MK10DX256ZVLQ10</li>
 *   <li>MK10DN512ZVMB10</li>
 *   <li>MK10DN512ZVMC10</li>
 *   <li>MK10DN512ZVMD10</li>
 *   <li>MK10DX256ZVMD10</li>
 *   <li>MK10DX128ZVMD10</li>
 * </ul>
 */
#undef  MK10DZ10

/**
 * Define for use with OHIBOARD-R1. 
 */
#undef OHIBOARD_R1

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
#elif defined(MK10DZ10)
#include "platforms/MK10DZ10.h"
#elif defined(MKL15Z4)
#include "platforms/MKL15Z4.h"
#elif defined(FRDMKL05Z)
#include "platforms/MKL05Z4.h"
#elif defined(FRDMKL25Z)
#include "platforms/MKL25Z4.h"
#elif defined(FRDMK20D50M)
#include "platforms/MK20D5.h"
#elif defined(OHIBOARD_R1)
#include "platforms/MK60DZ10.h"
#endif

#endif /* __PLATFORMS_H */
