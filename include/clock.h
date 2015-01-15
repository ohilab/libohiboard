/******************************************************************************
 * Copyright (C) 2014 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/clock.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @brief Clock definitions and prototypes.
 */

#ifndef __CLOCK_H
#define __CLOCK_H

#include "platforms.h"
#include "errors.h"
#include "types.h"


typedef enum
{
    CLOCK_BUS,
    CLOCK_SYSTEM,
    CLOCK_FLEXBUS,
    CLOCK_FLASH
} Clock_Source;

typedef enum
{
	CLOCK_INTERNAL,
	CLOCK_EXTERNAL,
	CLOCK_CRYSTAL,
#if defined(MK64F12)
	CLOCK_CRYSTAL_32K,
	CLOCK_INTERNAL_48M
#endif
#if defined(MK60F15)
	CLOCK_CRYSTAL_32K
#endif
} Clock_Origin;

typedef enum
{
#if (defined(MKL03Z4) || defined(FRDMKL03))
	CLOCK_LIRC2M,
	CLOCK_LIRC8M,
	CLOCK_HIRC,
	CLOCK_EXT,
#elif (defined(MK60DZ10) || defined(MK60F15) || defined(MK10DZ10) \
	|| defined(OHIBOARD_R1) || defined(FRDMKL25Z) || defined(MK64F12))
	CLOCK_FEI,
	CLOCK_FEE,
	CLOCK_FBI,
	CLOCK_FBE,
	CLOCK_PBE,
	CLOCK_PEE,
	CLOCK_BLPI,
	CLOCK_BLPE,
#endif
} Clock_State;

typedef struct _Clock_Config
{
	Clock_Origin source;

	uint32_t fext;
	uint32_t foutSys;

	uint8_t busDivider;
	uint8_t flexbusDivider;
	uint8_t flashDivider;
} Clock_Config;


System_Errors Clock_Init (Clock_Config *config);
System_Errors Clock_setDividers(uint8_t busDivider, uint8_t flexbusDivider, uint8_t flashDivider);

uint32_t Clock_getFrequency (Clock_Source source);

Clock_State Clock_getCurrentState(); 

#if (defined(MK60DZ10) || defined(MK60F15) || defined(MK64F12))
uint8_t Clock_getCoreDivider();
#endif

#endif /* __CLOCK_H */
