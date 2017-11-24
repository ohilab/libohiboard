/******************************************************************************
 * Copyright (C) 2014-2017 A. C. Open Hardware Ideas Lab
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

#if defined(LIBOHIBOARD_KV46F)   || \
	defined(LIBOHIBOARD_TWRKV46F)

    CLOCK_CORE,
    CLOCK_FAST_PERIPHERALS,
    CLOCK_NANOEDGE,

#else

    CLOCK_BUS,
    CLOCK_SYSTEM,
    CLOCK_FLEXBUS,

#endif
    CLOCK_FLASH,
} Clock_Source;

typedef enum
{
	CLOCK_INTERNAL,
	CLOCK_EXTERNAL,
	CLOCK_CRYSTAL,
#if defined (LIBOHIBOARD_KV31F12)    || \
	defined (LIBOHIBOARD_K64F12)     || \
	defined (LIBOHIBOARD_FRDMK64F)

	CLOCK_CRYSTAL_32K,
	CLOCK_INTERNAL_48M
#endif
} Clock_Origin;

typedef enum
{
#if defined (LIBOHIBOARD_KL03Z4) || \
    defined (LIBOHIBOARD_FRDMKL03Z)
	CLOCK_LIRC2M,
	CLOCK_LIRC8M,
	CLOCK_HIRC,
	CLOCK_EXT,
#elif defined (LIBOHIBOARD_KL15Z4)     || \
      defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)  || \
      defined (LIBOHIBOARD_K10D7)      || \
      defined (LIBOHIBOARD_K10D10)     || \
	  defined (LIBOHIBOARD_K12D5)      || \
	  defined (LIBOHIBOARD_K60DZ10)    || \
	  defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_KV31F12)    || \
	  defined (LIBOHIBOARD_FRDMK64F)   || \
	  defined (LIBOHIBOARD_KV46F)      || \
	  defined (LIBOHIBOARD_TWRKV46F)   || \
	  defined (LIBOHIBOARD_OHIBOARD_R1)
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

#if defined(LIBOHIBOARD_KV46F)   || \
	defined(LIBOHIBOARD_TWRKV46F)

	uint8_t coreDivider;
    uint8_t fastPerDivider;
    uint8_t flashDivider;
    bool enableHGO;

#else
	uint8_t busDivider;
	uint8_t flexbusDivider;
	uint8_t flashDivider;
#endif
} Clock_Config;


System_Errors Clock_Init (Clock_Config *config);
System_Errors Clock_setDividers(uint8_t busDivider, uint8_t flexbusDivider, uint8_t flashDivider);

uint32_t Clock_getFrequency (Clock_Source source);

Clock_State Clock_getCurrentState(); 

#if defined (LIBOHIBOARD_K10D10)       || \
	defined (LIBOHIBOARD_K10D7)        || \
    defined (LIBOHIBOARD_K12D5)        || \
    defined (LIBOHIBOARD_K60DZ10)      || \
    defined (LIBOHIBOARD_K64F12)       || \
    defined (LIBOHIBOARD_FRDMK64F)     || \
    defined (LIBOHIBOARD_KV31F12)      || \
	defined (LIBOHIBOARD_KV46F)        || \
    defined (LIBOHIBOARD_TWRKV46F)     || \
	defined (LIBOHIBOARD_OHIBOARD_R1)

uint8_t Clock_getCoreDivider();

#endif

#endif /* __CLOCK_H */
