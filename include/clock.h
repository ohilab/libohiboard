/*
 * Copyright (C) 2014-2018 A. C. Open Hardware Ideas Lab
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
 */

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

#if defined (LIBOHIBOARD_NXP_KINETIS)

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

// END IF: LIBOHIBOARD_NXP_KINETIS
#elif defined (LIBOHIBOARD_ST_STM32)

    CLOCK_BUS,
    CLOCK_SYSTEM,

// END IF: LIBOHIBOARD_ST_STM32
#endif

} Clock_Source;

typedef enum
{

    CLOCK_NO_SOURCE        = 0x0000,
    CLOCK_EXTERNAL         = 0x0001,
    CLOCK_CRYSTAL          = 0x0002,

#if defined (LIBOHIBOARD_NXP_KINETIS)

	CLOCK_INTERNAL,

#if defined (LIBOHIBOARD_KV31F12)    || \
    defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

    CLOCK_CRYSTAL_32K,
    CLOCK_INTERNAL_48M

#endif // LIBOHIBOARD_KV31F12 || LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F

// END IF: LIBOHIBOARD_NXP_KINETIS
#elif defined (LIBOHIBOARD_ST_STM32)

#if defined (LIBOHIBOARD_STM32L476)

    CLOCK_INTERNAL_32K     = 0x0004,
    CLOCK_INTERNAL_16M     = 0x0008,
    CLOCK_INTERNAL_MSI     = 0x0010,
    CLOCK_INTERNAL_PLL     = 0x0020,

#endif // LIBOHIBOARD_STM32L476

#if defined (LIBOHIBOARD_STM32L496)

    CLOCK_INTERNAL_32K,
    CLOCK_INTERNAL_16M,
    CLOCK_INTERNAL_48M,
    CLOCK_INTERNAL_MSI,

#endif // LIBOHIBOARD_STM32L476

// END IF: LIBOHIBOARD_ST_STM32
#endif

} Clock_Origin;

#if defined (LIBOHIBOARD_NXP_KINETIS)
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

Clock_State Clock_getCurrentState();
#endif


#if defined (LIBOHIBOARD_ST_STM32)
typedef enum
{
    CLOCK_OSCILLATORSTATE_OFF,
    CLOCK_OSCILLATORSTATE_ON,
} Clock_OscillatorState;
#endif

typedef struct _Clock_Config
{
    Clock_Origin source;

    uint32_t fext;
    uint32_t foutSys;

#if defined (LIBOHIBOARD_NXP_KINETIS)

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

// END IF: LIBOHIBOARD_NXP_KINETIS
#elif defined (LIBOHIBOARD_ST_STM32)

    Clock_OscillatorState hseState;
    Clock_OscillatorState hsiState;

// END IF: LIBOHIBOARD_ST_STM32
#endif

} Clock_Config;


System_Errors Clock_init (Clock_Config *config);
System_Errors Clock_setDividers(uint8_t busDivider, uint8_t flexbusDivider, uint8_t flashDivider);

uint32_t Clock_getFrequency (Clock_Source source);

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

#endif // __CLOCK_H
