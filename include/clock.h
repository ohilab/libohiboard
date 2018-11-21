/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2018 A. C. Open Hardware Ideas Lab
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

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"

#if defined (LIBOHIBOARD_STM32L476)

#define CLOCK_MIN_FREQ_HSE                     4000000u
#define CLOCK_MAX_FREQ_HSE                    48000000u
#define CLOCK_FREQ_HSI                        16000000u
#define CLOCK_FREQ_LSE                ((uint32_t)32768u)
#define CLOCK_FREQ_LSI                ((uint32_t)32000u)

#endif

typedef enum _Clock_Source
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

typedef enum _Clock_Origin
{

    CLOCK_NO_SOURCE            = 0x0000,
    CLOCK_EXTERNAL             = 0x0001,
    CLOCK_CRYSTAL              = 0x0002,

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

    CLOCK_INTERNAL_LSI         = 0x0004,
    CLOCK_INTERNAL_HSI         = 0x0008,
    CLOCK_INTERNAL_MSI         = 0x0010,
    CLOCK_INTERNAL_PLL         = 0x0020,
    CLOCK_EXTERNAL_LSE_CRYSTAL = 0x0040,

#endif // LIBOHIBOARD_STM32L476

// END IF: LIBOHIBOARD_ST_STM32
#endif

} Clock_Origin;

#if defined (LIBOHIBOARD_NXP_KINETIS)

typedef enum _Clock_State
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

typedef enum _Clock_OscillatorState
{
    CLOCK_OSCILLATORSTATE_OFF,
    CLOCK_OSCILLATORSTATE_ON,
} Clock_OscillatorState;

typedef enum _Clock_SystemSource
{
    CLOCK_SYSTEMSOURCE_HSE = 0x0001,
    CLOCK_SYSTEMSOURCE_MSI = 0x0002,
    CLOCK_SYSTEMSOURCE_HSI = 0x0004,
    CLOCK_SYSTEMSOURCE_PLL = 0x0008,
} Clock_SystemSource;

typedef enum _Clock_Output
{
    CLOCK_OUTPUT_SYSCLK = 0x0001,
    CLOCK_OUTPUT_HCLK   = 0x0002,
    CLOCK_OUTPUT_PCLK1  = 0x0004,
    CLOCK_OUTPUT_PCLK2  = 0x0008,
} Clock_Output;

typedef enum _Clock_AHBDivider
{
    CLOCK_AHBDIVIDER_1    = 0,
    CLOCK_AHBDIVIDER_2    = 1,
    CLOCK_AHBDIVIDER_4    = 2,
    CLOCK_AHBDIVIDER_8    = 3,
    CLOCK_AHBDIVIDER_16   = 4,
    CLOCK_AHBDIVIDER_64   = 5,
    CLOCK_AHBDIVIDER_128  = 6,
    CLOCK_AHBDIVIDER_256  = 7,
    CLOCK_AHBDIVIDER_512  = 8,

} Clock_AHBDivider;

typedef enum _Clock_APBDivider
{
    CLOCK_APBDIVIDER_1    = 0,
    CLOCK_APBDIVIDER_2    = 1,
    CLOCK_APBDIVIDER_4    = 2,
    CLOCK_APBDIVIDER_8    = 3,
    CLOCK_APBDIVIDER_16   = 4,

} Clock_APBDivider;

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
    Clock_OscillatorState lsiState;
    Clock_OscillatorState lseState;

    Clock_SystemSource sysSource;

    Clock_Output output;

    Clock_AHBDivider ahbDivider;
    Clock_APBDivider apb1Divider;
    Clock_APBDivider apb2Divider;

// END IF: LIBOHIBOARD_ST_STM32
#endif

} Clock_Config;

// Useful define
#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/clock_STM32L4.h"

#endif


System_Errors Clock_init (Clock_Config *config);

#if defined (LIBOHIBOARD_NXP_KINETIS)

System_Errors Clock_setDividers (uint8_t busDivider, uint8_t flexbusDivider, uint8_t flashDivider);

#elif defined (LIBOHIBOARD_ST_STM32)

System_Errors Clock_setDividers (uint32_t ahbDivider, uint32_t apb1Divider, uint32_t apb2Divider);

/**
 * Return the selected output clock.
 *
 * @param[in] output The selected output clock
 * @return The clock frequency in Hz
 */
uint32_t Clock_getOutputValue (Clock_Output output);

#endif

/**
 * Return the selected oscillator clock value.
 *
 * @param[in] source The selected oscillator value
 * @return The clock frequency in Hz
 */
uint32_t Clock_getOscillatorValue (Clock_Source source);

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

#ifdef __cplusplus
}
#endif

#endif // __CLOCK_H
