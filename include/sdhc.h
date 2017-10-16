/******************************************************************************
 * Copyright (C) 2017 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/include/sdhc.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief SDHC definitions and prototypes.
 */

#ifdef LIBOHIBOARD_SDHC

#ifndef __SDHC_H
#define __SDHC_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum {
    SDHC_DATAWIDTH_1BIT,
    SDHC_DATAWIDTH_4BIT,
    SDHC_DATAWIDTH_8BIT
} Sdhc_DataWidth;


typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTE1,

#endif

    SDHC_PINS_D1NONE,

} Sdhc_D0Pins;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTE0,

#endif

    SDHC_PINS_D1NONE,

} Sdhc_D1Pins;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTE5,

#endif

    SDHC_PINS_D2NONE,

} Sdhc_D2Pins;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTE4,

#endif

    SDHC_PINS_D3NONE,

} Sdhc_D3Pins;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTE3,

#endif

    SDHC_PINS_CMDNONE,

} Sdhc_CmdPins;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTE2,

#endif

    SDHC_PINS_CLOCKNONE,

} Sdhc_ClockPins;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTD11,

#endif

    SDHC_PINS_CLKINNONE,

} Sdhc_ClockInPins;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTD12,

#endif

    SDHC_PINS_D4NONE,

} Sdhc_D4Pins;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTD13,

#endif

    SDHC_PINS_D5NONE,

} Sdhc_D5Pins;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTD14,

#endif

    SDHC_PINS_D6NONE,

} Sdhc_D6Pins;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    SDHC_PINS_PTD15,

#endif

    SDHC_PINS_D7NONE,

} Sdhc_D7Pins;

typedef struct Sdhc_Device* Sdhc_DeviceHandle;

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

extern Sdhc_DeviceHandle OB_SDHC0;

#endif

typedef struct _Sdhc_Config
{
#if !defined (LIBOHIBOARD_K64F12)    && \
    !defined (LIBOHIBOARD_FRDMK64F)

    Sdhc_ClockPins clockPins;
    Sdhc_CmdPins cmdPins;
    Sdhc_D0Pins d0Pins;
    Sdhc_D0Pins d1Pins;
    Sdhc_D1Pins d2Pins;
    Sdhc_D2Pins d3Pins;
    Sdhc_D3Pins d4Pins;
    Sdhc_D4Pins d5Pins;
    Sdhc_D5Pins d6Pins;
    Sdhc_D6Pins d7Pins;
    Sdhc_ClockInPins clkinPins;

#endif

    Sdhc_DataWidth dataWidth;

    void (*cardRemoved)(void);
    void (*cardInserted)(void);

} Sdhc_Config;

/**
 * This function initialize the SDHC module
 *
 * @param dev The device handle
 * @retval ERRORS_NO_ERROR No problem during module initialization
 */
System_Errors Sdhc_init (Sdhc_DeviceHandle dev, Sdhc_Config* config);

/**
 * This function de-initialize SDHC module
 *
 * @param dev The device handle
 * @retval ERRORS_NO_ERROR No problem during module de-initialization
 */
System_Errors Sdhc_init (Sdhc_DeviceHandle dev);

#endif /* __FLASH_H */

#endif /* LIBOHIBOARD_FLASH */
