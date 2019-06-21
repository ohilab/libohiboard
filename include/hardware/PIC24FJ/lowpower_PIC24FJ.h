/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/include/hardware/PIC24FJ/lowpower_PIC24FJ.h
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief Low-Power useful definitions for PIC24FJ
 */

#ifndef __LOWPOWER_PIC24FJ_H
#define __LOWPOWER_PIC24FJ_H

#include "platforms.h"

#if defined(LIBOHIBOARD_LOWPOWER)

#ifdef __cplusplus
extern "C" {
#endif

#if defined (LIBOHIBOARD_PIC24FJ)

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */



/**
 * @addtogroup LOWPOWER
 * @{
 */

/**
 * @defgroup LOWPOWER_Hardware Low Power specific hardware types
 * @{
 */

/** Number of possible power modes. */
#define LOWPOWER_MODE_NUMBER           3

/**
 * List of all possible Low-Power modes for this kind of microcontroller.
 */
typedef enum _LowPower_Mode
{
    LOWPOWER_MODE_RUN,
    LOWPOWER_MODE_IDLE,
    LOWPOWER_MODE_SLEEP,
} LowPower_Mode;

/**
 *
 */
typedef struct _LowPower_ResetFlags
{
    uint16_t POR                         : 1; //Power-on Reset Flag bit
    uint16_t BOR                         : 1; //Brown-out Reset Flag bit
    uint16_t IDLE                        : 1; //Wake-up from Idle Flag bit
    uint16_t SLEEP                       : 1; //Wake from Sleep Flag bit
    uint16_t WDTO                        : 1; //Watchdog Timer Time-out Flag bit
    uint16_t SWDTEN                      : 1; //Software Enable/Disable of WDT bit
    uint16_t SWR                         : 1; //Software Reset
    uint16_t EXTR                        : 1; //External Reset
    uint16_t VREGS                       : 1; //Fast Wake-up from Sleep bit
    uint16_t CM                          : 1; //Configuration Word Mismatch Reset Flag bit
    uint16_t                             : 2;
    uint16_t RETEN                       : 1; //Retention Mode Enable bit
    uint16_t SBOREN                      : 1; //Software Enable/Disable of BOR bit
    uint16_t IOPUWR                      : 1; //Illegal Opcode or Uninitialized W Access Reset Flag bit(
    uint16_t TRAPR                       : 1; //Trap Reset Flag bit
} LowPower_ResetFlags;

/**
 *
 */
typedef union _LowPower_ResetControl
{
	uint16_t value;
	LowPower_ResetFlags flags;
} LowPower_ResetControl;

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_LOWPOWER

#endif // __LOWPOWER_PIC24FJ_H
