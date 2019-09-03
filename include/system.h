/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/system.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief 
 */

#ifndef __SYSTEM_H
#define __SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"

#define LIBOHIBOARD_VERSION_MAJOR         (0x2ul)
#define LIBOHIBOARD_VERSION_MINOR         (0x0ul)
#define LIBOHIBOARD_VERSION_BUG           (0x0ul)
#define LIBOHIBOARD_VERSION               ((LIBOHIBOARD_VERSION_MAJOR << 16)\
                                          |(LIBOHIBOARD_VERSION_MINOR << 8 )\
                                          |(LIBOHIBOARD_VERSION_BUG        ))

/**
 *
 */
System_Errors System_controlDevice (void);

#if (LIBOHIBOARD_VERSION >= 0x20000u)

// propostra pe poter determinmare systick period, ovvero via macros
#define LIBOHIBOARD_SYSTEM_TICK_100_MICROSEC  10000ul
#define LIBOHIBOARD_SYSTEM_TICK_1_MILLISEC     1000ul
#define LIBOHIBOARD_SYSTEM_TICK_10_MILLISEC     100ul

#ifndef LIBOHIBOARD_SYSTEM_TICK
#define LIBOHIBOARD_SYSTEM_TICK   LIBOHIBOARD_SYSTEM_TICK_1_MILLISEC
#endif




/**
 * Initialize and start the System Tick with interrupt enabled
 * Counter is in free running mode to generate periodic interrupts.
 *
 * @note This function is called automatically from @ref System_sysickInit
 * 
 * @note This function, for Microchip PIC microcontroller uses physical 32bit timer.
 *
 * @param[in] ticks Specifies the ticks number between two interrupts.
 * @return ERRORS_NO_ERROR when function succeeded.
 * @return ERRORS_SYSTEM_TICK_INIT_FAILED when function failed.
 */
System_Errors System_systickConfig (uint32_t ticks);

/**
 * This function configure the SysTick clock source, and configure the
 * timer to have 1ms time base.
 *
 * @note This function is called automatically from @ref Clock_init
 * 
 * @note This function, for Microchip PIC microcontroller uses physical 32bit timer.
 *
 * @param[in] priority Tick interrupt priority
 * @return ERRORS_NO_ERROR when function succeeded.
 * @return ERRORS_SYSTEM_NO_CLOCK when function failed because the lock source is zero.
 */
System_Errors System_systickInit (uint32_t priority);

/**
 * This function return the current tick value in millisecond
 * @return The current tick value
 */
uint32_t System_currentTick (void);

/**
 * This function provides delay in milliseconds.
 * The delay is blocking.
 * @param[in] msec The delay time length in millisecond
 */
void System_delay (uint32_t msec);

/**
 * This function provides delay in milliseconds from a given time tick previously taken.
 * The delay is blocking.
 * @param[in] msec The delay time length in millisecond
 */
void System_delayFrom (uint32_t from, uint32_t msec);

#if !defined (LIBOHIBOARD_MICROCHIP_PIC)
/**
 * Interrupt handler for SysTick interrupt.
 */
void SysTick_Handler (void);
#endif

/**
 * Suspend Tick increment.
 */
void System_suspendTick (void);

/**
 * Resume Tick increment.
 */
void System_resumeTick (void);

/*!
 * TODO
 */
void System_softwareBreakpoint (void);

/**
 * Force to be ON the configuration of MCUDBG peripheral to force clock to MCU on low power states.\n
 * MCUDBG on is generally default behaviour when debugger session is active.\n
 * This can be reset only by MCU power cycle.
 */
void System_forceEnableDebug(void);

/**
 * Force to be OFF the configuration of MCUDBG peripheral to force clock to MCU on low power states.\n
 * MCUDBG on is generally default behaviour when debugger session is active.\n
 * This can be reset only by MCU power cycle.
 */
void System_forceDisableDebug(void);

/**
 * Resume the HAL version.
 * The format is 4-byte data divided as:
 * @li 1-byte (MSB) - NOT USED
 * @li 1-byte - Major Version
 * @li 1-byte - Minor Version
 * @li 1-byte (LSB) - Bug fix
 *
 * @return An uint32_t that represent the version number.
 */
uint32_t System_getHalVersion (void);

#endif // LIBOHIBOARD_VERSION >= 0x20000

#ifdef __cplusplus
}
#endif

#endif // __SYSTEM_H
