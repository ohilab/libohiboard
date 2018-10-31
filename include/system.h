/*
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

#define LIBOHIBOARD_VERSION_MAJOR         (0x2u)
#define LIBOHIBOARD_VERSION_MINOR         (0x0u)
#define LIBOHIBOARD_VERSION_BUG           (0x0u)
#define LIBOHIBOARD_VERSION               ((LIBOHIBOARD_VERSION_MAJOR << 16)\
                                          |(LIBOHIBOARD_VERSION_MINOR << 8 )\
                                          |(LIBOHIBOARD_VERSION_BUG        ))

/**
 *
 */
System_Errors System_controlDevice (void);

#if (LIBOHIBOARD_VERSION >= 0x20000u)

/**
 * Initialize and start the System Tick with interrupt enabled
 * Counter is in free running mode to generate periodic interrupts.
 *
 * @note This function is called automatically from @ref System_sysickInit
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
 * @param[in] priority Tick interrupt priority
 * @return ERRORS_NO_ERROR when function succeeded.
 * @return ERRORS_SYSTEM_NO_CLOCK when function failed because the lock source is zero.
 */
System_Errors System_sysickInit(uint32_t priority);

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
 * Interrupt handler for SysTick interrupt.
 */
void SysTick_Handler(void);

#endif // LIBOHIBOARD_VERSION >= 0x20000

#ifdef __cplusplus
}
#endif

#endif // __SYSTEM_H
