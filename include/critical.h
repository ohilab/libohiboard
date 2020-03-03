/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 *  Stefano Gigli
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
 * @file libohiboard/include/critical.h
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @author Stefano Gigli
 * @brief Critical section definitions and prototypes.
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_CRITICAL

/**
 * @defgroup CRITICAL Critical Section
 * @brief CRITICAL Critical Section
 * @{
 */

#ifndef __CRITICAL_H
#define __CRITICAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "system.h"

/**
 * Begins critical section.
 *
 * @note For ARM Cortex M3/M4 device:
 *       from this <a href="http://infocenter.arm.com/help/topic/com.arm.doc.dui0553b/DUI0553.pdf">document</a> , section 2.1.3: Core Register
 *       The BASEPRI (Base Priority Mask Register) defines the minimum priority for exception processing.
 *       When BASEPRI is set to a nonzero value, it prevents the activation of all exceptions
 *       with the same or lower priority level as the BASEPRI.
 *       Priority mask bits:
 *       @li 0x00, no effect
 *       @li Nonzero, defines the base priority for exception processing.
 *           The processor does not process any exception with a priority
 *           value greater than or equal to BASEPRI
 *       Remember that higher priority field values correspond to lower exception priorities.
 *
 * @note For ARM Cortex M0/M0+: the behavior is different, and disable all interrupt, systick included.
 *
 * @param[in] mask: Pointer to a variable where to store the CPU IRQ mask
 */
void Critical_sectionBegin (uint32_t* mask);

/**
 * Ends critical section.
 *
 * @param[in] mask: Pointer to a variable where the CPU IRQ mask was stored
 */
void Critical_sectionEnd (uint32_t* mask);

#if defined (LIBOHIBOARD_PIC24FJ)
#include "hardware/PIC24FJ/critical_PIC24FJ.h"

/**
 * Max Value for priority interrupt
 */
#ifndef CRITICAL_MAX_PRIORITY
#define CRITICAL_MAX_PRIORITY                    7
#endif

/**
 * Define the default value for priority of timer
 */
#ifndef CRITICAL_DEFAULT_TICK_PRIO
#define CRITICAL_DEFAULT_TICK_PRIO               7
#endif

/**
 * Begins critical section
 */
#define CRITICAL_SECTION_BEGIN()                 {uint16_t ipl = Critical_getCpuIpl(); Critical_setCpuIpl(CRITICAL_MAX_PRIORITY);

/**
 * Ends critical section
 */
#define CRITICAL_SECTION_END()                   Critical_setCpuIpl(ipl);}

#elif defined (LIBOHIBOARD_STM32L4)

#include "hardware/STM32L4/critical_STM32L4.h"

/**
 * Max Value for priority interrupt
 */
#ifndef CRITICAL_MAX_PRIORITY
#define CRITICAL_MAX_PRIORITY                    0
#endif

/**
 * Define the default value for priority of timer
 */
#ifndef CRITICAL_DEFAULT_TICK_PRIO
#define CRITICAL_DEFAULT_TICK_PRIO               1
#endif

/**
 * Begins critical section
 */
#define CRITICAL_SECTION_BEGIN()                 {uint32_t mask = 0; Critical_sectionBegin(&mask);{}

/**
 * Ends critical section
 */
#define CRITICAL_SECTION_END()                   Critical_sectionEnd(&mask); }

#elif defined (LIBOHIBOARD_STM32L0)

#include "hardware/STM32L0/critical_STM32L0.h"

/**
 * Max Value for priority interrupt
 */
#ifndef CRITICAL_MAX_PRIORITY
#define CRITICAL_MAX_PRIORITY                    0
#endif

/**
 * Define the default value for priority of timer
 */
#ifndef CRITICAL_DEFAULT_TICK_PRIO
#define CRITICAL_DEFAULT_TICK_PRIO               1
#endif

/**
 * Begins critical section
 */
#define CRITICAL_SECTION_BEGIN()                 {uint32_t mask = 0; Critical_sectionBegin(&mask);{}

/**
 * Ends critical section
 */
#define CRITICAL_SECTION_END()                   Critical_sectionEnd(&mask); }

#endif

#ifdef __cplusplus
}
#endif

#endif // __CRITICAL_H

/**
 * @}
 */

#endif // LIBOHIBOARD_CRITICAL

/**
 * @}
 */
