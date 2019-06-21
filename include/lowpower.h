/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Leonardo Morichelli <leonardo.morichelli@live.com>
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
 * @file libohiboard/include/lowpower.h
 * @author Leonardo Morichelli <leonardo.morichelli@live.com>
 * @brief Low Power definitions and prototypes.
 *
 * This file supply a set of function for using various low power mode in microcontrollers.
 */
#ifdef LIBOHIBOARD_LOWPOWER

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

/**
 * @defgroup LOWPOWER Low Power
 * @brief Low power HAL driver
 * @{
 */

#ifndef __LOWPOWER_H
#define __LOWPOWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "utility.h"
#include "clock.h"

#if (defined (LIBOHIBOARD_STM32L0) || defined (LIBOHIBOARD_STM32L4))

/**
 *
 */
typedef enum _LowPower_WaitFor
{
	LOWPOWER_WAITFOR_INTERRUPT = 1,
	LOWPOWER_WAITFOR_EVENT     = 2,
} LowPower_WaitFor;

#endif // (defined (LIBOHIBOARD_STM32L0) || defined (LIBOHIBOARD_STM32L4))

/**
 *
 */
typedef enum _LowPower_Regulator
{
	LOWPOWER_REGULATOR_MAIN = 1,
	LOWPOWER_REGULATOR_LOW  = 2,
} LowPower_Regulator;

/**
 *
 */
typedef enum _LowPower_VoltageScaling
{
	LOWPOWER_VOLTAGESCALING_SCALE1 = 1,
	LOWPOWER_VOLTAGESCALING_SCALE2 = 2,
} LowPower_VoltageScaling;

#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/STM32L4/lowpower_STM32L4.h"

#endif // LIBOHIBOARD_STM32L4

#if defined (LIBOHIBOARD_PIC24FJ)

#include "hardware/PIC24FJ/lowpower_PIC24FJ.h"

#endif

/**
 * Initialize the low power control:
 * @li read the reset status flag and reset the status flags;
 * @li enable clock of PWR.
 *
 *
 * STM32L4: initial power state is RUN_MODE, the most performance one.
 *
 * @note This function must be called at the start of system.
 */
void LowPower_init(void);

/**
 * Return the reset status bits
 *
 * @return Reset status bits
 */
LowPower_ResetControl LowPower_getResetStatus(void);

#if (defined (LIBOHIBOARD_STM32L0) || defined (LIBOHIBOARD_STM32L4))

/**
 * Enable the WakeUp PINx functionality.
 *
 * @param[in] pins Specifies which wake up pin to enable.
 * @param[in] polarity Specifies the polarity for each pin,
 *                     @ref LOWPOWER_WAKEUPEDGE_RISING is unique for all pins.
 *
 * @note For STM32L476xx the wake up lines corresponding to:
 *       @li WKUP1 == PA0
 *       @li WKUP2 == PC13
 *       @li WKUP3 == PE6
 *       @li WKUP4 == PA2
 *       @li WKUP5 == PC5
 */
void LowPower_enableWakeUpPin (LowPower_WakeUpPins pins, LowPower_WakeUpEdge polarity);

/**
 * Disable the WakeUp PINx functionality.
 *
 * @param[in] pins specifies which wake up pin to enable.
 */
void LowPower_disableWakeUpPin (LowPower_WakeUpPins pins);

/**
 * Return wake-up pins flags
 *
 * @return Status flags corresponding to wake up pins.
 */
uint32_t LowPower_getWakeUpflags (void);

/**
 * Clear wake-up flags.
 *
 * @param[in] flags flags to clear, the @ref LowPower_WakeUpPins value can be used.
 */
void LowPower_clearWakeUpflags (uint32_t flags);

#endif // (defined (LIBOHIBOARD_STM32L0) || defined (LIBOHIBOARD_STM32L4))

/**
 * Set the frequency of CPU and the low-power mode.
 *
 * @param[in] frequency The CPU frequency for next mode
 * @param[in] mode The new low-power mode
 * @return System_Error
 */
System_Errors LowPower_setModeByFrequency (uint32_t frequency, LowPower_Mode mode);

/**
 * Set the clock configuration and the low power mode.
 *
 * @param[in] config The clock configuration
 * @param[in] mode The new low power mode
 * @return System_Error
 */
System_Errors LowPower_setModeByConfiguration (Clock_Config* config, LowPower_Mode mode);

/**
 * Return the current mode.
 *
 * @return current low-power mode.
 *
 * @note the returned value is in MCU-specific enumeration;  please compare with the comfort macros for a portable usage.
 */
LowPower_Mode LowPower_getMode(void);

#ifdef __cplusplus
}
#endif

#endif // __LOWPOWER_H

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_LOWPOWER
