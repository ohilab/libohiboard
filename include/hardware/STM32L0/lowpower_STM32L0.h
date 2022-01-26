/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Niccolò Paolinelli <ai03@hotmail.it>
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
 * @file libohiboard/include/hardware/STM32L0/lowpower_STM32L0.h
 * @author Niccolò Paolinelli <ai03@hotmail.it>
 * @brief Low Power useful definitions for STM32L0 series
 */

#ifndef __LOWPOWER_STM32L0_H
#define __LOWPOWER_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_LOWPOWER) && defined (LIBOHIBOARD_STM32L0)

/**
 * @addtogroup LOWPOWER
 * @{
 */

/**
 * @defgroup LOWPOWER_Hardware Low Power specific hardware types
 * @{
 */

/**
 * @warning dummy values/enumeration
 */
typedef enum _LowPower_WakeUpPins
{
    LOWPOWER_WAKEUPPINS_PIN1 = PWR_CSR_EWUP1,
    LOWPOWER_WAKEUPPINS_PIN2 = PWR_CSR_EWUP2,
    LOWPOWER_WAKEUPPINS_PIN3 = PWR_CSR_EWUP3,
} LowPower_WakeUpPins;

/**
 * @warning dummy values/enumeration
 */
typedef enum _LowPower_WakeUpEdge
{
    LOWPOWER_WAKEUPEDGE_RISING       = 0u,
    LOWPOWER_WAKEUPEDGE_FALLING_PIN1,
    LOWPOWER_WAKEUPEDGE_FALLING_PIN2,
    LOWPOWER_WAKEUPEDGE_FALLING_PIN3
} LowPower_WakeUpEdge;

/** Number of possible power modes. */
#define LOWPOWER_MODE_NUMBER           6

/**
 * List of all possible Low-Power modes for this kind of microcontroller.
 * @warning dummy values/enumeration
 */
typedef enum _LowPower_Mode
{
    LOWPOWER_MODE_RUN,
    LOWPOWER_MODE_LPRUN,
    LOWPOWER_MODE_SLEEP,
    LOWPOWER_MODE_LPSLEEP,
    LOWPOWER_MODE_STOP,
    LOWPOWER_MODE_STANDBY,
} LowPower_Mode;

/**
 * List of possible clock source to set after Wake Up from Stop mode
 */
typedef enum _LowPower_WakeUpClock
{
    LOWPOWER_WAKEUPCLOCK_MSI = 0,
    LOWPOWER_WAKEUPCLOCK_HSI,
} LowPower_WakeUpClock;

/**
 * @warning dummy values/enumeration
 */
typedef struct _LowPower_ResetFlags
{
    uint32_t pwrWuf                      : 1;  // PWR_CSR_WUF
    uint32_t pwrStandby                  : 1;  // PWR_CSR_SBF

    uint32_t rccLowPowerReset            : 1;  // RCC_CSR_LPWRRSTF
    uint32_t rccWatchdogReset            : 1;  // RCC_CSR_WWDGRSTF
    uint32_t rccIndependentWatchdogReset : 1;  // RCC_CSR_IWDGRSTF
    uint32_t rccSoftwareReset            : 1;  // RCC_CSR_SFTRSTF
    uint32_t rccPOR                      : 1;  // RCC_CSR_PORRSTF
    uint32_t rccPinReset                 : 1;  // RCC_CSR_PINRSTF
    uint32_t rccOptionbyteLoaderReset    : 1;  // RCC_CSR_OBLRSTF
    uint32_t rccFirewallReset            : 1;  // RCC_CSR_FWRSTF

    uint32_t                             : 22; // Not used
} LowPower_ResetFlags;

/**
 * @warning dummy values/enumeration
 */
typedef union _LowPower_ResetControl
{
    uint32_t value;
    LowPower_ResetFlags flags;
} LowPower_ResetControl;

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_LOWPOWER & LIBOHIBOARD_STM32L0

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __LOWPOWER_STM32L0_H
