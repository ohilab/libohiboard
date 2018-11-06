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
 * @file libohiboard/include/rtc.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief RTC definitions and prototypes.
 */

#ifdef LIBOHIBOARD_RTC

#ifndef __RTC_H
#define __RTC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "interrupt.h"
#include "gpio.h"

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Rtc_DeviceState
{
    RTC_DEVICESTATE_RESET,
    RTC_DEVICESTATE_READY,
    RTC_DEVICESTATE_BUSY,
    RTC_DEVICESTATE_ERROR,

} Rtc_DeviceState;

typedef enum _Rtc_ClockSource
{
#if defined (LIBOHIBOARD_NXP_KINETIS)

    RTC_CLOCK_SYSTEM,
    RTC_CLOCK_CLKIN,
    RTC_CLOCK_LPO

#elif defined (LIBOHIBOARD_ST_STM32)

    RTC_CLOCK_LSE     = 0x1u,
    RTC_CLOCK_LSI     = 0x2u,
    RTC_CLOCK_HSE_RTC = 0x3u,

#endif

} Rtc_ClockSource;

#if defined (LIBOHIBOARD_ST_STM32)

typedef enum _Rtc_HourFormat
{
    RTC_HOURFORMAT_12H,
    RTC_HOURFORMAT_24H,

} Rtc_HourFormat;

typedef enum _Rtc_OutputMode
{
    RTC_OUTPUTMODE_DISABLE  = 0x0u,
    RTC_OUTPUTMODE_ALARM_A  = 0x1u,
    RTC_OUTPUTMODE_ALARM_B  = 0x2u,
    RTC_OUTPUTMODE_WAKEUP   = 0x3u,

} Rtc_OutputMode;

typedef enum _Rtc_OutputType
{
    RTC_OUTPUTTYPE_OPEN_DRAIN  = 0u,
    RTC_OUTPUTTYPE_PUSH_PULL   = 1u,

} Rtc_OutputType;

typedef enum _Rtc_OutputRemap
{
    RTC_OUTPUTREMAP_DEFAULT  = 0u,
    RTC_OUTPUTREMAP_REMAP    = 1u,

} Rtc_OutputRemap;

#endif // LIBOHIBOARD_ST_STM32

#if (LIBOHIBOARD_VERSION >= 0x20000u)
typedef struct _Rtc_Device* Rtc_DeviceHandle;
#else
typedef struct Rtc_Device* Rtc_DeviceHandle;
#endif

#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/rtc_STM32L4.h"

#else

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

extern Rtc_DeviceHandle RTC0;

#elif defined (LIBOHIBOARD_KL15Z4)    || \
      defined (LIBOHIBOARD_KL25Z4)    || \
      defined (LIBOHIBOARD_FRDMKL25Z)

void RTC_IRQHandler (void);
void RTC_Seconds_IRQHandler (void);

extern Rtc_DeviceHandle OB_RTC0;

#endif

#endif

typedef struct _Rtc_Config
{
    Rtc_ClockSource clockSource;

    uint32_t alarm;
    void (*callbackAlarm)(void);    /**< The pointer for user alarm callback. */

    void (*callbackSecond)(void);  /**< The pointer for user second callback. */

#if defined (LIBOHIBOARD_ST_STM32)

    Rtc_HourFormat hourFormat;

    Rtc_OutputMode outputMode;
    Rtc_OutputType outputType;
    Rtc_OutputRemap outputRemap;
    Gpio_Level outputPolarity;

#endif

} Rtc_Config;

typedef uint32_t Rtc_Time;

/** @name RTC Configuration functions
 *  Functions to initialize and de-initialize the  peripheral.
 */
///@{

/**
 * This function initialize the selected peripheral, with the specified parameters.
 *
 * @param[in] dev Rtc device handle
 * @param[in] config Configuration parameters for the Rtc
 */
System_Errors Rtc_init (Rtc_DeviceHandle dev, Rtc_Config *config);

/**
 * This function de-initialize the selected peripheral.
 *
 * @param[in] dev Rtc device handle
 */
System_Errors Rtc_deInit (Rtc_DeviceHandle dev);

///@}

/** @name RTC Time functions
 *  Functions to write and read current RTC time.
 */
///@{

/**
 * This function set the current value of time.
 *
 * @param[in] dev Rtc device handle
 * @param[in] time The current time in unix format
 * @return
 */
System_Errors Rtc_setTime (Rtc_DeviceHandle dev, Rtc_Time time);

/**
 * This function return the current value of time.
 *
 * @param[in] dev Rtc device handle
 * @return Current RTC time in unix timestamp format
 */
Rtc_Time Rtc_getTime (Rtc_DeviceHandle dev);

///@}

void Rtc_enableAlarm (Rtc_DeviceHandle dev, void *callback, Rtc_Time alarm);
void Rtc_disableAlarm (Rtc_DeviceHandle dev);

void Rtc_enableSecond (Rtc_DeviceHandle dev, void *callback);
void Rtc_disableSecond (Rtc_DeviceHandle dev);

#ifdef __cplusplus
}
#endif

#endif // __RTC_H

#endif // LIBOHIBOARD_RTC
