/******************************************************************************
 * Copyright (C) 2012-2015 A. C. Open Hardware Ideas Lab
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
 ******************************************************************************/

/**
 * @file libohiboard/include/rtc.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief RTC definitions and prototypes.
 */

//#ifdef LIBOHIBOARD_RTC

#include "platforms.h"

#ifndef __RTC_H
#define __RTC_H

#include "errors.h"
#include "types.h"
#include "interrupt.h"

typedef enum {
    RTC_SYSTEM_OSCILLATOR,
    RTC_CLKIN,
    RTC_LPO_1kHz
} Rtc_ClockSource;

typedef struct _Rtc_Config
{
    Rtc_ClockSource clockSource;

} Rtc_Config;

typedef uint32_t Rtc_Time;

typedef struct Rtc_Device* Rtc_DeviceHandle;

System_Errors Rtc_init (Rtc_DeviceHandle dev, Rtc_Config *config);

void Rtc_setTime (Rtc_DeviceHandle dev, Rtc_Time time);
Rtc_Time Rtc_getTime (Rtc_DeviceHandle dev);

void Rtc_enableAlarm (Rtc_DeviceHandle dev, Rtc_Time alarm);
void Rtc_disableAlarm (Rtc_DeviceHandle dev);

void Rtc_enableSecond (Rtc_DeviceHandle dev);
void Rtc_disableSecond (Rtc_DeviceHandle dev);

#if defined(LIBOHIBOARD_KL15Z4)

extern Rtc_DeviceHandle RTC0;

#endif

#endif /* __RTC_H */

//#endif /* LIBOHIBOARD_RTC */
