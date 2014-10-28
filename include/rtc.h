/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Marco Giammarini <m.giammarini@warcomeb.it>
 *	
 * Project: libohiboard
 * Package: RTC
 * Version: 0.0
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>
 ******************************************************************************/

/**
 * @file libohiboard/include/rtc.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief RTC definitions and prototypes.
 */

#ifndef __RTC_H
#define __RTC_H

#include "platforms.h"

#if !defined (FRDMKL02Z) && !defined(MKL02Z4)

#include "errors.h"
#include "types.h"
#include "interrupt.h"

typedef enum {
    RTC_SYSTEM_OSCILLATOR,
    RTC_CLKIN,
    RTC_LPO_1kHz
} Rtc_ClockSource;

typedef uint32_t Rtc_Time;

typedef struct Rtc_Device* Rtc_DeviceHandle;

System_Errors Rtc_init (Rtc_DeviceHandle dev);

System_Errors Rtc_setClockSource (Rtc_DeviceHandle dev, Rtc_ClockSource clock);

void Rtc_setTime (Rtc_DeviceHandle dev, Rtc_Time time);
Rtc_Time Rtc_getTime (Rtc_DeviceHandle dev);

void Rtc_enableAlarm (Rtc_DeviceHandle dev, Rtc_Time alarm, Interrupt_Status irqEnable);
void Rtc_disableAlarm (Rtc_DeviceHandle dev, Interrupt_Status irqEnable);

void Rtc_enableSecond (Rtc_DeviceHandle dev);
void Rtc_disableSecond (Rtc_DeviceHandle dev);

extern Rtc_DeviceHandle RTC0;

#endif

#endif /* __RTC_H */
