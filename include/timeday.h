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
 * @file libohiboard/include/timeday.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Time manage functions declarations
 */

#ifndef __TIMEDAY_H
#define __TIMEDAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"

#define TIME_UNIX_YEAR       1970
#define TIME_UNIX_YEAR_LEAP  1968

#define TIME_SECOND_PER_DAY  86400
#define TIME_SECOND_PER_HOUR 3600

typedef struct
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} Time_TimeType;

typedef struct
{
    uint8_t  wday;
    uint8_t  day;
    uint8_t  month;
    uint16_t year;
} Time_DateType;

typedef uint32_t Time_UnixTime;

Time_UnixTime Time_getUnixTime (Time_DateType* date, Time_TimeType* time);

void Time_unixtimeToTime (Time_UnixTime unix, Time_DateType* date, Time_TimeType* time);
void Time_unixtimeToString (Time_UnixTime unix, char * dateString);
void Time_unixtimeToNumberString (Time_UnixTime unix, char * dateString, bool second);

#ifdef __cplusplus
}
#endif

#endif // __TIMEDAY_H


