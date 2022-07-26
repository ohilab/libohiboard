/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2019 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Leonardo Morichelli
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

typedef enum _Time_Month
{
    TIME_MONTH_JANUARY = 1,
    TIME_MONTH_FEBRUARY,
    TIME_MONTH_MARCH,
    TIME_MONTH_APRIL,
    TIME_MONTH_MAY,
    TIME_MONTH_JUNE,
    TIME_MONTH_JULY,
    TIME_MONTH_AUGUST,
    TIME_MONTH_SEPTEMBER,
    TIME_MONTH_OCTOBER,
    TIME_MONTH_NOVEMBER,
    TIME_MONTH_DECEMBER,
} __packed Time_Month;

typedef enum _Time_DayOfWeek
{
    TIME_DAYOFWEEK_SUNDAY = 0,
    TIME_DAYOFWEEK_MONDAY,
    TIME_DAYOFWEEK_TUESDAY,
    TIME_DAYOFWEEK_WEDNESDAY,
    TIME_DAYOFWEEK_THURSDAY,
    TIME_DAYOFWEEK_FRIDAY,
    TIME_DAYOFWEEK_SATURDAY,
} __packed Time_DayOfWeek;

typedef struct
{
    // hours since midnight - [0, 23] 
    uint8_t hours;
    // minutes after the hour - [0, 59] 
    uint8_t minutes;
    // seconds after the minute - [0, 59]
    uint8_t seconds;
} Time_TimeType;

typedef struct
{
    // days since Sunday - [0, 6]
    Time_DayOfWeek  wday;
    // day of the month - [1, 31] 
    uint8_t  day;
    // months since January - [1, 12]
    Time_Month  month;
    // years since 0 
    uint16_t year;
} Time_DateType;

typedef uint32_t Time_UnixTime;

bool Time_isValid (Time_DateType* date, Time_TimeType* time);
Time_UnixTime Time_getUnixTime (const Time_DateType* date, const Time_TimeType* time);

void Time_unixtimeToTime (Time_UnixTime unixEpoch, Time_DateType* date, Time_TimeType* time);
void Time_unixtimeToString (Time_UnixTime unixEpoch, char * dateString);
void Time_unixtimeToNumberString (Time_UnixTime unixEpoch, char * dateString, bool second);

/*!
 *
 */
static inline void Time_increaseDay (Time_DateType* date)
{
    date->day++;

    if (((date->month == TIME_MONTH_APRIL) ||
        (date->month == TIME_MONTH_JUNE)  ||
        (date->month == TIME_MONTH_SEPTEMBER) ||
        (date->month == TIME_MONTH_NOVEMBER)) && (date->day > 30))
    {
        date->day = 30;
        return;
    }

    if (date->month == TIME_MONTH_FEBRUARY)
    {
        if (!(date->year % 4) && (date->day > 29)) date->day = 29;
        else if ((date->year % 4) && (date->day > 28)) date->day = 28;

        return;
    }

    if (date->day > 31) date->day = 30;
}

/*!
 *
 */
static inline void Time_decreaseDay (Time_DateType* date)
{
    if (date->day > 1) date->day--;
}

static inline void Time_increaseMonth (Time_DateType* date)
{
    uint8_t t = date->month;
    if (date->month < 12) t++;
    date->month = (Time_Month)t;
}

static inline void Time_decreaseMonth (Time_DateType* date)
{
    uint8_t t = date->month;
    if (t > 1) t--;
    date->month = (Time_Month)t;
}

static inline void Time_increaseYear (Time_DateType* date)
{
    date->year++;
}

static inline void Time_decreaseYear (Time_DateType* date)
{
    if (date->year > 1970) date->year--;
}

static inline void Time_increaseHour (Time_TimeType* time)
{
    if (time->hours < 23) time->hours++;
}

static inline void Time_decreaseHour (Time_TimeType* time)
{
    if (time->hours > 0) time->hours--;
}

static inline void Time_increaseMinute (Time_TimeType* time)
{
    if (time->minutes < 59) time->minutes++;
}

static inline void Time_decreaseMinute (Time_TimeType* time)
{
    if (time->minutes > 0) time->minutes--;
}

#ifdef __cplusplus
}
#endif

#endif // __TIMEDAY_H


