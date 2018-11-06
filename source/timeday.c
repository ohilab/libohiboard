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
 * @file libohiboard/source/timeday.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Time manage functions
 */

#ifdef LIBOHIBOARD_TIMEDAY

#ifdef __cplusplus
extern "C" {
#endif

#include "timeday.h"
#include "utility.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_START_YEAR      1900

#define TIME_UNIX_YEAR       1970
#define TIME_UNIX_YEAR_LEAP  1968

#define TIME_SECOND_PER_DAY  86400
#define TIME_SECOND_PER_HOUR 3600

#define TIME_LEAP_YEAR(year) (!(year % 4) && ((year % 100) || !(year % 400)))
#define TIME_YEAR_SIZE(year) (TIME_LEAP_YEAR(year) ? 366 : 365)

static const uint8_t Time_dayPerMonth[2][12] =
{
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
    {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};

static const char* Time_monthString[] =
{
    "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
};

static const char* Time_dayString[] =
{
    "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun",
};

Time_UnixTime Time_getUnixTime (Time_DateType* date, Time_TimeType* time)
{
    Time_UnixTime result = 0;
    
    if (!(date->year % 4) && (date->month > 2)) result += TIME_SECOND_PER_DAY;
    date->month--;
    
    /* Save seconds for the months of the current year */
    while (date->month)
    {
        date->month--;
        result += Time_dayPerMonth[0][date->month] * TIME_SECOND_PER_DAY;
    }
    
    /* Save seconds for past years */
    result += (((date->year-TIME_UNIX_YEAR)*365) + ((date->year-TIME_UNIX_YEAR_LEAP)/4)) * (uint32_t)TIME_SECOND_PER_DAY;
    /* Save seconds for the days of the current month */
    result += (date->day-1) * (uint32_t)TIME_SECOND_PER_DAY;
    /* Save seconds for the hours of the current day */
    result += (time->hours) * (uint32_t)TIME_SECOND_PER_HOUR;
    /* Save seconds for the minutes and seconds of the current hour */
    result += (time->minutes * 60) + time->seconds;
    
    return result;
}

void Time_unixtimeToTime (Time_UnixTime unix, Time_DateType* date, Time_TimeType* time)
{
    uint32_t dayClock, dayNumber;
    uint16_t year = TIME_UNIX_YEAR;

    dayClock = (uint32_t) unix % TIME_SECOND_PER_DAY; /* Seconds of actual day */
    dayNumber = (uint32_t) unix / TIME_SECOND_PER_DAY;/* days from epoch year */

    time->seconds = dayClock % 60;
    time->minutes = (dayClock % 3600) / 60;
    time->hours   = dayClock / 3600;

    date->wday = (dayNumber + 4) % 7; /* The 0 day of epoch is thursday! */

    while (dayNumber >= TIME_YEAR_SIZE(TIME_UNIX_YEAR))
    {
        dayNumber -= TIME_YEAR_SIZE(year);
        year++;
    }

    date->year = year;
    date->month = 0;
    while (dayNumber >= Time_dayPerMonth[TIME_LEAP_YEAR(year)][date->month])
    {
        dayNumber -= Time_dayPerMonth[TIME_LEAP_YEAR(year)][date->month];
        date->month++;
    }
    date->day = dayNumber + 1;
    date->month++;
    date->wday++;
}

void Time_unixtimeToString (Time_UnixTime unix, char * dateString)
{
    Time_DateType date;
    Time_TimeType time;
    uint8_t counter = 0;

    Time_unixtimeToTime(unix,&date,&time);

    strcpy(dateString,Time_dayString[(date.wday)-1]);
    dateString += 3;

    *dateString = ',';
    dateString++;
    *dateString = ' ';
    dateString++;

    /* Day */
    if (date.day < 10)
    {
        *dateString = '0';
        dateString++;
        u16td(dateString,date.day);
        dateString++;
    }
    else
    {
        u16td(dateString,date.day);
        dateString += 2;
    }
    *dateString = ' ';
    dateString++;

    /* Month */
    strcpy(dateString,Time_monthString[(date.month)-1]);
    dateString += 3;

    *dateString = ' ';
    dateString++;

    /* Year */
    u16td(dateString,date.year);
    dateString += 4;

    *dateString = ' ';
    dateString++;

    /* Hours */
    if (time.hours < 10)
    {
        *dateString = '0';
        dateString++;
        u16td(dateString,time.hours);
        dateString++;
    }
    else
    {
        u16td(dateString,time.hours);
        dateString += 2;
    }

    *dateString = ':';
    dateString++;

    /* Minutes */
    if (time.minutes < 10)
    {
        *dateString = '0';
        dateString++;
        u16td(dateString,time.minutes);
        dateString++;
    }
    else
    {
        u16td(dateString,time.minutes);
        dateString += 2;
    }

    *dateString = ':';
    dateString++;

    /* Seconds */
    if (time.seconds < 10)
    {
        *dateString = '0';
        dateString++;
        u16td(dateString,time.seconds);
        dateString++;
    }
    else
    {
        u16td(dateString,time.seconds);
        dateString += 2;
    }

    /* Close string */
    *dateString = '\0';
}

void Time_unixtimeToNumberString (Time_UnixTime unix, char * dateString, bool second)
{
    Time_DateType date;
    Time_TimeType time;
    uint8_t counter = 0;

    Time_unixtimeToTime(unix,&date,&time);

    /* Day */
    if (date.day < 10)
    {
        *dateString = '0';
        dateString++;
        u16td(dateString,date.day);
        dateString++;
    }
    else
    {
        u16td(dateString,date.day);
        dateString += 2;
    }
    *dateString = '.';
    dateString++;

    /* Month */
    if (date.month < 10)
    {
        *dateString = '0';
        dateString++;
        u16td(dateString,date.month);
        dateString++;
    }
    else
    {
        u16td(dateString,date.month);
        dateString += 2;
    }
    *dateString = '.';
    dateString++;

    /* Year */
    u16td(dateString,date.year);
    dateString += 4;

    *dateString = ' ';
    dateString++;

    /* Hours */
    if (time.hours < 10)
    {
        *dateString = '0';
        dateString++;
        u16td(dateString,time.hours);
        dateString++;
    }
    else
    {
        u16td(dateString,time.hours);
        dateString += 2;
    }

    *dateString = ':';
    dateString++;

    /* Minutes */
    if (time.minutes < 10)
    {
        *dateString = '0';
        dateString++;
        u16td(dateString,time.minutes);
        dateString++;
    }
    else
    {
        u16td(dateString,time.minutes);
        dateString += 2;
    }

    if (second == TRUE)
    {
        *dateString = ':';
        dateString++;

        /* Seconds */
        if (time.seconds < 10)
        {
            *dateString = '0';
            dateString++;
            u16td(dateString,time.seconds);
            dateString++;
        }
        else
        {
            u16td(dateString,time.seconds);
            dateString += 2;
        }
    }
    /* Close string */
    *dateString = '\0';
}

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_TIMEDAY
