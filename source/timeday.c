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
 * @file libohiboard/source/timeday.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Time manage functions
 */

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
    "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat",
};

bool Time_isValid (Time_DateType* date, Time_TimeType* time)
{
    bool isValid = true;

    if(date->year < 1970)
    {
        isValid = false;
    }
    if(date->month > TIME_MONTH_DECEMBER)
    {
        isValid = false;
    }
    if(date->day < 1 || date->day > 31)
    {
        isValid = false;
    }
    if(date->wday > TIME_DAYOFWEEK_SATURDAY)
    {
        isValid = false;
    }

    if(time->hours > 23)
    {
        isValid = false;
    }
    if(time->minutes > 59)
    {
        isValid = false;
    }
    if(time->seconds > 59)
    {
        isValid = false;
    }

    return isValid;
}

Time_UnixTime Time_getUnixTime (Time_DateType* date, Time_TimeType* time)
{
    Time_UnixTime unixEpoch = 0;

    if (!(date->year % 4) && (date->month > TIME_MONTH_FEBRUARY)) unixEpoch += TIME_SECOND_PER_DAY;

    /* Save seconds for the months of the current year */
    while (date->month)
    {
        date->month--;
        unixEpoch += Time_dayPerMonth[0][date->month] * TIME_SECOND_PER_DAY;
    }

    /* Save seconds for past years */
    unixEpoch += (((date->year-TIME_UNIX_YEAR)*365) + (((date->year - 1) - TIME_UNIX_YEAR_LEAP)/4)) * (uint32_t)TIME_SECOND_PER_DAY;
    /* Save seconds for the days of the current month */
    unixEpoch += (date->day-1) * (uint32_t)TIME_SECOND_PER_DAY;
    /* Save seconds for the hours of the current day */
    unixEpoch += (time->hours) * (uint32_t)TIME_SECOND_PER_HOUR;
    /* Save seconds for the minutes and seconds of the current hour */
    unixEpoch += (time->minutes * 60) + time->seconds;

    return unixEpoch;
}

void Time_unixtimeToTime (Time_UnixTime unixEpoch, Time_DateType* date, Time_TimeType* time)
{
    uint32_t dayClock = 0, dayNumber = 0;
    uint16_t year = TIME_UNIX_YEAR;

    memset(date, 0, sizeof(Time_DateType));
    memset(time, 0, sizeof(Time_TimeType));

    dayClock = (uint32_t) unixEpoch % TIME_SECOND_PER_DAY; /* Seconds of actual day */
    dayNumber = (uint32_t) unixEpoch / TIME_SECOND_PER_DAY;/* days from epoch year */

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
}

void Time_unixtimeToString (Time_UnixTime unixEpoch, char * dateString)
{
    Time_DateType date = {0};
    Time_TimeType time = {0};

    Time_unixtimeToTime(unixEpoch,&date,&time);

    strcpy(dateString,Time_dayString[(date.wday)]);
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
        u16td((uint8_t *)dateString,(uint16_t)date.day);
        dateString++;
    }
    else
    {
        u16td((uint8_t *)dateString,(uint16_t)date.day);
        dateString += 2;
    }
    *dateString = ' ';
    dateString++;

    /* Month */
    strcpy(dateString,Time_monthString[(date.month)]);
    dateString += 3;

    *dateString = ' ';
    dateString++;

    /* Year */
    u16td((uint8_t *)dateString,(uint16_t)date.year);
    dateString += 4;

    *dateString = ' ';
    dateString++;

    /* Hours */
    if (time.hours < 10)
    {
        *dateString = '0';
        dateString++;
        u16td((uint8_t *)dateString,(uint16_t)time.hours);
        dateString++;
    }
    else
    {
        u16td((uint8_t *)dateString,(uint16_t)time.hours);
        dateString += 2;
    }

    *dateString = ':';
    dateString++;

    /* Minutes */
    if (time.minutes < 10)
    {
        *dateString = '0';
        dateString++;
        u16td((uint8_t *)dateString,(uint16_t)time.minutes);
        dateString++;
    }
    else
    {
        u16td((uint8_t *)dateString,(uint16_t)time.minutes);
        dateString += 2;
    }

    *dateString = ':';
    dateString++;

    /* Seconds */
    if (time.seconds < 10)
    {
        *dateString = '0';
        dateString++;
        u16td((uint8_t *)dateString,(uint16_t)time.seconds);
        dateString++;
    }
    else
    {
        u16td((uint8_t *)dateString,(uint16_t)time.seconds);
        dateString += 2;
    }

    /* Close string */
    *dateString = '\0';
}

void Time_unixtimeToNumberString (Time_UnixTime unixEpoch, char * dateString, bool second)
{
    Time_DateType date;
    Time_TimeType time;

    Time_unixtimeToTime(unixEpoch,&date,&time);

    /* Day */
    if (date.day < 10)
    {
        *dateString = '0';
        dateString++;
        u16td((uint8_t *)dateString,(uint16_t)date.day);
        dateString++;
    }
    else
    {
        u16td((uint8_t *)dateString,(uint16_t)date.day);
        dateString += 2;
    }
    *dateString = '.';
    dateString++;

    /* Month */
    if (date.month < 10)
    {
        *dateString = '0';
        dateString++;
        u16td((uint8_t *)dateString,(uint16_t)date.month + 1);
        dateString++;
    }
    else
    {
        u16td((uint8_t *)dateString,(uint16_t)date.month + 1);
        dateString += 2;
    }
    *dateString = '.';
    dateString++;

    /* Year */
    u16td((uint8_t *)dateString,(uint16_t)date.year);
    dateString += 4;

    *dateString = ' ';
    dateString++;

    /* Hours */
    if (time.hours < 10)
    {
        *dateString = '0';
        dateString++;
        u16td((uint8_t *)dateString,(uint16_t)time.hours);
        dateString++;
    }
    else
    {
        u16td((uint8_t *)dateString,(uint16_t)time.hours);
        dateString += 2;
    }

    *dateString = ':';
    dateString++;

    /* Minutes */
    if (time.minutes < 10)
    {
        *dateString = '0';
        dateString++;
        u16td((uint8_t *)dateString,(uint16_t)time.minutes);
        dateString++;
    }
    else
    {
        u16td((uint8_t *)dateString,(uint16_t)time.minutes);
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
            u16td((uint8_t *)dateString,(uint16_t)time.seconds);
            dateString++;
        }
        else
        {
            u16td((uint8_t *)dateString,(uint16_t)time.seconds);
            dateString += 2;
        }
    }
    /* Close string */
    *dateString = '\0';
}

#ifdef __cplusplus
}
#endif

