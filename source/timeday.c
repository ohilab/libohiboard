/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/timeday.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Time manage functions
 */

#include "timeday.h"

static uint8_t Time_dayPerMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

Time_UnixTime Time_getUnixTime (Time_DateType* date, Time_TimeType* time)
{
    Time_UnixTime result = 0;
    
    if (!(date->year % 4) && (date->month > 2)) result += TIME_SECOND_PER_DAY;
    date->month--;
    
    /* Save seconds for the months of the current year */
    while (date->month)
    {
        date->month--;
        result += Time_dayPerMonth[date->month] * TIME_SECOND_PER_DAY;
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
