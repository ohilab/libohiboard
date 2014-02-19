/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  
 * Project: libohiboard
 * Package: TIME
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
 * @file libohiboard/source/time.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Time manage functions
 */

#include "time.h"

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
