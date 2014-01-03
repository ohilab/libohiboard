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
 * @file libohiboard/include/timeday.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Time manage functions declarations
 */

#ifndef __TIMEDAY_H
#define __TIMEDAY_H

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
    uint8_t  day;
    uint8_t  month;
    uint16_t year;
} Time_DateType;

typedef uint32_t Time_UnixTime;

Time_UnixTime Time_getUnixTime (Time_DateType date, Time_TimeType time);

#endif /* __TIMEDAY_H */
