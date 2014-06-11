/******************************************************************************
 * Copyright (C) 2014 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  
 * Project: libohiboard
 * Package: MCG
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
 * @file libohiboard/include/clock.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Clock definitions and prototypes.
 */

#ifndef __CLOCK_H
#define __CLOCK_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum
{
    CLOCK_BUS,
    CLOCK_SYSTEM,
    CLOCK_FLEXBUS,
    CLOCK_FLASH
} Clock_Source;

uint32_t Clock_getFrequency (Clock_Source source);

#endif /* __CLOCK_H */
