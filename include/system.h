/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  
 * Project: libohiboard
 * Package: System
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
 * @file libohiboard/include/system.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief 
 */

#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

#if defined (FRDMKL25Z)
#define SYSTEM_CLOCK_KHZ 20000
#elif defined (MKL15Z4)
#define SYSTEM_CLOCK_KHZ 24000
#elif defined (MK10DZ10)
#define SYSTEM_CLOCK_KHZ 100000
#elif defined (MK60DZ10)
#define SYSTEM_CLOCK_KHZ 48000
#endif

System_Errors System_controlDevice (void);
System_Errors System_initClock (void);

#endif /* __SYSTEM_H */
