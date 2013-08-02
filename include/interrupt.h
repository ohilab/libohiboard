/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Marco Giammarini <m.giammarini@warcomeb.it>
 *	
 * Project: libohiboard
 * Package: Interrupt
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
 * @file libohiboard/include/interrupt.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Manage interrupt
 */

#include "platforms.h"
#include "errors.h"
#include "types.h"

#ifndef __INTERRUPT_H
#define __INTERRUPT_H

System_Errors Interrupt_enable (uint16_t vectorNumber);
System_Errors Interrupt_disable (uint16_t vectorNumber);

#endif /* __INTERRUPT_H */
