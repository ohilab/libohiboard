/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Edoardo Bezzeccheri <coolman3@gmail.com>
 *	
 * Project: libohiboard
 * Package: -
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
 * @file libohiboard/include/libohiboard.h
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @brief Library main file.
 */

#ifndef __LIBOHIBOARD_H
#define __LIBOHIBOARD_H

#include <stdio.h>

#include "types.h"
#include "errors.h"
#include "utility.h"

#include "interrupt.h"

#include "system.h"

#include "uart.h"

#include "i2c.h"

#include "spi.h"

void test();

#endif /* __LIBOHIBOARD_H */
