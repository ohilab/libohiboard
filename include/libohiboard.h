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
 */

#ifndef __LIBOHIBOARD_H
#define __LIBOHIBOARD_H

#include <stdio.h>

/* microcontroller selection */
#define KL15
#undef  K60

#if defined(K60)
#include "k60/MK60DZ10.h"
#else if defined(KL15)
#include "kl15/MKL15Z4.h"
#endif

#include "types.h"
#include "errors.h"
#include "uart.h"

void test();

#endif /* __LIBOHIBOARD_H */
