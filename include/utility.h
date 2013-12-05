/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/include/utility.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Useful functions and definitions.
 */

#ifndef __UTILITY_H
#define __UTILITY_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

/* Useful define */
#define GPIO_PIN_MASK               0x1Fu
#define GPIO_PIN(x)                 (((1)<<(x & GPIO_PIN_MASK)))

#define IS_DIGIT(c)                 ((c <= '9') && (c >= '0'))
#define IS_LOWERLETTER(c)           ((c <= 'z') && (c >= 'a'))
#define IS_UPPERLETTER(c)           ((c <= 'Z') && (c >= 'A'))

extern const char hexDigits[];

System_Errors xdigit (uint8_t digit, uint8_t* result);

System_Errors xtu8 (const uint8_t* xString, uint8_t* result, uint8_t slength);
System_Errors xtu16 (const uint8_t* xString, uint16_t* result, uint8_t slength);
System_Errors xtu32 (const uint8_t* xString, uint32_t* result, uint8_t slength);

void u8tx (uint8_t *xString, uint8_t number);
void u16tx (uint8_t *xString, uint16_t number, uint8_t slength);
void u32tx (uint8_t *xString, uint32_t number, uint8_t slength);

System_Errors ddigit (uint8_t digit, uint8_t* result);

System_Errors dtu8 (const uint8_t* dString, uint8_t* result, uint8_t slength);

System_Errors strtf (const uint8_t* fString, float* result);

uint8_t stringCompare (const char* string1, const char* string2);

#endif /* __UTILITY_H */
