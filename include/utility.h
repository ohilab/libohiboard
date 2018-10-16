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
 * @file libohiboard/include/utility.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Useful functions and definitions.
 */

#ifndef __UTILITY_H
#define __UTILITY_H

#include "platforms.h"
#include "errors.h"
#include "types.h"


//convert 2's complement P bit number to 16 bit signed int
#define C2SI(X,P)     (-((X)&(1<<(P-1)))+((X)&(~(1<<(P-1)))))

//#define SET_BIT(n)                  (1<<n)&0xFF
#define SHIFT_LEFT(X,n)             X<<n
#define SHIFT_RIGHT(X,n)            X>>n

#define IS_DIGIT(c)                 ((c <= '9') && (c >= '0'))
#define IS_XDIGIT(c)                (((c <= '9') && (c >= '0')) || ((c >= 'A') && (c <= 'F')) || ((c >= 'a') && (c <= 'f')))
#define IS_LOWERLETTER(c)           ((c <= 'z') && (c >= 'a'))
#define IS_UPPERLETTER(c)           ((c <= 'Z') && (c >= 'A'))

#define UTILITY_CLEAR_REGISTER_BIT(REGISTER,BIT) ((REGISTER) &= ~(BIT))
#define UTILITY_SET_REGISTER_BIT(REGISTER,BIT)   ((REGISTER) |= (BIT))
#define UTILITY_READ_REGISTER_BIT(REGISTER,BIT)  ((REGISTER) & (BIT))

#define UTILITY_WRITE_REGISTER(REGISTER,VALUE)   ((REGISTER) = (VALUE))

#define UTILITY_MODIFY_REGISTER(REGISTER,CLEARMASK,SETMASK) \
    UTILITY_WRITE_REGISTER((REGISTER), (((REGISTER) & (~(CLEARMASK))) | (SETMASK)))

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
System_Errors dtu16 (const uint8_t* dString, uint16_t* result, uint8_t slength);
System_Errors dtu32 (const uint8_t* dString, uint32_t* result, uint8_t slength);

uint8_t u16td (uint8_t *dString, uint16_t number);
uint8_t i16td (uint8_t *dString, int16_t number);

System_Errors strtf (const uint8_t* fString, float* result);
System_Errors ftstr (float value, uint8_t* fString, uint8_t precision);

uint8_t u32td (uint8_t *dString, uint32_t number);

void fti (float number, uint8_t precision, int16_t* integerPart, uint16_t* decimalPart);
void ftu (float number, uint8_t precision, uint16_t* integerPart, uint16_t* decimalPart);

uint8_t stringCompare (const char* string1, const char* string2);
uint8_t stringCompareBySize (const char* string1, const char* string2, uint8_t size);
int8_t stringFindFirstOf (const char* string, char find, uint8_t size);

/*******************************************************************************
 * String validation functions
 *******************************************************************************/

/**
 *
 * @return TRUE if the string is valid, FALSE otherwise.
 */
bool Utility_isValidIp4Address (char* str);

/**
 *
 * @return TRUE if the string is valid, FALSE otherwise.
 */
bool Utility_isValidMacAddress (char* str);

#endif /* __UTILITY_H */
