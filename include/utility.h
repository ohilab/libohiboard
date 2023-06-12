/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2018 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Leonardo Morichelli
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
 */

/**
 * @file libohiboard/include/utility.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Leonardo Morichelli
 * @brief Useful functions and definitions.
 */

#ifndef __UTILITY_H
#define __UTILITY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>

#include "platforms.h"
#include "errors.h"
#include "types.h"

/// Compose an 32 bit integer type starting from four 8 bit integer type numbers
#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) ((uint32_t)((uint32_t)((Byte0) & 0x00FF) + ((uint32_t)((Byte1) & 0x00FF) << 8) + ((uint32_t)((Byte2) & 0x00FF) << 16) + ((uint32_t)((Byte3) & 0x00FF) << 24)))

#define HI_UINT32(a) (((a) >> 24) & 0xFF)  /// Retrieve the most significant byte of a 32 bit integer type
#define MH_UINT32(a) (((a) >> 16) & 0xFF)  /// Retrieve the 2nd most significant byte of a 32 bit integer type
#define ML_UINT32(a) (((a) >> 8) & 0xFF)   /// Retrieve the 3rd most significant byte of a 32 bit integer type
#define LO_UINT32(a) ((a) & 0xFF)          /// Retrieve the least significant byte of a 32 bit integer type

/// Compose an 16 bit integer type starting from two 8 bit integer type numbers
#define BUILD_UINT16(loByte, hiByte) ((uint16_t)((((uint16_t)loByte) & 0x00FF) + ((((uint16_t)hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF) /// Retrieve the most significant byte of a 16 bit integer type
#define LO_UINT16(a) ((a) & 0xFF)        /// Retrieve the least significant byte of a 16 bit integer type

/// Compose an 8 bit integer type starting from two nibbles
#define BUILD_UINT8(hiByte, loByte) ((uint8_t)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

#define HI_UINT8(a) (((a) >> 4) & 0x0F)   /// Retrieve the most significant nibble of a byte
#define LO_UINT8(a) ((a) & 0x0F)          /// Retrieve the least significant nibble of a byte

/**
 * Convert 2's complement P bit number to 16 bit signed int
 */
#define C2SI(X,P)     (-((X)&(1<<(P-1)))+((X)&(~(1<<(P-1)))))

#define SHIFT_LEFT(X,n)             X<<n
#define SHIFT_RIGHT(X,n)            X>>n

#define IS_DIGIT(c)                 ((c <= '9') && (c >= '0'))
#define IS_XDIGIT(c)                (((c <= '9') && (c >= '0')) || ((c >= 'A') && (c <= 'F')) || ((c >= 'a') && (c <= 'f')))
#define IS_LOWERLETTER(c)           ((c <= 'z') && (c >= 'a'))
#define IS_UPPERLETTER(c)           ((c <= 'Z') && (c >= 'A'))

#define UTILITY_SET_REGISTER_BIT(REGISTER,BIT)       ((REGISTER) |= (BIT))
#define UTILITY_CLEAR_REGISTER_BIT(REGISTER,BIT)     ((REGISTER) &= ~(BIT))
#define UTILITY_READ_REGISTER_BIT(REGISTER,BIT)      ((REGISTER) & (BIT))

#define UTILITY_CLEAR_RGISTER(REGISTER)              ((REGISTER) = (0x0))
#define UTILITY_WRITE_REGISTER(REGISTER,VALUE)       ((REGISTER) = (VALUE))
#define UTILITY_MODIFY_REGISTER(REGISTER,MASK,VALUE) UTILITY_WRITE_REGISTER((REGISTER),\
		                              (((REGISTER) & (~(MASK))) | ((VALUE) & (MASK))))

typedef enum _Utility_State
{
    UTILITY_STATE_DISABLE = 0x00000000u,
    UTILITY_STATE_ENABLE  = 0x00000001u,
} Utility_State;

#define UTILITY_VALID_STATE(STATE) (((STATE) == UTILITY_STATE_ENABLE) || \
                                    ((STATE) == UTILITY_STATE_DISABLE))

#define UTILITY_STRING(x)              #x
#define UTILITY_STRING1(x)             UTILITY_STRING(x)
#define UTILITY_STRING2(x,y)           x##y
#define UTILITY_STRING3(x,y,z)         x##y##z

/**
 * This macro return the dimension of an array.
 *
 * @param ARRAYDIM The array
 */
#define UTILITY_DIMOF(ARRAYDIM) (sizeof(ARRAYDIM)/sizeof(ARRAYDIM[0]))

#define UTILITY_SWAP_INT16(a,b) do { \
    int16_t t = a;                   \
    a = b;                           \
    b = t;                           \
    } while (0);

extern const char hexDigits[];

typedef struct _Utility_VersionFields_t
{
    uint8_t  major;
    uint8_t  minor;
    uint16_t subminor;
    uint32_t time;
} __packed Utility_VersionFields_t;

typedef union _Utility_Version_t
{
	Utility_VersionFields_t f;
    uint8_t b[sizeof (Utility_VersionFields_t)];
} __packed Utility_Version_t;

System_Errors xdigit (uint8_t digit, uint8_t* result);

System_Errors xtu8 (const uint8_t* xString, uint8_t* result, uint8_t slength);
System_Errors xtu16 (const uint8_t* xString, uint16_t* result, uint8_t slength);
System_Errors xtu32 (const uint8_t* xString, uint32_t* result, uint8_t slength);

void u8tx (uint8_t *xString, uint8_t number);
void u16tx (uint8_t *xString, uint16_t number, uint8_t slength);
void u32tx (uint8_t *xString, uint32_t number, uint8_t slength);

System_Errors ddigit (uint8_t digit, uint8_t* result);

System_Errors dtu8 (const uint8_t* dString, uint8_t* result, uint8_t slength);
System_Errors u8td (uint8_t *dString, uint8_t number);
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

/* *****************************************************************************
 *   Useful types management
 * *****************************************************************************/

/**
 * Compose an 16 bit integer type starting from two 8 bit integer type numbers.
 */
#define UTILITY_BUILD_UINT16(loByte, hiByte) ((uint16_t)((((uint16_t)loByte) & 0x00FF) + ((((uint16_t)hiByte) & 0x00FF) << 8)))

/**
 * Retrieve the most significant byte of a 16 bit integer type
 */
#define UTILITY_HI_UINT16(a) (((a) >> 8) & 0xFF)
/**
 * Retrieve the least significant byte of a 16 bit integer type
 */
#define UTILITY_LO_UINT16(a) ((a) & 0xFF)

#define UTILITY_SWAP_UINT16(x) (((x) >> 8) | ((x) << 8))

/**
 * Compose an 32 bit integer type starting from four 8 bit integer type numbers.
 */
#define UTILITY_BUILD_UINT32(loByte, mloByte, mhiByte, hiByte) ((uint32_t)(((uint32_t)loByte) & 0x000000FF) + \
                                                                ((((uint32_t)mloByte) & 0x000000FF) << 8)   + \
                                                                ((((uint32_t)mhiByte) & 0x000000FF) << 16)  + \
                                                                ((((uint32_t)hiByte)  & 0x000000FF) << 24))

#define UTILITY_LO_UINT32(a)    ((uint8_t)((a) & 0x000000FF))
#define UTILITY_MIDLO_UINT32(a) ((uint8_t)((a >> 8) & 0x000000FF))
#define UTILITY_MIDHI_UINT32(a) ((uint8_t)((a >> 16) & 0x000000FF))
#define UTILITY_HI_UINT32(a)    ((uint8_t)((a >> 24) & 0x000000FF))

/* *****************************************************************************
 *   Useful types
 * *****************************************************************************/

typedef union _Utility_4Byte
{
    uint8_t  b[4];
    uint32_t d;
} __packed Utility_4Byte;

typedef union _Utility_2Word
{
    uint16_t w[2];
    uint32_t d;
    int32_t  i;
} __packed Utility_2Word;

typedef union _Utility_2Byte
{
    uint8_t  b[2];
    uint16_t d;
} __packed Utility_2Byte;

typedef union _Utility_Float2Word
{
    uint16_t w[2];
    float    f;
} __packed Utility_Float2Word;

/* *****************************************************************************
 *   BCD value management
 * *****************************************************************************/

/**
  * Convert a 2 digit decimal to BCD format.
  *
  * @param[in] value Byte to be converted (must be less than 99)
  * @return Converted byte
  */
uint8_t Utility_byteToBcd2 (uint8_t value);

/**
 * Convert from 2 digit BCD to binary (2 digit decimal).
 * @param[in] value BCD value to be converted
 * @return Converted value
 */
uint8_t Utility_bcd2ToByte (uint8_t value);

/* *****************************************************************************
 *   String validation functions
 * *****************************************************************************/

/**
 *
 * @return TRUE if the byte is Ascii character, FALSE otherwise.
 */
bool Utility_isAsciiChar  (uint8_t data);

/**
 *
 * @return TRUE if the byte is printable character, FALSE otherwise.
 */
bool Utility_isPrintableChar  (uint8_t data);

/**
 *
 * @return TRUE if the byte is special character, FALSE otherwise.
 */
bool Utility_isSpecialChar  (uint8_t data);

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

/*!
 * \defgroup Utility_other Other generic utility functions
 * \{
 */

/*!
 *
 */
void Utility_getVersionString (const Utility_Version_t* version, char* toString);

/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif // __UTILITY_H
