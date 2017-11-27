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
 * @file libohiboard/source/utility.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Useful functions and definitions.
 */

#include "utility.h"

const char hexDigits[] = "0123456789ABCDEF";

System_Errors xdigit (uint8_t digit, uint8_t* result)
{
    if (digit >= 'A' && digit <= 'F')
    {
        *result = digit - 'A' + 10;
        return ERRORS_UTILITY_CONVERSION_OK;
    }
    else if (digit >= 'a' && digit <= 'f')
    {
        *result = digit - 'a' + 10;
        return ERRORS_UTILITY_CONVERSION_OK;
    }
    else if (digit >= '0' && digit <= '9')
    {
        *result = digit - '0';
        return ERRORS_UTILITY_CONVERSION_OK;
    }

    return ERRORS_UTILITY_ILLEGAL_CHAR; /* Bad digit */
}

System_Errors xtu8 (const uint8_t* xString, uint8_t* result, uint8_t slength)
{
    uint8_t i, singleByte;

    if (slength > 0)
    {
        if (slength > 2) return ERRORS_UTILITY_LONG_STRING;

        /* Start conversion... */
        *result = 0;
        for (i = 0; i < slength; ++i)
        {
            if (xdigit(*xString,&singleByte) == ERRORS_UTILITY_CONVERSION_OK)
            {
                *result = 16 * (*result) + singleByte;
            }
            else
            {
                return ERRORS_UTILITY_ILLEGAL_CHAR;
            }
            xString++;
        }
        return ERRORS_UTILITY_CONVERSION_OK;
    }

    return ERRORS_UTILITY_EMPTY_STRING;
}

System_Errors xtu16 (const uint8_t* xString, uint16_t* result, uint8_t slength)
{
    uint8_t i, singleByte;

    if (slength > 0)
    {
        if (slength > 4) return ERRORS_UTILITY_LONG_STRING;

        /* Start conversion... */
        *result = 0;
        for (i = 0; i < slength; ++i)
        {
            if (xdigit(*xString,&singleByte) == ERRORS_UTILITY_CONVERSION_OK)
            {
                *result = 16 * (*result) + singleByte;
            }
            else
            {
                return ERRORS_UTILITY_ILLEGAL_CHAR;
            }
            xString++;
        }
        return ERRORS_UTILITY_CONVERSION_OK;
    }

    return ERRORS_UTILITY_EMPTY_STRING;
}

System_Errors xtu32 (const uint8_t* xString, uint32_t* result, uint8_t slength)
{
    uint8_t i, singleByte;
    

    if (slength > 0)
    {
        if (slength > 8) return ERRORS_UTILITY_LONG_STRING;

        /* Start conversion... */
        *result = 0;
        for (i = 0; i < slength; ++i)
        {
            if (xdigit(*xString,&singleByte) == ERRORS_UTILITY_CONVERSION_OK)
            {
                *result = 16 * (*result) + singleByte;
            }
            else
            {
                return ERRORS_UTILITY_ILLEGAL_CHAR;
            }
            xString++;
        }
        return ERRORS_UTILITY_CONVERSION_OK;
    }

    return ERRORS_UTILITY_EMPTY_STRING;
}

void u8tx (uint8_t *xString, uint8_t number)
{
    *xString = hexDigits[(number >> 4) & 0x0F];
    ++xString;
    *xString = hexDigits[(number >> 0) & 0x0F];
}

void u16tx (uint8_t *xString, uint16_t number, uint8_t slength)
{
    switch (slength)
    {
    case 4:
        *xString = hexDigits[(number >> 12) & 0x000F];
        ++xString;
    case 3:
        *xString = hexDigits[(number >> 8) & 0x000F];
        ++xString;
    case 2:
        *xString = hexDigits[(number >> 4) & 0x000F];
        ++xString;
    case 1:
        *xString = hexDigits[(number >> 0) & 0x000F];
        break;
    default:
        return;
    }
}

void u32tx (uint8_t *xString, uint32_t number, uint8_t slength)
{
    switch (slength)
    {
    case 8:
        *xString = hexDigits[(number >> 28) & 0x0000000F];
        ++xString;
    case 7:
        *xString = hexDigits[(number >> 24) & 0x0000000F];
        ++xString;
    case 6:
        *xString = hexDigits[(number >> 20) & 0x0000000F];
        ++xString;
    case 5:
        *xString = hexDigits[(number >> 16) & 0x0000000F];
        ++xString;
    case 4:
        *xString = hexDigits[(number >> 12) & 0x0000000F];
        ++xString;
    case 3:
        *xString = hexDigits[(number >> 8) & 0x0000000F];
        ++xString;
    case 2:
        *xString = hexDigits[(number >> 4) & 0x0000000F];
        ++xString;
    case 1:
        *xString = hexDigits[(number >> 0) & 0x0000000F];
        break;
    default:
        return;
    }
}

/**
 * @brief Decimal digit to uint8_t conversion.
 * 
 */
System_Errors ddigit (uint8_t digit, uint8_t* result)
{
    if (digit >= '0' && digit <= '9')
    {
        *result = digit - '0';
        return ERRORS_UTILITY_CONVERSION_OK;
    }

    return ERRORS_UTILITY_ILLEGAL_CHAR; /* Bad digit */
}

/**
 * @brief Decimal to uint8_t conversion. 
 * 
 */
System_Errors dtu8 (const uint8_t* dString, uint8_t* result, uint8_t slength)
{
    uint8_t i, singleByte;

    if (slength > 0)
    {
        if (slength > 3) return ERRORS_UTILITY_LONG_STRING;

        /* Start conversion... */
        *result = 0;
        for (i = 0; i < slength; ++i)
        {
            if (ddigit(*dString,&singleByte) == ERRORS_UTILITY_CONVERSION_OK)
            {
                *result = 10 * (*result) + singleByte;
            }
            else
            {
                return ERRORS_UTILITY_ILLEGAL_CHAR;
            }
            dString++;
        }
        return ERRORS_UTILITY_CONVERSION_OK;
    }

    return ERRORS_UTILITY_EMPTY_STRING;
}

/**
 * @brief Decimal to uint16_t conversion. 
 * 
 */
System_Errors dtu16 (const uint8_t* dString, uint16_t* result, uint8_t slength)
{
    uint8_t i, singleByte;

    if (slength > 0)
    {
        if (slength > 5) return ERRORS_UTILITY_LONG_STRING;

        /* Start conversion... */
        *result = 0;
        for (i = 0; i < slength; ++i)
        {
            if (ddigit(*dString,&singleByte) == ERRORS_UTILITY_CONVERSION_OK)
            {
                *result = 10 * (*result) + singleByte;
            }
            else
            {
                return ERRORS_UTILITY_ILLEGAL_CHAR;
            }
            dString++;
        }
        return ERRORS_UTILITY_CONVERSION_OK;
    }

    return ERRORS_UTILITY_EMPTY_STRING;
}

uint8_t u16td (uint8_t *dString, uint16_t number)
{
    uint16_t scale = 10000;
    char pad = 0;
    uint8_t charCount = 0;
    
    if (number == 0)
    {
        *dString = '0';
        dString++;
        *dString = '\0';
        charCount++;
        return charCount;
    }
    
    for (; scale; scale /= 10)
    {
        if (number >= scale)
        {
            *dString = hexDigits[number / scale];
            dString++;
            charCount++;
            number %= scale;
            pad = '0';
        }
        else
        {
            if (pad)
            {
                *dString = pad;
                dString++;
                charCount++;
            }
        }
    }
    *dString = '\0';
    return charCount;
}

uint8_t i16td (uint8_t *dString, int16_t number)
{
    int16_t scale = 10000;
    char pad = 0;
    uint8_t charCount = 0;

    if (number == 0)
    {
        *dString = '0';
        dString++;
        *dString = '\0';
        charCount++;
        return charCount;
    }
    
    if (number < 0)
    {
        number = -number;
        *dString = '-';
        dString++;
        charCount++;
    }
        
    for (; scale; scale /= 10)
    {
        if (number >= scale)
        {
            *dString = hexDigits[number / scale];
            dString++;
            charCount++;
            number %= scale;
            pad = '0';
        }
        else
        {
            if (pad)
            {
                *dString = pad;
                dString++;
                charCount++;
            }
        }
    }
    *dString = '\0';
    return charCount;
}

/**
 * @brief String to float conversion.
 * 
 * @param uint8_t* fString String that must be convert.
 * @param float* result The result of the conversion.
 * @result Return the status of the operation by an element of @ref System_Errors
 */
System_Errors strtf (const uint8_t* fString, float* result)
{
    uint8_t digit;
    const char* dotPosition = fString+2;

    uint8_t isNegative = 0;
    uint8_t isDecimal = 0;
    float decimalPart = 0.0, decimalDivisor = 1.0;
    
    *result = 0;
    while (*fString)
    {
        if (*fString == '-')
        {
            isNegative = 1;
            fString++;
            continue;
        }
        
        /* Skip without any operation */
        if (*fString == '+') 
        {
            fString++;        
            continue;
        }

        if (*fString == '.')
        {
            isDecimal = 1;
            fString++;        
            continue;
        }

        if (ddigit(*fString,&digit) != ERRORS_UTILITY_CONVERSION_OK)
            return ERRORS_UTILITY_ILLEGAL_CHAR;
        
        if (!isDecimal && (*dotPosition != '.'))
        {
            *result = (10 * (*result)) + digit;
            dotPosition++;
        }
        else if (isDecimal)
        {
            decimalPart = (10.0 * decimalPart) + digit;
            decimalDivisor *= 10.0;
        }
        else
        {
            decimalPart = (10.0 * decimalPart) + digit;
        }

        fString++;
    }
    
    *result += (decimalPart/decimalDivisor);
    *result *= (isNegative) ? -1.0 : +1.0;
    
    return ERRORS_UTILITY_CONVERSION_OK;
}

/**
 * 
 */
System_Errors ftstr (float value, uint8_t* fString, uint8_t precision)
{
    float rounding = 0.5;
    uint16_t decMultiplier = 1;
    uint16_t fractional;
    uint16_t intPart;
    
    uint8_t index;
    uint8_t addChar;

    if (precision > 4)
        return ERRORS_UTILITY_FLOAT_WRONG_PRECISION;
    
    if (value < 0.0)
    {
        value = -value;
        *fString = '-';
        fString++;
    }
    
    // Setup rounding and decimal multiplier
    if (precision > 0)
    {
        for (index=0; index < precision; ++index)
        {
            rounding /= 10.0;
            decMultiplier *= 10;
        }
    }
    
    // round number and save integer part
    value += rounding;
    intPart = (uint16_t) value;
    // Print integer part
    addChar = u16td(fString,value);
    fString += addChar;
    
    if (precision > 0)
    {
        *fString = '.'; 
        fString++;
        
        fractional = (uint16_t)((value - intPart) * decMultiplier);
        switch (precision)
        {
        case 2:
            if (fractional < 10)
            {
                *fString = '0';
                fString++;
            }
            break;
        case 3:
            if (fractional < 10)
            {
                *fString = '0';
                fString++;
                *fString = '0';
                fString++;
            }
            else if (fractional < 100)
            {
                *fString = '0';
                fString++;
            }
            break;
        case 4:
            if (fractional < 10)
            {
                *fString = '0';
                fString++;
                *fString = '0';
                fString++;
                *fString = '0';
                fString++;
            }
            else if (fractional < 100)
            {
                *fString = '0';
                fString++;
                *fString = '0';
                fString++;
            }
            else if (fractional < 1000)
            {
                *fString = '0';
                fString++;
            }
            break;
        }
        
        // Print fractional part
        u16td (fString,fractional);
    }
    
    return ERRORS_UTILITY_CONVERSION_OK;
}

void fti (float number, uint8_t precision, int16_t* integerPart, uint16_t* decimalPart)
{ 
    uint16_t decMultiplier = 1;
    uint8_t i;
    uint16_t intPart;
  
    for(i=precision; i > 0; --i) 
    {
        decMultiplier *= 10;
    }

    intPart = *integerPart = (int16_t) number;
    if (number < 0.0)
    {
        number = -number;
        intPart = -intPart;
    }
    *decimalPart = (uint16_t)((number - intPart) * decMultiplier);
}

void ftu (float number, uint8_t precision, uint16_t* integerPart, uint16_t* decimalPart)
{ 
    uint16_t decMultiplier = 1;
    uint8_t i;
  
    for(i=precision; i > 0; --i) 
    {
        decMultiplier *= 10;
    }

    *integerPart = (uint16_t) number;
    *decimalPart = (uint16_t)((number - *integerPart) * decMultiplier);
}

uint8_t stringCompare (const char* string1, const char* string2)
{
    while ((*string1 && *string2) && (*string1 == *string2))
    {
        string1++;
        string2++;
    }
    return (*string1 - *string2);
}

uint8_t stringCompareBySize (const char* string1, const char* string2, uint8_t size)
{
    while (size != 0)
    {
        if ((*string1 == '\0') && (*string2 == '\0')) break;

        if (*string1 != *string2) return 0;

        string1++;
        string2++;
        size--;
    }
    return 1;
}

int8_t stringFindFirstOf (const char* string, char find, uint8_t size)
{
    int8_t position = 0;

    while (size != 0)
    {
        if (*string == '\0') break;

        if (*string == find) return position;

        string++;
        position++;
        size--;
    }
    return -1;
}

uint8_t u32td (uint8_t *dString, uint32_t number)
{
    uint32_t scale = 1000000000;
    char pad = 0;
    uint8_t charCount = 0;

    if (number == 0)
    {
        *dString = '0';
        dString++;
        *dString = '\0';
        charCount++;
        return charCount;
    }

    for (; scale; scale /= 10)
    {
        if (number >= scale)
        {
            *dString = hexDigits[number / scale];
            dString++;
            charCount++;
            number %= scale;
            pad = '0';
        }
        else
        {
            if (pad)
            {
                *dString = pad;
                dString++;
                charCount++;
            }
        }
    }
    *dString = '\0';
    return charCount;
}

/**
 * @brief Decimal string to uint32_t conversion.
 *
 */
System_Errors dtu32 (const uint8_t* dString, uint32_t* result, uint8_t slength)
{
    uint8_t i, singleByte;

    if (slength > 0)
    {
        if (slength > 10) return ERRORS_UTILITY_LONG_STRING;

        /* Start conversion... */
        *result = 0;
        for (i = 0; i < slength; ++i)
        {
            if (ddigit(*dString,&singleByte) == ERRORS_UTILITY_CONVERSION_OK)
            {
                *result = 10 * (*result) + singleByte;
            }
            else
            {
                return ERRORS_UTILITY_ILLEGAL_CHAR;
            }
            dString++;
        }
        return ERRORS_UTILITY_CONVERSION_OK;
    }

    return ERRORS_UTILITY_EMPTY_STRING;
}

