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
        if (slength > 2) return ERRORS_UTILITY_LONG_STRING;

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
        if (slength > 2) return ERRORS_UTILITY_LONG_STRING;

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
            continue;
        }
        
        /* Skip without any operation */
        if (*fString == '+') continue;
        
        if (*fString == '.')
        {
            isDecimal = 1;
            continue;
        }

        if (ddigit(*fString,&digit) != ERRORS_UTILITY_CONVERSION_OK)
            return ERRORS_UTILITY_ILLEGAL_CHAR;
        
        if (!isDecimal && (*dotPosition != '.'))
        {
            *result = (10 * (*result)) + digit;
            dotPosition++;
        }
        else
        {
            decimalPart = (10.0 * decimalPart) + digit;
            decimalDivisor *= 10.0; 
        }
        fString++;
    }
    
    *result += (decimalPart/decimalDivisor);
    *result *= (isNegative) ? -1.0 : +1.0;
    
    return ERRORS_UTILITY_CONVERSION_OK;
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
