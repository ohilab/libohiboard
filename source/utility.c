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

