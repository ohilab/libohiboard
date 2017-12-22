/******************************************************************************
 * Copyright (C) 2017 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Francesco Maria Rietti
 *  Matteo Civale
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
 * @file libohiboard/source/utility-valid.c
 * @author Francesco Maria Rietti
 * @author Matteo Civale
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Useful functions to validate strings.
 */

#include "utility.h"

bool Utility_isValidIp4Address (char* str)
{
    int segs = 0;   // Segment count
    int chcnt = 0;  // Character count within segment
    int accum = 0;  // Accumulator for segment

    // Catch NULL pointer.
    if (str == 0) return FALSE;

    // Process every character in string.
    while (*str != '\0')
    {
        // Segment changeover.
        if (*str == '.')
        {
            // Must have some digits in segment.
            if (chcnt == 0) return FALSE;
            // Limit number of segments
            if (++segs == 4) return FALSE;
            // Reset segment values and restart loop.
            chcnt = accum = 0;
            str++;
            continue;
        }
        // Check numeric.
        if ((*str < '0') || (*str > '9')) return FALSE;
        // Accumulate and check segment.
        if ((accum = accum * 10 + *str - '0') > 255) return FALSE;
        // Advance other segment specific stuff and continue loop.
        chcnt++;
        str++;
    }
    // Check enough segments and enough characters in last segment.
    if (segs != 3) return FALSE;
    if (chcnt == 0) return FALSE;

    return TRUE;
}

bool Utility_isValidMacAddress (char* str)
{
    uint8_t i = 0, s = 0;

    // Catch NULL pointer.
    if (str == 0) return FALSE;

    while (*str)
    {
       if (IS_XDIGIT(*str))
       {
          i++;
       }
       else if (*str == ':')
       {
          if ((i == 0) || (i / 2 - 1 != s)) break;

          ++s;
       }
       else
       {
           // Wrong char
           return FALSE;
       }
       ++str;
    }

    return ((i == 12) && (s == 5));
}
