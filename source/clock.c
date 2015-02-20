/******************************************************************************
 * Copyright (C) 2014-2015 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/clock.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Clock implementations.
 */

#include "platforms.h"
#include "clock.h"

#if defined (LIBOHIBOARD_FRDMKL02Z) || \
	defined (LIBOHIBOARD_KL02Z4)

uint32_t Clock_getFrequency (Clock_Source source)
{
    switch (source)
    {
    case CLOCK_BUS:
#if defined (FRDMKL02Z)
        return 23986176; /* TODO: FIXME: */
#endif
        break;
    case CLOCK_SYSTEM:
#if defined (FRDMKL02Z)
        return 47972352; /* TODO: FIXME: */
#endif    
        break;
    }
}

#endif
