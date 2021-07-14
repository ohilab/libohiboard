/*
 * This file is part of the libohiboard project.
 * 
 * Copyright (C) 2020 A. C. Open Hardware Ideas Lab
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
 */

/**
 * @file libohiboard/source/NXPMKL/critical_MKL.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Function for implementing Critical Section on NXP MKL series.
 */

#if defined(LIBOHIBOARD_CRITICAL)

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "critical.h"
#include "utility.h"

#if defined (LIBOHIBOARD_MKL)

inline void Critical_sectionBegin (uint32_t* mask)
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

inline void Critical_sectionEnd (uint32_t* mask)
{
    __set_PRIMASK( *mask );
}

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_CRITICAL
