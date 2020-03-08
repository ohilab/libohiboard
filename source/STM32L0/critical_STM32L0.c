/*
 * This file is part of the libohiboard project.
 * 
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/STM32L0/critical_STM32L0.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Function for implementing Critical Section on STM32L0
 */

#if defined(LIBOHIBOARD_CRITICAL)

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "critical.h"
#include "utility.h"

#if defined (LIBOHIBOARD_STM32L0)

inline void Critical_sectionBegin (uint32_t* mask)
{
    *mask = __get_BASEPRI();
    __set_BASEPRI(CRITICAL_DEFAULT_TICK_PRIO + 1);
}

inline void Critical_sectionEnd (uint32_t* mask)
{
    __set_BASEPRI( *mask );
}

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_CRITICAL
