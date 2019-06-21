/*
 * This file is part of the libohiboard project.
 * 
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/source/PIC24FJ/critical_PIC24FJ.c
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief CRITICAL Function for implementing Critical Section on PIC24FJ
 */

#if defined(LIBOHIBOARD_CRITICAL)

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "utility.h"
#include "critical.h"

#if defined (LIBOHIBOARD_PIC24FJ)

inline uint8_t Critical_getCpuIpl(void)
{
    uint8_t value = (UTILITY_READ_REGISTER_BIT(CPU->SR, _SR_IPL_MASK) >> _SR_IPL_POSITION);
    return value;
}

inline void Critical_setCpuIpl(uint8_t value)
{
    UTILITY_MODIFY_REGISTER(CPU->SR, _SR_IPL_MASK, (value << _SR_IPL_POSITION));
}

inline bool Critical_isActive(void)
{
    uint8_t value = Critical_getCpuIpl();
    return (value == CRITICAL_MAX_PRIORITY)?(true):(false);
}

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_CRITICAL
