/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/include/hardware/PIC24FJ/critical_PIC24FJ.h
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief CRITICAL Function for implementing Critical Section on PIC24FJ
 */

#ifndef __CRITICAL_PIC24FJ_H
#define __CRITICAL_PIC24FJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_CRITICAL) && defined(LIBOHIBOARD_PIC24FJ)

/**
 * @addtogroup CRITICAL
 * @{
 */

/**
 * @defgroup CRITICAL Critical Section for specific Hardware
 * @{
 */

/**
 * Return the CPU Interrupt Priority Level.
 * 
 * @retval current value of IPL
 */
uint16_t Critical_getCpuIpl(void);

/**
 * Set the CPU Interrupt Priority Level.
 * 
 * @note If IPL is 7 (15) then user interrupts are disabled.
 * 
 * @param[IN] value, new value of IPL;
 */
void Critical_setCpuIpl(uint16_t value);

/*!
 * Return if critical section is active.
 * 
 * \retval TRUE if Critical Section is active, FALSE otherwise
 */
bool Critical_isActive(void);

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_CRITICAL & LIBOHIBOARD_PIC24FJ

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __CRITICAL_PIC24FJ_H
