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

#ifndef __UTILITY_DEBOUNCING_H
#define __UTILITY_DEBOUNCING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"

/*!
 *
 */
typedef struct _UtilityDeboucing_Config
{
    uint32_t checkTime;
    uint32_t holdTime;
    uint32_t debounceTime;

    uint32_t count;
    uint32_t countTimeout;

    Gpio_Level holdLevel;

    bool newEvent;
} UtilityDeboucing_Config;

/*!
 *
 * \note This function was inspired by http://www.ganssle.com/debouncing-pt2.htm
 *
 * \param[in] pin:
 * \param[in] config:
 *
 * \return
 */
System_Errors UtilityDebouncing_debounce (Gpio_Pins pin, UtilityDeboucing_Config* config);

#ifdef __cplusplus
}
#endif

#endif // __UTILITY_DEBOUNCING_H
