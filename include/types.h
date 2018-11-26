/*
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
 */

/**
 * @file libohiboard/include/types.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Useful type definitions.
 */

#ifndef __TYPES_H
#define __TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
#ifndef __MCUXPRESSO
typedef unsigned long      uint32_t;
#endif
typedef unsigned long long uint64_t;

typedef signed char        int8_t;
typedef short              int16_t;
#ifndef __MCUXPRESSO
typedef long               int32_t;
#endif
typedef long long          int64_t;

#ifndef __MCUXPRESSO
typedef unsigned char      bool;
#else
#include "stdbool.h"
#endif
#define TRUE               1
#define FALSE              0

typedef void (*voidFuncPtr)(void);
typedef void (*voidArgumentFuncPtr)(void *);

#if defined ( __GNUC__ ) && !defined (__CC_ARM)

#ifndef _weak
#define _weak   __attribute__((weak))
#endif // _weak

#ifndef __weak
#define __weak   __attribute__((weak))
#endif // __weak

#endif // defined ( __GNUC__ ) && !defined (__CC_ARM)

#ifdef __cplusplus
}
#endif

#endif // __TYPES_H

