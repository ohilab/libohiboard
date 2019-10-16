/*
 * Copyright (C) 2012-2019 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Stefano Gigli
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
 * @author Stefano Gigli
 * @brief Useful type definitions.
 */

#ifndef __TYPES_H
#define __TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define TRUE               1
#define FALSE              0

typedef void (*voidFuncPtr)(void);
typedef void (*voidArgumentFuncPtr)(void *);

typedef void (*pFunc)(void);

typedef void (*pFuncPtrParam)(void *);

typedef void (*pFuncU32Param) (uint32_t param);
typedef void (*pFuncU32Param2)(uint32_t param1, uint32_t param2);
typedef void (*pFuncU32Param3)(uint32_t param1, uint32_t param2, uint32_t param3);
typedef void (*pFuncArrayParam)(void *data, size_t length);

#if ((defined ( __GNUC__ ) && !defined (__CC_ARM)) || defined( __XC16__ ))


#ifndef _inline
#define _inline   inline
#endif // _inline

#ifndef __inline
#define __inline   inline
#endif // __inline

#ifndef _weak
#define _weak   __attribute__((weak))
#endif // _weak

#ifndef __weak
#define __weak   __attribute__((weak))
#endif // __weak

#ifndef _isr
#define _isr             __attribute__((interrupt,"IRQ"))
#endif // _isr

#ifndef __isr
#define __isr             __attribute__((interrupt,"IRQ"))
#endif // __isr

#ifndef _isr_autopsv
#define _isr_autopsv     __attribute__((interrupt,auto_psv))
#endif // _isr_autopsv

#ifndef __isr_autopsv
#define __isr_autopsv     __attribute__((interrupt,auto_psv))
#endif // __isr_autopsv

#ifndef _isr_noautopsv
#define _isr_noautopsv   __attribute__((interrupt,no_auto_psv))
#endif // _isr_noautopsv

#ifndef __isr_noautopsv
#define __isr_noautopsv   __attribute__((interrupt,no_auto_psv))
#endif // __isr_noautopsv

#ifndef _nacked
#define _nacked            __attribute__((nacked))
#endif // _nacked

#ifndef __nacked
#define __nacked            __attribute__((nacked))
#endif // __nacked

#ifndef _packed
#define _packed            __attribute__((packed))
#endif // _packed

#ifndef __packed
#define __packed            __attribute__((packed))
#endif // __packed

#ifndef __unused
#define __unused            __attribute__((unused))
#endif // __unused

#ifndef __io
#define __io   volatile
#endif // __packed

#ifndef _aligned
#define _aligned           __attribute__((aligned))
#endif // _aligned

#ifndef __aligned
#define __aligned           __attribute__((aligned))
#endif // __aligned

#ifndef _alias
#define _alias( _ALIAS_ )  __attribute__( (alias (_ALIAS_)) )
#endif // _alias

#ifndef __alias
#define __alias( _ALIAS_ )  __attribute__( (alias (_ALIAS_)) )
#endif // __alias

#ifndef _optimize_none
#define _optimize_none     __attribute__((optimize("-O0")) )
#endif // _optimize_none

#ifndef __optimize_none
#define __optimize_none     __attribute__((optimize("-O0")) )
#endif // __optimize_none

#ifndef _optimize_1
#define _optimize_1        __attribute__((optimize("-O1")) )
#endif // _optimize_1

#ifndef __optimize_1
#define __optimize_1        __attribute__((optimize("-O1")) )
#endif // __optimize_1

#ifndef _optimize_2
#define _optimize_2        __attribute__((optimize("-O2")) )
#endif // _optimize_2

#ifndef __optimize_2
#define __optimize_2        __attribute__((optimize("-O2")) )
#endif // __optimize_2

#ifndef _optimize_3
#define _optimize_3        __attribute__((optimize("-O3")) )
#endif // _optimize_3

#ifndef __optimize_3
#define __optimize_3        __attribute__((optimize("-O3")) )
#endif // __optimize_3

#ifndef _optimize_s
#define _optimize_s        __attribute__((optimize("-Os")) )
#endif // _optimize_s

#ifndef __optimize_s
#define __optimize_s        __attribute__((optimize("-Os")) )
#endif // __optimize_s

#ifndef _persistent
#define _persistent        __attribute__((persistent))
#endif // _persistent

#ifndef __persistent
#define __persistent        __attribute__((persistent))
#endif // __persistent

#ifndef null
#define null NULL
#endif // null

#ifndef nullptr
#define nullptr ((void *)NULL)
#endif // nullptr

#endif // defined ( __GNUC__ ) && !defined (__CC_ARM)


#ifdef __cplusplus
}
#endif

#endif // __TYPES_H

