/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/include/hardware/gpio_MKL.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPIO pins and device definitions for NXP MKL series
 */

#ifndef __GPIO_MKL_H
#define __GPIO_MKL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_MKL)

typedef enum
{
    GPIO_PINS_NONE = 0,

#if defined (LIBOHIBOARD_MKL15)

    GPIO_PINS_PTA0,
    GPIO_PINS_PTA1,
    GPIO_PINS_PTA2,
    GPIO_PINS_PTA3,
    GPIO_PINS_PTA4,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTA5,
    GPIO_PINS_PTA12,
    GPIO_PINS_PTA13,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTA14,
    GPIO_PINS_PTA15,
    GPIO_PINS_PTA16,
    GPIO_PINS_PTA17,
#endif
    GPIO_PINS_PTA18,
    GPIO_PINS_PTA19,
    GPIO_PINS_PTA20,

    GPIO_PINS_PTB0,
    GPIO_PINS_PTB1,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTB2,
    GPIO_PINS_PTB3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTB8,
    GPIO_PINS_PTB9,
    GPIO_PINS_PTB10,
    GPIO_PINS_PTB11,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTB16,
    GPIO_PINS_PTB17,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTB18,
    GPIO_PINS_PTB19,
#endif

#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTC0,
#endif
    GPIO_PINS_PTC1,
    GPIO_PINS_PTC2,
    GPIO_PINS_PTC3,
    GPIO_PINS_PTC4,
    GPIO_PINS_PTC5,
    GPIO_PINS_PTC6,
    GPIO_PINS_PTC7,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTC8,
    GPIO_PINS_PTC9,
    GPIO_PINS_PTC10,
    GPIO_PINS_PTC11,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTC12,
    GPIO_PINS_PTC13,
    GPIO_PINS_PTC16,
    GPIO_PINS_PTC17,
#endif

#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTD0,
    GPIO_PINS_PTD1,
    GPIO_PINS_PTD2,
    GPIO_PINS_PTD3,
#endif
    GPIO_PINS_PTD4,
    GPIO_PINS_PTD5,
    GPIO_PINS_PTD6,
    GPIO_PINS_PTD7,

#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTE0,
    GPIO_PINS_PTE1,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTE2,
    GPIO_PINS_PTE3,
    GPIO_PINS_PTE4,
    GPIO_PINS_PTE5,
#endif
    GPIO_PINS_PTE16,
    GPIO_PINS_PTE17,
    GPIO_PINS_PTE18,
    GPIO_PINS_PTE19,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTE20,
    GPIO_PINS_PTE21,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTE22,
    GPIO_PINS_PTE23,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTE24,
    GPIO_PINS_PTE25,
    GPIO_PINS_PTE29,
#endif
    GPIO_PINS_PTE30,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    GPIO_PINS_PTE31,
#endif

// ENDIF_ LIBOHIBOARD_MKL15
#endif

} Gpio_Pins;

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // __GPIO_MKL_H
