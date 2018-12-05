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
 * @file libohiboard/include/hardware/clock_MKL.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Clock useful definitions for NXP MKL series
 */

#ifndef __CLOCK_MKL_H
#define __CLOCK_MKL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_MKL)

#define CLOCK_FREQ_INTERNAL_LIRC          ((uint32_t)32000u)
#define CLOCK_FREQ_INTERNAL_HIRC        ((uint32_t)4000000u)

#define CLOCK_FREQ_OSC_IN_RANGE0_MIN      ((uint32_t)32000u)
#define CLOCK_FREQ_OSC_IN_RANGE0_MAX      ((uint32_t)40000u)
#define CLOCK_FREQ_OSC_IN_RANGE1_MIN    ((uint32_t)3000000u)
#define CLOCK_FREQ_OSC_IN_RANGE1_MAX    ((uint32_t)8000000u)
#define CLOCK_FREQ_OSC_IN_RANGE2_MIN    ((uint32_t)8000000u)
#define CLOCK_FREQ_OSC_IN_RANGE2_MAX   ((uint32_t)32000000u)

#define CLOCK_FREQ_EXT_MAX             ((uint32_t)48000000u)

#define CLOCK_FREQ_FLL_LOWRANGE_MIN    ((uint32_t)20000000u)
#define CLOCK_FREQ_FLL_LOWRANGE_CENTER ((uint32_t)20970000u)
#define CLOCK_FREQ_FLL_LOWRANGE_MAX    ((uint32_t)25000000u)
#define CLOCK_FREQ_FLL_MIDRANGE_MIN    ((uint32_t)40000000u)
#define CLOCK_FREQ_FLL_MIDRANGE_CENTER ((uint32_t)41940000u)
#define CLOCK_FREQ_FLL_MIDRANGE_MAX    ((uint32_t)48000000u)

#define CLOCK_FREQ_FLL_INPUT_MAX                   (39062.5)

#define CLOCK_FREQ_PLL_INPUT_MIN        ((uint32_t)2000000u)
#define CLOCK_FREQ_PLL_INPUT_MAX        ((uint32_t)4000000u)

#define CLOCK_FREQ_MCGOUT_MAX         ((uint32_t)100000000u)
#define CLOCK_FREQ_MCGFLL_MAX          ((uint32_t)48000000u)
#define CLOCK_FREQ_MCGPLL_MAX         ((uint32_t)100000000u)

#define CLOCK_REG_PRDIV_MIN                             (1u)
#define CLOCK_REG_PRDIV_MAX                            (25u)

#define CLOCK_REG_VDIV_MIN                             (24u)
#define CLOCK_REG_VDIV_MAX                             (55u)

#define CLOCK_REG_OUTDIV1_MIN                           (1u)
#define CLOCK_REG_OUTDIV1_MAX                          (16u)

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // __CLOCK_MKL_H
