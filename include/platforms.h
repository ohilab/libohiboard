/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2018 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/platforms.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Platform definition.
 */

#ifndef __PLATFORMS_H
#define __PLATFORMS_H

/* microcontroller selection: */

#if defined(LIBOHIBOARD_K10DZ10) // NXP/Freescale Microcontrollers
#include "platforms/MK10DZ10.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_K10D10)
#include "platforms/MK10D10.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_K10D7)
#include "platforms/MK10D7.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_K12D5)
#include "platforms/MK12D5.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_K60DZ10)
#include "platforms/MK60DZ10.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_K60F15)
#include "platforms/MK60F15.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_KL02Z4)
#include "platforms/MKL02Z4.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_KL03Z4)
#include "platforms/MKL03Z4.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_KL15Z4)
#include "platforms/MKL15Z4.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_KL25Z4)
#include "platforms/MKL25Z4.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_K64F12)
#include "platforms/MK64F12.h"
#include "platforms/MK64F12_portability.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_FRDMKL02Z)
#include "platforms/MKL02Z4.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_FRDMKL03Z)
#include "platforms/MKL03Z4.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_FRDMKL05Z)
#include "platforms/MKL05Z4.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_FRDMKL25Z) || defined(LIBOHIBOARD_KL25Z4)
#include "platforms/MKL25Z4.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_FRDMK20D50M)
#include "platforms/MK20D5.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_FRDMK64F)
#include "platforms/MK64F12.h"
#include "platforms/MK64F12_portability.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_OHIBOARD_R1)
#include "platforms/MK60DZ10.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_KV31F12)
#include "platforms/MKV31F51212.h"
#define LIBOHIBOARD_NXP_KINETIS
#elif defined(LIBOHIBOARD_KV46F) || defined(LIBOHIBOARD_TWRKV46F)
#include "platforms/MKV46F16.h"
#define LIBOHIBOARD_NXP_KINETIS

#elif defined (LIBOHIBOARD_MKL15ZxFM) || \
      defined (LIBOHIBOARD_MKL15ZxFT) || \
      defined (LIBOHIBOARD_MKL15ZxLH) || \
      defined (LIBOHIBOARD_MKL15ZxLK)

#include "platforms/MKL15/MKL15Z4.h"

#ifndef LIBOHIBOARD_MKL
#define LIBOHIBOARD_MKL
#endif

#ifndef LIBOHIBOARD_MKL15
#define LIBOHIBOARD_MKL15
#endif

#ifndef LIBOHIBOARD_NXP_KINETIS
#define LIBOHIBOARD_NXP_KINETIS
#endif

#elif defined (LIBOHIBOARD_MKL25ZxFM) || \
      defined (LIBOHIBOARD_MKL25ZxFT) || \
      defined (LIBOHIBOARD_MKL25ZxLH) || \
      defined (LIBOHIBOARD_MKL25ZxLK)

#include "platforms/MKL25/MKL25Z4.h"
#include "platforms/MKL25/MKL25Z4_features.h"

#ifndef LIBOHIBOARD_MKL
#define LIBOHIBOARD_MKL
#endif

#ifndef LIBOHIBOARD_MKL25
#define LIBOHIBOARD_MKL25
#endif

#ifndef LIBOHIBOARD_NXP_KINETIS
#define LIBOHIBOARD_NXP_KINETIS
#endif

#elif defined (LIBOHIBOARD_STM32L073RxT) || \
      defined (LIBOHIBOARD_STM32L073RxI) || \
      defined (LIBOHIBOARD_STM32L073CxT) || \
      defined (LIBOHIBOARD_STM32L073VxT) || \
      defined (LIBOHIBOARD_STM32L073VxI)

#include "platforms/STM32L073/stm32l073xx.h"
#include "platforms/STM32L073/stm32l0xx.h"
#include "platforms/STM32L073/system_stm32l0xx.h"

#ifndef LIBOHIBOARD_STM32L0
#define LIBOHIBOARD_STM32L0
#endif

#ifndef LIBOHIBOARD_STM32L073
#define LIBOHIBOARD_STM32L073
#endif

#ifndef LIBOHIBOARD_ST_STM32
#define LIBOHIBOARD_ST_STM32
#endif

#elif defined (LIBOHIBOARD_STM32L476Jx) || \
      defined (LIBOHIBOARD_STM32L476Rx)

#include "platforms/STM32L476/stm32l476xx.h"
#include "platforms/STM32L476/stm32l4xx.h"
#include "platforms/STM32L476/system_stm32l4xx.h"

#ifndef LIBOHIBOARD_STM32L4
#define LIBOHIBOARD_STM32L4
#endif

#ifndef LIBOHIBOARD_STM32L476
#define LIBOHIBOARD_STM32L476
#endif

#ifndef LIBOHIBOARD_ST_STM32
#define LIBOHIBOARD_ST_STM32
#endif

#elif defined (LIBOHIBOARD_STM32WB55Rx)

#include "platforms/STM32WB55/stm32wb55xx.h"
#include "platforms/STM32WB55/stm32wbxx.h"
#include "platforms/STM32WB55/system_stm32wbxx.h"

#ifndef LIBOHIBOARD_STM32WB
#define LIBOHIBOARD_STM32WB
#endif

#ifndef LIBOHIBOARD_STM32WB55
#define LIBOHIBOARD_STM32WB55
#endif

#ifndef LIBOHIBOARD_ST_STM32
#define LIBOHIBOARD_ST_STM32
#endif

#elif defined (LIBOHIBOARD_PIC24FJxGA606) || \
      defined (LIBOHIBOARD_PIC24FJxGA610) || \
      defined (LIBOHIBOARD_PIC24FJxGB606) || \
      defined (LIBOHIBOARD_PIC24FJxGB610)

#include "platforms/PIC24FJ/PIC24FJxGy6z.h"
#include <xc.h>

#ifndef LIBOHIBOARD_PIC24FJ
#define LIBOHIBOARD_PIC24FJ
#endif

#if defined (__PIC24FJ1024GA606__) || defined (__PIC24FJ1024GA610__) || \
    defined (__PIC24FJ1024GB606__) || defined (__PIC24FJ1024GB610__)
#ifndef LIBOHIBOARD_PIC24FJ1024
#define LIBOHIBOARD_PIC24FJ1024
#endif
#endif

#ifndef LIBOHIBOARD_MICROCHIP_PIC
#define LIBOHIBOARD_MICROCHIP_PIC
#endif

#endif

// Features files
// To use this file the user must define the correct microcontroller
#if defined(LIBOHIBOARD_K64F12) & defined(LIBOHIBOARD_FLASH)
#include "platforms/MK64F12_features.h"
#endif

#endif // __PLATFORMS_H
