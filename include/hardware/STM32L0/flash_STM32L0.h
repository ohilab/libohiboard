/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2023 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/STM32L0/clock_STM32L0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Flash useful definitions for STM32L0 series
 */

#ifndef __FLASH_STM32L0_H
#define __FLASH_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_STM32L0)

#define FLASH_WORD_SIZE                   (4)
#define FLASH_ROW_SIZE                    (8)

#define FLASH_PAGE_SIZE                   (128U)

#define FLASH_SIZE                        (uint32_t)((*((uint32_t *)FLASHSIZE_BASE)&0xFFFF) * 1024U)
#define FLASH_MAX_PAGE_NUMBER             (FLASH_SIZE/FLASH_PAGE_SIZE)

#define FLASH_END                         (FLASH_BASE + FLASH_SIZE - 1)


extern Flash_DeviceHandle OB_FLASH0;

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // __FLASH_STM32L0_H
