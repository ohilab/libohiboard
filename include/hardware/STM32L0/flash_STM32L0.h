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

#define FLASH_WORD_SIZE                          (4)
#define FLASH_ROW_SIZE                           (8)

#define FLASH_PAGE_SIZE                          (128U)

#define FLASH_SIZE                               (uint32_t)((*((uint32_t *)FLASHSIZE_BASE)&0xFFFF) * 1024U)
#define FLASH_MAX_PAGE_NUMBER                    (FLASH_SIZE/FLASH_PAGE_SIZE)

#define FLASH_START                              (FLASH_BASE)
#define FLASH_END                                (FLASH_BASE + FLASH_SIZE - 1)

#define FLASH_TIMEOUT_VALUE                      (50000u) // 50s

/** @defgroup FLASH_Keys FLASH Keys
  * @{
  */

#define FLASH_PDKEY1                             (0x04152637U) /*!< Flash power down key1 */
#define FLASH_PDKEY2                             (0xFAFBFCFDU) /*!< Flash power down key2: used with FLASH_PDKEY1
                                                                    to unlock the RUN_PD bit in FLASH_ACR */

#define FLASH_PEKEY1                             (0x89ABCDEFU) /*!< Flash program erase key1 */
#define FLASH_PEKEY2                             (0x02030405U) /*!< Flash program erase key: used with FLASH_PEKEY2
                                                                   to unlock the write access to the FLASH_PECR register and
                                                                   data EEPROM */

#define FLASH_PRGKEY1                            (0x8C9DAEBFU) /*!< Flash program memory key1 */
#define FLASH_PRGKEY2                            (0x13141516U) /*!< Flash program memory key2: used with FLASH_PRGKEY2
                                                                   to unlock the program memory */

#define FLASH_OPTKEY1                            (0xFBEAD9C8U) /*!< Flash option key1 */
#define FLASH_OPTKEY2                            (0x24252627U) /*!< Flash option key2: used with FLASH_OPTKEY1 to
                                                                            unlock the write access to the option byte block */
/**
  * @}
  */

extern Flash_DeviceHandle OB_FLASH0;

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // __FLASH_STM32L0_H
