/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2020 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/NXPMKL/clock_MKL.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Flash useful definitions for NXP MKL series
 */

#ifndef __FLASH_MKL_H
#define __FLASH_MKL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_MKL)

#define FLASH_WORD_SIZE                   (4)
#define FLASH_ROW_SIZE                    (8)
#define FLASH_PAGE_SIZE                   FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
//#define FLASH_BANK_SIZE                   (524288)  // = 0x00080000 [256 pages:  524288 byte]
#define FLASH_SIZE                        FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE

#define FLASH_MAX_PAGE_NUMBER             (FLASH_SIZE/FLASH_PAGE_SIZE)

/**
 *The list of all commands usable for manage flash.
 */
typedef enum _Flash_Command
{
    FLASH_COMMAND_READ_1S_SECTION    = 0x01,
    FLASH_COMMAND_PROGRAM_CHECK      = 0x02,
    FLASH_COMMAND_READ_RESOURCE      = 0x03,
    FLASH_COMMAND_PROGRAM_LONGWORD   = 0x06,
    FLASH_COMMAND_ERASE_FLASH_SECTOR = 0x09,
    FLASH_COMMAND_READ_1S_ALL_BLOCKS = 0x40,
    FLASH_COMMAND_READ_ONCE          = 0x41,
    FLASH_COMMAND_PROGRAM_ONCE       = 0x43,
    FLASH_COMMAND_ERASE_ALL_BLOCKS   = 0x44,
    FLASH_COMMAND_VERIFY_BACKDOOR    = 0x45,
} Flash_Command;

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // __FLASH_MKL_H
