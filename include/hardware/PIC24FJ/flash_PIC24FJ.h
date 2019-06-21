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
 * @file libohiboard/include/hardware/PIC24FJ/flash_PIC24FJ.h
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief FLASH Function for implementing Reading and Writing on memory code
 */

#ifndef __FLASH_PIC24FJ_H
#define __FLASH_PIC24FJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "types.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_FLASH) && defined(LIBOHIBOARD_PIC24FJ)

/**
 * @addtogroup FLASH
 * @{
 */

/**
 * @defgroup FLASH Reading and Writing on memory code
 * @{
 */

#if defined (LIBOHIBOARD_PIC24FJ1024)

#define FLASH_ENABLE_OP                   0x4000
#define FLASH_ERASE_IP                    (FLASH_ENABLE_OP | 0b0100) //Inactive Partition erase operation
#define FLASH_ERASE_PAGE                  (FLASH_ENABLE_OP | 0b0011) //Page erase operation
#define FLASH_ROW_WRITE                   (FLASH_ENABLE_OP | 0b0010) //Row program operation
#define FLASH_DWORD_WRITE                 (FLASH_ENABLE_OP | 0b0001) //Double-word program operation

#define FLASH_ROW_SIZE_B                  (0x00000200ul) //512 byte
#define FLASH_ROW_MASK_B                  (0x007FFC00ul)
#define FLASH_ROW_SIZE_W                  (0x00000100ul) //256 word
#define FLASH_ROW_MASK_W                  (0x007FFE00ul)
#define FLASH_ROW_SIZE_I                  (0x00000080ul) //128 instructions (384 bytes)

#define FLASH_PAGE_SIZE_B                 (0x00001000ul) //4096 byte
#define FLASH_PAGE_MASK_B                 (0x007FF000ul)
#define FLASH_PAGE_SIZE_W                 (0x00000800ul) //2048 word
#define FLASH_PAGE_MASK_W                 (0x007FF800ul)
#define FLASH_PAGE_SIZE_I                 (0x00000400ul) //1024 instructions (3072 bytes)

#define FLASH_EMPTY_INSTRUCTION           (0x00FFFFFFul)
#define FLASH_ADDRESS_MASK                (0x007FFFFEul)

#define ADDRESS_START_PRIM_PARTITION      (0x00000000ul)
#define ADDRESS_END_PRIM_PARTITION        (0x00050000ul)
#define LENGTH_PRIM_PARTITION             (ADDRESS_END_PRIM_PARTITION - ADDRESS_START_PRIM_PARTITION)
#define ADDRESS_START_DUAL_PARTITION      (0x00400000ul)
#define ADDRESS_END_DUAL_PARTITION        (0x00450000ul)
#define LENGTH_DUAL_PARTITION             (ADDRESS_END_DUAL_PARTITION - ADDRESS_START_DUAL_PARTITION)
#define BOOT_SECTOR_DATA_ADR              (ADDRESS_END_DUAL_PARTITION)

#endif // defined (LIBOHIBOARD_PIC24FJ1024)

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_FLASH & LIBOHIBOARD_PIC24FJ

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __FLASH_PIC24FJ_H
