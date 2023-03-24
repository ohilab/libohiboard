/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2023 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 *  Lorenzo Compagnucci
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
 * @file libohiboard/include/flash.h
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief FLASH definitions and prototypes.
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_FLASH

/**
 * @defgroup FLASH
 * @brief FLASH Reading and Writing on memory code
 * @{
 */

#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "system.h"

typedef struct _Flash_Device* Flash_DeviceHandle;

#if defined (LIBOHIBOARD_PIC24FJ)

#include "hardware/PIC24FJ/flash_PIC24FJ.h"

#elif defined (LIBOHIBOARD_STM32L4)

//#include "hardware/STM32L4/flash_STM32L4.h"

#elif defined (LIBOHIBOARD_STM32L0)

#include "hardware/STM32L0/flash_STM32L0.h"

#elif defined (LIBOHIBOARD_MKL)

#include "hardware/NXPMKL/flash_MKL.h"

#endif

/**
 * Initialize the FLASH control register.
 * 
 * @retval error
 */
System_Errors Flash_init (Flash_DeviceHandle dev);

/**
 * Erase a page.
 *
 * @param[in]        dev: The peripheral handle
 * @param[in] pageNumber: progression number of page
 * @retval error
 */
System_Errors Flash_erasePage (Flash_DeviceHandle dev, uint16_t pageNumber);

/**
 * Write data to the given page.
 *
 * @param[in]        dev: The peripheral handle
 * @param[in] pageNumber: The page number of the page to write
 * @param[out]    buffer: Array of data
 * @param[in]     length: Length of data
 * @retval error
 */
System_Errors Flash_writePage (Flash_DeviceHandle dev, uint16_t pageNumber, uint8_t* buffer, uint32_t length);

/**
 * Read data from given page.
 *
 * @param[in]        dev: The peripheral handle
 * @param[in] pageNumber: The page number of the page to read
 * @param[in]     buffer: Array of data to save data
 * @param[in]     length: Length of data
 * @retval error
 */
System_Errors Flash_readPage (Flash_DeviceHandle dev, uint16_t pageNumber, uint8_t* buffer, uint32_t length);

#if 0
/**
 * Unlock the FLASH control register access.
 * 
 * @retval error
 */
System_Errors Flash_unlock (void);

/**
 * Lock the FLASH control register access.
 * 
 * @retval error
 */
System_Errors Flash_lock (void);

/**
 * Gets the bank of a given address
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @retval The bank of a given address
 */
uint32_t Flash_getBank (uint32_t address);

/**
 * Gets the page of a given address
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @retval The page of a given address
 */
uint32_t Flash_getPage (uint32_t address);

/**
 * Returns the size in word of a Page
 */
uint32_t Flash_getPageSize (void);

/**
 * Gets the row of a given address
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @retval The row of a given address
 */
uint32_t Flash_getRow (uint32_t address);

/**
 * Returns the size in word of a Row
 */
uint32_t Flash_getRowSize (void);

/**
 * Read a word of code from a given address
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @retval The word of a given address
 */
uint32_t Flash_readWord (uint32_t address);

/**
 * Read a row of code from a given address
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @param[OUT] words: The row of a given address
 * @retval error
 */
System_Errors Flash_readRow (uint32_t address, uint32_t *words);

/**
 * Erase a bank
 * 
 * @param[IN] bank: progression number of bank
 * @retval error
 */
System_Errors Flash_eraseBank (uint32_t bank);

/**
 * Erase a page
 * 
 * @param[IN] page: progression number of page
 * @retval error
 */
System_Errors Flash_erasePage (uint32_t page);

/**
 * Erase a sector
 * 
 * @param[IN] pageStart: page number from which the deletion begins
 * @param[IN] numPages: number of pages to erase
 * @retval error
 */
System_Errors Flash_eraseSector (uint32_t pageStart, uint32_t numPages);

/**
 * Erase data from specified address 
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @param[IN] length: Length of data [Byte]
 * @retval error
 */
System_Errors Flash_erase (uint32_t address, uint32_t length);

/**
 * Write double word at given address
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @param[IN] word1: most significant word
 * @param[IN] word2: least significant word
 * @retval error
 */
System_Errors Flash_writeDoubleWord (uint32_t address, uint32_t word1, uint32_t word2);

/**
 * Write a row from given address
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @param[IN] word: array of words
 * @retval error
 */
System_Errors Flash_writeRow (uint32_t address, uint32_t *words);

/**
 * Read data from given address
 * 
 * @note The data is stored in code are using only the two less significat
 *       byte of word.
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @param[IN] data: Array of data
 * @param[IN] length: Length of data [Byte]
 * @retval error
 */
System_Errors Flash_readData (uint32_t address, uint8_t *data, uint16_t length);

/**
 * Write data from given address
 * 
 * @note The data is stored in code are using only the two less significat
 *       byte of word.
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @param[OUT] data: Array of data
 * @param[IN] length: Length of data [Byte]
 * @retval error
 */
System_Errors Flash_writeData (uint32_t address, uint8_t *data, uint16_t length);

/**
 * Test if page of given address is empty
 * 
 * @param[IN] address: Address of the FLASH Memory
 * @retval true: page is empty; false: otherwise.
 */
bool Flash_isPageEmpty (uint32_t address);

/**
 * Test if two page of given addresses are equal
 * 
 * @param[IN] address1: First address of the FLASH Memory
 * @param[IN] address2: Second address of the FLASH Memory
 * @retval true: pages are equal, false: otherwise.
 */
bool Flash_pagecmp (uint32_t address1, uint32_t address2);

/**
 * Copy the content from source to destination for the specified length
 * 
 * @param[IN] destAddress: Destination address of the FLASH Memory
 * @param[IN] sourceAddress: Source address of the FLASH Memory
 * @param[IN] length: Length of data [Byte]
 * @retval true: copy operation worked well, false: otherwise.
 */
bool Flash_memcpy (uint32_t destAddress, uint32_t sourceAddress, uint32_t length);

/**
 * Test the Flash functions.
 * @note erase the settings of device.
 * @retval result of test: true: operation worked well, false: otherwise.
 */
bool Flash_test (void);
#endif

#ifdef __cplusplus
}
#endif

#endif // __FLASH_H

/**
 * @}
 */

#endif // LIBOHIBOARD_UART

/**
 * @}
 */
