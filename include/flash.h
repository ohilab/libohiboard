/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2017 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Matteo Piersantelli
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
 * @file libohiboard/include/flash.h
 * @author Matteo Piersantelli
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief FLASH definitions and prototypes.
 */

#ifdef LIBOHIBOARD_FLASH

#ifndef __FLASH_H
#define __FLASH_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

// Usefull defines
#define FLASH_WRITE16(address, value)         (*(volatile uint16_t*)(address) = (value))
#define FLASH_READ16(address)                 ((uint16_t)(*(volatile uint16_t*)(address)))

#define FLASH_WRITE8(address, value)          (*(volatile uint8_t*)(address) = (value))
#define FLASH_READ8(address)                  ((uint8_t)(*(volatile uint8_t*)(address)))

#define FLASH_GET_BIT_0_7(value)              ((uint8_t)((value) & 0xFFU))
#define FLASH_GET_BIT_8_15(value)             ((uint8_t)(((value)>>8) & 0xFFU))
#define FLASH_GET_BIT_16_23(value)            ((uint8_t)(((value)>>16) & 0xFFU))
#define FLASH_GET_BIT_24_31(value)            ((uint8_t)((value)>>24))

typedef struct Flash_Device* Flash_DeviceHandle;

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

extern Flash_DeviceHandle OB_FLASH0;

#endif

/**
 * This function initialize the flash informations for user data
 *
 * @param dev The device handle
 * @param sectorNumbers The number of memory sector to save user data
 * @retval ERRORS_NO_ERROR No problem during flash initialization
 */
System_Errors Flash_init (Flash_DeviceHandle dev, uint8_t sectorNumbers);

/**
 * This function read a 16bit value at the specified index. The index start
 * from user location start address. In this case each index value represent
 * a 16bit location.
 *
 * @param dev The device handle
 * @param index The position of the data.
 * @return the read value
 */
uint16_t Flash_readLocation (Flash_DeviceHandle dev, uint16_t index);

/**
 * This function read a 8bit value at the specified index. The index start
 * from user location start address. In this case each index value represent
 * a 8bit location.
 *
 * @param dev The device handle
 * @param index The position of the data.
 * @return the read value
 */
uint8_t Flash_readLocation8 (Flash_DeviceHandle dev, uint16_t index);

/**
 * This function erase the specified sector.
 *
 * @param dev The device handle
 * @param sectorNumber The current sector number to erase
 * @retval ERRORS_NO_ERROR No problem during erasing
 * @retval ERRORS_FLASH_PROTECTION_VIOLATION The flash sector is protected
 * @retval ERRORS_FLASH_ACCESS In case of invalid argument
 */
System_Errors Flash_EraseSector (Flash_DeviceHandle dev, uint8_t sectorNumber);

/**
 * This function write data into memory starting from start address of user dedicated
 * location.
 *
 * @param dev The device handle
 * @param buffer The data buffer to write into flash memory
 * @param size The number of byte must be write
 */
System_Errors Flash_writeBuffer (Flash_DeviceHandle dev, uint8_t *buffer, uint32_t size);

#endif /* __FLASH_H */

#endif /* LIBOHIBOARD_FLASH */
