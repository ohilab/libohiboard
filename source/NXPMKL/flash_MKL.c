/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019-2023 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/source/NXPMKL/flash_MKL.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief FLASH Function for implementing Reading and Writing on memory code for NXP MKL series
 */

#if defined(LIBOHIBOARD_FLASH)

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_MKL)

#include "utility.h"
#include "critical.h"
#include "flash.h"

#define FLASH_READ8(address)                  ((uint8_t)(*(volatile uint8_t*)(address)))

typedef struct _Flash_Device
{
    FTFA_Type* regmap;

} Flash_Device;

static Flash_Device flash0 =
{
    .regmap = FTFA,
};
Flash_DeviceHandle OB_FLASH0 = &flash0;

static inline __ramcode void  Flash_executeCommand (Flash_DeviceHandle dev)
{
    // Send commands
    dev->regmap->FSTAT |= FTFA_FSTAT_CCIF_MASK;
    // wait until hardware is hidle...
    while (!(dev->regmap->FSTAT & FTFA_FSTAT_CCIF_MASK));
}

#define FLASH_ERROR_MASK  (FTFA_FSTAT_FPVIOL_MASK   | \
                           FTFA_FSTAT_ACCERR_MASK   | \
                           FTFA_FSTAT_RDCOLERR_MASK | \
                           FTFA_FSTAT_MGSTAT0_MASK)

#define FLASH_ERROR_CLEAR_MASK  (FTFA_FSTAT_FPVIOL_MASK   | \
                                 FTFA_FSTAT_ACCERR_MASK   | \
                                 FTFA_FSTAT_RDCOLERR_MASK)

static System_Errors Flash_checkErrors (Flash_DeviceHandle dev)
{
    uint32_t error = (dev->regmap->FSTAT & (FLASH_ERROR_MASK));
    if (error)
    {
        dev->regmap->FSTAT |= (FLASH_ERROR_CLEAR_MASK);
        return ERRORS_FLASH_ERROR;
    }
    else
    {
        return ERRORS_NO_ERROR;
    }
}

System_Errors Flash_init (Flash_DeviceHandle dev)
{
    return Flash_checkErrors(dev);
}

System_Errors Flash_erasePage (Flash_DeviceHandle dev, uint16_t pageNumber)
{
    uint32_t pageAddr = pageNumber * FLASH_PAGE_SIZE;

    if (pageNumber > FLASH_MAX_PAGE_NUMBER)
    {
        return ERRORS_FLASH_ACCESS;
    }

    while (!(FTFA->FSTAT & FTFA_FSTAT_CCIF_MASK));

    // Erase Flash Sector Command - Erase all bytes in a program flash sector
    UTILITY_WRITE_REGISTER(dev->regmap->FCCOB0, FLASH_COMMAND_ERASE_FLASH_SECTOR);
    UTILITY_WRITE_REGISTER(dev->regmap->FCCOB1, (uint8_t)((pageAddr >> 16) & 0x000000FF));
    UTILITY_WRITE_REGISTER(dev->regmap->FCCOB2, (uint8_t)((pageAddr >> 8)  & 0x000000FF));
    UTILITY_WRITE_REGISTER(dev->regmap->FCCOB3, (uint8_t)( pageAddr        & 0x000000FF));

    __disable_irq();
    Flash_executeCommand(dev);
    __enable_irq();
    System_Errors err = Flash_checkErrors(dev);
    System_delay(100);
    return err;
}

System_Errors Flash_writePage (Flash_DeviceHandle dev, uint16_t pageNumber, uint8_t* buffer, uint32_t length)
{
    uint32_t pageAddr = pageNumber * FLASH_PAGE_SIZE;

    if (pageNumber > FLASH_MAX_PAGE_NUMBER)
    {
        return ERRORS_FLASH_ACCESS;
    }

    if (length > FLASH_PAGE_SIZE)
    {
        return ERRORS_FLASH_WRONG_PARAMS;
    }

    uint32_t tempSize = length;

    while (!(dev->regmap->FSTAT & FTFA_FSTAT_CCIF_MASK));

    while (tempSize > 0)
    {
        UTILITY_WRITE_REGISTER(dev->regmap->FCCOB0, FLASH_COMMAND_PROGRAM_LONGWORD);
        UTILITY_WRITE_REGISTER(dev->regmap->FCCOB1, (uint8_t)((pageAddr >> 16) & 0x000000FF));
        UTILITY_WRITE_REGISTER(dev->regmap->FCCOB2, (uint8_t)((pageAddr >> 8)  & 0x000000FF));
        UTILITY_WRITE_REGISTER(dev->regmap->FCCOB3, (uint8_t)( pageAddr        & 0x000000FF));

        UTILITY_WRITE_REGISTER(dev->regmap->FCCOB4, (uint8_t)(*(buffer + 3)));
        UTILITY_WRITE_REGISTER(dev->regmap->FCCOB5, (uint8_t)(*(buffer + 2)));
        UTILITY_WRITE_REGISTER(dev->regmap->FCCOB6, (uint8_t)(*(buffer + 1)));
        UTILITY_WRITE_REGISTER(dev->regmap->FCCOB7, (uint8_t)(*(buffer)));

        __disable_irq();
        Flash_executeCommand(dev);
        __enable_irq();

        Flash_checkErrors(dev);

        buffer   += 4;
        tempSize -= 4;
        pageAddr += 4;
    }
    return Flash_checkErrors(dev);
}

System_Errors Flash_readPage (Flash_DeviceHandle dev, uint16_t pageNumber, uint8_t* buffer, uint32_t length)
{
    uint32_t pageAddr = pageNumber * FLASH_PAGE_SIZE;

    if (pageNumber > FLASH_MAX_PAGE_NUMBER)
    {
        return ERRORS_FLASH_ACCESS;
    }

    if (length > FLASH_PAGE_SIZE)
    {
        return ERRORS_FLASH_WRONG_PARAMS;
    }

    for (uint32_t i = 0; i < length; ++i)
    {
        *buffer = FLASH_READ8((uint8_t *) pageAddr + i);
        buffer++;
    }

    return ERRORS_NO_ERROR;
}

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_FLASH


