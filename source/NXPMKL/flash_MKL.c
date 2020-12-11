/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
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

typedef struct _Flash_Device
{
    FTFA_Type* regmap;

} Flash_Device;

static Flash_Device flash0 =
{
    .regmap = FTFA,
};

static inline void __attribute__((always_inline)) Flash_executeCommand (void)
{
    // Send commands
    flash0.regmap->FSTAT |= FTFA_FSTAT_CCIF_MASK;
    // wait until hardware is hidle...
    while (!(flash0.regmap->FSTAT & FTFA_FSTAT_CCIF_MASK));
}

#define FLASH_ERROR_MASK  (FTFA_FSTAT_FPVIOL_MASK   | \
                           FTFA_FSTAT_ACCERR_MASK   | \
                           FTFA_FSTAT_RDCOLERR_MASK | \
                           FTFA_FSTAT_MGSTAT0_MASK)

#define FLASH_ERROR_CLEAR_MASK  (FTFA_FSTAT_FPVIOL_MASK   | \
                                 FTFA_FSTAT_ACCERR_MASK   | \
                                 FTFA_FSTAT_RDCOLERR_MASK)

static System_Errors Flash_checkErrors (void)
{
    uint32_t error = (flash0.regmap->FSTAT & (FLASH_ERROR_MASK));
    if (error) {
        flash0.regmap->FSTAT |= (FLASH_ERROR_CLEAR_MASK);
        return ERRORS_FLASH_ERROR;
    }
    else {
        return ERRORS_NO_ERROR;
    }
}

System_Errors Flash_init (void)
{

}

System_Errors Flash_eraseBank (uint32_t bank)
{
    // Not implemented!
    return ohiassert(0);
}

System_Errors Flash_erasePage (uint32_t page)
{
    uint32_t address = page * FLASH_PAGE_SIZE;

    if (page > FLASH_MAX_PAGE_NUMBER)
    {
        return ERRORS_FLASH_ACCESS;
    }

    while(!(FTFA->FSTAT & FTFA_FSTAT_CCIF_MASK));

    UTILITY_WRITE_REGISTER(flash0.regmap->FCCOB0, FLASH_COMMAND_ERASE_FLASH_SECTOR);
    UTILITY_WRITE_REGISTER(flash0.regmap->FCCOB1, (uint8_t)((address >> 16) & 0x000000FF));
    UTILITY_WRITE_REGISTER(flash0.regmap->FCCOB2, (uint8_t)((address >> 8)  & 0x000000FF));
    UTILITY_WRITE_REGISTER(flash0.regmap->FCCOB3, (uint8_t)( address        & 0x000000FF));

    Flash_executeCommand();
}

System_Errors Flash_eraseSector (uint32_t pageStart, uint32_t numPages)
{
    for (uint32_t i = 0; i < numPages; i++)
    {
        Flash_erasePage(pageStart + i);
    }
//
//    return Flash_init();
}

System_Errors Flash_erase (uint32_t address, uint32_t length)
{

}


#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_FLASH


