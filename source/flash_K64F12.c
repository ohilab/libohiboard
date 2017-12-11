/******************************************************************************
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
 ******************************************************************************/

/**
 * @file libohiboard/source/flash_K64F12.h
 * @author Matteo Piersantelli
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief FLASH implementations for K64F12 and FRDMK64F.
 */

#ifdef LIBOHIBOARD_FLASH

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

#include "platforms.h"
#include "flash.h"

#define FTFx_VERIFY_BLOCK               0x00U
#define FTFx_VERIFY_SECTION             0x01U
#define FTFx_PROGRAM_CHECK              0x02U
#define FTFx_READ_RESOURCE              0x03U
//#define FTFx_PROGRAM_LONGWORD           0x06U
#define FTFx_PROGRAM_PHRASE             0x07U
#define FTFx_ERASE_BLOCK                0x08U
#define FTFx_ERASE_SECTOR               0x09U
#define FTFx_PROGRAM_SECTION            0x0BU
#define FTFx_VERIFY_ALL_BLOCK           0x40U
#define FTFx_READ_ONCE                  0x41U
#define FTFx_PROGRAM_ONCE               0x43U
#define FTFx_ERASE_ALL_BLOCK            0x44U
#define FTFx_SECURITY_BY_PASS           0x45U
#define FTFx_PFLASH_SWAP                0x46U
#define FTFx_PROGRAM_PARTITION          0x80U
#define FTFx_SET_EERAM                  0x81U


typedef struct Flash_Device
{
    FTFE_MemMapPtr regMap;                          /**< Device memory pointer */

    uint32_t startAddr;
    uint32_t flashSize;
    uint32_t sectorSize;

    uint32_t userStartAddr;
    uint32_t userStopAddr;

    uint8_t blockSize;

    uint8_t commandArray[12];

    bool isInit;

} Flash_Device;

static Flash_Device flash0 = {
        .regMap           = FTFE_BASE_PTR,
        .isInit           = FALSE,
};
Flash_DeviceHandle OB_FLASH0 = &flash0;

static System_Errors Flash_sequenceCommand (Flash_DeviceHandle dev)
{
    // check CCIF bit of the flash status register
    // wait until CCIF bit is set
    while(0 == (FTFE_FSTAT_REG(dev->regMap) && FTFE_FSTAT_CCIF_MASK));

    // Clear RDCOLLERR, ACCERR and FPVIOL flag into status register
    FTFE_FSTAT_REG(dev->regMap) = FTFE_FSTAT_RDCOLERR_MASK |
                                  FTFE_FSTAT_ACCERR_MASK |
                                  FTFE_FSTAT_FPVIOL_MASK;

    FTFE_FCCOB0_REG(dev->regMap) = dev->commandArray[0];
    FTFE_FCCOB1_REG(dev->regMap) = dev->commandArray[1];
    FTFE_FCCOB2_REG(dev->regMap) = dev->commandArray[2];
    FTFE_FCCOB3_REG(dev->regMap) = dev->commandArray[3];
    FTFE_FCCOB4_REG(dev->regMap) = dev->commandArray[4];
    FTFE_FCCOB5_REG(dev->regMap) = dev->commandArray[5];
    FTFE_FCCOB6_REG(dev->regMap) = dev->commandArray[6];
    FTFE_FCCOB7_REG(dev->regMap) = dev->commandArray[7];
    FTFE_FCCOB8_REG(dev->regMap) = dev->commandArray[8];
    FTFE_FCCOB9_REG(dev->regMap) = dev->commandArray[9];
    FTFE_FCCOBA_REG(dev->regMap) = dev->commandArray[10];
    FTFE_FCCOBB_REG(dev->regMap) = dev->commandArray[11];

    // Clear CCIF flag into status register
    FTFE_FSTAT_REG(dev->regMap) = FTFE_FSTAT_CCIF_MASK;

    if (FTFE_FSTAT_REG(dev->regMap) & FTFE_FSTAT_ACCERR_MASK)
        return ERRORS_FLASH_ACCESS;
    else if (FTFE_FSTAT_REG(dev->regMap) & FTFE_FSTAT_RDCOLERR_MASK)
        return ERRORS_FLASH_ACCESS;
    else if (FTFE_FSTAT_REG(dev->regMap) & FTFE_FSTAT_FPVIOL_MASK)
        return ERRORS_FLASH_PROTECTION_VIOLATION;
    else
        return ERRORS_NO_ERROR;
}

static System_Errors Flash_writeLongWord (Flash_DeviceHandle dev, uint32_t address, uint8_t* data)
{
    dev->commandArray[0] = FTFx_PROGRAM_PHRASE;
    dev->commandArray[1] = FLASH_GET_BIT_16_23(address);
    dev->commandArray[2] = FLASH_GET_BIT_8_15(address);
    dev->commandArray[3] = FLASH_GET_BIT_0_7(address);
    dev->commandArray[4] = FLASH_READ8((data + 3));
    dev->commandArray[5] = FLASH_READ8((data + 2));
    dev->commandArray[6] = FLASH_READ8((data + 1));
    dev->commandArray[7] = FLASH_READ8((data + 0));
    dev->commandArray[8] = FLASH_READ8((data + 7));
    dev->commandArray[9] = FLASH_READ8((data + 6));
    dev->commandArray[10] = FLASH_READ8((data + 5));
    dev->commandArray[11] = FLASH_READ8((data + 4));

    return Flash_sequenceCommand(dev);
}

System_Errors Flash_init (Flash_DeviceHandle dev, uint8_t sectorNumbers)
{
    if (dev->isInit == TRUE) return ERRORS_FLASH_JUST_INIT;

    // Save memory address
    dev->startAddr = FSL_FEATURE_FLASH_PFLASH_START_ADDRESS;
    dev->flashSize = FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE * FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT;
    dev->sectorSize = FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE;

    dev->userStartAddr = dev->startAddr + dev->flashSize - (dev->sectorSize * sectorNumbers);
    dev->userStopAddr = dev->startAddr + dev->flashSize;

    dev->blockSize = FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE;

    // Init command array to 0x00
    for (uint8_t i = 0; i < 12; ++i)
        dev->commandArray[i] = 0x00;

    return ERRORS_NO_ERROR;
}

uint16_t Flash_readLocation (Flash_DeviceHandle dev, uint16_t index)
{
    return FLASH_READ16((uint16_t *) dev->userStartAddr + index);
}

uint8_t Flash_readLocation8 (Flash_DeviceHandle dev, uint16_t index)
{
    return FLASH_READ8((uint8_t *) dev->userStartAddr + index);
}

System_Errors Flash_EraseSector (Flash_DeviceHandle dev, uint8_t sectorNumber)
{
    uint32_t sectorAddr = dev->userStartAddr + (dev->sectorSize * sectorNumber);

    // Erase Flash Sector Command - Erase all bytes in a program flash sector
    dev->commandArray[0] = FTFx_ERASE_SECTOR;
    dev->commandArray[1] = FLASH_GET_BIT_16_23(sectorAddr);
    dev->commandArray[2] = FLASH_GET_BIT_8_15(sectorAddr);
    dev->commandArray[3] = FLASH_GET_BIT_0_7(sectorAddr);

    return Flash_sequenceCommand(dev);
}

System_Errors Flash_writeBuffer (Flash_DeviceHandle dev, uint8_t *buffer, uint32_t size)
{
    System_Errors error = ERRORS_NO_ERROR;
    uint32_t addr = dev->userStartAddr;

    while (size > 0x00U)
    {
        error = Flash_writeLongWord(dev,addr,buffer);

        if (error != ERRORS_NO_ERROR)
            return error;

        addr += dev->blockSize;
        size -= dev->blockSize;
        buffer += dev->blockSize;

    }
}

#endif /* LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F */

#endif /* LIBOHIBOARD_FLASH */
