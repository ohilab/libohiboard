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
 * @file libohiboard/source/PIC24FJ/flash_PIC24FJ.c
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief FLASH Function for implementing Reading and Writing on memory code
 */

#if defined(LIBOHIBOARD_FLASH)

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "utility.h"
#include "critical.h"

#if defined (LIBOHIBOARD_PIC24FJ)
#include "flash.h"
#include "hardware/PIC24FJ/flash_PIC24FJ.h"

void _isr_noautopsv _NVMInterrupt (void)
{
    UTILITY_CLEAR_REGISTER_BIT(INTERRUPT->IFS[0], _IFS0_NVMIF_MASK);
}

System_Errors Flash_init (void)
{
    UTILITY_WRITE_REGISTER(NVM->NVMCON, 0x0000);
    UTILITY_WRITE_REGISTER(NVM->NVMADR, 0x0000);
    UTILITY_WRITE_REGISTER(NVM->NVMADRU, 0x0000);
    UTILITY_WRITE_REGISTER(NVM->NVMKEY, 0x0000);
    UTILITY_CLEAR_REGISTER_BIT(INTERRUPT->IFS[0], _IFS0_NVMIF_MASK);
    return ERRORS_NO_ERROR;
}

System_Errors Flash_unlock (void)
{
    UTILITY_SET_REGISTER_BIT(NVM->NVMCON, _NVMCON_WREN_MASK);
    return ERRORS_NO_ERROR;
}

System_Errors Flash_lock (void)
{
    UTILITY_CLEAR_REGISTER_BIT(NVM->NVMCON, _NVMCON_WREN_MASK);
    return ERRORS_NO_ERROR;
}

uint32_t Flash_getBank (uint32_t address)
{
    uint32_t bank = 0;
    if (address >= ADDRESS_START_DUAL_PARTITION)
    {
        bank = 1;
    }
    return bank;
}

uint32_t Flash_getPage (uint32_t address)
{
    uint32_t page = (address & FLASH_PAGE_MASK_W) / FLASH_PAGE_SIZE_W;
    return page;
}

uint32_t Flash_getPageSize(void)
{
    return FLASH_PAGE_SIZE_W;
}

uint32_t Flash_getRow (uint32_t address)
{
    uint32_t row = (address & FLASH_ROW_MASK_W) / FLASH_ROW_SIZE_W;
    return row;   
}

uint32_t Flash_getRowSize(void)
{
    return FLASH_ROW_SIZE_W;
}

uint32_t Flash_readWord (uint32_t address)
{
    Flash_MemoryLatch_t adr  = {.value = (address & FLASH_ADDRESS_MASK)};
    Flash_MemoryLatch_t code = {.value = 0};

    UTILITY_WRITE_REGISTER(CPU->TBLPAG, adr.address.page);
    code.word.low  = __builtin_tblrdl(adr.address.offset);
    code.word.high = __builtin_tblrdh(adr.address.offset);
    return code.value;
}

System_Errors Flash_readRow (uint32_t address, uint32_t *value_array)
{
    Flash_MemoryLatch_t adr = {.value = (address & FLASH_ROW_MASK_W)};
    for (int i = 0; i < FLASH_ROW_SIZE_I; i++)
    {
        *(value_array + i) = Flash_readWord(adr.value + i);
    }
    return ERRORS_NO_ERROR;
}

System_Errors Flash_eraseBank (uint32_t bank)
{
    if (bank == 0)
    {
        Flash_erase(ADDRESS_START_PRIM_PARTITION, LENGTH_PRIM_PARTITION);
        return Flash_init();
    }
    else if (bank == 1)
    {
        Flash_erase(ADDRESS_START_DUAL_PARTITION, LENGTH_DUAL_PARTITION);
        return Flash_init();
    }

    return ERRORS_FLASH_ACCESS;
}

System_Errors Flash_erasePage (uint32_t page)
{
    Flash_MemoryLatch_t adr = {.value = (page * FLASH_PAGE_SIZE_W)};

    UTILITY_WRITE_REGISTER(NVM->NVMCON,  FLASH_ERASE_PAGE);
    UTILITY_WRITE_REGISTER(NVM->NVMADR,  adr.word.low);
    UTILITY_WRITE_REGISTER(NVM->NVMADRU, adr.word.high);

    CRITICAL_SECTION_BEGIN();
    __builtin_write_NVM();
    while(UTILITY_READ_REGISTER_BIT(NVM->NVMCON, _NVMCON_WR_MASK) != 0);
    CRITICAL_SECTION_END();

    return Flash_init();
}

System_Errors Flash_eraseSector (uint32_t pageStart, uint32_t numPages)
{
    for (uint32_t i = 0; i < numPages; i++)
    {
        Flash_erasePage(pageStart + i);
    }

    return Flash_init();
}

System_Errors Flash_erase (uint32_t address, uint32_t length)
{
    uint32_t pageStart = Flash_getPage(address);
    uint32_t numPages = (length / FLASH_ROW_SIZE_B);
    if ((length % FLASH_ROW_SIZE_B) > 0)
    {
        numPages++;
    }
    return Flash_eraseSector(pageStart, numPages);
}

System_Errors Flash_writeDoubleWord (uint32_t address, uint32_t word1, uint32_t word2)
{
    Flash_MemoryLatch_t adr   = {.value = (address & FLASH_ADDRESS_MASK)};
    Flash_MemoryLatch_t code1 = {.value = word1};
    Flash_MemoryLatch_t code2 = {.value = word2};
    Flash_MemoryLatch_t test1 = {.value = 0};
    Flash_MemoryLatch_t test2 = {.value = 0};

    UTILITY_WRITE_REGISTER(NVM->NVMCON,  FLASH_DWORD_WRITE);
    UTILITY_WRITE_REGISTER(CPU->TBLPAG,  0xFA);
    UTILITY_WRITE_REGISTER(NVM->NVMADR,  adr.word.low);
    UTILITY_WRITE_REGISTER(NVM->NVMADRU, adr.word.high);

    __builtin_tblwtl(0, code1.word.low);
    __builtin_tblwth(0, code1.byte.mhigh);
    __builtin_tblwtl(2, code2.word.low);
    __builtin_tblwth(2, code2.byte.mhigh);
    CRITICAL_SECTION_BEGIN();
    __builtin_write_NVM();
    while(UTILITY_READ_REGISTER_BIT(NVM->NVMCON, _NVMCON_WR_MASK) != 0);
    CRITICAL_SECTION_END();

    test1.value = Flash_readWord(address);
    test2.value = Flash_readWord(address + 2);

    Flash_init();
    if ((code1.program.code == test1.program.code) &&
        (code2.program.code == test2.program.code))
    {
        return ERRORS_NO_ERROR;
    }
    return ERRORS_FLASH_ACCESS;
}

System_Errors Flash_writeRow (uint32_t address, uint32_t *words)
{
    Flash_MemoryLatch_t adr        = {.value = (address & FLASH_ROW_MASK_B)};
    Flash_MemoryLatch_t *codearray = (Flash_MemoryLatch_t *)words;

    UTILITY_WRITE_REGISTER(NVM->NVMCON,  FLASH_ROW_WRITE);
    UTILITY_WRITE_REGISTER(CPU->TBLPAG,  0xFA);
    UTILITY_WRITE_REGISTER(NVM->NVMADR,  adr.word.low);
    UTILITY_WRITE_REGISTER(NVM->NVMADRU, adr.word.high);

    for (uint16_t i = 0; i < FLASH_ROW_SIZE_I; i++)
    {
        __builtin_tblwtl(i * 2, codearray[i].word.low);
        __builtin_tblwth(i * 2, codearray[i].byte.mhigh);
    }
    CRITICAL_SECTION_BEGIN();
    __builtin_write_NVM();
    while (NVMCONbits.WR == 1);
    CRITICAL_SECTION_END();

    return Flash_init();
}

System_Errors Flash_readData (uint32_t startAddress, uint8_t *data, uint16_t length)
{
    uint16_t buffer[FLASH_ROW_SIZE_I];
    Flash_MemoryLatch_t addressSource = {.value = startAddress};
    uint16_t fixedLength = ((length % 2) == 0) ? (length) : (length + 1);
    uint16_t i = 0;
    memset((void *) buffer, 0x00, sizeof (buffer));
    fixedLength = fixedLength / 2;
    fixedLength = MIN(fixedLength, FLASH_ROW_SIZE_I);

    for (i = 0; i < fixedLength; i += 1)
    {
        Flash_MemoryLatch_t latch = {.value = FLASH_EMPTY_INSTRUCTION};
        latch.value = Flash_readWord(addressSource.value);
        addressSource.value += 2;
        buffer[i] = latch.word.low;
    }

    memcpy(data, buffer, MIN(length, FLASH_ROW_SIZE_W));
    return Flash_init();
}

System_Errors Flash_writeData (uint32_t startAddress, uint8_t *data, uint16_t length)
{
    uint16_t buffer[FLASH_ROW_SIZE_I];
    Flash_MemoryLatch_t addressDest = {.value = startAddress};
    uint16_t fixedLength = ((length % 2) == 0) ? (length) : (length + 1);
    memset((void *) buffer, 0xFF, sizeof (buffer));
    memcpy((void *) buffer, data, MIN(length, FLASH_ROW_SIZE_W));
    fixedLength = fixedLength / 2;

    for (uint16_t i = 0; i < fixedLength; i += 2)
    {
        Flash_MemoryLatch_t latch1 = {.word.high = 0, .word.low = *(buffer + i)};
        Flash_MemoryLatch_t latch2 = {.word.high = 0, .word.low = *(buffer + i + 1)};
        Flash_writeDoubleWord(addressDest.value, latch1.value, latch2.value);
        if (Flash_writeDoubleWord(addressDest.value, latch1.value, latch2.value) == ERRORS_NO_ERROR)
        {
            addressDest.value += 4;
        }
        else
        {
            return ERRORS_FLASH_ACCESS;
        }
    }

    return Flash_init();
}

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_FLASH
