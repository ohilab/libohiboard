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
 * @author Leonardo Morichelli
 * @brief FLASH implementations for PIC24FJ Series.
 */

#include "platforms.h"

#if defined(LIBOHIBOARD_FLASH)

#ifdef __cplusplus
extern "C" {
#endif

#include "flash.h"
#include "string.h"

void __weak Flash_notifyError (void) 
{

}

void __weak Flash_notifyProgress (uint16_t p, uint16_t pages) 
{

}

bool Flash_isPageEmpty (uint32_t address) 
{
    uint32_t fixedAddressPage = (address & FLASH_PAGE_MASK_W);

    for (uint16_t i = 0; i < FLASH_PAGE_SIZE_I; i++) 
    {
        uint32_t code = FLASH_EMPTY_INSTRUCTION;
        code = Flash_readWord(fixedAddressPage);
        if (code != FLASH_EMPTY_INSTRUCTION) 
        {
            return false;
        }
        fixedAddressPage += 2;
    }
    return true;
}

bool Flash_pagecmp (uint32_t address1, uint32_t address2) 
{
    uint32_t fixedAddressPage1 = (address1 & FLASH_PAGE_MASK_W);
    uint32_t fixedAddressPage2 = (address2 & FLASH_PAGE_MASK_W);

    for (uint16_t i = 0; i < FLASH_PAGE_SIZE_I; i++) 
    {
        uint32_t code1 = FLASH_EMPTY_INSTRUCTION;
        uint32_t code2 = FLASH_EMPTY_INSTRUCTION;
        code1 = Flash_readWord(fixedAddressPage1);
        code2 = Flash_readWord(fixedAddressPage2);
        if (code1 != code2) 
        {
            return true;
        }
        fixedAddressPage1 += 2;
        fixedAddressPage2 += 2;
    }
    return false;
}

bool Flash_memcpy (uint32_t destAddress, uint32_t sourceAddress, uint32_t length) 
{
    uint32_t rowBuffer[FLASH_ROW_SIZE_I];
    uint32_t rowCheckBuffer[FLASH_ROW_SIZE_I];

    uint16_t r = 0, rows = (FLASH_PAGE_SIZE_W / FLASH_ROW_SIZE_W);
    uint32_t p = 0, pages = ((length + FLASH_PAGE_SIZE_W - 1) / FLASH_PAGE_SIZE_W);
    bool rtnValue = true, isGood = true;

    while (p < pages) 
    {
        uint32_t currentAddressSource = (sourceAddress + (p * FLASH_PAGE_SIZE_W));
        uint32_t currentAddressDest = (destAddress + (p * FLASH_PAGE_SIZE_W));

        Flash_erasePage(Flash_getPage(currentAddressDest));
        rtnValue = true;
        for (r = 0; (r < rows)&& (rtnValue == true); r++) 
        {
            memset(rowBuffer, 0xFF, sizeof (rowBuffer));
            Flash_readRow(currentAddressSource, &rowBuffer[0]);
            Flash_writeRow(currentAddressDest, &rowBuffer[0]);

            memset(rowCheckBuffer, 0xFF, sizeof (rowCheckBuffer));
            Flash_readRow(currentAddressDest, & rowCheckBuffer[0]);
            if (memcmp((void *) rowBuffer, (void *) rowCheckBuffer, sizeof (rowBuffer))) 
            {
                rtnValue = false;
            }

            currentAddressSource += (FLASH_ROW_SIZE_W);
            currentAddressDest += (FLASH_ROW_SIZE_W);
        }

        // se una delle righe della pagina non è scritta correttamente, 
        //   allora si esegue un solo tentativo di cancellazione e 
        //   riscrittura dell'intera pagina;
        //   se anche il secondo tentativo non va a buon fine, 
        //   allora eseguire un ciclo in ci si notifica che la 
        //   programmazione è falita e non è possibile recuperare via OTA 
        //   il device (lampeggio o vibrazione continua))
        // se tutte le righe della pagina sono scritte correttamente, 
        //     allora si procede alla pagina successiva 
        if (rtnValue == true) 
        {
            p++;
            isGood = true;
            Flash_notifyProgress(p, pages);
        } 
        else 
        {
            if (isGood == false) 
            {
                Flash_notifyError();
            }
            isGood = false;
        }
    }
    return rtnValue;
}

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_FLASH
