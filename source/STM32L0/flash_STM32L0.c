/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2023 A. C. Open Hardware Ideas Lab
 *
 * Authors:
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
 * @file libohiboard/source/STM32L0/flash_STM32L0.c
 * @author Lorenzo Compagnucci
 * @brief FLASH Function for implementing Reading and Writing on memory code for STM32L0 series
 */

#if defined(LIBOHIBOARD_FLASH)

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L0)

#include "utility.h"
#include "critical.h"
#include "flash.h"

#define FLASH_READ8(address)                  ((uint8_t)(*(volatile uint8_t*)(address)))

#define FLASH_ERROR_MASK  (FLASH_SR_WRPERR     | \
                           FLASH_SR_PGAERR     | \
                           FLASH_SR_SIZERR     | \
                           FLASH_SR_OPTVERR    | \
                           FLASH_SR_RDERR      | \
                           FLASH_SR_FWWERR     | \
                           FLASH_SR_NOTZEROERR)

typedef struct _Flash_Device
{
    FLASH_TypeDef* regmap;

} Flash_Device;

static Flash_Device flash0 =
{
    .regmap = FLASH,
};
Flash_DeviceHandle OB_FLASH0 = &flash0;

static System_Errors Flash_checkErrors (Flash_DeviceHandle dev)
{
    if ((dev->regmap->SR & FLASH_ERROR_MASK) != 0U)
    {
        return ERRORS_FLASH_ERROR;
    }
    return ERRORS_NO_ERROR;
}

__attribute__((always_inline)) static inline System_Errors Flash_waitLastOperation (Flash_DeviceHandle dev,
                                                                                    uint32_t timeout)

{
    uint32_t tickstart = System_currentTick();

    while ((dev->regmap->SR & FLASH_SR_BSY) == FLASH_SR_BSY)
    {
        if ((timeout == 0U) || ((System_currentTick() - tickstart) > timeout))
        {
            return ERRORS_FLASH_TIMEOUT;
        }
    }

    // Check FLASH End of Operation flag
    if ((dev->regmap->SR & FLASH_SR_EOP) == FLASH_SR_EOP)
    {
        // Clear flag!
        UTILITY_WRITE_REGISTER(dev->regmap->SR,FLASH_SR_EOP);
    }

    return ERRORS_NO_ERROR;
}

__attribute__((always_inline)) static inline System_Errors Flash_unlock (Flash_DeviceHandle dev)
{
    uint32_t primask_bit;

    // Unlocking FLASH_PECR register access
    if ((dev->regmap->PECR & FLASH_PECR_PELOCK) == FLASH_PECR_PELOCK)
    {
        // Disable interrupts to avoid any interruption during unlock sequence
        primask_bit = __get_PRIMASK();
        __disable_irq();

        UTILITY_WRITE_REGISTER(dev->regmap->PEKEYR, FLASH_PEKEY1);
        UTILITY_WRITE_REGISTER(dev->regmap->PEKEYR, FLASH_PEKEY2);

        // Re-enable the interrupts: restore previous priority mask
        __set_PRIMASK(primask_bit);

        if ((dev->regmap->PECR & FLASH_PECR_PELOCK) == FLASH_PECR_PELOCK)
        {
            return ERRORS_FLASH_PROTECTION_VIOLATION;
        }
    }

    if ((dev->regmap->PECR & FLASH_PECR_PRGLOCK) == FLASH_PECR_PRGLOCK)
    {
        // Disable interrupts to avoid any interruption during unlock sequence
        primask_bit = __get_PRIMASK();
        __disable_irq();

        // Unlocking the program memory access
        UTILITY_WRITE_REGISTER(dev->regmap->PRGKEYR, FLASH_PRGKEY1);
        UTILITY_WRITE_REGISTER(dev->regmap->PRGKEYR, FLASH_PRGKEY2);

        // Re-enable the interrupts: restore previous priority mask */
        __set_PRIMASK(primask_bit);

        if ((dev->regmap->PECR & FLASH_PECR_PRGLOCK) == FLASH_PECR_PRGLOCK)
        {
            return ERRORS_FLASH_PROTECTION_VIOLATION;
        }
    }

    return ERRORS_NO_ERROR;
}

__attribute__((always_inline)) static inline void Flash_lock (Flash_DeviceHandle dev)
{
    // Set the PRGLOCK Bit to lock the FLASH Registers access
    UTILITY_SET_REGISTER_BIT(dev->regmap->PECR,FLASH_PECR_PRGLOCK);

    // Set the PELOCK Bit to lock the PECR Register access
    UTILITY_SET_REGISTER_BIT(dev->regmap->PECR, FLASH_PECR_PELOCK);
}

System_Errors Flash_init (Flash_DeviceHandle dev)
{
    return Flash_checkErrors(dev);
}

System_Errors Flash_erasePage (Flash_DeviceHandle dev, uint16_t pageNumber)
{
    uint32_t pageAddr = (pageNumber * FLASH_PAGE_SIZE) + FLASH_START;

    if (pageNumber > FLASH_MAX_PAGE_NUMBER)
    {
        return ERRORS_FLASH_ACCESS;
    }

    Flash_unlock(dev);

    Flash_waitLastOperation(dev,FLASH_TIMEOUT_VALUE);

    // Set the ERASE bit
    UTILITY_SET_REGISTER_BIT(dev->regmap->PECR, FLASH_PECR_ERASE);
    // Set PROG bit
    UTILITY_SET_REGISTER_BIT(dev->regmap->PECR, FLASH_PECR_PROG);

    // Write 00000000h to the first word of the program page to erase
    *(__IO uint32_t *)(uint32_t)(pageAddr & ~(FLASH_PAGE_SIZE - 1)) = 0x00000000;

    Flash_waitLastOperation(dev,FLASH_TIMEOUT_VALUE);

    Flash_lock(dev);

    return ERRORS_NO_ERROR;
}

System_Errors Flash_writePage (Flash_DeviceHandle dev, uint16_t pageNumber, uint8_t* buffer, uint32_t length)
{
    uint32_t pageAddr = (pageNumber * FLASH_PAGE_SIZE) + FLASH_START;

    if (pageNumber > FLASH_MAX_PAGE_NUMBER)
    {
        return ERRORS_FLASH_ACCESS;
    }

    if (length > FLASH_PAGE_SIZE)
    {
        return ERRORS_FLASH_WRONG_PARAMS;
    }

    Flash_unlock(dev);

    Flash_waitLastOperation(dev,FLASH_TIMEOUT_VALUE);

    uint32_t tempSize = length;
    Utility_4Byte data;
    while (tempSize > 0)
    {
        data.b[0] = (uint8_t)(*(buffer + 3));
        data.b[1] = (uint8_t)(*(buffer + 2));
        data.b[2] = (uint8_t)(*(buffer + 1));
        data.b[3] = (uint8_t)(*(buffer + 0));

        *(volatile uint32_t *)pageAddr = (uint32_t)data.d;

        buffer   += 4;
        tempSize -= 4;
        pageAddr += 4;
    }

    Flash_waitLastOperation(dev,FLASH_TIMEOUT_VALUE);

    Flash_lock(dev);

    return Flash_checkErrors(dev);
}

System_Errors Flash_readPage (Flash_DeviceHandle dev, uint16_t pageNumber, uint8_t* buffer, uint32_t length)
{
    uint32_t pageAddr = (pageNumber * FLASH_PAGE_SIZE) + FLASH_START;

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

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_FLASH

