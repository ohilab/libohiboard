/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Marco Giammarini <m.giammarini@warcomeb.it>
 *	
 * Project: libohiboard
 * Package: Interrupt
 * Version: 0.0
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>
 ******************************************************************************/

/**
 * @file libohiboard/source/interrupt.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Manage interrupt
 */

#include "interrupt.h"

#if defined (FRDMKL25Z)
#define NVIC_NUM_CORE_VECTORS           16
#define NVIC_NUM_MCU_VECTORS            32
#define NVIC_NUM_VECTORS                NVIC_NUM_CORE_VECTORS + NVIC_NUM_MCU_VECTORS
#elif defined (MK60DZ10)
#define NVIC_NUM_CORE_VECTORS           16
#define NVIC_NUM_MCU_VECTORS            95
#define NVIC_NUM_VECTORS                NVIC_NUM_CORE_VECTORS + NVIC_NUM_MCU_VECTORS
#endif

System_Errors Interrupt_enable (uint16_t vectorNumber)
{
#if defined (MK60DZ10)
    uint16_t div;
#endif

    /* Make sure that the IRQ is an allowable number. */
    if (vectorNumber > NVIC_NUM_MCU_VECTORS)
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

#if defined (FRDMKL25Z)
    NVIC_ICPR = 1 << (vectorNumber%32);
    NVIC_ISER = 1 << (vectorNumber%32);
#elif defined (MK60DZ10)
    /* Determine which of the NVICISERs corresponds to the irq */
    div = vectorNumber/32;

    switch (div)
    {
    case 0x0:
        NVICICPR0 = 1 << (vectorNumber%32);
        NVICISER0 = 1 << (vectorNumber%32);
        break;
    case 0x1:
        NVICICPR1 = 1 << (vectorNumber%32);
        NVICISER1 = 1 << (vectorNumber%32);
        break;
    case 0x2:
        NVICICPR2 = 1 << (vectorNumber%32);
        NVICISER2 = 1 << (vectorNumber%32);
        break;
    }
#endif

    return ERRORS_NO_ERROR;
}

System_Errors Interrupt_disable (uint16_t vectorNumber)
{
#if defined (MK60DZ10)
    uint16_t div;
#endif

    /* Make sure that the IRQ is an allowable number. */
    if (vectorNumber > NVIC_NUM_MCU_VECTORS)
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

#if defined (FRDMKL25Z)
    NVIC_ICER = 1 << (vectorNumber%32);
#elif defined (MK60DZ10)
    /* Determine which of the NVICISERs corresponds to the irq */
    div = vectorNumber/32;

    switch (div)
    {
    case 0x0:
        NVICICER0 = 1 << (vectorNumber%32);
        break;
    case 0x1:
        NVICICER1 = 1 << (vectorNumber%32);
        break;
    case 0x2:
        NVICICER2 = 1 << (vectorNumber%32);
        break;
    }
#endif

    return ERRORS_NO_ERROR;
}
