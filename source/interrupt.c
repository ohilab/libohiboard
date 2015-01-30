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
 * @file libohiboard/source/interrupt.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Manage interrupt
 */


#include "platforms.h"

#if defined (FRDMKL25Z) || defined(LIBOHIBOARD_KL15Z4) || defined(FRDMKL05Z) || \
	defined (FRDMKL02Z) || defined(MKL02Z4) ||                       \
	defined (FRDMKL03Z) || defined(MKL03Z4) || defined (MK60DZ10) || \
	defined (MK10DZ10) || defined(MK10D10)

#include "interrupt.h"

#if defined (FRDMKL25Z) || defined(LIBOHIBOARD_KL15Z4) || defined(FRDMKL05Z) || \
	defined (FRDMKL02Z) || defined(MKL02Z4) ||                       \
	defined (FRDMKL03Z) || defined(MKL03Z4)

#define NVIC_NUM_CORE_VECTORS           16
#define NVIC_NUM_MCU_VECTORS            32
#define NVIC_NUM_VECTORS                NVIC_NUM_CORE_VECTORS + NVIC_NUM_MCU_VECTORS

#elif defined (MK60DZ10)

#define NVIC_NUM_CORE_VECTORS           16
#define NVIC_NUM_MCU_VECTORS            95
#define NVIC_NUM_VECTORS                NVIC_NUM_CORE_VECTORS + NVIC_NUM_MCU_VECTORS

#elif defined (MK10DZ10) || defined(MK10D10)

#define NVIC_NUM_CORE_VECTORS           16
#define NVIC_NUM_MCU_VECTORS            95
#define NVIC_NUM_VECTORS                NVIC_NUM_CORE_VECTORS + NVIC_NUM_MCU_VECTORS

#endif

System_Errors Interrupt_enable (Interrupt_Vector vectorNumber)
{
#if defined (MK60DZ10) || defined (MK10DZ10) || defined(MK10D10)
    uint16_t div;
#endif

    /* Make sure that the IRQ is an allowable number. */
    if (vectorNumber > NVIC_NUM_MCU_VECTORS)
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

#if defined (FRDMKL25Z) || defined(LIBOHIBOARD_KL15Z4) || defined(FRDMKL05Z) || \
	defined (FRDMKL02Z) || defined(MKL02Z4)
    NVIC_ICPR = 1 << (vectorNumber%32);
    NVIC_ISER = 1 << (vectorNumber%32);
#elif defined (FRDMKL03Z) || defined(MKL03Z4) /* FIXME: This kind of use is for KDS! */
    NVIC->ICPR[0] = 1 << (vectorNumber%32);
    NVIC->ISER[0] = 1 << (vectorNumber%32);
#elif defined (MK60DZ10) || defined (MK10DZ10) || defined(MK10D10)
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

System_Errors Interrupt_disable (Interrupt_Vector vectorNumber)
{
#if defined (MK60DZ10) || defined (MK10DZ10) || defined(MK10D10)
    uint16_t div;
#endif

    /* Make sure that the IRQ is an allowable number. */
    if (vectorNumber > NVIC_NUM_MCU_VECTORS)
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

#if defined (FRDMKL25Z) || defined(LIBOHIBOARD_KL15Z4) || defined(FRDMKL05Z) || \
	defined (FRDMKL02Z) || defined(MKL02Z4)
    NVIC_ICER = 1 << (vectorNumber%32);
#elif defined (FRDMKL03Z) || defined(MKL03Z4) /* FIXME: This kind of use is for KDS! */
    NVIC->ICER[0] = 1 << (vectorNumber%32);
#elif defined (MK60DZ10) || defined (MK10DZ10) || defined(MK10D10)
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

#endif
