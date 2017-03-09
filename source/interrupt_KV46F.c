/* Copyright (C) 2014-2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Matteo Civale <m.civale@gmail.com>
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
 * @file libohiboard/source/interrupt_KV46F.c
 * @author Matteo Civale <m.civale@gmail.com>
 * @brief Interrupt implementations for KV46F and TWRKV46F.
 */

#if defined (LIBOHIBOARD_KV46F)     || \
    defined (LIBOHIBOARD_TRWKV46F)

#include "platforms.h"
#include "interrupt.h"

#define NVIC_NUM_CORE_VECTORS           16
#define NVIC_NUM_MCU_VECTORS            86
#define NVIC_NUM_VECTORS                NVIC_NUM_CORE_VECTORS + NVIC_NUM_MCU_VECTORS
#define NVIC_MAX_PRIORITY               15

System_Errors Interrupt_enable (Interrupt_Vector vectorNumber)
{
    /* Make sure that the IRQ is an allowable number. */
    if (vectorNumber > NVIC_NUM_MCU_VECTORS)
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

    /* Clear pending request */
    NVIC_ClearPendingIRQ(vectorNumber);

    /* Enable request */
    NVIC_EnableIRQ(vectorNumber);

    return ERRORS_NO_ERROR;
}

System_Errors Interrupt_disable (Interrupt_Vector vectorNumber)
{

    /* Make sure that the IRQ is an allowable number. */
    if (vectorNumber > NVIC_NUM_MCU_VECTORS)
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

    NVIC_DisableIRQ(vectorNumber);
    return ERRORS_NO_ERROR;
}

System_Errors Interrupt_setPriority (Interrupt_Vector vectorNumber, uint8_t priority)
{
    if (priority > NVIC_MAX_PRIORITY) return ERRORS_IRQ_PRIORITY_LEVEL_WRONG;

    /* Set interrupt priority note priority 0 is the higher priority level*/
    NVIC_SetPriority(vectorNumber,priority);
    return ERRORS_NO_ERROR;
}

#endif /* LIBOHIBOARD_KV46F || LIBOHIBOARD_TWRKV46F */






