/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/source/STM32L4/interrupt_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Interrupt implementations for STM32L4-WB Series.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L4) || defined (LIBOHIBOARD_STM32WB)

#include "interrupt.h"

#define NVIC_NUM_CORE_VECTORS                  16
#define NVIC_NUM_MCU_VECTORS                   91
#define NVIC_NUM_VECTORS                       NVIC_NUM_CORE_VECTORS + NVIC_NUM_MCU_VECTORS

#define INTERRUPT_IS_NVIC_IRQ(IRQ)             (((IRQ) >= -16) && ((IRQ) <= NVIC_NUM_VECTORS))
#define INTERRUPT_IS_VALID_PRIORITY(PRIORITY)  ((PRIORITY >= 0x00u) && (PRIORITY < 16))

System_Errors Interrupt_enable (Interrupt_Vector vectorNumber)
{
    ohiassert(INTERRUPT_IS_NVIC_IRQ(vectorNumber));

    // Make sure that the IRQ is an allowable number.
    if (!INTERRUPT_IS_NVIC_IRQ(vectorNumber))
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

    NVIC_EnableIRQ((IRQn_Type)vectorNumber);

    return ERRORS_NO_ERROR;
}

System_Errors Interrupt_disable (Interrupt_Vector vectorNumber)
{
    ohiassert(INTERRUPT_IS_NVIC_IRQ(vectorNumber));

    // Make sure that the IRQ is an allowable number.
    if (!INTERRUPT_IS_NVIC_IRQ(vectorNumber))
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

    NVIC_DisableIRQ((IRQn_Type)vectorNumber);

    return ERRORS_NO_ERROR;
}

System_Errors Interrupt_setPriority (Interrupt_Vector vectorNumber,
                                     uint8_t priority)
{
    ohiassert(INTERRUPT_IS_NVIC_IRQ(vectorNumber));
    ohiassert(INTERRUPT_IS_VALID_PRIORITY(priority));

    // Make sure that the IRQ is an allowable number.
    if (!INTERRUPT_IS_NVIC_IRQ(vectorNumber))
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

    if (!INTERRUPT_IS_VALID_PRIORITY(priority))
        return ERRORS_IRQ_PRIORITY_LEVEL_WRONG;

    // Call core function
    NVIC_SetPriority((IRQn_Type)vectorNumber,priority);

    return ERRORS_NO_ERROR;
}

uint8_t Interrupt_getPriority (Interrupt_Vector vectorNumber)
{
    ohiassert(INTERRUPT_IS_NVIC_IRQ(vectorNumber));

    // Call core function
    return (uint8_t) NVIC_GetPriority((IRQn_Type)vectorNumber);
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif
