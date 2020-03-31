/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/PIC24FJ/interrupt_PIC24FJ.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Interrupt implementations for PIC24FJ Series.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_PIC24FJ)

#include "interrupt.h"
#include "utility.h"

#define NVIC_NUM_VECTORS                       118

#define INTERRUPT_IS_NVIC_IRQ(IRQ)             (((IRQ) >= 0) && ((IRQ) < NVIC_NUM_VECTORS) && \
                                                ((IRQ) != 21) && ((IRQ) != 34) && ((IRQ) != 35) && \
                                                ((IRQ) != 52) && ((IRQ) != 55) && ((IRQ) != 56) && \
                                                ((IRQ) != 57) && ((IRQ) != 74) && ((IRQ) != 75) && \
                                                ((IRQ) != 76) && ((IRQ) != 78) && ((IRQ) != 79) && \
                                                ((IRQ) != 80) && ((IRQ) != 92) && ((IRQ) != 93) && \
                                                ((IRQ) != 100) && ((IRQ) != 103) && ((IRQ) != 104) && \
                                                ((IRQ) != 105) && ((IRQ) != 107) && ((IRQ) != 108))

System_Errors Interrupt_enable (Interrupt_Vector vectorNumber)
{
    ohiassert(INTERRUPT_IS_NVIC_IRQ(vectorNumber));
    
    uint8_t div = 0;

    // Make sure that the IRQ is an allowable number.
    if (!INTERRUPT_IS_NVIC_IRQ(vectorNumber))
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

    // Determine which of the IEC/IFS corresponds to the irq
    div = vectorNumber/16;
    
    // Clear Interrupt flag
    UTILITY_CLEAR_REGISTER_BIT(INTERRUPT->IFS[div], (1 << (vectorNumber%16)));
    // Enable Interrupt
    UTILITY_SET_REGISTER_BIT(INTERRUPT->IEC[div], (1 << (vectorNumber%16)));

    return ERRORS_NO_ERROR;
}

System_Errors Interrupt_disable (Interrupt_Vector vectorNumber)
{
    ohiassert(INTERRUPT_IS_NVIC_IRQ(vectorNumber));

    uint8_t div = 0;

    // Make sure that the IRQ is an allowable number.
    if (!INTERRUPT_IS_NVIC_IRQ(vectorNumber))
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

    // Determine which of the IEC/IFS corresponds to the irq
    div = vectorNumber/16;
    
    // Disable Interrupt
    UTILITY_CLEAR_REGISTER_BIT(INTERRUPT->IEC[div], (1 << (vectorNumber%16)));
    // Clear Interrupt flag
    UTILITY_CLEAR_REGISTER_BIT(INTERRUPT->IFS[div], (1 << (vectorNumber%16)));

    return ERRORS_NO_ERROR;
}

System_Errors Interrupt_setPriority (Interrupt_Vector vectorNumber,
                                     uint8_t priority)
{
    ohiassert(INTERRUPT_IS_NVIC_IRQ(vectorNumber));
    ohiassert(INTERRUPT_IS_VALID_PRIORITY(priority));
    
    uint8_t div = 0;

    // Make sure that the IRQ is an allowable number.
    if (!INTERRUPT_IS_NVIC_IRQ(vectorNumber))
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

    if (!INTERRUPT_IS_VALID_PRIORITY(priority))
        return ERRORS_IRQ_PRIORITY_LEVEL_WRONG;

    // Determine which of the IEP corresponds to the irq
    div = vectorNumber/4;
    // Write Priority Level
    UTILITY_MODIFY_REGISTER(INTERRUPT->IPC[div], (0b0111 << ((vectorNumber%4)*4)), (priority << ((vectorNumber%4)*4)));

    return ERRORS_NO_ERROR;
}

uint8_t Interrupt_getPriority (Interrupt_Vector vectorNumber)
{
    ohiassert(INTERRUPT_IS_NVIC_IRQ(vectorNumber));
    
    uint8_t div = 0;
    uint16_t currentPriority = 0;

    // Determine which of the IEC/IFS corresponds to the irq
    div = vectorNumber/4;
    // Read Priority Level
    currentPriority = UTILITY_READ_REGISTER_BIT(INTERRUPT->IPC[div], (0b0111 << ((vectorNumber%4)*4)));
    currentPriority >>= ((vectorNumber%4)*4);
    
    return (uint8_t)currentPriority;
}

bool Interrupt_isFlag (Interrupt_Vector vectorNumber)
{
    ohiassert(INTERRUPT_IS_NVIC_IRQ(vectorNumber));

    uint8_t div = 0;
    uint16_t flag = false;

    // Make sure that the IRQ is an allowable number.
    if (!INTERRUPT_IS_NVIC_IRQ(vectorNumber))
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

    // Determine which of the IEC/IFS corresponds to the irq
    div = vectorNumber/16;
    // Clear Interrupt flag
    flag = UTILITY_READ_REGISTER_BIT(INTERRUPT->IFS[div], (1 << (vectorNumber%16)));
    flag >>= (vectorNumber%16);
    
    return (flag != 0)?(true):(false);
}

System_Errors Interrupt_clearFlag (Interrupt_Vector vectorNumber)
{
    ohiassert(INTERRUPT_IS_NVIC_IRQ(vectorNumber));

    uint8_t div = 0;

    // Make sure that the IRQ is an allowable number.
    if (!INTERRUPT_IS_NVIC_IRQ(vectorNumber))
        return ERRORS_IRQ_NUM_VECTOR_WRONG;

    // Determine which of the IEC/IFS corresponds to the irq
    div = vectorNumber/16;
    // Clear Interrupt flag
    UTILITY_CLEAR_REGISTER_BIT(INTERRUPT->IFS[div], (1 << (vectorNumber%16)));
    
    return ERRORS_NO_ERROR;
}

#if defined (LIBOHIBOARD_INTERRUPT_DEBUG)
void Interrupt_callbackCheck (void)
{
    asm("NOP");
}
#endif

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif
