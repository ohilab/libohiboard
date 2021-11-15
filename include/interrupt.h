/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2021 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Marco Contigiani <m.contigiani86@gmail.com>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
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
 */

/**
 * @file libohiboard/include/interrupt.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Marco Contigiani <m.contigiani86@gmail.com>
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @author Matteo Civale <m.civale@gmail.com>
 * @brief Interrupt definitions.
 */

#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum {
    INTERRUPT_ENABLE_OFF,
    INTERRUPT_ENABLE_ON,
} Interrupt_Status;

#if defined (LIBOHIBOARD_STM32G0)

#include "hardware/STM32G0/interrupt_STM32G0.h"

#elif defined (LIBOHIBOARD_STM32L0)

#include "hardware/STM32L0/interrupt_STM32L0.h"

#elif defined (LIBOHIBOARD_STM32L4) || \
      defined (LIBOHIBOARD_STM32WB)

#include "hardware/STM32L4/interrupt_STM32L4.h"

#elif defined (LIBOHIBOARD_MKL)

#include "hardware/NXPMKL/interrupt_MKL.h"

#elif defined (LIBOHIBOARD_PIC24FJ)

#include "hardware/PIC24FJ/interrupt_PIC24FJ.h"

#else

// Dummy
typedef enum _Interrupt_Vector
{
    INTERRUPT_SYSTICK = -1,
    INTERRUPT_NONE    = 0,

} Interrupt_Vector;

#endif

System_Errors Interrupt_enable (Interrupt_Vector vectorNumber);
System_Errors Interrupt_disable (Interrupt_Vector vectorNumber);

/**
 * Set interrupt priority.
 * Note priority 0 is the higher priority level.
 */
System_Errors Interrupt_setPriority (Interrupt_Vector vectorNumber,
                                     uint8_t priority);

/**
 * Return the current priority level.
 */
uint8_t Interrupt_getPriority (Interrupt_Vector vectorNumber);

#if defined (LIBOHIBOARD_MICROCHIP_PIC)
/**
 * Return value of flag for specific IRQ.
 * 
 * @param[in] vectorNumber: index of IRQ
 * 
 * @return <bool>: true if flag is active, false otherwise 
 */
bool Interrupt_isFlag (Interrupt_Vector vectorNumber);

/**
 * Clear the interrupt flag for specific IRQ.
 * 
 * @param[in] vectorNumber: index of IRQ
 */
System_Errors Interrupt_clearFlag (Interrupt_Vector vectorNumber);

#if defined (LIBOHIBOARD_INTERRUPT_DEBUG)
void Interrupt_callbackCheck (void);
#endif

#endif

#ifdef __cplusplus
}
#endif

#endif // __INTERRUPT_H


