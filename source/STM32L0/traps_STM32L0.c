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
 * @file libohiboard/source/STM32L0/traps_STM32L0.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Traps implementations for STM32L0 Series.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L0)

#include "traps.h"
#include "system.h"

static pFunc mHardFaultCallback = NULL;

void Traps_addHardFaultCallback (pFunc c)
{
    mHardFaultCallback = c;
}

void Traps_haltOnError (Traps_ErrorCode code)
{
    Traps_ErrorCode current = code;

    System_softwareBreakpoint();
    asm("NOP");
}

_weak void HardFault_Handler (void)
{
    if (mHardFaultCallback != NULL) mHardFaultCallback();
    Traps_haltOnError(TRAPS_ERRORCODE_NONE);
}

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif
