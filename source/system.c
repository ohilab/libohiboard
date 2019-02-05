/*
 * Copyright (C) 2012-2018 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/system.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief 
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "system.h"
#include "clock.h"
#include "interrupt.h"

/**
 * This function checks if the program has been downloaded to the right device.
 */
System_Errors System_controlDevice (void)
{
    /* TODO: implement... */

    return ERRORS_NO_ERROR;
}

#if (LIBOHIBOARD_VERSION >= 0x20000u)

System_Errors System_systickConfig (uint32_t ticks)
{
    return (SysTick_Config(ticks) ? ERRORS_SYSTEM_TICK_INIT_FAILED : ERRORS_NO_ERROR);
}

__weak System_Errors System_systickInit (uint32_t priority)
{
    uint32_t basetime = 0;
    // Configure the systick interrupt every 1ms
#if defined (LIBOHIBOARD_STM32L0)
    basetime = (Clock_getOutputValue(CLOCK_OUTPUT_HCLK) / 1000ul);
#elif defined (LIBOHIBOARD_STM32L4)
    basetime = (Clock_getOutputValue(CLOCK_OUTPUT_HCLK) / 1000ul);
#elif defined (LIBOHIBOARD_MKL)
    basetime = (Clock_getOutputValue(CLOCK_OUTPUT_SYSCLK) / 1000ul);
#endif

    if (basetime == 0)
        return ERRORS_SYSTEM_NO_CLOCK;

    System_systickConfig(basetime);

    // Configure the SysTick IRQ priority
    Interrupt_setPriority(INTERRUPT_SYSTICK,priority);

    return ERRORS_NO_ERROR;
}

static volatile uint32_t System_ticks = 0u;

void SysTick_Handler(void)
{
    System_ticks++;
}

uint32_t System_currentTick ()
{
    return System_ticks;
}

void System_delay (uint32_t msec)
{
    uint32_t timeout = System_currentTick() + msec;

    while (timeout > System_currentTick());
}

void System_suspendTick (void)
{
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

void System_resumeTick (void)
{
    SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
}

uint32_t System_getHalVersion (void)
{
    return LIBOHIBOARD_VERSION;
}

#endif // LIBOHIBOARD_VERSION >= 0x20000

#ifdef __cplusplus
}
#endif
