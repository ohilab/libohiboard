/*
 * Copyright (C) 2012-2019 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Leonardo Morichelli
 *  Stefano Gigli
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
 * @author Leonardo Morichelli
 * @author Stefano Gigli
 * @brief 
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "system.h"
#include "clock.h"
#include "interrupt.h"

#if defined (LIBOHIBOARD_MICROCHIP_PIC)
#include "timer.h"
#include "critical.h"
void SysTick_Handler (Timer_DeviceHandle dev);
#endif

/**
 * This function checks if the program has been downloaded to the right device.
 */
System_Errors System_controlDevice (void)
{
    /* TODO: implement... */

    return ERRORS_NO_ERROR;
}

bool System_systickIsRunning (void)
{
#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    return Timer_isRunning(OB_TIM23);
#else
    return true; //TODO: review
#endif
}

#if (LIBOHIBOARD_VERSION >= 0x20000u)

System_Errors System_systickConfig (uint32_t ticks)
{
    System_Errors error = ERRORS_SYSTEM_TICK_INIT_FAILED;
#if !defined (LIBOHIBOARD_MICROCHIP_PIC)
    error = (SysTick_Config(ticks) ? ERRORS_SYSTEM_TICK_INIT_FAILED : ERRORS_NO_ERROR);
#else

    // Timer configuration...
    Timer_Config systickTimer =
    {
        .mode = TIMER_MODE_FREE,
        .modulo = 0,
        .prescaler = 0,
        .timerFrequency = 1000, // The base time is 1ms!
        //.configurationBits = 0,
        .clockSource  = TIMER_CLOCKSOURCE_INTERNAL,

        .freeCounterCallback = SysTick_Handler,
        .pwmPulseFinishedCallback = nullptr,
        .outputCompareCallback = nullptr,
        .inputCaptureCallback = nullptr,

        .isrPriority = 6,
        .counterMode = TIMER_COUNTERMODE_UP,
    };

//    Timer_stop(OB_TIM23);
    Timer_init(OB_TIM23,&systickTimer);
    Timer_start(OB_TIM23);
#endif
    return error;
}

__weak System_Errors System_systickInit (uint32_t priority)
{
    uint32_t basetime = 0;

    // Configure the systick interrupt every 1ms
#if defined (LIBOHIBOARD_STM32L0)
    basetime = (Clock_getOutputValue(CLOCK_OUTPUT_HCLK) / 1000ul);
    //basetime = (Clock_getOutputValue(CLOCK_OUTPUT_HCLK) / LIBOHIBOARD_SYSTEM_TICK );
#elif defined (LIBOHIBOARD_STM32L4) || defined (LIBOHIBOARD_STM32WB)
    //basetime = (Clock_getOutputValue(CLOCK_OUTPUT_HCLK) / LIBOHIBOARD_SYSTEM_TICK );
    basetime = (Clock_getOutputValue(CLOCK_OUTPUT_HCLK) / 1000ul);
#elif defined (LIBOHIBOARD_MKL)
    basetime = (Clock_getOutputValue(CLOCK_OUTPUT_SYSCLK) / 1000ul);
    //basetime = (Clock_getOutputValue(CLOCK_OUTPUT_SYSCLK) / LIBOHIBOARD_SYSTEM_TICK );
#elif defined (LIBOHIBOARD_MICROCHIP_PIC)
    // It's just for bypass the control below!
    basetime = 1;
#endif

    if (basetime == 0)
        return ERRORS_SYSTEM_NO_CLOCK;

    System_systickConfig(basetime);

#if !defined (LIBOHIBOARD_MICROCHIP_PIC)
    // Configure the SysTick IRQ priority
    Interrupt_setPriority(INTERRUPT_SYSTICK,priority);
#else
    // Configure the Timer3 IRQ priority
//    Interrupt_setPriority(INTERRUPT_TIMER3,priority);
#endif

    return ERRORS_NO_ERROR;
}

static volatile uint32_t System_ticks = 0u;

#if !defined (LIBOHIBOARD_MICROCHIP_PIC)
void SysTick_Handler (void)
{
#else
void SysTick_Handler (Timer_DeviceHandle dev)
{
    (void)dev;
#endif

    System_ticks++;
}

uint32_t System_currentTick (void)
{
    return System_ticks;
}

#if defined (LIBOHIBOARD_MICROCHIP_PIC)
#include <libpic30.h>
static void delay(uint32_t msec)
{
    uint64_t milli = 1000ull;
    uint64_t time = (uint64_t)msec;
    uint64_t fcy = (uint64_t)Clock_getOutputValue(CLOCK_OUTPUT_PERIPHERAL);

    uint32_t value = (uint32_t)((time * fcy) / milli);
    __delay32(value);
}
#endif

static void wait(uint32_t msec)
{
    uint32_t timeout = System_currentTick() + msec;
    while (timeout > System_currentTick());
}

void System_delay (uint32_t msec)
{
#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    if ((Critical_isActive() == true) || (System_systickIsRunning() == false))
    {
        delay(msec);
    }
    else
    {
        wait(msec);
    }
#else
    wait(msec);
#endif
}


void System_suspendTick (void)
{
#if !defined (LIBOHIBOARD_MICROCHIP_PIC)
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
#endif
}

void System_resumeTick (void)
{
#if !defined (LIBOHIBOARD_MICROCHIP_PIC)
    SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
#endif
}

void System_softwareBreakpoint (void)
{
#if defined (__DEBUG)

#if defined (LIBOHIBOARD_STM32L0) || defined (LIBOHIBOARD_STM32L4) || defined (LIBOHIBOARD_STM32WB)
    asm("BKPT #1");
    asm("NOP");
#endif
#if defined (LIBOHIBOARD_MKL)
    // TODO
#endif
#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    __builtin_software_breakpoint();
#endif

#endif
}

void System_forceEnableDebug (void)
{
#if defined (LIBOHIBOARD_STM32L0) || defined (LIBOHIBOARD_STM32L4) || defined (LIBOHIBOARD_STM32WB)
    DBGMCU_TypeDef* mcuDgb = (DBGMCU_TypeDef *) DBGMCU;
    mcuDgb->CR |=  ( DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY );
#endif
#if defined (LIBOHIBOARD_MKL)
    // FIXME: intentionally empty
#endif
#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    // intentionally empty
#endif
}

void System_forceDisableDebug (void)
{
#if defined (LIBOHIBOARD_STM32L0) || defined (LIBOHIBOARD_STM32L4) || defined (LIBOHIBOARD_STM32WB)
    DBGMCU_TypeDef* mcuDgb = ( DBGMCU_TypeDef * )DBGMCU;
    mcuDgb->CR &= ~( DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY );
#endif
#if defined (LIBOHIBOARD_MKL)
    // FIXME: intentionally empty
#endif
#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    // intentionally empty
#endif
}

uint32_t System_getHalVersion (void)
{
    return LIBOHIBOARD_VERSION;
}

#endif // LIBOHIBOARD_VERSION >= 0x20000

#ifdef __cplusplus
}
#endif
