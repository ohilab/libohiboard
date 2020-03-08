/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/source/PIC24FJ/lowpower-timer_PIC24FJ.c
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief Low-Power Timer implementations for PIC24FJ
 */

#include "platforms.h"

#if defined (LIBOHIBOARD_LOWPOWER_TIMER)

#ifdef __cplusplus
extern "C" {
#endif

#include "lowpower-timer.h"

#include "platforms.h"
#include "utility.h"
#include "system.h"
#include "clock.h"
#include "interrupt.h"

#if defined (LIBOHIBOARD_PIC24FJ)

#define LOWPOWERTIMER_CLOCK_ENABLE(REG,MASK) do { \
                                               UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                               asm("nop"); \
                                               (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                             } while (0)

#define LOWPOWERTIMER_CLOCK_DISABLE(REG,MASK) do { \
                                                UTILITY_CLEAR_REGISTER_BIT(REG,MASK); \
                                                asm("nop"); \
                                                (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                              } while (0)

#define LOWPOWERTIMER_VALID_CLOCK_SOURCE(CLOCKSOURCE) (((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_FRC)    || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL)        || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_LPRC)   || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL_CLK)    || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL_SOSC))

#define LOWPOWERTIMER_VALID_PRESCALER(PRESCALER) (((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV1)  || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV8)  || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV64) || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV256))

#define LOWPOWERTIMER_VALID_COUNTER(COUNTER) (((uint32_t)COUNTER) <= 0x0000FFFFul)

typedef struct _LowPowerTimer_Device
{
    LPTIM_TypeDef* regmap;                         /**< Device memory pointer */
    INTERRUPT_TypeDef* regmapInt;
    PMD_Typedef* regmapPmd;
    
    uint16_t bckConfValue;
    uint16_t bckConfMask;
    
    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    LowPowerTimer_DeviceState state;           /**< Current peripheral state. */
    LowPowerTimer_Config config;
} LowPowerTimer_Device;

#if defined (LIBOHIBOARD_PIC24FJ)

#define LOWPOWERTIMER_IS_DEVICE(DEVICE) ((DEVICE) == OB_LPTIM1)

static LowPowerTimer_Device lptim1 =
{
    .regmap = LPTIM,
    .regmapInt = INTERRUPT,
    .regmapPmd = PMD,
    
    .bckConfValue = 0x0000,
    .bckConfMask = (_T1CON_TSIDL_MASK | _T1CON_TECS_MASK | _T1CON_TGATE_MASK | _T1CON_TCKPS_MASK | _T1CON_TSYNC_MASK | _T1CON_TCS_MASK),
    .isrNumber = INTERRUPT_TIMER1,
    
    .state = LOWPOWERTIMER_DEVICESTATE_RESET,
};
LowPowerTimer_DeviceHandle OB_LPTIM1 = &lptim1;

#endif // LIBOHIBOARD_PIC24FJ

const LowPowerTimer_ClockPrescaler LOWPOWERTIMER_PRESCALER_REGISTER_TABLE[8] =
{
    LOWPOWERTIMER_CLOCKPRESCALER_DIV1,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV8,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV64,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV256,
};

const uint32_t LOWPOWERTIMER_PRESCALER_REGISTER_VALUE[8] =
{
    1,
    8,
    64,
    256,
};

static inline void __attribute__((always_inline)) LowPowerTimer_callbackInterrupt (LowPowerTimer_DeviceHandle dev)
{
    // Update event
    if(Interrupt_isFlag(INTERRUPT_TIMER1))
    {
        // Clear flag
        Interrupt_clearFlag(INTERRUPT_TIMER1);
        
        // Call callback
        dev->config.counterCallback(dev);
    }
}

static void LowPowerTimer_config (LowPowerTimer_DeviceHandle dev,
                                  LowPowerTimer_Config *config)
{
    // Stop the TON bit (= 0)
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->T1CON, _T1CON_TON_MASK);
    
    // Set Clock Source
    if(config->clockSource == LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_FRC)
    {
        // Set clock source for FOSC/2
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->T1CON, _T1CON_TCS_MASK);
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->T1CON, _T1CON_TECS_MASK);
    }
    else
    {
        // Set clock source for other input
        UTILITY_SET_REGISTER_BIT(dev->regmap->T1CON, _T1CON_TCS_MASK);
        UTILITY_MODIFY_REGISTER(dev->regmap->T1CON, _T1CON_TECS_MASK, (config->clockSource << _T1CON_TECS_POSITION));
    }
    
    // Save parameters to device structure
    dev->config = *config;
    
    // Enable Interrupt if CallBack is set
    if ((config->counterCallback != 0))
    {
        // Set Interrupt Priority
        Interrupt_setPriority(dev->isrNumber, config->intPriority);

        // Enable Interrupt
        Interrupt_enable(dev->isrNumber);
    }
    else
    {
        // Disable Interrupt
        Interrupt_disable(dev->isrNumber);
    }

    // Backup configuration
    dev->bckConfValue = UTILITY_READ_REGISTER_BIT(dev->regmap->T1CON, dev->bckConfMask);
}

System_Errors LowPowerTimer_init (LowPowerTimer_DeviceHandle dev,
                                  LowPowerTimer_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance type
    err = ohiassert(LOWPOWERTIMER_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    err |= ohiassert(LOWPOWERTIMER_VALID_CLOCK_SOURCE(config->clockSource));
    err |= ohiassert(LOWPOWERTIMER_VALID_PRESCALER(config->prescaler));

    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }

    // Enable peripheral if needed
    if (dev->state == LOWPOWERTIMER_DEVICESTATE_RESET)
    {
        // Peripheral Module Disabling
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapPmd->PMD1, _PMD1_T1MD_MASK);
        dev->state = LOWPOWERTIMER_DEVICESTATE_BUSY;
    }

    LowPowerTimer_config(dev,config);
    LowPowerTimer_setPrescaler(dev, config->prescaler);

    dev->state = LOWPOWERTIMER_DEVICESTATE_READY;

    return err;
}

System_Errors LowPowerTimer_deInit (LowPowerTimer_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance type
    err = ohiassert(LOWPOWERTIMER_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    dev->state = LOWPOWERTIMER_DEVICESTATE_BUSY;
    
    // Disable Interrupt
    Interrupt_disable(dev->isrNumber);

    // Clear configuration
    UTILITY_WRITE_REGISTER(dev->regmap->T1CON, 0x0000u);

    // Peripheral Module Disabling
    UTILITY_SET_REGISTER_BIT(dev->regmapPmd->PMD1, _PMD1_T1MD_MASK);
    dev->state = LOWPOWERTIMER_DEVICESTATE_RESET;

    return err;
}

void LowPowerTimer_setPrescaler (LowPowerTimer_DeviceHandle dev,
                                 LowPowerTimer_ClockPrescaler prescaler)
{
    // Check the TIMER instance type
    ohiassert(LOWPOWERTIMER_IS_DEVICE(dev));
    // Check prescaler value
    ohiassert(LOWPOWERTIMER_VALID_PRESCALER(prescaler));
    dev->config.prescaler = prescaler;

    if (dev->state == LOWPOWERTIMER_DEVICESTATE_RESET)
    {
        // Enable the peripheral
        UTILITY_CLEAR_REGISTER_BIT(dev->regmapPmd->PMD1, _PMD1_T1MD_MASK);        
        dev->state = LOWPOWERTIMER_DEVICESTATE_BUSY;

        // Reload Configuration
        UTILITY_MODIFY_REGISTER(dev->regmap->T1CON, dev->bckConfMask, dev->bckConfValue);
    }
    
    // Stop timer
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->T1CON, _T1CON_TON_MASK);
    // Set prescaler
    UTILITY_MODIFY_REGISTER(dev->regmap->T1CON, _T1CON_TCKPS_MASK, (((uint16_t)prescaler) << _T1CON_TCKPS_POSITION));
    
    // Backup configuration
    dev->bckConfValue = UTILITY_READ_REGISTER_BIT(dev->regmap->T1CON, dev->bckConfMask);
}

System_Errors LowPowerTimer_startCounter (LowPowerTimer_DeviceHandle dev,
                                          uint32_t counter)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance type
    err = ohiassert(LOWPOWERTIMER_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    // Check counter value
    ohiassert(LOWPOWERTIMER_VALID_COUNTER(counter));

    // Enable the peripheral
    UTILITY_CLEAR_REGISTER_BIT(dev->regmapPmd->PMD1, _PMD1_T1MD_MASK);
    dev->state = LOWPOWERTIMER_DEVICESTATE_BUSY;

    //Reload Configuration
    UTILITY_MODIFY_REGISTER(dev->regmap->T1CON, dev->bckConfMask, dev->bckConfValue);

    // Check if callback is present
    if (dev->config.counterCallback != 0)
    {
        // Enable Interrupt
        Interrupt_enable(dev->isrNumber);
    }

    // Update reload register
    UTILITY_WRITE_REGISTER(dev->regmap->TMR1, 0x0000);
    uint16_t regVal = (uint16_t)counter;
    UTILITY_WRITE_REGISTER(dev->regmap->PR1, regVal);

    // Star the TON
    UTILITY_SET_REGISTER_BIT(dev->regmap->T1CON, _T1CON_TON_MASK);
    dev->state = LOWPOWERTIMER_DEVICESTATE_READY;
    
    return err;
}

System_Errors LowPowerTimer_stopCounter (LowPowerTimer_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance type
    err = ohiassert(LOWPOWERTIMER_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    dev->state = LOWPOWERTIMER_DEVICESTATE_BUSY;

    // Disable interrupt
    Interrupt_disable(INTERRUPT_TIMER1);

    // Disable the peripheral
    //UTILITY_SET_REGISTER_BIT(dev->regmapPmd->PMD1, _PMD1_T1MD_MASK);
    dev->state = LOWPOWERTIMER_DEVICESTATE_RESET;

    return err;
}

uint32_t LowPowerTimer_getCurrentCounter (LowPowerTimer_DeviceHandle dev)
{
    // Check the TIMER instance type
    ohiassert(LOWPOWERTIMER_IS_DEVICE(dev));

    return (dev->regmap->TMR1 & 0x0000FFFF);
}

_weak void __attribute__ ( ( interrupt, no_auto_psv ) ) _T1Interrupt (  )
{
    LowPowerTimer_callbackInterrupt(OB_LPTIM1);
}

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_LOWPOWER_TIMER
