/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2020 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/NXPMKL/lowpower-timer_MKL.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Low-Power Timer implementations for NXP MKL Series
 */

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

#if defined (LIBOHIBOARD_MKL)

#define LOWPOWERTIMER_VALID_CLOCK_SOURCE(CLOCKSOURCE) (((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_MCGIRCLK) || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_LPO)      || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_ERCLK32K) || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL_OSCERCLK) || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL))

#define LOWPOWERTIMER_VALID_PRESCALER(PRESCALER) (((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV1)     || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV2)     || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV4)     || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV8)     || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV16)    || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV32)    || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV64)    || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV128)   || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV256)   || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV512)   || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV1024)  || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV2048)  || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV4096)  || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV8192)  || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV16384) || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV32768) || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV65536))

#define LOWPOWERTIMER_VALID_POLARITY(POLARITY) (((POLARITY) == LOWPOWERTIMER_CLOCKPOLARITY_RISING)  || \
                                                ((POLARITY) == LOWPOWERTIMER_CLOCKPOLARITY_FALLING))

#define LOWPOWERTIMER_VALID_TRIGGER_SOURCE(TRIGGER) (((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_SOFTWARE) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_0) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_1) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_2) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_3))

#define LOWPOWERTIMER_VALID_UPDATE_MODE(UPDATE) (((UPDATE) == LOWPOWERTIMER_UPDATEMODE_COMPARE)     || \
                                                 ((UPDATE) == LOWPOWERTIMER_UPDATEMODE_FREE_RUNNING))

#define LOWPOWERTIMER_VALID_COUNTER_SOURCE(SOURCE) (((SOURCE) == LOWPOWERTIMER_COUNTERSOURCE_INTERNAL) || \
                                                    ((SOURCE) == LOWPOWERTIMER_COUNTERSOURCE_EXTERNAL))

#define LOWPOWERTIMER_VALID_COUNTER(COUNTER) ((COUNTER) <= 0x0000FFFF)

typedef struct _LowPowerTimer_Device
{
    LPTMR_Type* regmap;                            /**< Device memory pointer */

    volatile uint32_t* simScgcPtr;          /**< Register for clock enabling. */
    uint32_t simScgcBitEnable;         /**< Register mask for current device. */

    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    LowPowerTimer_DeviceState state;           /**< Current peripheral state. */

    LowPowerTimer_Config config;

} LowPowerTimer_Device;


#define LOWPOWERTIMER_IS_DEVICE(DEVICE) ((DEVICE) == OB_LPTIM0)

static LowPowerTimer_Device lptim0 =
{
        .regmap              = LPTMR0,

        .simScgcPtr          = &SIM->SCGC5,
        .simScgcBitEnable    = SIM_SCGC5_LPTMR_MASK,

        .isrNumber           = INTERRUPT_LPTMR0,

        .state               = LOWPOWERTIMER_DEVICESTATE_RESET,
};
LowPowerTimer_DeviceHandle OB_LPTIM0 = &lptim0;

const LowPowerTimer_ClockPrescaler LOWPOWERTIMER_PRESCALER_REGISTER_TABLE[16] =
{
    LOWPOWERTIMER_CLOCKPRESCALER_DIV2,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV4,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV8,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV16,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV32,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV64,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV128,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV256,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV512,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV1024,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV2048,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV4096,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV8192,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV16384,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV32768,
    LOWPOWERTIMER_CLOCKPRESCALER_DIV65536,
};

const uint32_t LOWPOWERTIMER_PRESCALER_REGISTER_VALUE[16] =
{
    2,
    4,
    8,
    16,
    32,
    64,
    128,
    256,
    512,
    1024,
    2048,
    4096,
    8192,
    16384,
    32768,
    65536,
};

static inline void __attribute__((always_inline)) LowPowerTimer_callbackInterrupt (LowPowerTimer_DeviceHandle dev)
{
    // Update event
    if ((dev->regmap->CSR & LPTMR_CSR_TCF_MASK) == LPTMR_CSR_TCF_MASK)
    {
        // Clear flag
        dev->regmap->CSR |= LPTMR_CSR_TCF_MASK;
        // Call callback
        dev->config.counterCallback(dev);
    }
}

static System_Errors LowPowerTimer_config (LowPowerTimer_DeviceHandle dev,
                                  LowPowerTimer_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    err |= ohiassert(LOWPOWERTIMER_VALID_CLOCK_SOURCE(config->clockSource));
    err |= ohiassert(LOWPOWERTIMER_VALID_PRESCALER(config->prescaler));
    // Check for the ultra low-power mode
    if (config->clockSource == LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL)
    {
        err |= ohiassert(LOWPOWERTIMER_VALID_POLARITY(config->polarity));
    }
    // Check the trigger source, and if it is not software, check trigger polarity and sample
    err |= ohiassert(LOWPOWERTIMER_VALID_TRIGGER_SOURCE(config->triggerSource));
    if (config->triggerSource != LOWPOWERTIMER_TRIGGERSOURCE_SOFTWARE)
    {
        // TODO
    }
    err |= ohiassert(LOWPOWERTIMER_VALID_UPDATE_MODE(config->updateMode));
    err |= ohiassert(LOWPOWERTIMER_VALID_COUNTER_SOURCE(config->counterSource));

    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }

    // Save parameters to device structure
    dev->config = *config;

    // Disable the peripheral and interrupt... just for safety!
    dev->regmap->CSR &= ~LPTMR_CSR_TEN_MASK;
    dev->regmap->CSR &= ~LPTMR_CSR_TIE_MASK;

    dev->regmap->CSR |= dev->config.counterSource;

    // TIME MODE
    if (dev->config.counterSource == LOWPOWERTIMER_COUNTERSOURCE_INTERNAL)
    {
        dev->regmap->PSR = 0;
        dev->regmap->PSR |= dev->config.clockSource;
        dev->regmap->PSR |= dev->config.prescaler;
        // Configure update mode
        dev->regmap->CSR |= dev->config.updateMode;
    }
    // PULSE MODE
    else
    {
//    // TODO: add clock source external configuration
//
//    // TODO: add trigger configuration
//
//    // TODO: add configuration input sources
    }

    // Check callback and enable interrupts
    if ((config->counterCallback != 0))
    {
        // Enable interrupt
        Interrupt_enable(dev->isrNumber);
    }
    return err;
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

    // Enable peripheral clock if needed
    if (dev->state == LOWPOWERTIMER_DEVICESTATE_RESET)
    {
        // Enable peripheral clock
        UTILITY_SET_REGISTER_BIT(*dev->simScgcPtr,dev->simScgcBitEnable);
    }
    dev->state = LOWPOWERTIMER_DEVICESTATE_BUSY;

    LowPowerTimer_config(dev,config);

    dev->state = LOWPOWERTIMER_DEVICESTATE_READY;

    return ERRORS_NO_ERROR;
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

    // Disable the peripheral
    dev->regmap->CSR &= ~LPTMR_CSR_TEN_MASK;
    // Disable peripheral clock
    UTILITY_CLEAR_REGISTER_BIT(*dev->simScgcPtr,dev->simScgcBitEnable);

    dev->state = LOWPOWERTIMER_DEVICESTATE_RESET;
    return ERRORS_NO_ERROR;
}

void LowPowerTimer_setPrescaler (LowPowerTimer_DeviceHandle dev,
                                 LowPowerTimer_ClockPrescaler prescaler)
{
    // Check the TIMER instance type
    ohiassert(LOWPOWERTIMER_IS_DEVICE(dev));
    // Check prescaler value
    ohiassert(LOWPOWERTIMER_VALID_PRESCALER(prescaler));
    dev->config.prescaler = prescaler;

    // Disable the peripheral
    dev->regmap->CSR &= ~LPTMR_CSR_TEN_MASK;
    // Set prescaler
    dev->regmap->PSR &= ~(LPTMR_PSR_PRESCALE_MASK);
    dev->regmap->PSR |= LPTMR_PSR_PRESCALE(prescaler);
    // Enable the peripheral
    dev->regmap->CSR |= LPTMR_CSR_TEN_MASK;
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

    dev->state = LOWPOWERTIMER_DEVICESTATE_BUSY;

    dev->regmap->CMR = counter;

    // Check if callback is present
    if (dev->config.counterCallback != 0)
    {
        dev->regmap->CSR |= (LPTMR_CSR_TCF_MASK | LPTMR_CSR_TIE_MASK);
    }

    // Enable the peripheral
    dev->regmap->CSR |= LPTMR_CSR_TEN_MASK;

    dev->state = LOWPOWERTIMER_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
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

    // Disable the peripheral
    dev->regmap->CSR &= ~LPTMR_CSR_TEN_MASK;

    // Disable interrupt
    dev->regmap->CSR &= ~LPTMR_CSR_TIE_MASK;

    dev->state = LOWPOWERTIMER_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

uint32_t LowPowerTimer_getCurrentCounter (LowPowerTimer_DeviceHandle dev)
{
    // Check the TIMER instance type
    ohiassert(LOWPOWERTIMER_IS_DEVICE(dev));

    return (dev->regmap->CNR & 0x0000FFFF);
}

void LPTMR0_IRQHandler (void)
{
    LowPowerTimer_callbackInterrupt(OB_LPTIM0);
}

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_LOWPOWER_TIMER
