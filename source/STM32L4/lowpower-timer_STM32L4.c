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
 * @file libohiboard/include/STM32L4/lowpower-timer_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Low-Power Timer implementations for STM32L4 Series
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

#if defined (LIBOHIBOARD_STM32L4)

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

#define LOWPOWERTIMER_VALID_CLOCK_SOURCE(CLOCKSOURCE) (((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_PCLK)  || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_LSI)   || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_HSI16) || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_INTERNAL_LSE)   || \
                                                       ((CLOCKSOURCE) == LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL))

#define LOWPOWERTIMER_VALID_PRESCALER(PRESCALER) (((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV1)  || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV2)  || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV4)  || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV8)  || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV16) || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV32) || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV64) || \
                                                  ((PRESCALER) == LOWPOWERTIMER_CLOCKPRESCALER_DIV128))

#define LOWPOWERTIMER_VALID_POLARITY(POLARITY) (((POLARITY) == LOWPOWERTIMER_CLOCKPOLARITY_RISING)  || \
                                                ((POLARITY) == LOWPOWERTIMER_CLOCKPOLARITY_FALLING) || \
                                                ((POLARITY) == LOWPOWERTIMER_CLOCKPOLARITY_BOTH))

#define LOWPOWERTIMER_VALID_TRIGGER_SOURCE(TRIGGER) (((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_SOFTWARE) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_0) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_1) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_2) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_3) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_4) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_5) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_6) || \
                                                     ((TRIGGER) == LOWPOWERTIMER_TRIGGERSOURCE_7))

#define LOWPOWERTIMER_VALID_UPDATE_MODE(UPDATE) (((UPDATE) == LOWPOWERTIMER_UPDATEMODE_IMMEDIATE) || \
                                                 ((UPDATE) == LOWPOWERTIMER_UPDATEMODE_END_PERIOD))

#define LOWPOWERTIMER_VALID_COUNTER_SOURCE(SOURCE) (((SOURCE) == LOWPOWERTIMER_COUNTERSOURCE_INTERNAL) || \
                                                    ((SOURCE) == LOWPOWERTIMER_COUNTERSOURCE_EXTERNAL))

#define LOWPOWERTIMER_VALID_COUNTER(COUNTER) ((COUNTER) <= 0x0000FFFF)

typedef struct _LowPowerTimer_Device
{
    LPTIM_TypeDef* regmap;                         /**< Device memory pointer */

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    volatile uint32_t* rccTypeRegisterPtr;  /**< Register for clock enabling. */
    uint32_t rccTypeRegisterMask;      /**< Register mask for user selection. */
    uint32_t rccTypeRegisterPos;       /**< Mask position for user selection. */

    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    LowPowerTimer_DeviceState state;           /**< Current peripheral state. */

} LowPowerTimer_Device;

#if defined (LIBOHIBOARD_STM32L476)

#define LOWPOWERTIMER_IS_DEVICE(DEVICE) (((DEVICE) == OB_LPTIM1)  || \
                                         ((DEVICE) == OB_LPTIM2))

static LowPowerTimer_Device lptim1 =
{
        .regmap              = LPTIM1,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_LPTIM1EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_LPTIM1SEL_Msk,
        .rccTypeRegisterPos  = RCC_CCIPR_LPTIM1SEL_Pos,

        .isrNumber           = INTERRUPT_LPTIM1,

        .state               = LOWPOWERTIMER_DEVICESTATE_RESET,
};
LowPowerTimer_DeviceHandle OB_LPTIM1 = &lptim1;

static LowPowerTimer_Device lptim2 =
{
        .regmap              = LPTIM2,

        .rccRegisterPtr      = &RCC->APB1ENR2,
        .rccRegisterEnable   = RCC_APB1ENR2_LPTIM2EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_LPTIM2SEL_Msk,
        .rccTypeRegisterPos  = RCC_CCIPR_LPTIM2SEL_Pos,

        .isrNumber           = INTERRUPT_LPTIM2,

        .state               = LOWPOWERTIMER_DEVICESTATE_RESET,
};
LowPowerTimer_DeviceHandle OB_LPTIM2 = &lptim2;

#endif // LIBOHIBOARD_STM32L476

static inline void __attribute__((always_inline)) LowPowerTimer_callbackInterrupt (LowPowerTimer_DeviceHandle dev)
{

}

static void LowPowerTimer_config (LowPowerTimer_DeviceHandle dev,
                                  LowPowerTimer_Config *config)
{
    // read CFGR value
    uint32_t tmpreg = dev->regmap->CFGR;

    // When external clock is selected, clear CKPOL and CKFLT
    if (config->clockSource == LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL)
    {
        tmpreg &= (uint32_t)(~(LPTIM_CFGR_CKPOL_Msk | LPTIM_CFGR_CKFLT_Msk));
    }

    // When the trigger is not software, clear TRGFLT and TRIGSEL
    if (config->triggerSource != LOWPOWERTIMER_TRIGGERSOURCE_SOFTWARE)
    {
        tmpreg &= (uint32_t)(~(LPTIM_CFGR_TRGFLT_Msk | LPTIM_CFGR_TRIGSEL_Msk));
    }

    // Clear other common bits..
    tmpreg &= (uint32_t)(~(LPTIM_CFGR_CKSEL_Msk  | LPTIM_CFGR_TRIGEN_Msk | LPTIM_CFGR_PRELOAD_Msk |
                           LPTIM_CFGR_WAVPOL_Msk | LPTIM_CFGR_PRESC_Msk  | LPTIM_CFGR_COUNTMODE_Msk ));

    // Write new configuration
    // TODO: add all configuration
    tmpreg |= ((config->clockSource == LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL) ? LPTIM_CFGR_CKSEL : 0x00000000) |
              config->prescaler     |
              config->counterSource |
              config->updateMode;

    // TODO: add clock source external configuration

    // TODO: add trigger configuration

    // TODO: add configuration input sources

    // Save configurations into register
    dev->regmap->CFGR = tmpreg;
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
    // Check for the ultra low-power mode
    if (config->clockSource == LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL)
    {
        err |= ohiassert(LOWPOWERTIMER_VALID_POLARITY(config->polarity));
        // TODO: add check on sample time!
    }
    // Check the trigger source, and if it is not software, check trigger polarity and sample
    err |= ohiassert(LOWPOWERTIMER_VALID_TRIGGER_SOURCE(config->triggerSource));
    if (config->triggerSource != LOWPOWERTIMER_TRIGGERSOURCE_SOFTWARE)
    {
        // TODO
    }
    // TODO: check output polarity
    err |= ohiassert(LOWPOWERTIMER_VALID_UPDATE_MODE(config->updateMode));
    err |= ohiassert(LOWPOWERTIMER_VALID_COUNTER_SOURCE(config->counterSource));

    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }

    // Enable peripheral clock if needed
    if (dev->state == LOWPOWERTIMER_DEVICESTATE_RESET)
    {
        if (config->clockSource != LOWPOWERTIMER_CLOCKSOURCE_EXTERNAL)
        {
            // Select clock source
            UTILITY_MODIFY_REGISTER(*dev->rccTypeRegisterPtr,
                                    dev->rccTypeRegisterMask,
                                    (config->clockSource << dev->rccTypeRegisterPos));
        }

        // Enable peripheral clock
        LOWPOWERTIMER_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);
    }
    dev->state = LOWPOWERTIMER_DEVICESTATE_BUSY;

    LowPowerTimer_config(dev,config);

    dev->state = LOWPOWERTIMER_DEVICESTATE_READY;

    return ERRORS_NO_ERROR;
}

System_Errors LowPowerTimer_deInit (LowPowerTimer_DeviceHandle dev)
{
    // TODO
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

    // TODO: check prescaler value...

    // Check counter value
    ohiassert(LOWPOWERTIMER_VALID_COUNTER(counter));

    dev->state = LOWPOWERTIMER_DEVICESTATE_BUSY;

    // Enable the peripheral
    dev->regmap->CR |= LPTIM_CR_ENABLE;

    // Update reload register
    dev->regmap->ARR = counter;

    // Enable continuous mode
    dev->regmap->CR |= LPTIM_CR_CNTSTRT;

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

}


_weak void LPTIM1_IRQHandler (void)
{
    LowPowerTimer_callbackInterrupt(OB_LPTIM1);
}

_weak void LPTIM2_IRQHandler (void)
{
    LowPowerTimer_callbackInterrupt(OB_LPTIM2);
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_LOWPOWER_TIMER
