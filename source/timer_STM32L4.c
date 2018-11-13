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
 * @file libohiboard/include/timer_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Timer implementations for STM32L4 Series
 */

#if defined (LIBOHIBOARD_TIMER)

#ifdef __cplusplus
extern "C" {
#endif

#include "timer.h"

#include "platforms.h"
#include "utility.h"
#include "system.h"
#include "clock.h"
#include "interrupt.h"

#if defined (LIBOHIBOARD_STM32L4)

#define TIMER_CLOCK_ENABLE(REG,MASK)      do { \
                                            UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                          } while (0)

#define TIMER_CLOCK_DISABLE(REG,MASK)     do { \
                                            UTILITY_CLEAR_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                          } while (0)


#define TIMER_CCER_CCxE_MASK              ((uint32_t)(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E))
#define TIMER_CCER_CCxNE_MASK             ((uint32_t)(TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE))

/**
 * Enable selected peripheral
 */
#define TIMER_DEVICE_ENABLE(DEVICE)       ((DEVICE)->regmap->CR1 |= (TIM_CR1_CEN))

/**
 * Disable the selected peripheral.
 */
#define TIMER_DEVICE_DISABLE(DEVICE)      do {                                                              \
                                            if (((DEVICE)->regmap->CCER & TIMER_CCER_CCxE_MASK) == 0uL)   { \
                                              if (((DEVICE)->regmap->CCER & TIMER_CCER_CCxNE_MASK) == 0uL) {\
                                                (DEVICE)->regmap->CR1 &= ~(TIM_CR1_CEN);                    \
                                              }                                                             \
                                            }                                                               \
                                          } while(0)

/**
 * Check if selected peripheral is enable
 */
#define TIMER_DEVICE_IS_ENABLE(DEVICE)    ((((DEVICE)->regmap->CR1 & TIM_CR1_CEN) == TIM_CR1_CEN)   || \
		                                   (((DEVICE)->regmap->CCER & TIMER_CCER_CCxE_MASK) != 0uL) || \
	                                       (((DEVICE)->regmap->CCER & TIMER_CCER_CCxNE_MASK) != 0uL))

#define TIMER_VALID_MODE(MODE) (((MODE) == TIMER_MODE_FREE)           || \
                                ((MODE) == TIMER_MODE_PWM)            || \
                                ((MODE) == TIMER_MODE_INPUT_CAPTURE)  || \
                                ((MODE) == TIMER_MODE_OUTPUT_COMPARE))

#define TIMER_VALID_CLOCKSOURCE(CLOCKSOURCE) (((CLOCKSOURCE) == TIMER_CLOCKSOURCE_INTERNAL)        || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_INTERNAL_ITR0)   || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_INTERNAL_ITR1)   || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_INTERNAL_ITR2)   || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_INTERNAL_ITR3)   || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_EXTERNAL_MODE_1) || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_EXTERNAL_MODE_2))

#define TIMER_VALID_COUNTERMODE(COUNTERMODE) (((COUNTERMODE) == TIMER_COUNTERMODE_UP)               || \
                                              ((COUNTERMODE) == TIMER_COUNTERMODE_DOWN)             || \
                                              ((COUNTERMODE) == TIMER_COUNTERMODE_CENTER_ALIGNED_1) || \
                                              ((COUNTERMODE) == TIMER_COUNTERMODE_CENTER_ALIGNED_2) || \
                                              ((COUNTERMODE) == TIMER_COUNTERMODE_CENTER_ALIGNED_3))

#define TIMER_VALID_AUTORELOAD_PRELOAD(AUTORELOAD) (((AUTORELOAD) == TRUE) || \
                                                    ((AUTORELOAD) == FALSE))

typedef struct _Timer_Device
{
    TIM_TypeDef* regmap;                         /**< Device memory pointer */
    LPTIM_TypeDef* regmapLp;

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    void (* freeCounterCallback)(struct _Timer_Device *dev);

    uint32_t inputClock;                            /**< Current CK_INT value */

    Timer_Mode mode;
    Timer_ClockSource clockSource;
    /**< Define the counter type for a specific operational mode */
    Timer_CounterMode counterMode;

    bool autoreload; /**< Auto-reload preload enable, ARR register is buffered */

    Timer_DeviceState state;                   /**< Current peripheral state. */

} Timer_Device;

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

#define TIMER_IS_DEVICE(DEVICE) (((DEVICE) == OB_TIM1)   || \
                                 ((DEVICE) == OB_TIM2)   || \
                                 ((DEVICE) == OB_TIM3)   || \
                                 ((DEVICE) == OB_TIM4)   || \
                                 ((DEVICE) == OB_TIM5)   || \
                                 ((DEVICE) == OB_TIM6)   || \
                                 ((DEVICE) == OB_TIM7)   || \
                                 ((DEVICE) == OB_TIM8)   || \
                                 ((DEVICE) == OB_TIM15)  || \
                                 ((DEVICE) == OB_TIM16)  || \
                                 ((DEVICE) == OB_TIM17))

#define TIMER_IS_LOWPOWER_DEVICE(DEVICE) (((DEVICE) == OB_LPTIM1)  || \
                                          ((DEVICE) == OB_LPTIM2))

#define TIMER_IS_DEVICE_COUNTER_MODE(DEVICE) (((DEVICE) == OB_TIM1)   || \
                                              ((DEVICE) == OB_TIM2)   || \
                                              ((DEVICE) == OB_TIM3)   || \
                                              ((DEVICE) == OB_TIM4)   || \
                                              ((DEVICE) == OB_TIM5)   || \
                                              ((DEVICE) == OB_TIM8))

#define TIMER_IS_APB1_DEVICE(DEVICE) (((DEVICE) == OB_TIM2)   || \
                                      ((DEVICE) == OB_TIM3)   || \
                                      ((DEVICE) == OB_TIM4)   || \
                                      ((DEVICE) == OB_TIM5)   || \
                                      ((DEVICE) == OB_TIM6)   || \
                                      ((DEVICE) == OB_TIM7)   || \
                                      ((DEVICE) == OB_LPTIM1) || \
                                      ((DEVICE) == OB_LPTIM2))

#define TIMER_IS_APB2_DEVICE(DEVICE) (((DEVICE) == OB_TIM1)   || \
                                      ((DEVICE) == OB_TIM8)   || \
                                      ((DEVICE) == OB_TIM15)  || \
                                      ((DEVICE) == OB_TIM16)  || \
                                      ((DEVICE) == OB_TIM17))

#define TIMER_IS_32BIT_COUNTER_DEVICE(DEVICE) (((DEVICE) == OB_TIM2)   || \
                                               ((DEVICE) == OB_TIM5))

static Timer_Device tim1 =
{
        .regmap              = TIM1,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM1EN,

        .isrNumber           = INTERRUPT_TIM1BRK_TIM15,
};
Timer_DeviceHandle OB_TIM1 = &tim1;

static Timer_Device tim2 =
{
        .regmap              = TIM2,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM2EN,

        .isrNumber           = INTERRUPT_TIM2,
};
Timer_DeviceHandle OB_TIM2 = &tim2;

static Timer_Device tim3 =
{
        .regmap              = TIM3,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM3EN,

        .isrNumber           = INTERRUPT_TIM3,
};
Timer_DeviceHandle OB_TIM3 = &tim3;

static Timer_Device tim4 =
{
        .regmap              = TIM4,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM4EN,

        .isrNumber           = INTERRUPT_TIM4,
};
Timer_DeviceHandle OB_TIM4 = &tim4;

static Timer_Device tim5 =
{
        .regmap              = TIM5,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM5EN,

        .isrNumber           = INTERRUPT_TIM5,
};
Timer_DeviceHandle OB_TIM5 = &tim5;

static Timer_Device tim6 =
{
        .regmap              = TIM6,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM6EN,

        .isrNumber           = INTERRUPT_TIM6DACUNDER,
};
Timer_DeviceHandle OB_TIM6 = &tim6;

static Timer_Device tim7 =
{
        .regmap              = TIM7,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM7EN,

        .isrNumber           = INTERRUPT_TIM7,
};
Timer_DeviceHandle OB_TIM7 = &tim7;

static Timer_Device tim8 =
{
        .regmap              = TIM8,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM8EN,

        .isrNumber           = INTERRUPT_TIM8BRK,
};
Timer_DeviceHandle OB_TIM8 = &tim8;

static Timer_Device tim15 =
{
        .regmap              = TIM15,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM15EN,

        .isrNumber           = INTERRUPT_TIM1BRK_TIM15,
};
Timer_DeviceHandle OB_TIM15 = &tim15;

static Timer_Device tim16 =
{
        .regmap              = TIM16,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM16EN,

        .isrNumber           = INTERRUPT_TIM1UP_TIM16,
};
Timer_DeviceHandle OB_TIM16 = &tim16;

static Timer_Device tim17 =
{
        .regmap              = TIM17,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM17EN,

        .isrNumber           = INTERRUPT_TIM1TRG_TIM17,
};
Timer_DeviceHandle OB_TIM17 = &tim17;

static Timer_Device lptim1 =
{
        .regmapLp            = LPTIM1,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_LPTIM1EN,
};
Timer_DeviceHandle OB_LPTIM1 = &lptim1;

static Timer_Device lptim2 =
{
        .regmapLp            = LPTIM2,

        .rccRegisterPtr      = &RCC->APB1ENR2,
        .rccRegisterEnable   = RCC_APB1ENR2_LPTIM2EN,
};
Timer_DeviceHandle OB_LPTIM2 = &lptim2;

#endif // LIBOHIBOARD_STM32L476Jx || LIBOHIBOARD_STM32L476Rx

static inline void __attribute__((always_inline)) Timer_callbackInterrupt (Timer_DeviceHandle dev)
{
    // Update event
    if ((dev->regmap->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        if ((dev->regmap->DIER & TIM_DIER_UIE) == TIM_DIER_UIE)
        {
            // Clear flag
            dev->regmap->SR &= ~(TIM_SR_UIF);
            // Call callback
            dev->freeCounterCallback(dev);
        }
    }
}

static void Timer_computeCounterValues (Timer_DeviceHandle dev,
                                        Timer_Config *config,
                                        uint16_t* prescaler,
                                        uint32_t* modulo)
{
    uint32_t moduloComputed = 0;
    uint32_t prescalerComputed = 1;

    // Search the correct prescaler
    for (; prescalerComputed < 65536; ++prescalerComputed)
    {
        moduloComputed = (uint32_t)(dev->inputClock / (prescalerComputed * config->timerFrequency));

        if (!TIMER_IS_32BIT_COUNTER_DEVICE(dev))
        {
            if (moduloComputed > 65536)
                continue;
            else
                break;
        }
        else
        {
            break;
        }
    }

    *prescaler = (uint16_t)(prescalerComputed - 1);
    *modulo = (uint32_t)(moduloComputed - 1);
}

static System_Errors Timer_configBase (Timer_DeviceHandle dev, Timer_Config *config)
{
    // Set counter mode: direction and alignment
    if (TIMER_IS_DEVICE_COUNTER_MODE(dev))
    {
        dev->regmap->CR1 &= (~(TIM_CR1_DIR_Msk | TIM_CR1_CMS_Msk));
        dev->counterMode = config->counterMode;
        switch (config->counterMode)
        {
        case TIMER_COUNTERMODE_UP:
            // Nothing to do!
            break;
        case TIMER_COUNTERMODE_DOWN:
            dev->regmap->CR1 |= TIM_CR1_DIR;
            break;
        case TIMER_COUNTERMODE_CENTER_ALIGNED_1:
            dev->regmap->CR1 |= TIM_CR1_CMS_0;
            break;
        case TIMER_COUNTERMODE_CENTER_ALIGNED_2:
            dev->regmap->CR1 |= TIM_CR1_CMS_1;
            break;
        case TIMER_COUNTERMODE_CENTER_ALIGNED_3:
            dev->regmap->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CMS_1;
            break;
        }
    }

    // Set auto-reload
    dev->autoreload = config->autoreload;
    MODIFY_REG(dev->regmap->CR1, TIM_CR1_ARPE_Msk, ((config->autoreload == TRUE) ? TIM_CR1_ARPE : 0));

    if ((dev->clockSource == TIMER_CLOCKSOURCE_INTERNAL) && (config->timerFrequency > 0))
    {
        uint32_t modulo = 0;
        uint16_t prescaler = 0;
        Timer_computeCounterValues(dev,config,&prescaler,&modulo);

        // Write values into register
        // Set the Autoreload value
        dev->regmap->ARR = (uint32_t) modulo;
        // Set the Prescaler value
        dev->regmap->PSC = prescaler;
    }
    // Save Autoreload and Prescaler value chose by user
    else
    {
        // Set the Autoreload value
        dev->regmap->ARR = (uint32_t)(config->modulo - 1);
        // Set the Prescaler value
        dev->regmap->PSC = (config->prescaler - 1);
    }

    // Check callback and interrupt
    if (config->freeCounterCallback != 0)
    {
        // Save callback
        dev->freeCounterCallback = config->freeCounterCallback;
        // Enable interrupt
        Interrupt_enable(dev->isrNumber);
    }

    return ERRORS_NO_ERROR;
}

System_Errors Timer_configClockSource (Timer_DeviceHandle dev, Timer_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    if (ohiassert(TIMER_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    // Check the selected clock source
    if (ohiassert(TIMER_VALID_CLOCKSOURCE(config->clockSource)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }
    dev->clockSource = config->clockSource;

    // Clear Slave Mode Control Register (SMCR)
    // Disable Slave mode, Prescaler, Filter, Polarity
    dev->regmap->SMCR &= ~(TIM_SMCR_SMS  |
                           TIM_SMCR_TS   |
                           TIM_SMCR_ETF  |
                           TIM_SMCR_ETPS |
                           TIM_SMCR_ECE  |
                           TIM_SMCR_ETP);

    switch (dev->clockSource)
    {
    case TIMER_CLOCKSOURCE_INTERNAL:
        {
            uint32_t apbValue = 0, ahbValue = 0;
            ahbValue = Clock_getOutputValue(CLOCK_OUTPUT_HCLK);
            // Compute current CK_INT value
            // Get current APB frequency value
            if (TIMER_IS_APB1_DEVICE(dev))
            {
                apbValue = Clock_getOutputValue(CLOCK_OUTPUT_PCLK1);
            }
            else
            {
                apbValue = Clock_getOutputValue(CLOCK_OUTPUT_PCLK2);
            }
            // Now compute prescaler and save current timer clock value
            if ((ahbValue/apbValue) == 1)
            {
                dev->inputClock = apbValue;
            }
            else
            {
                dev->inputClock = apbValue * 2;
            }
            break;
        }
    }

    dev->state = TIMER_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Timer_init (Timer_DeviceHandle dev, Timer_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    err = ohiassert((TIMER_IS_DEVICE(dev)) || (TIMER_IS_LOWPOWER_DEVICE(dev)));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    err  = ohiassert(TIMER_VALID_MODE(config->mode));
    err |= ohiassert(TIMER_VALID_CLOCKSOURCE(config->clockSource));
    err |= ohiassert(TIMER_VALID_COUNTERMODE(config->counterMode));
    err |= ohiassert(TIMER_VALID_AUTORELOAD_PRELOAD(config->autoreload));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }
    dev->mode = config->mode;

    // Enable peripheral clock if needed
    if (dev->state == TIMER_DEVICESTATE_RESET)
    {
        // Enable peripheral clock
        TIMER_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);
    }

    // Configure clock source
    Timer_configClockSource(dev,config);

    // Now the peripheral is busy
    dev->state = TIMER_DEVICESTATE_BUSY;

    // Configure the peripheral
    switch (dev->mode)
    {
    case TIMER_MODE_FREE:
    case TIMER_MODE_PWM:
        // Check user choices
        ohiassert((config->timerFrequency > 0) || ((config->prescaler > 0) && (config->modulo > 0)));

        Timer_configBase(dev,config);
        break;

    }

    return ERRORS_NO_ERROR;
}

System_Errors Timer_deInit (Timer_DeviceHandle dev)
{

}

System_Errors Timer_start (Timer_DeviceHandle dev)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    if (ohiassert((TIMER_IS_DEVICE(dev)) || (TIMER_IS_LOWPOWER_DEVICE(dev))) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;

    switch (dev->mode)
    {
    case TIMER_MODE_FREE:
        // In case of callback... enable interrupt
        if (dev->freeCounterCallback != 0)
        {
            dev->regmap->DIER |= TIM_DIER_UIE;
        }

        // Enable device
        TIMER_DEVICE_ENABLE(dev);
        break;
    }

    dev->state = TIMER_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Timer_stop (Timer_DeviceHandle dev)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    if (ohiassert((TIMER_IS_DEVICE(dev)) || (TIMER_IS_LOWPOWER_DEVICE(dev))) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;

    switch (dev->mode)
    {
    case TIMER_MODE_FREE:
        // In case of callback... enable interrupt
        if (dev->freeCounterCallback != 0)
        {
            dev->regmap->DIER &=  ~(TIM_DIER_UIE);
        }

        // Enable device
        TIMER_DEVICE_DISABLE(dev);
        break;
    }

    dev->state = TIMER_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

_weak void TIM1_BRK_TIM15_IRQHandler (void)
{
    if (TIMER_DEVICE_IS_ENABLE(OB_TIM1))
    {
        Timer_callbackInterrupt(OB_TIM1);
    }
    else if (TIMER_DEVICE_IS_ENABLE(OB_TIM15))
    {
        Timer_callbackInterrupt(OB_TIM15);
    }
}

_weak void TIM1_UP_TIM16_IRQHandler (void)
{
    if (TIMER_DEVICE_IS_ENABLE(OB_TIM1))
    {
        Timer_callbackInterrupt(OB_TIM1);
    }
    else if (TIMER_DEVICE_IS_ENABLE(OB_TIM16))
    {
        Timer_callbackInterrupt(OB_TIM16);
    }
}

_weak void TIM1_TRG_COM_TIM17_IRQHandler (void)
{
    if (TIMER_DEVICE_IS_ENABLE(OB_TIM1))
    {
        Timer_callbackInterrupt(OB_TIM1);
    }
    else if (TIMER_DEVICE_IS_ENABLE(OB_TIM17))
    {
        Timer_callbackInterrupt(OB_TIM17);
    }
}

_weak void TIM1_CC_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM1);
}

_weak void TIM2_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM2);
}

_weak void TIM3_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM3);
}

_weak void TIM4_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM4);
}

_weak void TIM5_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM5);
}

_weak void TIM6_DAC_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM6);
}

_weak void TIM7_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM7);
}

_weak void TIM8_BRK_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM8);
}

_weak void TIM8_UP_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM8);
}

_weak void TIM8_TRG_COM_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM8);
}

_weak void TIM8_CC_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM8);
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_TIMER
