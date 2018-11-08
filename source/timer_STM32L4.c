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

#ifdef __cplusplus
extern "C" {
#endif

#include "timer.h"

#include "platforms.h"
#include "utility.h"
#include "system.h"

#if defined (LIBOHIBOARD_STM32L4) && defined (LIBOHIBOARD_TIMER)

typedef struct _Timer_Device
{
    TIM_TypeDef* regmap;                         /**< Device memory pointer */
    LPTIM_TypeDef* regmapLp;

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    volatile uint32_t* rccTypeRegisterPtr;  /**< Register for clock enabling. */
    uint32_t rccTypeRegisterMask;      /**< Register mask for user selection. */
    uint32_t rccTypeRegisterPos;       /**< Mask position for user selection. */

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

static Timer_Device tim1 =
{
        .regmap              = TIM1,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM1EN,
};
Timer_DeviceHandle OB_TIM1 = &tim1;

static Timer_Device tim2 =
{
        .regmap              = TIM2,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM2EN,
};
Timer_DeviceHandle OB_TIM2 = &tim2;

static Timer_Device tim3 =
{
        .regmap              = TIM3,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM3EN,
};
Timer_DeviceHandle OB_TIM3 = &tim3;

static Timer_Device tim4 =
{
        .regmap              = TIM4,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM4EN,
};
Timer_DeviceHandle OB_TIM4 = &tim4;

static Timer_Device tim5 =
{
        .regmap              = TIM5,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM5EN,
};
Timer_DeviceHandle OB_TIM5 = &tim5;

static Timer_Device tim6 =
{
        .regmap              = TIM6,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM6EN,
};
Timer_DeviceHandle OB_TIM6 = &tim6;

static Timer_Device tim7 =
{
        .regmap              = TIM7,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM7EN,
};
Timer_DeviceHandle OB_TIM7 = &tim7;

static Timer_Device tim8 =
{
        .regmap              = TIM8,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM8EN,
};
Timer_DeviceHandle OB_TIM8 = &tim8;

static Timer_Device tim15 =
{
        .regmap              = TIM15,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM15EN,
};
Timer_DeviceHandle OB_TIM15 = &tim15;

static Timer_Device tim16 =
{
        .regmap              = TIM16,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM16EN,
};
Timer_DeviceHandle OB_TIM16 = &tim16;

static Timer_Device tim17 =
{
        .regmap              = TIM17,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM17EN,
};
Timer_DeviceHandle OB_TIM17 = &tim17;

#endif // LIBOHIBOARD_STM32L476Jx || LIBOHIBOARD_STM32L476Rx

System_Errors Timer_init (Timer_DeviceHandle dev, Timer_Config *config)
{

}

System_Errors Timer_deInit (Timer_DeviceHandle dev)
{

}

#endif // LIBOHIBOARD_STM32L4 && LIBOHIBOARD_TIMER

#ifdef __cplusplus
}
#endif
