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

#define TIMER_VALID_CHANNEL(CHANNEL) (((CHANNEL) == TIMER_CHANNELS_CH1) || \
                                      ((CHANNEL) == TIMER_CHANNELS_CH2) || \
                                      ((CHANNEL) == TIMER_CHANNELS_CH3) || \
                                      ((CHANNEL) == TIMER_CHANNELS_CH4) || \
                                      ((CHANNEL) == TIMER_CHANNELS_CH5) || \
                                      ((CHANNEL) == TIMER_CHANNELS_CH6))

#define TIMER_VALID_PWM_MODE(MODE) (((MODE) == TIMER_OUTPUTCOMPAREMODE_PWM1)            || \
                                    ((MODE) == TIMER_OUTPUTCOMPAREMODE_PWM2)            || \
                                    ((MODE) == TIMER_OUTPUTCOMPAREMODE_COMBINED_PWM1)   || \
                                    ((MODE) == TIMER_OUTPUTCOMPAREMODE_COMBINED_PWM2)   || \
                                    ((MODE) == TIMER_OUTPUTCOMPAREMODE_ASYMMETRIC_PWM1) || \
                                    ((MODE) == TIMER_OUTPUTCOMPAREMODE_ASYMMETRIC_PWM2))

#define TIMER_VALID_OC_MODE(MODE) (((MODE) == TIMER_OUTPUTCOMPAREMODE_COUNTING)           || \
                                   ((MODE) == TIMER_OUTPUTCOMPAREMODE_ACTIVE_ON_MATCH)    || \
                                   ((MODE) == TIMER_OUTPUTCOMPAREMODE_INACTIVE_ON_MATCH)  || \
                                   ((MODE) == TIMER_OUTPUTCOMPAREMODE_TOGGLE)             || \
                                   ((MODE) == TIMER_OUTPUTCOMPAREMODE_FORCED_INACTIVE)    || \
                                   ((MODE) == TIMER_OUTPUTCOMPAREMODE_FORCED_ACTIVE)      || \
                                   ((MODE) == TIMER_OUTPUTCOMPAREMODE_RETRIGGERABLE_OPM1) || \
                                   ((MODE) == TIMER_OUTPUTCOMPAREMODE_RETRIGGERABLE_OPM2))

#define TIMER_VALID_OC_POLARITY(POLARITY) (((POLARITY) == GPIO_HIGH) || \
                                           ((POLARITY) == GPIO_LOW))

#define TIMER_VALID_OC_FAST_MODE(FASTMODE) (((FASTMODE) == TRUE)  || \
                                            ((FASTMODE) == FALSE))

#define TIMER_VALID_IC_POLARITY(POLARITY) (((POLARITY) == TIMER_INPUTCAPTUREPOLARITY_RISING)  || \
                                           ((POLARITY) == TIMER_INPUTCAPTUREPOLARITY_FALLING) || \
                                           ((POLARITY) == TIMER_INPUTCAPTUREPOLARITY_BOTH))

#define TIMER_VALID_IC_PRESCALER(PRESCALER) (((PRESCALER) == TIMER_INPUTCAPTUREPRESCALER_DIV1) || \
                                             ((PRESCALER) == TIMER_INPUTCAPTUREPRESCALER_DIV2) || \
                                             ((PRESCALER) == TIMER_INPUTCAPTUREPRESCALER_DIV4) || \
                                             ((PRESCALER) == TIMER_INPUTCAPTUREPRESCALER_DIV8))

#define TIMER_VALID_IC_SELECTION(SELECTION) (((SELECTION) == TIMER_INPUTCAPTURESELECTION_DIRECT)   || \
                                             ((SELECTION) == TIMER_INPUTCAPTURESELECTION_INDIRECT) || \
                                             ((SELECTION) == TIMER_INPUTCAPTURESELECTION_TRC))

#define TIMER_VALID_AUTORELOAD_PRELOAD(AUTORELOAD) (((AUTORELOAD) == TRUE) || \
                                                    ((AUTORELOAD) == FALSE))

#define TIMER_MAX_PINS                   20

typedef struct _Timer_Device
{
    TIM_TypeDef* regmap;                         /**< Device memory pointer */
    LPTIM_TypeDef* regmapLp;

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    Timer_Pins pins[TIMER_MAX_PINS];/**< List of the pin for the timer channel. */
    Timer_Channels pinsChannel[TIMER_MAX_PINS];
    Gpio_Pins pinsGpio[TIMER_MAX_PINS];
    Gpio_Alternate pinsMux[TIMER_MAX_PINS];

    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Timer_ActiveChannels currentChannel;

    void (* freeCounterCallback)(struct _Timer_Device *dev);
    void (* pwmPulseFinishedCallback)(struct _Timer_Device *dev);
    void (* outputCompareCallback)(struct _Timer_Device *dev);
    void (* inputCaptureCallback)(struct _Timer_Device *dev);

    uint32_t inputClock;                            /**< Current CK_INT value */

    Timer_Mode mode;
    Timer_ClockSource clockSource;
    /**< Define the counter type for a specific operational mode */
    Timer_CounterMode counterMode;

    bool autoreload; /**< Auto-reload preload enable, ARR register is buffered */

    Timer_DeviceState state;                   /**< Current peripheral state. */

} Timer_Device;

#if defined (LIBOHIBOARD_STM32L476)

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

#define TIMER_IS_OC_DEVICE(DEVICE) (((DEVICE) == OB_TIM1)   || \
                                    ((DEVICE) == OB_TIM2)   || \
                                    ((DEVICE) == OB_TIM3)   || \
                                    ((DEVICE) == OB_TIM4)   || \
                                    ((DEVICE) == OB_TIM5)   || \
                                    ((DEVICE) == OB_TIM8)   || \
                                    ((DEVICE) == OB_TIM15)  || \
                                    ((DEVICE) == OB_TIM16)  || \
                                    ((DEVICE) == OB_TIM17))

#define TIMER_IS_IC_DEVICE(DEVICE) (((DEVICE) == OB_TIM1)   || \
                                    ((DEVICE) == OB_TIM2)   || \
                                    ((DEVICE) == OB_TIM3)   || \
                                    ((DEVICE) == OB_TIM4)   || \
                                    ((DEVICE) == OB_TIM5)   || \
                                    ((DEVICE) == OB_TIM8)   || \
                                    ((DEVICE) == OB_TIM15)  || \
                                    ((DEVICE) == OB_TIM16)  || \
                                    ((DEVICE) == OB_TIM17))

#define TIMER_IS_CHANNEL1_DEVICE(DEVICE) (((DEVICE) == OB_TIM1)  || \
                                          ((DEVICE) == OB_TIM2)  || \
                                          ((DEVICE) == OB_TIM3)  || \
                                          ((DEVICE) == OB_TIM4)  || \
                                          ((DEVICE) == OB_TIM5)  || \
                                          ((DEVICE) == OB_TIM8)  || \
                                          ((DEVICE) == OB_TIM15) || \
                                          ((DEVICE) == OB_TIM16) || \
                                          ((DEVICE) == OB_TIM17))

#define TIMER_IS_CHANNEL2_DEVICE(DEVICE) (((DEVICE) == OB_TIM1)  || \
                                          ((DEVICE) == OB_TIM2)  || \
                                          ((DEVICE) == OB_TIM3)  || \
                                          ((DEVICE) == OB_TIM4)  || \
                                          ((DEVICE) == OB_TIM5)  || \
                                          ((DEVICE) == OB_TIM8)  || \
                                          ((DEVICE) == OB_TIM15))

#define TIMER_IS_CHANNEL3_DEVICE(DEVICE) (((DEVICE) == OB_TIM1)  || \
                                          ((DEVICE) == OB_TIM2)  || \
                                          ((DEVICE) == OB_TIM3)  || \
                                          ((DEVICE) == OB_TIM4)  || \
                                          ((DEVICE) == OB_TIM5)  || \
                                          ((DEVICE) == OB_TIM8))

#define TIMER_IS_CHANNEL4_DEVICE(DEVICE) (((DEVICE) == OB_TIM1) || \
                                          ((DEVICE) == OB_TIM2) || \
                                          ((DEVICE) == OB_TIM3) || \
                                          ((DEVICE) == OB_TIM4) || \
                                          ((DEVICE) == OB_TIM5) || \
                                          ((DEVICE) == OB_TIM8))

#define TIMER_IS_CHANNEL5_DEVICE(DEVICE) (((DEVICE) == OB_TIM1) || \
                                          ((DEVICE) == OB_TIM8))

#define TIMER_IS_CHANNEL6_DEVICE(DEVICE) (((DEVICE) == OB_TIM1) || \
                                          ((DEVICE) == OB_TIM8))

#define TIMER_IS_CHANNEL_DEVICE(DEVICE, CHANNEL)                                          \
                                                ((((DEVICE) == OB_TIM1) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH2) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH3) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH4) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH5) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH6)))  || \
                                                 (((DEVICE) == OB_TIM2) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH2) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH3) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH4)))  || \
                                                 (((DEVICE) == OB_TIM3) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH2) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH3) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH4)))  || \
                                                 (((DEVICE) == OB_TIM4) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH2) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH3) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH4)))  || \
                                                 (((DEVICE) == OB_TIM5) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH2) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH3) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH4)))  || \
												 (((DEVICE) == OB_TIM8) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH2) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH3) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH4) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH5) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH6)))  || \
                                                 (((DEVICE) == OB_TIM15) &&               \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH2)))  || \
                                                 (((DEVICE) == OB_TIM16) &&               \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1)))  || \
                                                 (((DEVICE) == OB_TIM17) &&               \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1))))

#define TIMER_IS_NCHANNEL_DEVICE(DEVICE, CHANNEL)                                          \
                                                ((((DEVICE) == OB_TIM1) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH2) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH3)))  || \
                                                 (((DEVICE) == OB_TIM8) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH2) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH3)))  || \
                                                 (((DEVICE) == OB_TIM15) &&               \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1)))  || \
                                                 (((DEVICE) == OB_TIM16) &&               \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1)))  || \
                                                 (((DEVICE) == OB_TIM17) &&               \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH1))))


static Timer_Device tim1 =
{
        .regmap              = TIM1,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM1EN,

        .pins             =
        {
                               TIMER_PINS_PA8,
                               TIMER_PINS_PA9,
                               TIMER_PINS_PA10,
                               TIMER_PINS_PA11,
        },
        .pinsChannel      =
        {
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
                               TIMER_CHANNELS_CH3,
                               TIMER_CHANNELS_CH4,
        },
        .pinsGpio         =
        {
                               GPIO_PINS_PA8,
                               GPIO_PINS_PA9,
                               GPIO_PINS_PA10,
                               GPIO_PINS_PA11,
        },
        .pinsMux          =
        {
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
        },

        .isrNumber           = INTERRUPT_TIM1BRK_TIM15,
};
Timer_DeviceHandle OB_TIM1 = &tim1;

static Timer_Device tim2 =
{
        .regmap              = TIM2,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM2EN,

        .pins             =
        {
                               TIMER_PINS_PA0,
                               TIMER_PINS_PA1,
                               TIMER_PINS_PA2,
                               TIMER_PINS_PA3,
                               TIMER_PINS_PA5,
                               TIMER_PINS_PA15,
                               TIMER_PINS_PB3,
                               TIMER_PINS_PB10,
                               TIMER_PINS_PB11,
        },
        .pinsChannel      =
        {
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
                               TIMER_CHANNELS_CH3,
                               TIMER_CHANNELS_CH4,
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
                               TIMER_CHANNELS_CH3,
                               TIMER_CHANNELS_CH4,
        },
        .pinsGpio         =
        {
                               GPIO_PINS_PA0,
                               GPIO_PINS_PA1,
                               GPIO_PINS_PA2,
                               GPIO_PINS_PA3,
                               GPIO_PINS_PA5,
                               GPIO_PINS_PA15,
                               GPIO_PINS_PB3,
                               GPIO_PINS_PB10,
                               GPIO_PINS_PB11,
        },
        .pinsMux          =
        {
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
                               GPIO_ALTERNATE_1,
        },

        .isrNumber           = INTERRUPT_TIM2,
};
Timer_DeviceHandle OB_TIM2 = &tim2;

static Timer_Device tim3 =
{
        .regmap              = TIM3,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM3EN,

        .pins             =
        {
                               TIMER_PINS_PA6,
                               TIMER_PINS_PA7,
                               TIMER_PINS_PB0,
                               TIMER_PINS_PB1,
                               TIMER_PINS_PB4,
                               TIMER_PINS_PB5,
                               TIMER_PINS_PC6,
                               TIMER_PINS_PC7,
                               TIMER_PINS_PC8,
                               TIMER_PINS_PC9,
        },
        .pinsChannel      =
        {
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
                               TIMER_CHANNELS_CH3,
                               TIMER_CHANNELS_CH4,
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
                               TIMER_CHANNELS_CH3,
                               TIMER_CHANNELS_CH4,
        },
        .pinsGpio         =
        {
                               GPIO_PINS_PA6,
                               GPIO_PINS_PA7,
                               GPIO_PINS_PB0,
                               GPIO_PINS_PB1,
                               GPIO_PINS_PB4,
                               GPIO_PINS_PB5,
                               GPIO_PINS_PC6,
                               GPIO_PINS_PC7,
                               GPIO_PINS_PC8,
                               GPIO_PINS_PC9,
        },
        .pinsMux          =
        {
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
        },

        .isrNumber           = INTERRUPT_TIM3,
};
Timer_DeviceHandle OB_TIM3 = &tim3;

static Timer_Device tim4 =
{
        .regmap              = TIM4,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM4EN,

        .pins             =
        {
                               TIMER_PINS_PB6,
                               TIMER_PINS_PB7,
                               TIMER_PINS_PB8,
                               TIMER_PINS_PB9,
        },
        .pinsChannel      =
        {
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
                               TIMER_CHANNELS_CH3,
                               TIMER_CHANNELS_CH4,
        },
        .pinsGpio         =
        {
                               GPIO_PINS_PB6,
                               GPIO_PINS_PB7,
                               GPIO_PINS_PB8,
                               GPIO_PINS_PB9,
        },
        .pinsMux          =
        {
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
        },

        .isrNumber           = INTERRUPT_TIM4,
};
Timer_DeviceHandle OB_TIM4 = &tim4;

static Timer_Device tim5 =
{
        .regmap              = TIM5,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_TIM5EN,

        .pins             =
        {
                               TIMER_PINS_PA0,
                               TIMER_PINS_PA1,
                               TIMER_PINS_PA2,
                               TIMER_PINS_PA3,
        },
        .pinsChannel      =
        {
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
                               TIMER_CHANNELS_CH3,
                               TIMER_CHANNELS_CH4,
        },
        .pinsGpio         =
        {
                               GPIO_PINS_PA0,
                               GPIO_PINS_PA1,
                               GPIO_PINS_PA2,
                               GPIO_PINS_PA3,
        },
        .pinsMux          =
        {
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_2,
        },


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

        .pins             =
        {
                               TIMER_PINS_PC6,
                               TIMER_PINS_PC7,
                               TIMER_PINS_PC8,
                               TIMER_PINS_PC9,
        },
        .pinsChannel      =
        {
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
                               TIMER_CHANNELS_CH3,
                               TIMER_CHANNELS_CH4,
        },
        .pinsGpio         =
        {
                               GPIO_PINS_PC6,
                               GPIO_PINS_PC7,
                               GPIO_PINS_PC8,
                               GPIO_PINS_PC9,
        },
        .pinsMux          =
        {
                               GPIO_ALTERNATE_3,
                               GPIO_ALTERNATE_3,
                               GPIO_ALTERNATE_3,
                               GPIO_ALTERNATE_3,
        },

        .isrNumber           = INTERRUPT_TIM8BRK,
};
Timer_DeviceHandle OB_TIM8 = &tim8;

static Timer_Device tim15 =
{
        .regmap              = TIM15,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM15EN,

        .pins             =
        {
                               TIMER_PINS_PA2,
                               TIMER_PINS_PA3,
                               TIMER_PINS_PB14,
                               TIMER_PINS_PB15,
#if defined (LIBOHIBOARD_STM32L476Jx)
                               TIMER_PINS_PG10,
                               TIMER_PINS_PG11,
#endif
        },
        .pinsChannel      =
        {
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
#if defined (LIBOHIBOARD_STM32L476Jx)
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH2,
#endif
        },
        .pinsGpio         =
        {
                               GPIO_PINS_PA2,
                               GPIO_PINS_PA3,
                               GPIO_PINS_PB14,
                               GPIO_PINS_PB15,
#if defined (LIBOHIBOARD_STM32L476Jx)
                               GPIO_PINS_PG10,
                               GPIO_PINS_PG11,
#endif
        },
        .pinsMux          =
        {
                               GPIO_ALTERNATE_14,
                               GPIO_ALTERNATE_14,
                               GPIO_ALTERNATE_14,
                               GPIO_ALTERNATE_14,
#if defined (LIBOHIBOARD_STM32L476Jx)
                               GPIO_ALTERNATE_14,
                               GPIO_ALTERNATE_14,
#endif
        },

        .isrNumber           = INTERRUPT_TIM1BRK_TIM15,
};
Timer_DeviceHandle OB_TIM15 = &tim15;

static Timer_Device tim16 =
{
        .regmap              = TIM16,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM16EN,

        .pins             =
        {
                               TIMER_PINS_PA6,
                               TIMER_PINS_PB8,
        },
        .pinsChannel      =
        {
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH1,
        },
        .pinsGpio         =
        {
                               GPIO_PINS_PA6,
                               GPIO_PINS_PB8,
        },
        .pinsMux          =
        {
                               GPIO_ALTERNATE_14,
                               GPIO_ALTERNATE_14,
        },

        .isrNumber           = INTERRUPT_TIM1UP_TIM16,
};
Timer_DeviceHandle OB_TIM16 = &tim16;

static Timer_Device tim17 =
{
        .regmap              = TIM17,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_TIM17EN,

        .pins             =
        {
                               TIMER_PINS_PA7,
                               TIMER_PINS_PB9,
        },
        .pinsChannel      =
        {
                               TIMER_CHANNELS_CH1,
                               TIMER_CHANNELS_CH1,
        },
        .pinsGpio         =
        {
                               GPIO_PINS_PA7,
                               GPIO_PINS_PB9,
        },
        .pinsMux          =
        {
                               GPIO_ALTERNATE_14,
                               GPIO_ALTERNATE_14,
        },

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

#endif // LIBOHIBOARD_STM32L476

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

    // Capture/Compare Channel 1
    if ((dev->regmap->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF)
    {
        // Check the interrupt configuration
        if ((dev->regmap->DIER & TIM_DIER_CC1IE) == TIM_DIER_CC1IE)
        {
            // Clear flag
            dev->regmap->SR &= ~(TIM_SR_CC1IF);
            // Save current channel
            dev->currentChannel = TIMER_ACTIVECHANNELS_CH1;
            // Callback for input compare
            if ((dev->regmap->CCMR1 & TIM_CCMR1_CC1S) != 0u)
            {
                dev->inputCaptureCallback(dev);
            }
            // Callback for output compare
            else
            {
                dev->pwmPulseFinishedCallback(dev);
                dev->outputCompareCallback(dev);
            }
            // Clear current channel
            dev->currentChannel = TIMER_ACTIVECHANNELS_NONE;
        }
    }

    // Capture/Compare Channel 2
    if ((dev->regmap->SR & TIM_SR_CC2IF) == TIM_SR_CC2IF)
    {
        // Check the interrupt configuration
        if ((dev->regmap->DIER & TIM_DIER_CC2IE) == TIM_DIER_CC2IE)
        {
            // Clear flag
            dev->regmap->SR &= ~(TIM_SR_CC2IF);
            // Save current channel
            dev->currentChannel = TIMER_ACTIVECHANNELS_CH2;
            // Callback for input compare
            if ((dev->regmap->CCMR1 & TIM_CCMR1_CC2S) != 0u)
            {
                dev->inputCaptureCallback(dev);
            }
            // Callback for output compare
            else
            {
                dev->pwmPulseFinishedCallback(dev);
                dev->outputCompareCallback(dev);
            }
            // Clear current channel
            dev->currentChannel = TIMER_ACTIVECHANNELS_NONE;
        }
    }

    // Capture/Compare Channel 3
    if ((dev->regmap->SR & TIM_SR_CC3IF) == TIM_SR_CC3IF)
    {
        // Check the interrupt configuration
        if ((dev->regmap->DIER & TIM_DIER_CC3IE) == TIM_DIER_CC3IE)
        {
            // Clear flag
            dev->regmap->SR &= ~(TIM_SR_CC3IF);
            // Save current channel
            dev->currentChannel = TIMER_ACTIVECHANNELS_CH3;
            // Callback for input compare
            if ((dev->regmap->CCMR2 & TIM_CCMR2_CC3S) != 0u)
            {
                dev->inputCaptureCallback(dev);
            }
            // Callback for output compare
            else
            {
                dev->pwmPulseFinishedCallback(dev);
                dev->outputCompareCallback(dev);
            }
            // Clear current channel
            dev->currentChannel = TIMER_ACTIVECHANNELS_NONE;
        }
    }

    // Capture/Compare Channel 4
    if ((dev->regmap->SR & TIM_SR_CC4IF) == TIM_SR_CC4IF)
    {
        // Check the interrupt configuration
        if ((dev->regmap->DIER & TIM_DIER_CC4IE) == TIM_DIER_CC4IE)
        {
            // Clear flag
            dev->regmap->SR &= ~(TIM_SR_CC4IF);
            // Save current channel
            dev->currentChannel = TIMER_ACTIVECHANNELS_CH4;
            // Callback for input compare
            if ((dev->regmap->CCMR2 & TIM_CCMR2_CC4S) != 0u)
            {
                dev->inputCaptureCallback(dev);
            }
            // Callback for output compare
            else
            {
             dev->pwmPulseFinishedCallback(dev);
                dev->outputCompareCallback(dev);
            }
            // Clear current channel
            dev->currentChannel = TIMER_ACTIVECHANNELS_NONE;
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

    // Check callback and interrupt for free-counter
    if (config->freeCounterCallback != 0)
    {
        // Save callback
        dev->freeCounterCallback = config->freeCounterCallback;
        // Enable interrupt
        Interrupt_enable(dev->isrNumber);
    }

    // Check callback and interrupt for PWM
    if (config->pwmPulseFinishedCallback != 0)
    {
        // Save callback
        dev->pwmPulseFinishedCallback = config->pwmPulseFinishedCallback;
        // Enable interrupt
        Interrupt_enable(dev->isrNumber);
    }

    if (config->outputCompareCallback != 0)
    {
        // Save callback
        dev->outputCompareCallback = config->outputCompareCallback;
        // Enable interrupt
        Interrupt_enable(dev->isrNumber);
    }

    if (config->inputCaptureCallback != 0)
    {
        // Save callback
        dev->inputCaptureCallback = config->inputCaptureCallback;
        // Enable interrupt
        Interrupt_enable(dev->isrNumber);
    }

    // Reset CNT and CNT_PSC, and generate an update event to reload all
    // value immediately
    dev->regmap->EGR = TIM_EGR_UG;

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

    case TIMER_MODE_OUTPUT_COMPARE:
        // Check user choices
        ohiassert(config->prescaler > 0);

        Timer_configBase(dev,config);
        break;

    case TIMER_MODE_INPUT_CAPTURE:
        Timer_configBase(dev,config);
        break;

    default:
        ohiassert(0);
        break;
    }

    return ERRORS_NO_ERROR;
}

System_Errors Timer_deInit (Timer_DeviceHandle dev)
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

    // Disable the peripheral
    TIMER_CLOCK_DISABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

    dev->state = TIMER_DEVICESTATE_RESET;
    return ERRORS_NO_ERROR;
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

    // In case of callback... enable interrupt
    if (dev->freeCounterCallback != 0)
    {
        dev->regmap->DIER |= TIM_DIER_UIE;
    }

    // Enable device
    TIMER_DEVICE_ENABLE(dev);

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

    // In case of callback... enable interrupt
    if (dev->freeCounterCallback != 0)
    {
        dev->regmap->DIER &=  ~(TIM_DIER_UIE);
    }

    // Disable device
    TIMER_DEVICE_DISABLE(dev);

    dev->state = TIMER_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

/**
 * This function enable or disable the TIM Capture/Compare Channel.
 *
 * @param[in] dev Timer device handle
 * @param[in] channel The output channel
 * @param[in] enable The enable status of CC channel (TRUE for the enable, FALSE otherwise)
 */
static inline void __attribute__((always_inline)) Timer_manageCCxChannel (Timer_DeviceHandle dev,
                                                                          Timer_Channels channel,
                                                                          bool enable)
{
    uint32_t tmp = 0;

    ohiassert(TIMER_VALID_CHANNEL(channel));
    ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,channel));

    tmp = TIM_CCER_CC1E << channel;
    // Reset channel state
    dev->regmap->CCER &= ~tmp;

    // Enable or disable the channel
    dev->regmap->CCER |= (((enable == TRUE) ? TIM_CCER_CC1E : 0u) << channel);
}

/**
 * This function return the address of CCRn register of selected channel.
 *
 * @param[in] dev Timer device handle
 * @param[in] channel The output channel
 * @return The register CCRn address
 */
static inline volatile uint32_t* __attribute__((always_inline)) Timer_getCCRnRegister (Timer_DeviceHandle dev,
                                                                                       Timer_Channels channel)
{
    switch (channel)
    {
    case TIMER_CHANNELS_CH1:
        return &(dev->regmap->CCR1);
    case TIMER_CHANNELS_CH2:
        return &(dev->regmap->CCR2);
    case TIMER_CHANNELS_CH3:
        return &(dev->regmap->CCR3);
    case TIMER_CHANNELS_CH4:
        return &(dev->regmap->CCR4);
    case TIMER_CHANNELS_CH5:
        return &(dev->regmap->CCR5);
    case TIMER_CHANNELS_CH6:
        return &(dev->regmap->CCR6);
    default:
        ohiassert(0);
        return 0;
    }
}

/**
 * This function return the address of CCMRn register of selected channel.
 *
 * @param[in] dev Timer device handle
 * @param[in] channel The output channel
 * @return The register CCMRn address
 */
static inline volatile uint32_t* __attribute__((always_inline)) Timer_getCCMRnRegister (Timer_DeviceHandle dev,
                                                                                        Timer_Channels channel)
{
    switch (channel)
    {
    case TIMER_CHANNELS_CH1:
    case TIMER_CHANNELS_CH2:
        return &(dev->regmap->CCMR1);
    case TIMER_CHANNELS_CH3:
    case TIMER_CHANNELS_CH4:
        return &(dev->regmap->CCMR2);
    case TIMER_CHANNELS_CH5:
    case TIMER_CHANNELS_CH6:
        return &(dev->regmap->CCMR3);
    default:
        ohiassert(0);
        return 0;
    }
}

System_Errors Timer_configPwmPin (Timer_DeviceHandle dev,
                                  Timer_OutputCompareConfig* config,
                                  Timer_Pins pin)
{
    System_Errors err = ERRORS_NO_ERROR;

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

    err  = ohiassert(TIMER_IS_OC_DEVICE(dev));
    err |= ohiassert(TIMER_VALID_PWM_MODE(config->mode));
    err |= ohiassert(TIMER_VALID_OC_FAST_MODE(config->fastMode));
    err |= ohiassert(TIMER_VALID_OC_POLARITY(config->polarity));
    err |= ohiassert(config->duty <= 100);
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;

    // Configure alternate function on selected pin
    // And save selected channel
    Timer_Channels channel;
    bool isPinFound = FALSE;
    for (uint16_t i = 0; i < TIMER_MAX_PINS; ++i)
    {
        if (dev->pins[i] == pin)
        {
            Gpio_configAlternate(dev->pinsGpio[i],
                                 dev->pinsMux[i],
                                 0);
            channel = dev->pinsChannel[i];
            isPinFound = TRUE;
            break;
        }
    }
    if (isPinFound == FALSE)
    {
        dev->state = TIMER_DEVICESTATE_ERROR;
        return ERRORS_TIMER_NO_PWM_PIN_FOUND;
    }

    // Configure Output Compare functions
    // Temporary variables
    volatile uint32_t* regCCMRxPtr = Timer_getCCMRnRegister(dev,channel);
    uint32_t tmpccmrx = *regCCMRxPtr;
    uint32_t tmpccer;

    uint32_t shiftccmrx;

    // Disable Channel
    dev->regmap->CCER &= ~(TIM_CCER_CC1E << channel);

    // Set-up registers for specific channel
    switch (channel)
    {
    case TIMER_CHANNELS_CH1:
        shiftccmrx = 0u;
        break;
    case TIMER_CHANNELS_CH2:
        shiftccmrx = 8u;
        break;
    case TIMER_CHANNELS_CH3:
        shiftccmrx = 0u;
        break;
    case TIMER_CHANNELS_CH4:
        shiftccmrx = 8u;
        break;
    case TIMER_CHANNELS_CH5:
        shiftccmrx = 0u;
        break;
    case TIMER_CHANNELS_CH6:
        shiftccmrx = 8u;
        break;
    default:
        ohiassert(0);
    }

    // Get common register values
    tmpccer = dev->regmap->CCER;

    // Reset output compare mode and selection bits
    tmpccmrx &= ~(TIM_CCMR1_OC1M_Msk << shiftccmrx);
    tmpccmrx &= ~(TIM_CCMR1_CC1S_Msk << shiftccmrx);

    // Set selected compare mode
    tmpccmrx |= (config->mode << shiftccmrx);

    // Set preload bit
    tmpccmrx |= ((TIM_CCMR1_OC1PE) << shiftccmrx);

    // Set fast mode
    tmpccmrx &= ~((TIM_CCMR1_OC1FE) << shiftccmrx);
    tmpccmrx |= (((config->fastMode == TRUE) ? TIM_CCMR1_OC1FE : 0u) << shiftccmrx);

    // Set polarity
    tmpccer &= ~(TIM_CCER_CC1P_Msk << channel);
    tmpccer |= (((config->polarity == GPIO_LOW) ? TIM_CCER_CC1P : 0u) << channel);

    // Save new register value
    *regCCMRxPtr = tmpccmrx;
    dev->regmap->CCER = tmpccer;

    // Compute duty-cycle pulse value
    uint32_t pulse = (((dev->regmap->ARR + 1) / 100) * config->duty);
    volatile uint32_t* regCCRn = Timer_getCCRnRegister(dev,channel);
    // Write new pulse value
    *regCCRn = pulse - 1;

    dev->state = TIMER_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Timer_startPwm (Timer_DeviceHandle dev, Timer_Channels channel)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    if (ohiassert(TIMER_IS_OC_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check the channel: exist into the device?
    if (ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,channel)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }

    // In case of callback, enable CC Interrupt
    if (dev->pwmPulseFinishedCallback != 0)
    {
        switch (channel)
        {
        case TIMER_CHANNELS_CH1:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC1IE);
            break;
        case TIMER_CHANNELS_CH2:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC2IE);
            break;
        case TIMER_CHANNELS_CH3:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC3IE);
            break;
        case TIMER_CHANNELS_CH4:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC4IE);
            break;
        default:
            ohiassert(0);
        }
    }

    // Enable channel in the selected pin
    Timer_manageCCxChannel(dev,channel,TRUE);

    // Enable device
    TIMER_DEVICE_ENABLE(dev);

    return ERRORS_NO_ERROR;
}

System_Errors Timer_stopPwm (Timer_DeviceHandle dev, Timer_Channels channel)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    if (ohiassert(TIMER_IS_OC_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check the channel: exist into the device?
    if (ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,channel)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }

    // Disable CC Interrupt
    if (dev->pwmPulseFinishedCallback != 0)
    {
        switch (channel)
        {
        case TIMER_CHANNELS_CH1:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC1IE);
            break;
        case TIMER_CHANNELS_CH2:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC2IE);
            break;
        case TIMER_CHANNELS_CH3:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC3IE);
            break;
        case TIMER_CHANNELS_CH4:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC4IE);
            break;
        default:
            ohiassert(0);
        }
    }

    // Disable channel in the selected pin
    Timer_manageCCxChannel(dev,channel,FALSE);

    // Disable device if all CC channel is not active
    TIMER_DEVICE_DISABLE(dev);

    return ERRORS_NO_ERROR;
}

System_Errors Timer_setPwmDuty (Timer_DeviceHandle dev,
                                Timer_Channels channel,
                                uint8_t duty)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    if (ohiassert(TIMER_IS_OC_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check the channel: exist into the device?
    if (ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,channel)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }
    // Check duty-cycle value
    if (ohiassert(duty <= 100) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }

    volatile uint32_t* regCCRn = Timer_getCCRnRegister(dev,channel);
    // Compute duty-cycle pulse value
    uint32_t pulse = (((dev->regmap->ARR + 1) / 100) * duty);
    // Write new pulse value
    *regCCRn = pulse - 1;

    return ERRORS_NO_ERROR;
}

System_Errors Timer_configOutputComparePin (Timer_DeviceHandle dev,
                                            Timer_OutputCompareConfig* config,
                                            Timer_Pins pin)
{
    System_Errors err = ERRORS_NO_ERROR;

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

    err  = ohiassert(TIMER_IS_OC_DEVICE(dev));
    err |= ohiassert(TIMER_VALID_OC_MODE(config->mode));
    err |= ohiassert(TIMER_VALID_OC_FAST_MODE(config->fastMode));
    err |= ohiassert(TIMER_VALID_OC_POLARITY(config->polarity));
    err |= ohiassert(config->pulse <= 0xFFFF);
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;

    // Configure alternate function on selected pin
    // And save selected channel
    Timer_Channels channel;
    bool isPinFound = FALSE;
    for (uint16_t i = 0; i < TIMER_MAX_PINS; ++i)
    {
        if (dev->pins[i] == pin)
        {
            Gpio_configAlternate(dev->pinsGpio[i],
                                 dev->pinsMux[i],
                                 0);
            channel = dev->pinsChannel[i];
            isPinFound = TRUE;
            break;
        }
    }
    if (isPinFound == FALSE)
    {
        dev->state = TIMER_DEVICESTATE_ERROR;
        return ERRORS_TIMER_NO_OC_PIN_FOUND;
    }

    // Configure Output Compare functions
    // Temporary variables
    volatile uint32_t* regCCMRxPtr = Timer_getCCMRnRegister(dev,channel);
    uint32_t tmpccmrx = *regCCMRxPtr;
    uint32_t tmpccer;

    uint32_t shiftccmrx;

    // Disable Channel
    dev->regmap->CCER &= ~(TIM_CCER_CC1E << channel);

    // Set-up registers for specific channel
    switch (channel)
    {
    case TIMER_CHANNELS_CH1:
        shiftccmrx = 0u;
        break;
    case TIMER_CHANNELS_CH2:
        shiftccmrx = 8u;
        break;
    case TIMER_CHANNELS_CH3:
        shiftccmrx = 0u;
        break;
    case TIMER_CHANNELS_CH4:
        shiftccmrx = 8u;
        break;
    case TIMER_CHANNELS_CH5:
        shiftccmrx = 0u;
        break;
    case TIMER_CHANNELS_CH6:
        shiftccmrx = 8u;
        break;
    default:
        ohiassert(0);
    }

    // Get common register values
    tmpccer = dev->regmap->CCER;

    // Reset output compare mode and selection bits
    tmpccmrx &= ~(TIM_CCMR1_OC1M_Msk << shiftccmrx);
    tmpccmrx &= ~(TIM_CCMR1_CC1S_Msk << shiftccmrx);

    // Set selected compare mode
    tmpccmrx |= (config->mode << shiftccmrx);

    // Set preload bit
    // FIXME: is useful?
    //tmpccmrx |= ((TIM_CCMR1_OC1PE) << shiftccmrx);

    // Set polarity
    tmpccer &= ~(TIM_CCER_CC1P_Msk << channel);
    tmpccer |= (((config->polarity == GPIO_LOW) ? TIM_CCER_CC1P : 0u) << channel);

// FIXME: which kind of microcontroller has this feature?
#if !defined (LIBOHIBOARD_STM32L476)
    // Enable complementary signal
    if (TIMER_IS_NCHANNEL_DEVICE(dev,channel))
    {
        ohiassert(TIMER_VALID_OC_POLARITY(config->nPolarity));
        // Set polarity
        tmpccer &= ~(TIM_CCER_CC1NP_Msk << channel);
        tmpccer |= (((config->nPolarity == GPIO_LOW) ? TIM_CCER_CC1NP : 0u) << channel);

        // Disable the Output N State
        tmpccer &= ~(TIM_CCER_CC1NE << channel);
    }
#endif

    // Save new register value
    *regCCMRxPtr = tmpccmrx;
    volatile uint32_t* regCCRn = Timer_getCCRnRegister(dev,channel);
    // Write the compare value
    *regCCRn = config->pulse;
    dev->regmap->CCER = tmpccer;

    return ERRORS_NO_ERROR;
}

System_Errors Timer_startOutputCompare (Timer_DeviceHandle dev, Timer_Channels channel)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    if (ohiassert(TIMER_IS_OC_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check the channel: exist into the device?
    if (ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,channel)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_OC_CHANNEL;
    }

    // In case of callback, enable CC Interrupt
    if (dev->outputCompareCallback != 0)
    {
        switch (channel)
        {
        case TIMER_CHANNELS_CH1:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC1IE);
            break;
        case TIMER_CHANNELS_CH2:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC2IE);
            break;
        case TIMER_CHANNELS_CH3:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC3IE);
            break;
        case TIMER_CHANNELS_CH4:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC4IE);
            break;
        default:
            ohiassert(0);
        }
    }

    // Enable channel in the selected pin
    Timer_manageCCxChannel(dev,channel,TRUE);

    // Enable device
    TIMER_DEVICE_ENABLE(dev);

    return ERRORS_NO_ERROR;
}

System_Errors Timer_stopOutputCompare (Timer_DeviceHandle dev, Timer_Channels channel)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    if (ohiassert(TIMER_IS_OC_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check the channel: exist into the device?
    if (ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,channel)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_OC_CHANNEL;
    }

    // Disable CC Interrupt
    if (dev->pwmPulseFinishedCallback != 0)
    {
        switch (channel)
        {
        case TIMER_CHANNELS_CH1:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC1IE);
            break;
        case TIMER_CHANNELS_CH2:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC2IE);
            break;
        case TIMER_CHANNELS_CH3:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC3IE);
            break;
        case TIMER_CHANNELS_CH4:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC4IE);
            break;
        default:
            ohiassert(0);
        }
    }

    // Disable channel in the selected pin
    Timer_manageCCxChannel(dev,channel,FALSE);

    // Disable device if all CC channel is not active
    TIMER_DEVICE_DISABLE(dev);

    return ERRORS_NO_ERROR;
}

System_Errors Timer_configInputCapturePin (Timer_DeviceHandle dev,
                                           Timer_InputCaptureConfig* config,
                                           Timer_Pins pin)
{
    System_Errors err = ERRORS_NO_ERROR;

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

    err  = ohiassert(TIMER_IS_IC_DEVICE(dev));
    err |= ohiassert(TIMER_VALID_IC_POLARITY(config->polarity));
    err |= ohiassert(TIMER_VALID_IC_SELECTION(config->selection));
    err |= ohiassert(TIMER_VALID_IC_PRESCALER(config->prescaler));
    err |= ohiassert((config->filter >= 0x0u) && (config->filter <= 0xFu));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;

    // Configure alternate function on selected pin
    // And save selected channel
    Timer_Channels channel;
    bool isPinFound = FALSE;
    for (uint16_t i = 0; i < TIMER_MAX_PINS; ++i)
    {
        if (dev->pins[i] == pin)
        {
            Gpio_configAlternate(dev->pinsGpio[i],
                                 dev->pinsMux[i],
                                 0);
            channel = dev->pinsChannel[i];
            isPinFound = TRUE;
            break;
        }
    }
    if (isPinFound == FALSE)
    {
        dev->state = TIMER_DEVICESTATE_ERROR;
        return ERRORS_TIMER_NO_IC_PIN_FOUND;
    }

    // Configure Input Capture functions
    // Temporary variables
    volatile uint32_t* regCCMRxPtr = Timer_getCCMRnRegister(dev,channel);
    uint32_t tmpccmrx = *regCCMRxPtr;
    uint32_t tmpccer;

    uint32_t shiftccmrx;

    // Disable Channel
    dev->regmap->CCER &= ~(TIM_CCER_CC1E << channel);
    // Get common register values
    tmpccer = dev->regmap->CCER;

    // Set-up registers for specific channel
    // Check if complementary channel is available
    bool isComplementary = FALSE;
    switch (channel)
    {
    case TIMER_CHANNELS_CH1:
        shiftccmrx = 0u;
        isComplementary = (TIMER_IS_CHANNEL2_DEVICE(dev) ? TRUE : FALSE);
        break;
    case TIMER_CHANNELS_CH2:
        shiftccmrx = 8u;
        isComplementary = (TIMER_IS_CHANNEL1_DEVICE(dev) ? TRUE : FALSE);
        break;
    case TIMER_CHANNELS_CH3:
        shiftccmrx = 0u;
        isComplementary = (TIMER_IS_CHANNEL4_DEVICE(dev) ? TRUE : FALSE);
        break;
    case TIMER_CHANNELS_CH4:
        shiftccmrx = 8u;
        isComplementary = (TIMER_IS_CHANNEL3_DEVICE(dev) ? TRUE : FALSE);
        break;
    case TIMER_CHANNELS_CH5:
        shiftccmrx = 0u;
        isComplementary = (TIMER_IS_CHANNEL6_DEVICE(dev) ? TRUE : FALSE);
        break;
    case TIMER_CHANNELS_CH6:
        shiftccmrx = 8u;
        isComplementary = (TIMER_IS_CHANNEL5_DEVICE(dev) ? TRUE : FALSE);
        break;
    default:
        ohiassert(0);
    }

    // Select the input signal
    // If the channel has the complementary, the user can choose the complementary
    // channel as input
    if (isComplementary == TRUE)
    {
        tmpccmrx &= ~(TIM_CCMR1_CC1S << shiftccmrx);
        tmpccmrx |= ((config->selection & TIM_CCMR1_CC1S) << shiftccmrx);
    }
    else
    {
        // In this case, only the current channel can be choose
        tmpccmrx |= (TIM_CCMR1_CC1S_0 << shiftccmrx);
    }

    // Select the filter
    tmpccmrx &= ~(TIM_CCMR1_IC1F << shiftccmrx);
    tmpccmrx |= (((config->filter << 4u) & TIM_CCMR1_IC1F) << shiftccmrx);

    // Select polarity
    tmpccer &= ~((TIM_CCER_CC1P | TIM_CCER_CC1NP) << channel);
    tmpccer |= (config->polarity << channel);

    // Select the prescaler
    tmpccmrx &= ~(TIM_CCMR1_IC1PSC << shiftccmrx);
    tmpccmrx |= (config->prescaler << shiftccmrx);

    // Save register values
    *regCCMRxPtr = tmpccmrx;
    dev->regmap->CCER = tmpccer;

    dev->state = TIMER_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Timer_startInputCapture (Timer_DeviceHandle dev, Timer_Channels channel)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    if (ohiassert(TIMER_IS_IC_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check the channel: exist into the device?
    if (ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,channel)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_IC_CHANNEL;
    }

    // In case of callback, enable CC Interrupt
    if (dev->outputCompareCallback != 0)
    {
        switch (channel)
        {
        case TIMER_CHANNELS_CH1:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC1IE);
            break;
        case TIMER_CHANNELS_CH2:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC2IE);
            break;
        case TIMER_CHANNELS_CH3:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC3IE);
            break;
        case TIMER_CHANNELS_CH4:
            UTILITY_SET_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC4IE);
            break;
        default:
            ohiassert(0);
        }
    }

    // Enable channel in the selected pin
    Timer_manageCCxChannel(dev,channel,TRUE);

    // Enable device
    TIMER_DEVICE_ENABLE(dev);

    return ERRORS_NO_ERROR;
}

System_Errors Timer_stopInputCapture (Timer_DeviceHandle dev, Timer_Channels channel)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // Check the TIMER instance
    if (ohiassert(TIMER_IS_IC_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check the channel: exist into the device?
    if (ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,channel)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_IC_CHANNEL;
    }

    // Disable CC Interrupt
    if (dev->pwmPulseFinishedCallback != 0)
    {
        switch (channel)
        {
        case TIMER_CHANNELS_CH1:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC1IE);
            break;
        case TIMER_CHANNELS_CH2:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC2IE);
            break;
        case TIMER_CHANNELS_CH3:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC3IE);
            break;
        case TIMER_CHANNELS_CH4:
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->DIER,TIM_DIER_CC4IE);
            break;
        default:
            ohiassert(0);
        }
    }

    // Disable channel in the selected pin
    Timer_manageCCxChannel(dev,channel,FALSE);

    // Disable device if all CC channel is not active
    TIMER_DEVICE_DISABLE(dev);

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
