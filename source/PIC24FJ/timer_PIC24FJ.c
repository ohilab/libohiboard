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
 * @file libohiboard/include/PIC24FJ/timer_PIC24FJ.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Timer implementations for PIC24FJ Series
 */

#if defined (LIBOHIBOARD_TIMER)

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_PIC24FJ)

#include "timer.h"

#include "utility.h"
#include "system.h"
#include "clock.h"
#include "interrupt.h"

/**
 * Enable selected peripheral, it is a double timer
 */
#define TIMER_DOUBLE_DEVICE_ENABLE(REGMAP)       ((REGMAP)->TxCON |= (_T2CON_TON_MASK))

/**
 * Enable selected peripheral
 */
#define TIMER_DEVICE_ENABLE(REGISTER)            ((*REGISTER) |= (_T2CON_TON_MASK))

/**
 * Disable the selected peripheral, it is a double timer.
 */
#define TIMER_DOUBLE_DEVICE_DISABLE(REGMAP)      ((REGMAP)->TxCON &= ~(_T2CON_TON_MASK))

/**
 * Disable the selected peripheral.
 */
#define TIMER_DEVICE_DISABLE(REGISTER)           ((*REGISTER) &= ~(_T2CON_TON_MASK))

#if 0
#define TIMER_VALID_MODE(MODE) (((MODE) == TIMER_MODE_FREE)           || \
                                ((MODE) == TIMER_MODE_PWM)            || \
                                ((MODE) == TIMER_MODE_INPUT_CAPTURE)  || \
                                ((MODE) == TIMER_MODE_OUTPUT_COMPARE))
#endif

// FIXME: Right now FREE counter timer is the only mode usable!!
#define TIMER_VALID_MODE(MODE) (((MODE) == TIMER_MODE_FREE))

#define TIMER_VALID_CLOCKSOURCE(CLOCKSOURCE) (((CLOCKSOURCE) == TIMER_CLOCKSOURCE_INTERNAL) || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_SOSC)     || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_TyCK)     || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_LPRC_OSC) || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_TxCK))

#define TIMER_VALID_COUNTERMODE(COUNTERMODE) ((COUNTERMODE) == TIMER_COUNTERMODE_UP)

#define TIMER_MAX_PWM_PINS                       6
    
typedef struct _Timer_Device
{
    TMR_TypeDef* regmap;                         /**< Device memory pointer */

    bool isDouble; /**< This field is TRUE when two timers are used together, FALSE otherwise. */
     /**
      * This field is TRUE when single timer is just used and can't be used
      * as double timer, FALSE otherwise.
      */
    bool justUsed;

    /**
     * This field is used to save the timer stutus: running or not.
     */
    bool isRunning;

    volatile uint16_t* pmdRegisterPtr1;    /**< Register for device enabling. */
    uint16_t pmdRegisterEnable1;       /**< Register mask for current device. */
    volatile uint16_t* pmdRegisterPtr2;    /**< Register for device enabling. */
    uint16_t pmdRegisterEnable2;       /**< Register mask for current device. */

    volatile uint16_t* tconRegisterPtr;
    volatile uint16_t* tmrRegisterPtr;
    volatile uint16_t* prRegisterPtr;

    
    Gpio_PpsOutputFunction ppsOCRegisterValue[OC_NUM];
    OC_TypeDef* ocRegisterPointer[OC_NUM];

    // Timer_Pins pins[TIMER_MAX_PINS];/**< List of the pin for the timer channel. */
    // Timer_Channels pinsChannel[TIMER_MAX_PINS];
    // Gpio_Pins pinsGpio[TIMER_MAX_PINS];
    // Gpio_Alternate pinsMux[TIMER_MAX_PINS];

    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Timer_ActiveChannels currentChannel;

    void (* freeCounterCallback)(struct _Timer_Device *dev);
    // void (* pwmPulseFinishedCallback)(struct _Timer_Device *dev);
    // void (* outputCompareCallback)(struct _Timer_Device *dev);
    // void (* inputCaptureCallback)(struct _Timer_Device *dev);

    Timer_Config config;

    Timer_DeviceState state;                   /**< Current peripheral state. */

} Timer_Device;

static Timer_Device tim2 =
{
        .regmap              = TMR23,

        .isDouble            = FALSE,
        .justUsed            = FALSE,
        .isRunning           = FALSE,

        .pmdRegisterPtr1     = &PMD->PMD1,
        .pmdRegisterEnable1  = _PMD1_T2MD_MASK,
        .pmdRegisterPtr2     = 0,
        .pmdRegisterEnable2  = 0,

        .tconRegisterPtr     = &TMR23->TxCON,
        .tmrRegisterPtr      = &TMR23->TMRx,
        .prRegisterPtr       = &TMR23->PRx,

        .isrNumber           = INTERRUPT_TIMER2,

        .state               = TIMER_DEVICESTATE_RESET,
};
Timer_DeviceHandle OB_TIM2 = &tim2;

static Timer_Device tim3 =
{
        .regmap              = TMR23,

        .isDouble            = FALSE,
        .justUsed            = FALSE,
        .isRunning           = FALSE,

        .pmdRegisterPtr1     = &PMD->PMD1,
        .pmdRegisterEnable1  = _PMD1_T3MD_MASK,
        .pmdRegisterPtr2     = 0,
        .pmdRegisterEnable2  = 0,

        .tconRegisterPtr     = &TMR23->TyCON,
        .tmrRegisterPtr      = &TMR23->TMRy,
        .prRegisterPtr       = &TMR23->PRy,

        .isrNumber           = INTERRUPT_TIMER3,

        .state               = TIMER_DEVICESTATE_RESET,
};
Timer_DeviceHandle OB_TIM3 = &tim3;

static Timer_Device tim23 =
{
        .regmap              = TMR23,

        .isDouble            = TRUE,
        .justUsed            = FALSE,
        .isRunning           = FALSE,

        .pmdRegisterPtr1     = &PMD->PMD1,
        .pmdRegisterEnable1  = _PMD1_T2MD_MASK,
        .pmdRegisterPtr2     = &PMD->PMD1,
        .pmdRegisterEnable2  = _PMD1_T3MD_MASK,

        .tconRegisterPtr     = 0,
        .tmrRegisterPtr      = 0,
        .prRegisterPtr       = 0,

        .isrNumber           = INTERRUPT_TIMER3,

        .state               = TIMER_DEVICESTATE_RESET,
};
Timer_DeviceHandle OB_TIM23 = &tim23;

static Timer_Device tim4 =
{
        .regmap              = TMR45,

        .isDouble            = FALSE,
        .justUsed            = FALSE,
        .isRunning           = FALSE,

        .pmdRegisterPtr1     = &PMD->PMD1,
        .pmdRegisterEnable1  = _PMD1_T4MD_MASK,
        .pmdRegisterPtr2     = 0,
        .pmdRegisterEnable2  = 0,

        .tconRegisterPtr     = &TMR45->TxCON,
        .tmrRegisterPtr      = &TMR45->TMRx,
        .prRegisterPtr       = &TMR45->PRx,

        .isrNumber           = INTERRUPT_TIMER4,

        .state               = TIMER_DEVICESTATE_RESET,
};
Timer_DeviceHandle OB_TIM4 = &tim4;

static Timer_Device tim5 =
{
        .regmap              = TMR45,

        .isDouble            = FALSE,
        .justUsed            = FALSE,
        .isRunning           = FALSE,

        .pmdRegisterPtr1     = &PMD->PMD1,
        .pmdRegisterEnable1  = _PMD1_T5MD_MASK,
        .pmdRegisterPtr2     = 0,
        .pmdRegisterEnable2  = 0,

        .tconRegisterPtr     = &TMR45->TyCON,
        .tmrRegisterPtr      = &TMR45->TMRy,
        .prRegisterPtr       = &TMR45->PRy,

        .isrNumber           = INTERRUPT_TIMER5,

        .state               = TIMER_DEVICESTATE_RESET,
};
Timer_DeviceHandle OB_TIM5 = &tim5;

static Timer_Device tim45 =
{
        .regmap              = TMR45,

        .isDouble            = TRUE,
        .justUsed            = FALSE,
        .isRunning           = FALSE,

        .pmdRegisterPtr1     = &PMD->PMD1,
        .pmdRegisterEnable1  = _PMD1_T4MD_MASK,
        .pmdRegisterPtr2     = &PMD->PMD1,
        .pmdRegisterEnable2  = _PMD1_T5MD_MASK,

        .isrNumber           = INTERRUPT_TIMER5,

        .state               = TIMER_DEVICESTATE_RESET,
};
Timer_DeviceHandle OB_TIM45 = &tim45;

static Timer_Device timPwm =
{
/* Unused for PWM *************************************************************/
        .regmap              = nullptr,

        .isDouble            = FALSE,
        .justUsed            = FALSE,
        .isRunning           = FALSE,

        .pmdRegisterPtr1     = 0,
        .pmdRegisterEnable1  = 0,
        .pmdRegisterPtr2     = 0,
        .pmdRegisterEnable2  = 0,

        .tconRegisterPtr     = 0,
        .tmrRegisterPtr      = 0,
        .prRegisterPtr       = 0,

        .isrNumber           = INTERRUPT_TIMER3,

        .state               = TIMER_DEVICESTATE_RESET,
/* END - Unused for PWM *******************************************************/
        
        .ppsOCRegisterValue = 
        {
                              GPIO_PPSOUTPUTFUNCTION_OC1,
                              GPIO_PPSOUTPUTFUNCTION_OC2,
                              GPIO_PPSOUTPUTFUNCTION_OC3,
                              GPIO_PPSOUTPUTFUNCTION_NULL,
                              GPIO_PPSOUTPUTFUNCTION_NULL,
                              GPIO_PPSOUTPUTFUNCTION_NULL,
        },
        
        .ocRegisterPointer  = 
        {
                              OC1,
                              OC2,
                              OC3,
                              OC4,
                              OC5,
                              OC6,
        },
};
Timer_DeviceHandle OB_TIMPWM = &timPwm;

#define TIMER_IS_DEVICE(DEVICE) (((DEVICE) == OB_TIM2)   || \
                                 ((DEVICE) == OB_TIM3)   || \
                                 ((DEVICE) == OB_TIM4)   || \
                                 ((DEVICE) == OB_TIM5)   || \
                                 ((DEVICE) == OB_TIM23)  || \
                                 ((DEVICE) == OB_TIM45))

static inline void __attribute__((always_inline)) Timer_callbackInterrupt (Timer_DeviceHandle dev)
{
    Interrupt_clearFlag(dev->isrNumber);
    dev->freeCounterCallback(dev);
}

/**
 * Useful constant to detect timer prescaler register value.
 */
static const uint16_t TIMER_CLOCK_PRESCALER[4]  =
{
    TIMER_CLOCKPRESCALER_1,
    TIMER_CLOCKPRESCALER_8,
    TIMER_CLOCKPRESCALER_64,
    TIMER_CLOCKPRESCALER_256,
};

/**
 * Useful constant to compute timer prescaler.
 */
static const uint16_t TIMER_CLOCK_PRESCALER_VALUE[4]  =
{
    1,
    8,
    64,
    256,
};

static void Timer_computeCounterValues (Timer_DeviceHandle dev,
                                        Timer_Config *config,
                                        uint16_t* prescaler,
                                        uint32_t* modulo)
{
    uint32_t moduloComputed = 0;
    uint32_t prescalerComputed = 0;

    uint32_t clock = Clock_getOutputValue(CLOCK_OUTPUT_PERIPHERAL);

     // Search the correct prescaler
     for (; prescalerComputed < UTILITY_DIMOF(TIMER_CLOCK_PRESCALER); ++prescalerComputed)
     {
         moduloComputed = (uint32_t)(clock / (TIMER_CLOCK_PRESCALER_VALUE[prescalerComputed] * config->timerFrequency));

         if (!dev->isDouble)
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

     *prescaler = TIMER_CLOCK_PRESCALER[prescalerComputed];
     *modulo = (uint32_t)(moduloComputed - 1);
}

static System_Errors Timer_configBase (Timer_DeviceHandle dev, Timer_Config *config)
{
    uint32_t modulo = 0;
    uint16_t prescaler = 0;

    if (config->timerFrequency > 0)
    {
        Timer_computeCounterValues(dev,config,&prescaler,&modulo);
    }
    // Save Autoreload and Prescaler value chose by user
    else
    {
        // Set the Autoreload value
        modulo = (uint32_t)(config->modulo - 1);
        // Set the Prescaler value
        prescaler = (uint16_t) config->prescaler;
    }

    if (dev->isDouble)
    {
        UTILITY_SET_REGISTER_BIT(dev->regmap->TxCON,_T2CON_T32_MASK);

        // Store prescaler and modulo
        dev->regmap->TxCON &= ~(_T2CON_TCKPS_MASK);
        dev->regmap->TxCON |= prescaler;

        // LSB period part
        dev->regmap->PRx = (uint16_t) (modulo & 0x0000FFFF);
        // MSB period part
        dev->regmap->PRy = (uint16_t) ((modulo & 0xFFFF0000) >> 16);

        // Clear counter
        dev->regmap->TMRx = 0;
        dev->regmap->TMRy = 0;
    }
    else
    {
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->TxCON,_T2CON_T32_MASK);

        // Store prescaler
        *dev->tconRegisterPtr &= ~(_T2CON_TCKPS_MASK);
        *dev->tconRegisterPtr |= prescaler;

        // write period
        *dev->prRegisterPtr = (uint16_t)(modulo & 0x0000FFFF);

        // Clear counter
        *dev->tmrRegisterPtr = 0;
    }

    // Check callback and enable interrupts
    if (config->freeCounterCallback != 0)
    {
        // Save callback
        dev->freeCounterCallback = config->freeCounterCallback;
    }

    return ERRORS_NO_ERROR;
}

void Timer_configClockSource (Timer_DeviceHandle dev, Timer_Config* config)
{
    // Set Clock Source
    if(config->clockSource == TIMER_CLOCKSOURCE_INTERNAL)
    {
        // Set clock source for FOSC/2
        if (dev->isDouble)
        {
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->TxCON, _T2CON_TCS_MASK);
            UTILITY_CLEAR_REGISTER_BIT(dev->regmap->TxCON, _T2CON_TECS_MASK);
        }
        else
        {
            UTILITY_CLEAR_REGISTER_BIT((*dev->tconRegisterPtr), _T2CON_TCS_MASK);
            UTILITY_CLEAR_REGISTER_BIT((*dev->tconRegisterPtr), _T2CON_TECS_MASK);
        }
    }
    else
    {
        // Set clock source for other input
        if (dev->isDouble)
        {
            UTILITY_MODIFY_REGISTER(dev->regmap->TxCON,(_T1CON_TECS_MASK | _T2CON_TCS_MASK),(config->clockSource));
        }
        else
        {
            UTILITY_MODIFY_REGISTER((*dev->tconRegisterPtr),(_T1CON_TECS_MASK | _T2CON_TCS_MASK),(config->clockSource));
        }
    }
}

System_Errors Timer_init (Timer_DeviceHandle dev, Timer_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }
    // In case of PWM module, nothing to do into this function
    if (dev == OB_TIMPWM)
    {
        return ERRORS_NO_ERROR;
    }
    // Check the TIMER instance
    if (ohiassert(TIMER_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    err  = ohiassert(TIMER_VALID_MODE(config->mode));
    err |= ohiassert(TIMER_VALID_CLOCKSOURCE(config->clockSource));
    err |= ohiassert(TIMER_VALID_COUNTERMODE(config->counterMode));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }
    dev->config = *config;

    // Check if the double device is just used and vice-versa!
    if (((dev == OB_TIM23) && ((OB_TIM2->justUsed) || (OB_TIM3->justUsed))) ||
       (((dev == OB_TIM2) || (dev == OB_TIM3)) && (OB_TIM23->justUsed)))
    {
        return ERRORS_TIMER_DEVICE_JUST_USED;
    }

    if (((dev == OB_TIM45) && ((OB_TIM4->justUsed) || (OB_TIM5->justUsed))) ||
       (((dev == OB_TIM4) || (dev == OB_TIM5)) && (OB_TIM45->justUsed)))
    {
        return ERRORS_TIMER_DEVICE_JUST_USED;
    }

    // Enable peripheral clock if needed
    if (dev->state == TIMER_DEVICESTATE_RESET)
    {
        // Enable peripheral
        if (dev->isDouble)
        {
            UTILITY_CLEAR_REGISTER_BIT(*dev->pmdRegisterPtr1, dev->pmdRegisterEnable1);
            UTILITY_CLEAR_REGISTER_BIT(*dev->pmdRegisterPtr2, dev->pmdRegisterEnable2);
        }
        else
        {
            UTILITY_CLEAR_REGISTER_BIT(*dev->pmdRegisterPtr1, dev->pmdRegisterEnable1);
        }
    }
    // Now the peripheral is busy
    dev->state = TIMER_DEVICESTATE_BUSY;

    // Configure clock source
    Timer_configClockSource(dev,config);

    // Configure the peripheral
    switch (dev->config.mode)
    {
    case TIMER_MODE_FREE:
        // Check user choices
        ohiassert((config->timerFrequency > 0) || (config->modulo > 0));

        Timer_configBase(dev,config);
        break;

    default:
        ohiassert(0);
        break;
    }

    dev->isRunning = FALSE;

    // Now the peripheral is busy
    dev->state = TIMER_DEVICESTATE_READY;
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
    if (ohiassert(TIMER_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;
    dev->isRunning = FALSE;

    // Disable the peripheral
    if (dev->isDouble)
    {
        UTILITY_SET_REGISTER_BIT(*dev->pmdRegisterPtr1, dev->pmdRegisterEnable1);
        UTILITY_SET_REGISTER_BIT(*dev->pmdRegisterPtr2, dev->pmdRegisterEnable2);
    }
    else
    {
        UTILITY_SET_REGISTER_BIT(*dev->pmdRegisterPtr1, dev->pmdRegisterEnable1);
    }

    Interrupt_disable(dev->isrNumber);
    Interrupt_clearFlag(dev->isrNumber);

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
    if (ohiassert(TIMER_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check whether the device is just initialized
    if (dev->state != TIMER_DEVICESTATE_READY)
    {
        return ERRORS_TIMER_DEVICE_NOT_READY;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;

    // In case of callback... enable interrupt
    if (dev->freeCounterCallback != 0)
    {
        Interrupt_clearFlag(dev->isrNumber);
        Interrupt_setPriority(dev->isrNumber,dev->config.isrPriority);
        Interrupt_enable(dev->isrNumber);
    }

    // Enable device
    if (dev->isDouble)
    {
        TIMER_DOUBLE_DEVICE_ENABLE(dev->regmap);
    }
    else
    {
        TIMER_DEVICE_ENABLE(dev->tconRegisterPtr);
    }

    dev->isRunning = TRUE;
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
    if (ohiassert(TIMER_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check whether the device is just initialized
    if (dev->state != TIMER_DEVICESTATE_READY)
    {
        return ERRORS_TIMER_DEVICE_NOT_READY;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;

    // In case of callback... disable interrupt
    if (dev->freeCounterCallback != 0)
    {
        Interrupt_clearFlag(dev->isrNumber);
        Interrupt_disable(dev->isrNumber);
    }

    // Disable device
    if (dev->isDouble)
    {
        TIMER_DOUBLE_DEVICE_DISABLE(dev->regmap);
    }
    else
    {
        TIMER_DEVICE_DISABLE(dev->tconRegisterPtr);
    }

    dev->isRunning = FALSE;
    dev->state = TIMER_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

bool Timer_isRunning (Timer_DeviceHandle dev)
{
    // Check the TIMER instance type
    ohiassert(TIMER_IS_DEVICE(dev));

    return dev->isRunning;
}

uint32_t Timer_getCurrentCounter (Timer_DeviceHandle dev)
{
    // Check the TIMER instance type
    ohiassert(TIMER_IS_DEVICE(dev));

    if (dev->isDouble)
    {
        return (uint32_t)((((uint32_t)dev->regmap->TMRy << 16) & 0xFFFF0000) && (((uint32_t)dev->regmap->TMRx) & 0x0000FFFF));
    }
    else
    {
        return (uint32_t)(*dev->tmrRegisterPtr);
    }
}

void Timer_resetCurrentCounter (Timer_DeviceHandle dev)
{
    // Check the TIMER instance type
    ohiassert(TIMER_IS_DEVICE(dev));

    if (dev->isDouble)
    {
        dev->regmap->TMRy = 0;
        dev->regmap->TMRx = 0;
    }
    else
    {
        *dev->tmrRegisterPtr = 0;
    }
}

#define TIMER_IS_CHANNEL(CHANNEL) (((CHANNEL) == TIMER_CHANNELS_CH1) || \
                                   ((CHANNEL) == TIMER_CHANNELS_CH2) || \
                                   ((CHANNEL) == TIMER_CHANNELS_CH3) || \
                                   ((CHANNEL) == TIMER_CHANNELS_CH4) || \
                                   ((CHANNEL) == TIMER_CHANNELS_CH5) || \
                                   ((CHANNEL) == TIMER_CHANNELS_CH6))

System_Errors Timer_configPwmPin (Timer_DeviceHandle dev,
                                  Timer_OutputCompareConfig* config,
                                  Timer_Pins pin)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }

    if (dev != OB_TIMPWM)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    if (!TIMER_IS_CHANNEL(config->channel))
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }
    
    uint8_t ch = config->channel - 1;
    
    switch (config->channel)
    {
    case TIMER_CHANNELS_CH1:
    case TIMER_CHANNELS_CH2:
    case TIMER_CHANNELS_CH3:
        {
            uint8_t regIndex = pin / 2;
            uint8_t regPosIndex = ((pin % 2 ) * 8);

            UTILITY_MODIFY_REGISTER(PPS->RPOR[regIndex], (0x3F << regPosIndex),(dev->ppsOCRegisterValue[ch] << regPosIndex));
        }
        break;
    default:
        // Nothing to do!
        // Channel 4,5 and 6 are bound to the selected pin!
        break;
    }
    
    uint16_t frequency = (uint16_t)((uint32_t)Clock_getOutputValue(CLOCK_OUTPUT_PERIPHERAL)/config->frequency);
    uint16_t period = (uint16_t)(((uint32_t)frequency * (uint32_t)config->duty) / 100);
    
    UTILITY_WRITE_REGISTER(dev->ocRegisterPointer[ch]->OCRS,frequency);
    UTILITY_WRITE_REGISTER(dev->ocRegisterPointer[ch]->OCR,period);
    
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

    if (dev != OB_TIMPWM)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    if (!TIMER_IS_CHANNEL(channel))
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }
    
    if (duty > 100)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }

    uint8_t ch = channel - 1;
    uint16_t period = (uint16_t)(((uint32_t)dev->ocRegisterPointer[ch]->OCRS * (uint32_t)duty) / 100);
    UTILITY_WRITE_REGISTER(dev->ocRegisterPointer[ch]->OCR,period);

    return ERRORS_NO_ERROR;
}

System_Errors Timer_startPwm (Timer_DeviceHandle dev, Timer_Channels channel)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }

    if (dev != OB_TIMPWM)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    if (!TIMER_IS_CHANNEL(channel))
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }

    uint8_t ch = channel - 1;
    
    UTILITY_WRITE_REGISTER(dev->ocRegisterPointer[ch]->OCCON1,0x1C06);
    UTILITY_WRITE_REGISTER(dev->ocRegisterPointer[ch]->OCCON2,0x001F);
    
    return ERRORS_NO_ERROR;
}

System_Errors Timer_stopPwm (Timer_DeviceHandle dev, Timer_Channels channel)
{
    // Check the TIMER device
    if (dev == NULL)
    {
        return ERRORS_TIMER_NO_DEVICE;
    }

    if (dev != OB_TIMPWM)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    if (!TIMER_IS_CHANNEL(channel))
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }

    uint8_t ch = channel - 1;
    
    UTILITY_MODIFY_REGISTER(dev->ocRegisterPointer[ch]->OCCON1,0x1C06,0x0000);
    
    return ERRORS_NO_ERROR;
}

void __attribute__ (( interrupt, no_auto_psv )) _T2Interrupt ( void )
{
    Timer_callbackInterrupt(OB_TIM2);
}

void __attribute__ (( interrupt, no_auto_psv )) _T3Interrupt ( void )
{
    if (TMR23->TxCON & _T2CON_T32_MASK)
    {
        Timer_callbackInterrupt(OB_TIM23);
    }
    else
    {
        Timer_callbackInterrupt(OB_TIM3);
    }
}

void __attribute__ (( interrupt, no_auto_psv )) _T4Interrupt ( void )
{
    Timer_callbackInterrupt(OB_TIM4);
}

void __attribute__ (( interrupt, no_auto_psv )) _T5Interrupt ( void )
{
    if (TMR45->TxCON & _T2CON_T45_MASK)
    {
        Timer_callbackInterrupt(OB_TIM45);
    }
    else
    {
        Timer_callbackInterrupt(OB_TIM5);
    }
}

#endif // LIBOHIBOARD_PIC242FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_TIMER
