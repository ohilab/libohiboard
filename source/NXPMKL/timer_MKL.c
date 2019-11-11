/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2014-2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/NXPMKL/timer_MKL.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Timer implementations for NXP MKL Series.
 */

#if defined (LIBOHIBOARD_TIMER)

#ifdef __cplusplus
extern "C" {
#endif

#include "timer.h"

#include "platforms.h"
#include "utility.h"
#include "system.h"
#include "interrupt.h"
#include "clock.h"

#if defined (LIBOHIBOARD_MKL)

const Timer_ClockPrescaler TIMER_PRESCALER_REGISTER_TABLE[TIMER_CLOCKPRESCALER_NUMBER] =
{
    TIMER_CLOCKPRESCALER_1,
    TIMER_CLOCKPRESCALER_2,
    TIMER_CLOCKPRESCALER_4,
    TIMER_CLOCKPRESCALER_8,
    TIMER_CLOCKPRESCALER_16,
    TIMER_CLOCKPRESCALER_32,
    TIMER_CLOCKPRESCALER_64,
    TIMER_CLOCKPRESCALER_128,
};

const uint32_t TIMER_PRESCALER_REGISTER_VALUE[TIMER_CLOCKPRESCALER_NUMBER] =
{
    1,
    2,
    4,
    8,
    16,
    32,
    64,
    128,
};

typedef struct _Timer_Device
{
    TPM_Type* regmap;                          /**< Device memory pointer */

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Timer_Pins pins[TIMER_PINS_NUMBER];  /**< List of the pin for the FTM channel. */
    Gpio_Pins pinsGpio[TIMER_PINS_NUMBER];
    volatile uint32_t* pinsPtr[TIMER_PINS_NUMBER];
    Timer_Channels channel[TIMER_PINS_NUMBER];
    uint8_t pinsMux[TIMER_PINS_NUMBER];     /**< Mux of the pin of the FTM channel. */

    uint32_t channelConfiguration[TIMER_CHANNELS_NUMBER];

    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    void (* freeCounterCallback)(struct _Timer_Device *dev);
    void (* pwmPulseFinishedCallback)(struct _Timer_Device *dev);
    void (* outputCompareCallback)(struct _Timer_Device *dev);
    void (* inputCaptureCallback)(struct _Timer_Device *dev);

    Timer_Mode mode;                                /**< Modes of operations. */
    uint32_t inputClock;                       /**< Current clock input value */

    Timer_Config config;

    Timer_DeviceState state;                   /**< Current peripheral state. */
} Timer_Device;

#define TIMER_IS_DEVICE(DEVICE) (((DEVICE) == OB_TIM0)   || \
                                 ((DEVICE) == OB_TIM1)   || \
                                 ((DEVICE) == OB_TIM2))

#define TIMER_VALID_MODE(MODE) (((MODE) == TIMER_MODE_FREE)           || \
                                ((MODE) == TIMER_MODE_PWM)            || \
                                ((MODE) == TIMER_MODE_INPUT_CAPTURE)  || \
                                ((MODE) == TIMER_MODE_OUTPUT_COMPARE))

#define TIMER_VALID_CLOCKSOURCE(CLOCKSOURCE) (((CLOCKSOURCE) == TIMER_CLOCKSOURCE_INTERNAL)        || \
                                              ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_OSCILLATOR)      || \
											  ((CLOCKSOURCE) == TIMER_CLOCKSOURCE_MCG))

#define TIMER_VALID_COUNTERMODE(COUNTERMODE) (((COUNTERMODE) == TIMER_COUNTERMODE_UP)             || \
                                              ((COUNTERMODE) == TIMER_COUNTERMODE_DOWN)           || \
                                              ((COUNTERMODE) == TIMER_COUNTERMODE_EDGE_ALIGNED)   || \
                                              ((COUNTERMODE) == TIMER_COUNTERMODE_CENTER_ALIGNED))

#define TIMER_IS_CHANNEL_DEVICE(DEVICE, CHANNEL)                                          \
                                                ((((DEVICE) == OB_TIM0) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH0) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH1) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH2) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH3) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH4) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH5)))  || \
                                                 (((DEVICE) == OB_TIM1) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH0) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH1)))  || \
                                                 (((DEVICE) == OB_TIM2) &&                \
                                                 (((CHANNEL) == TIMER_CHANNELS_CH0) ||    \
                                                  ((CHANNEL) == TIMER_CHANNELS_CH1))))

#define TIMER_DISABLE_CHANNEL_MASK               (TPM_CnSC_MSB_MASK  | \
                                                  TPM_CnSC_MSA_MASK  | \
                                                  TPM_CnSC_ELSB_MASK | \
                                                  TPM_CnSC_ELSA_MASK)
#define TIMER_CHANNEL_MODE_MASK                  (TPM_CnSC_MSB_MASK  | \
                                                  TPM_CnSC_MSA_MASK)
#define TIMER_CHANNEL_INPUT_CAPTURE_MASK         (0ul)
#define TIMER_CHANNEL_OUTPUT_COMPARE_MASK        (TPM_CnSC_MSA_MASK)
#define TIMER_CHANNEL_PWM_MASK                   (TPM_CnSC_MSB_MASK)

static Timer_Device tim0 =
{
        .regmap           = TPM0,

        .simScgcPtr       = &SIM->SCGC6,
        .simScgcBitEnable = SIM_SCGC6_TPM0_MASK,

        .pins             =
        {
                            TIMER_PINS_PTA0,
                            TIMER_PINS_PTA3,
                            TIMER_PINS_PTA4,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_PINS_PTA5,
#endif
                            TIMER_PINS_PTC1,
                            TIMER_PINS_PTC2,
                            TIMER_PINS_PTC3,
                            TIMER_PINS_PTC4,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_PINS_PTC8,
                            TIMER_PINS_PTC9,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_PINS_PTD0,
                            TIMER_PINS_PTD1,
                            TIMER_PINS_PTD2,
                            TIMER_PINS_PTD3,
#endif
                            TIMER_PINS_PTD4,
                            TIMER_PINS_PTD5,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_PINS_PTE24,
                            TIMER_PINS_PTE25,
                            TIMER_PINS_PTE29,
#endif
                            TIMER_PINS_PTE30,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_PINS_PTE31,
#endif
        },
        .pinsGpio          =
        {
                            GPIO_PINS_PTA0,
                            GPIO_PINS_PTA3,
                            GPIO_PINS_PTA4,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            GPIO_PINS_PTA5,
#endif
                            GPIO_PINS_PTC1,
                            GPIO_PINS_PTC2,
                            GPIO_PINS_PTC3,
                            GPIO_PINS_PTC4,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            GPIO_PINS_PTC8,
                            GPIO_PINS_PTC9,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            GPIO_PINS_PTD0,
                            GPIO_PINS_PTD1,
                            GPIO_PINS_PTD2,
                            GPIO_PINS_PTD3,
#endif
                            GPIO_PINS_PTD4,
                            GPIO_PINS_PTD5,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            GPIO_PINS_PTE24,
                            GPIO_PINS_PTE25,
                            GPIO_PINS_PTE29,
#endif
                            GPIO_PINS_PTE30,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            GPIO_PINS_PTE31,
#endif
        },
        .pinsPtr          =
        {
                            &PORTA->PCR[0],
                            &PORTA->PCR[3],
                            &PORTA->PCR[4],
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            &PORTA->PCR[5],
#endif
                            &PORTC->PCR[1],
                            &PORTC->PCR[2],
                            &PORTC->PCR[3],
                            &PORTC->PCR[4],
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            &PORTC->PCR[8],
                            &PORTC->PCR[9],
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            &PORTD->PCR[0],
                            &PORTD->PCR[1],
                            &PORTD->PCR[2],
                            &PORTD->PCR[3],
#endif
                            &PORTD->PCR[4],
                            &PORTD->PCR[5],
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            &PORTE->PCR[24],
                            &PORTE->PCR[25],
                            &PORTE->PCR[29],
#endif
                            &PORTE->PCR[30],
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            &PORTE->PCR[31],
#endif
        },
        .pinsMux          =
        {
                            3,
                            3,
                            3,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            3,
#endif
                            4,
                            4,
                            4,
                            4,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            3,
                            3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            4,
                            4,
                            4,
                            4,
#endif
                            4,
                            4,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            3,
                            3,
                            3,
#endif
                            3,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            3,
#endif
        },
        .channel          =
        {
                            TIMER_CHANNELS_CH5,
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_CHANNELS_CH2,
#endif
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
                            TIMER_CHANNELS_CH2,
                            TIMER_CHANNELS_CH3,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_CHANNELS_CH4,
                            TIMER_CHANNELS_CH5,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
                            TIMER_CHANNELS_CH2,
                            TIMER_CHANNELS_CH3,
#endif
                            TIMER_CHANNELS_CH4,
                            TIMER_CHANNELS_CH5,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
                            TIMER_CHANNELS_CH2,
#endif
                            TIMER_CHANNELS_CH3,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_CHANNELS_CH4,
#endif
        },

        .config           = {0},
        .inputClock       = 0,

        .isrNumber        = INTERRUPT_TPM0,

        .state            = TIMER_DEVICESTATE_RESET,

};
Timer_DeviceHandle OB_TIM0 = &tim0;

static Timer_Device tim1 =
{
        .regmap           = TPM1,

        .simScgcPtr       = &SIM->SCGC6,
        .simScgcBitEnable = SIM_SCGC6_TPM1_MASK,

        .pins             =
        {
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_PINS_PTA12,
                            TIMER_PINS_PTA13,
#endif
                            TIMER_PINS_PTB0,
                            TIMER_PINS_PTB1,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_PINS_PTE20,
                            TIMER_PINS_PTE21,
#endif
        },
        .pinsGpio         =
        {
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            GPIO_PINS_PTA12,
                            GPIO_PINS_PTA13,
#endif
                            GPIO_PINS_PTB0,
                            GPIO_PINS_PTB1,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            GPIO_PINS_PTE20,
                            GPIO_PINS_PTE21,
#endif
        },
        .pinsPtr          =
        {
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            &PORTA->PCR[12],
                            &PORTA->PCR[13],
#endif
                            &PORTB->PCR[0],
                            &PORTB->PCR[1],
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            &PORTE->PCR[20],
                            &PORTE->PCR[21],
#endif
        },
        .pinsMux          =
        {
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            3,
                            3,
#endif
                            3,
                            3,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            3,
                            3,
#endif
        },
        .channel          =
        {
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
#endif
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
#endif
        },

        .config           = {0},
        .inputClock       = 0,

        .isrNumber        = INTERRUPT_TPM1,

        .state            = TIMER_DEVICESTATE_RESET,
};
Timer_DeviceHandle OB_TIM1 = &tim1;

static Timer_Device tim2 =
{
        .regmap           = TPM2,

        .simScgcPtr       = &SIM->SCGC6,
        .simScgcBitEnable = SIM_SCGC6_TPM2_MASK,

        .pins             =
        {
                            TIMER_PINS_PTA1,
                            TIMER_PINS_PTA2,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_PINS_PTB2,
                            TIMER_PINS_PTB3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_PINS_PTB18,
                            TIMER_PINS_PTB19,
                            TIMER_PINS_PTE22,
                            TIMER_PINS_PTE23,
#endif
        },
        .pinsGpio         =
        {
                            GPIO_PINS_PTA1,
                            GPIO_PINS_PTA2,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            GPIO_PINS_PTB2,
                            GPIO_PINS_PTB3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            GPIO_PINS_PTB18,
                            GPIO_PINS_PTB19,
                            GPIO_PINS_PTE22,
                            GPIO_PINS_PTE23,
#endif
        },
        .pinsPtr          =
        {
                            &PORTA->PCR[1],
                            &PORTA->PCR[2],
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            &PORTB->PCR[2],
                            &PORTB->PCR[3],
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            &PORTB->PCR[18],
                            &PORTB->PCR[19],
                            &PORTE->PCR[22],
                            &PORTE->PCR[23],
#endif
        },
        .pinsMux          =
        {
                            3,
                            3,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            3,
                            3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            3,
                            3,
                            3,
                            3,
#endif
        },
        .channel          =
        {
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
                            TIMER_CHANNELS_CH0,
                            TIMER_CHANNELS_CH1,
#endif
        },

        .config           = {0},
        .inputClock       = 0,

        .isrNumber        = INTERRUPT_TPM2,

        .state            = TIMER_DEVICESTATE_RESET,
};
Timer_DeviceHandle OB_TIM2 = &tim2;


static inline void __attribute__((always_inline)) Timer_callbackInterrupt (Timer_DeviceHandle dev)
{
    // Update event / free couter
    if ((dev->regmap->SC & TPM_SC_TOF_MASK) == TPM_SC_TOF_MASK)
    {
        if ((dev->regmap->SC & TPM_SC_TOIE_MASK) == TPM_SC_TOIE_MASK)
        {
            // Clear flag... and call callback!
            dev->regmap->SC |= TPM_SC_TOF_MASK;
            dev->config.freeCounterCallback(dev);
        }
    }

    // Capture/Compare Channel x
    for (uint8_t i = 0; i < TIMER_CHANNELS_NUMBER; ++i)
    {
        if ((dev->regmap->CONTROLS[i].CnSC & TPM_CnSC_CHF_MASK) == TPM_CnSC_CHF_MASK)
        {
            if ((dev->regmap->CONTROLS[i].CnSC & TPM_CnSC_CHIE_MASK) == TPM_CnSC_CHIE_MASK)
            {
                // Clear flag...
                dev->regmap->CONTROLS[i].CnSC |= TPM_CnSC_CHF_MASK;

                if ((dev->regmap->CONTROLS[i].CnSC & TIMER_CHANNEL_MODE_MASK) == TIMER_CHANNEL_INPUT_CAPTURE_MASK)
                {
                    dev->inputCaptureCallback(dev);
                }
                else if ((dev->regmap->CONTROLS[i].CnSC & TIMER_CHANNEL_MODE_MASK) == TIMER_CHANNEL_OUTPUT_COMPARE_MASK)
                {
                    dev->outputCompareCallback(dev);
                }
                else if ((dev->regmap->CONTROLS[i].CnSC & TIMER_CHANNEL_MODE_MASK) == TIMER_CHANNEL_PWM_MASK)
                {
                    dev->pwmPulseFinishedCallback(dev);
                }
            }
        }
    }
}

System_Errors Timer_configClockSource (Timer_DeviceHandle dev, Timer_Config *config)
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

    switch (config->clockSource)
    {
    default:
        ohiassert(0);
        return ERRORS_TIMER_WRONG_PARAM;
        break;

    case TIMER_CLOCKSOURCE_INTERNAL:
        dev->inputClock = Clock_getOutputValue(CLOCK_OUTPUT_INTERNAL);
        SIM->SOPT2 |= SIM_SOPT2_TPMSRC(3);
        break;
    case TIMER_CLOCKSOURCE_MCG:
        dev->inputClock = Clock_getOutputValue(CLOCK_OUTPUT_MCG);
        SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
        break;
    case TIMER_CLOCKSOURCE_OSCILLATOR:
        dev->inputClock = Clock_getOscillatorValue();
        SIM->SOPT2 |= SIM_SOPT2_TPMSRC(2);
        break;
    }

    return ERRORS_NO_ERROR;
}

static Timer_ClockPrescaler Timer_computeFrequencyPrescale (Timer_DeviceHandle dev,
                                                            uint32_t frequency)
{
    uint32_t clock = dev->inputClock;
    uint8_t prescaler = 0;

    if (dev->mode == TIMER_MODE_INPUT_CAPTURE)
    {
        prescaler = (clock / frequency);
    }
    else
    {
        prescaler = (clock / frequency) / 65536;
    }

    // Scan all prescaler table, but start from the second from the high to low
    for (int8_t i = TIMER_CLOCKPRESCALER_NUMBER - 2; i >= 0; i--)
    {
        if (prescaler > TIMER_PRESCALER_REGISTER_VALUE[i])
        {
            return TIMER_PRESCALER_REGISTER_TABLE[i+1];
        }
    }

    // Just in case...
    return TIMER_CLOCKPRESCALER_1;
}

static uint16_t Timer_computeModulo (Timer_DeviceHandle dev,
                                     uint32_t frequency,
                                     Timer_ClockPrescaler prescaler)
{
    uint32_t clock  = dev->inputClock;
    uint32_t modulo = 0;

    modulo = (uint16_t) ((uint32_t) clock / frequency * TIMER_PRESCALER_REGISTER_VALUE[prescaler]);

    ohiassert(modulo < 0x0000FFFF);
    return (uint16_t) modulo;
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
    err = ohiassert(TIMER_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
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

    // Enable peripheral clock if needed
    if (dev->state == TIMER_DEVICESTATE_RESET)
    {
        // Enable peripheral clock
        UTILITY_SET_REGISTER_BIT(*dev->simScgcPtr,dev->simScgcBitEnable);
    }

    // Configure clock source
    Timer_configClockSource(dev,config);

    // Now the peripheral is busy
    dev->state = TIMER_DEVICESTATE_BUSY;

    Timer_ClockPrescaler prescaler;
    uint16_t modulo;
    // Configure the peripheral
    switch (dev->mode)
    {
    case TIMER_MODE_FREE:
        // Check user choices
        ohiassert((config->timerFrequency > 0) || ((config->prescaler > 0) && (config->modulo > 0)));

        if (config->timerFrequency > 0)
        {
            // Compute prescale factor
            prescaler = Timer_computeFrequencyPrescale(dev,config->timerFrequency);

            // Compute timer modulo
            modulo = Timer_computeModulo(dev,config->timerFrequency,prescaler);
        }
        else
        {
            prescaler = config->prescaler;
            modulo = config->modulo;
        }

        dev->regmap->CNT = 0;
        dev->regmap->MOD = modulo - 1;
        dev->regmap->SC  = TPM_SC_PS(prescaler) | 0;
        break;

    case TIMER_MODE_PWM:
        // Check user choices
        ohiassert((config->timerFrequency > 0) || ((config->prescaler > 0) && (config->modulo > 0)));

        if (config->timerFrequency > 0)
        {
            // Compute prescale factor
            prescaler = Timer_computeFrequencyPrescale(dev,config->timerFrequency);
            // Compute timer modulo
            modulo = Timer_computeModulo(dev,config->timerFrequency,prescaler);
        }
        else
        {
            prescaler = config->prescaler;
            modulo = config->modulo;
        }

        if (config->counterMode == TIMER_COUNTERMODE_CENTER_ALIGNED)
        {
            dev->regmap->SC |= TPM_SC_CPWMS_MASK;
            modulo = (modulo / 2) - 1;
        }
        else
        {
            dev->regmap->SC &= ~TPM_SC_CPWMS_MASK;
            modulo -= 1;
        }

        dev->regmap->CNT = 0;
        dev->regmap->MOD = modulo;
        // The timer is enabled, but all channel are disabled
        dev->regmap->SC  = TPM_SC_CMOD(1) | TPM_SC_PS(prescaler) | 0;
        break;

    case TIMER_MODE_OUTPUT_COMPARE:
        // FIXME: is not implemented now!

        break;

    case TIMER_MODE_INPUT_CAPTURE:
        // FIXME: is not implemented now!

        break;

    default:
        ohiassert(0);
        break;
    }

    // Check callback and enable interrupts
    if ((config->freeCounterCallback != 0)      ||
        (config->pwmPulseFinishedCallback != 0) ||
        (config->outputCompareCallback != 0)    ||
        (config->inputCaptureCallback != 0))
    {
        // Enable interrupt
        Interrupt_enable(dev->isrNumber);
    }

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
    if (ohiassert((TIMER_IS_DEVICE(dev))) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;

    // In case of callback... enable interrupt
    if (dev->config.freeCounterCallback != 0)
    {
        dev->regmap->SC |= TPM_SC_TOIE_MASK;
    }

    // Enable device
    dev->regmap->SC |= TPM_SC_CMOD(1);

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
    if (ohiassert((TIMER_IS_DEVICE(dev))) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;

    // In case of callback... enable interrupt
    if (dev->config.freeCounterCallback != 0)
    {
        dev->regmap->SC &= ~(TPM_SC_TOIE_MASK);
    }

    // Disable device
    dev->regmap->SC &= ~(TPM_SC_CMOD_MASK);

    dev->state = TIMER_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}


static inline  volatile uint32_t* __attribute__((always_inline))  Timer_getCnVRegister (Timer_DeviceHandle dev,
                                                                                        Timer_Channels channel)
{
    switch (channel)
    {
    case TIMER_CHANNELS_CH0:
        return &dev->regmap->CONTROLS[0].CnV;
    case TIMER_CHANNELS_CH1:
        return &dev->regmap->CONTROLS[1].CnV;
    case TIMER_CHANNELS_CH2:
        return &dev->regmap->CONTROLS[2].CnV;
    case TIMER_CHANNELS_CH3:
        return &dev->regmap->CONTROLS[3].CnV;
    case TIMER_CHANNELS_CH4:
        return &dev->regmap->CONTROLS[4].CnV;
    case TIMER_CHANNELS_CH5:
        return &dev->regmap->CONTROLS[5].CnV;
    default:
        ohiassert(0);
        return 0;
    }
}

static inline volatile uint32_t* __attribute__((always_inline)) Timer_getCnSCRegister (Timer_DeviceHandle dev,
                                                                                       Timer_Channels channel)
{
    switch (channel)
    {
    case TIMER_CHANNELS_CH0:
        return &dev->regmap->CONTROLS[0].CnSC;
    case TIMER_CHANNELS_CH1:
        return &dev->regmap->CONTROLS[1].CnSC;
    case TIMER_CHANNELS_CH2:
        return &dev->regmap->CONTROLS[2].CnSC;
    case TIMER_CHANNELS_CH3:
        return &dev->regmap->CONTROLS[3].CnSC;
    case TIMER_CHANNELS_CH4:
        return &dev->regmap->CONTROLS[4].CnSC;
    case TIMER_CHANNELS_CH5:
        return &dev->regmap->CONTROLS[5].CnSC;
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
    if (ohiassert((TIMER_IS_DEVICE(dev))) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check the channel: exist into the device?
    if (ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,config->channel)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }

    //err |= ohiassert(TIMER_VALID_PWM_MODE(config->mode));
    err |= ohiassert(config->duty <= 100);
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }

    dev->state = TIMER_DEVICESTATE_BUSY;

    // Configure alternate function on selected pin
    // And save selected channel
    bool isPinFound = FALSE;
    for (uint16_t i = 0; i < TIMER_PINS_NUMBER; ++i)
    {
        if (dev->pins[i] == pin)
        {
            Gpio_configAlternate(dev->pinsGpio[i],
                                 dev->pinsMux[i],
                                 0);
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
    volatile uint32_t* regCSCPtr = Timer_getCnSCRegister(dev,config->channel);
    // Disable channel
    if (regCSCPtr)
    {
        *regCSCPtr &= ~(TIMER_DISABLE_CHANNEL_MASK);

        if (config->polarity == GPIO_HIGH)
        {
            dev->channelConfiguration[config->channel] = TPM_CnSC_MSB_MASK  | TPM_CnSC_ELSB_MASK;
        }
        else
        {
            dev->channelConfiguration[config->channel] = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK;
        }
    }
    else
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }

    Timer_setPwmDuty(dev, config->channel, config->duty);

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
    if (ohiassert(TIMER_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check the channel: exist into the device?
    if (ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,channel)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }

    volatile uint32_t* regCSCPtr = Timer_getCnSCRegister(dev,channel);

    // In case of callback, enable Interrupt
    if (dev->pwmPulseFinishedCallback != 0)
    {
        UTILITY_SET_REGISTER_BIT(*regCSCPtr,TPM_CnSC_CHIE_MASK);
    }

    // Enable channel in the selected pin
    UTILITY_MODIFY_REGISTER(*regCSCPtr,TIMER_DISABLE_CHANNEL_MASK,dev->channelConfiguration[channel]);

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
    if (ohiassert(TIMER_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check the channel: exist into the device?
    if (ohiassert(TIMER_IS_CHANNEL_DEVICE(dev,channel)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PWM_CHANNEL;
    }

    volatile uint32_t* regCSCPtr = Timer_getCnSCRegister(dev,channel);

    // In case of callback, disable Interrupt
    if (dev->pwmPulseFinishedCallback != 0)
    {
        UTILITY_CLEAR_REGISTER_BIT(*regCSCPtr,TPM_CnSC_CHIE_MASK);
    }

    // Disable channel in the selected pin
    UTILITY_MODIFY_REGISTER(*regCSCPtr,TIMER_DISABLE_CHANNEL_MASK,0ul);

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
    if (ohiassert(TIMER_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_DEVICE;
    }
    // Check duty-cycle value
    if (ohiassert(duty <= 100) != ERRORS_NO_ERROR)
    {
        return ERRORS_TIMER_WRONG_PARAM;
    }

    volatile uint32_t* regCVPtr = Timer_getCnVRegister(dev,channel);
    // Compute duty-cycle pulse value
    uint32_t pulse = (((dev->regmap->MOD + 1) / 100) * duty);
    // Write new pulse value
    *regCVPtr = pulse - 1;

    return ERRORS_NO_ERROR;
}

void TPM0_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM0);
}

void TPM1_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM1);
}

void TPM2_IRQHandler (void)
{
    Timer_callbackInterrupt(OB_TIM2);
}

#if 0

void Ftm_init (Ftm_DeviceHandle dev, void *callback, Ftm_Config *config)
{
    Ftm_Prescaler prescaler;
    uint16_t modulo;

    uint8_t configPinIndex;
    uint8_t devPinIndex;

    /* Enable the clock to the selected FTM/TPM */


    /* If call back exist save it */
    if (callback)
    {
        dev->callback = callback;
        /* Enable interrupt */
        Interrupt_enable(dev->isrNumber);
        TPM_SC_REG(dev->regMap) |= TPM_SC_TOIE_MASK;
    }

    dev->mode = config->mode;

    dev->devInitialized = 1;

    switch (dev->mode)
    {
    case FTM_MODE_INPUT_CAPTURE:
        prescaler = Ftm_computeFrequencyPrescale(dev,config->timerFrequency);

        dev->configurationBits = config->configurationBits;
        TPM_SC_REG(dev->regMap) &=  ~TPM_SC_CPWMS_MASK;

        /* Initialize every selected channels */
        for (configPinIndex = 0; configPinIndex < FTM_MAX_CHANNEL; ++configPinIndex)
        {
            Ftm_Pins pin = config->pins[configPinIndex];

            if (pin == FTM_PINS_STOP)
                break;

            Ftm_addInputCapturePin(dev,pin,dev->configurationBits);
        }

        TPM_SC_REG(dev->regMap) = TPM_SC_CMOD(1) | TPM_SC_PS(prescaler) | 0;
        break;
    case FTM_MODE_OUTPUT_COMPARE:
        break;
    case FTM_MODE_QUADRATURE_DECODE:
        break;
    }
}

void Ftm_resetCounter (Ftm_DeviceHandle dev)
{
    TPM_CNT_REG(dev->regMap) = 0;
}

void Ftm_enableInterrupt (Ftm_DeviceHandle dev)
{
    /* disable interrupt */
    TPM_SC_REG(dev->regMap) &=~ TPM_SC_TOIE_MASK;
    /* set to zero cont */
    TPM_CNT_REG(dev->regMap) = 0;
    /* enable interrupt */
    TPM_SC_REG(dev->regMap) |=TPM_SC_TOIE_MASK;
}

void Ftm_disableInterrupt (Ftm_DeviceHandle dev)
{
    /* disable interrupt */
    TPM_SC_REG(dev->regMap) &=~ TPM_SC_TOIE_MASK;
}

void Ftm_enableChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel)
{
    volatile uint32_t* regCSCPtr = Ftm_getCnSCRegister(dev,channel);

    if (regCSCPtr)
    {
        *regCSCPtr |= TPM_CnSC_CHIE_MASK;
    }
}

void Ftm_disableChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel)
{
    volatile uint32_t* regCSCPtr = Ftm_getCnSCRegister(dev,channel);

    if (regCSCPtr)
    {
        *regCSCPtr &= ~TPM_CnSC_CHIE_MASK;
    }
}

bool Ftm_isChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel)
{
    volatile uint32_t* regCSCPtr = Ftm_getCnSCRegister(dev,channel);

    if (regCSCPtr && (*regCSCPtr & TPM_CnSC_CHF_MASK))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void Ftm_clearChannelFlagInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel)
{
    volatile uint32_t* regCSCPtr = Ftm_getCnSCRegister(dev,channel);

    if (regCSCPtr)
    {
        *regCSCPtr |= TPM_CnSC_CHF_MASK;
    }
}


uint16_t Ftm_getChannelCount (Ftm_DeviceHandle dev, Ftm_Channels channel)
{
    volatile uint32_t* regCVPtr = Ftm_getCnVRegister(dev,channel);

    if (regCVPtr)
    {
        return (uint16_t) *regCVPtr;
    }
}

System_Errors Ftm_addInputCapturePin (Ftm_DeviceHandle dev, Ftm_Pins pin, uint16_t configurations)
{
    uint8_t devPinIndex;

    volatile uint32_t* regCSCPtr;
    volatile uint32_t* regCVPtr;

    uint32_t tempReg = 0;

    if (dev->devInitialized == 0)
        return ERRORS_FTM_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < FTM_MAX_PINS; ++devPinIndex)
    {
        if (dev->pins[devPinIndex] == pin)
        {
            *(dev->pinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->pinMux[devPinIndex]) | PORT_PCR_IRQC(0);
            break;
        }
    }

    /* Select the right register */
    regCSCPtr = Ftm_getCnSCRegister(dev,dev->channel[devPinIndex]);

    /* Enable channel */
    if (regCSCPtr)
    {
        /* Input capture mode */
        *regCSCPtr &=  ~TPM_CnSC_MSA_MASK;
        *regCSCPtr &=  ~TPM_CnSC_MSB_MASK;

        *regCSCPtr &= ~(TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK);

        if (configurations & FTM_CONFIG_INPUT_RISING_EDGE)
        {
            *regCSCPtr |= TPM_CnSC_ELSA_MASK;
        }
        else if (configurations & FTM_CONFIG_INPUT_FALLING_EDGE)
        {
            *regCSCPtr |= TPM_CnSC_ELSB_MASK;
        }
        else
        {
            *regCSCPtr |= (TPM_CnSC_ELSB_MASK | TPM_CnSC_MSB_MASK);
        }

        /*Enable Selected Channel Interrupt*/
        *regCSCPtr |= TPM_CnSC_CHIE_MASK;
    }
    else
    {
        return ERRORS_FTM_CHANNEL_NOT_FOUND;
    }

    return ERRORS_FTM_OK;
}

#endif

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif //  LIBOHIBOARD_TIMER
