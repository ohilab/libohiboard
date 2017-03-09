/* Copyright (C) 2014-2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *
 *  Matteo Civale <matteo.civale@gmail.com>
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
 ******************************************************************************/

/**
 * @file libohiboard/source/ftm_KV46F.c
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief Ftm implementations for KV46F and TRWKV46F.
 */

#ifdef LIBOHIBOARD_FTM

#include "platforms.h"

#include "ftm.h"
#include "interrupt.h"
#include "utility.h"
#include "clock.h"

#if defined (LIBOHIBOARD_KV46F)     || \
    defined (LIBOHIBOARD_TRWKV46F)

#define FTM_MAX_PINS                     31
#define FTM_MAX_FAULT_PINS               9

typedef enum
{
    FTM_PRESCALER_1   = 0,
    FTM_PRESCALER_2   = 1,
    FTM_PRESCALER_4   = 2,
    FTM_PRESCALER_8   = 3,
    FTM_PRESCALER_16  = 4,
    FTM_PRESCALER_32  = 5,
    FTM_PRESCALER_64  = 6,
    FTM_PRESCALER_128 = 7,
} Ftm_Prescaler;

typedef enum
{
    FTM_DEADTIMEPRESCALER_1 = 0,
    FTM_DEADTIMEPRESCALER_2 = 1,
    FTM_DEADTIMEPRESCALER_4 = 2,
    FTM_DEADTIMEPRESCALER_8 = 3,

} Ftm_DeadTimePrescaler;

typedef struct Ftm_Device
{
    FTM_MemMapPtr regMap;                          /**< Device memory pointer */

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Ftm_Pins pins[FTM_MAX_PINS];    /**< List of the pin for the FTM channel. */
    volatile uint32_t* pinsPtr[FTM_MAX_PINS];
    Ftm_Channels channel[FTM_MAX_PINS];
    uint8_t pinMux[FTM_MAX_PINS];     /**< Mux of the pin of the FTM channel. */

    /** List of the fault pins for the FTM channel. */
    Ftm_FaultPins faultPins[FTM_MAX_FAULT_PINS];
    Ftm_FaultChannels faultChannel[FTM_MAX_FAULT_PINS];
    uint8_t faultMux[FTM_MAX_FAULT_PINS];
    volatile uint32_t* faultPinsPtr[FTM_MAX_FAULT_PINS];

    void (*isr)(void);                     /**< The function pointer for ISR. */
    void (*callback)(void);      /**< The function pointer for user callback. */
    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Ftm_Mode mode;                                  /**< Modes of operations. */

    uint8_t configurationBits;       /**< A useful variable to configure FTM. */

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Ftm_Device;

static Ftm_Device ftm0 = {
        .regMap           = FTM0_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_FTM0_MASK,

        .pins             = {FTM_PINS_PTE29,//PTE29
                             FTM_PINS_PTE30,//PTE30
                             FTM_PINS_PTE24,//PTE24
                             FTM_PINS_PTE25,//PTE25
                             FTM_PINS_PTA0, //PTA0
                             FTM_PINS_PTA1, //PTA1
                             FTM_PINS_PTA2, //PTA2
                             FTM_PINS_PTA3, //PTA3
                             FTM_PINS_PTA4, //PTA4
                             FTM_PINS_PTA5, //PTA5
                             FTM_PINS_PTC1, //PTC1
                             FTM_PINS_PTC2, //PTC2
                             FTM_PINS_PTC3, //PTC3
                             FTM_PINS_PTC4, //PTC4
                             FTM_PINS_PTC5, //PTC5
                             FTM_PINS_PTD0, //PTD0
                             FTM_PINS_PTD1, //PTD1
                             FTM_PINS_PTD2, //PTD2
                             FTM_PINS_PTD3, //PTD3
        },
        .pinsPtr          = {&PORTE_PCR29,//PTE29
                             &PORTE_PCR30,//PTE30
                             &PORTE_PCR24,//PTE24
                             &PORTE_PCR25,//PTE25
                             &PORTA_PCR0, //PTA0
                             &PORTA_PCR1, //PTA1
                             &PORTA_PCR2, //PTA2
                             &PORTA_PCR3, //PTA3
                             &PORTA_PCR4, //PTA4
                             &PORTA_PCR5, //PTA5
                             &PORTC_PCR1, //PTC1
                             &PORTC_PCR2, //PTC2
                             &PORTC_PCR3, //PTC3
                             &PORTC_PCR4, //PTC4
                             &PORTC_PCR5, //PTC5
                             &PORTD_PCR0, //PTD0
                             &PORTD_PCR1, //PTD1
                             &PORTD_PCR2, //PTD2
                             &PORTD_PCR3, //PTD3
        },
        .pinMux           = {3,//PTE29
                             3,//PTE30
                             3,//PTE24
                             3,//PTE25
                             3,//PTA0
                             3,//PTA1
                             3,//PTA2
                             3,//PTA3
                             3,//PTA4
                             3,//PTA5
                             4,//PTC1
                             4,//PTC2
                             4,//PTC3
                             4,//PTC4
                             7,//PTC5
                             5,//PTD0
                             5,//PTD1
                             5,//PTD2
                             5,//PTD3
        },
        .channel          = {FTM_CHANNELS_CH2,//PTE29
                             FTM_CHANNELS_CH3,//PTE30
                             FTM_CHANNELS_CH0,//PTE24
                             FTM_CHANNELS_CH1,//PTE25
                             FTM_CHANNELS_CH5,//PTA0
                             FTM_CHANNELS_CH6,//PTA1
                             FTM_CHANNELS_CH7,//PTA2
                             FTM_CHANNELS_CH0,//PTA3
                             FTM_CHANNELS_CH1,//PTA4
                             FTM_CHANNELS_CH2,//PTA5
                             FTM_CHANNELS_CH0,//PTC1
                             FTM_CHANNELS_CH1,//PTC2
                             FTM_CHANNELS_CH2,//PTC3
                             FTM_CHANNELS_CH3,//PTC4
                             FTM_CHANNELS_CH2,//PTC5
                             FTM_CHANNELS_CH0,//PTD0
                             FTM_CHANNELS_CH1,//PTD1
                             FTM_CHANNELS_CH2,//PTD2
                             FTM_CHANNELS_CH3,//PTD3

        },

        .faultPins        = {FTM_FAULTPINS_PTE16,//PTE16
                             FTM_FAULTPINS_PTA4, //PTA4
                             FTM_FAULTPINS_PTA18,//PTA18
                             FTM_FAULTPINS_PTB1, //PTB1
                             FTM_FAULTPINS_PTB2, //PTB2
                             FTM_FAULTPINS_PTB3, //PTB3
                             FTM_FAULTPINS_PTB10,//PTB10
                             FTM_FAULTPINS_PTB11,//PTB11
                             FTM_FAULTPINS_PTC0, //PTC0
        },
        .faultChannel     = {FTM_FAULTCHANNELS_3,//PTE16
                             FTM_FAULTCHANNELS_3,//PTA4
                             FTM_FAULTCHANNELS_2,//PTA18
                             FTM_FAULTCHANNELS_2,//PTB1
                             FTM_FAULTCHANNELS_1,//PTB2
                             FTM_FAULTCHANNELS_0,//PTB3
                             FTM_FAULTCHANNELS_1,//PTB10
                             FTM_FAULTCHANNELS_2,//PTB11
                             FTM_FAULTCHANNELS_1, //PTC0
        },
        .faultPinsPtr    = {&PORTE_PCR16,//PTE16
                            &PORTA_PCR4, //PTA4
                            &PORTA_PCR18,//PTA18
                            &PORTB_PCR1, //PTB1
                            &PORTB_PCR2, //PTB2
                            &PORTB_PCR3, //PTB3
                            &PORTB_PCR10,//PTB10
                            &PORTB_PCR11,//PTB11
                            &PORTC_PCR0, //PTC0
        },
        .faultMux        = {6,//PTE16
                            5,//PTAA
                            3,//PTA18
                            4,//PTB1
                            4,//PTB2
                            6,//PTB3
                            6,//PTB10
                            6,//PTB11
                            6,//PTC0

        },

        .isr              = FTM0_IRQHandler,
        .isrNumber        = INTERRUPT_FTM0,

        .devInitialized   = 0,
};
Ftm_DeviceHandle OB_FTM0 = &ftm0;

static Ftm_Device ftm1 = {
        .regMap           = FTM1_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_FTM1_MASK,

        .pins             = {FTM_PINS_PTE20,//PTE20
                             FTM_PINS_PTE21,//PTE21
                             FTM_PINS_PTA1, //PTA1
                             FTM_PINS_PTA2, //PTA2
                             FTM_PINS_PTA12,//PTA12
                             FTM_PINS_PTA12,//PTA13
                             FTM_PINS_PTB0, //PTB0
                             FTM_PINS_PTB1, //PTB1
                             FTM_PINS_PTD6, //PTD6

        },
        .pinsPtr          = {&PORTE_PCR20,//PTE20
                             &PORTE_PCR21,//PTE21
                             &PORTA_PCR1, //PTA1
                             &PORTA_PCR2, //PTA2
                             &PORTA_PCR12,//PTA12
                             &PORTA_PCR13,//PTA13
                             &PORTB_PCR0, //PTB0
                             &PORTB_PCR1, //PTB1
                             &PORTD_PCR1, //PTD6

        },
        .pinMux           = {3,//PTE20
                             3,//PTE21
                             6,//PTA1
                             6,//PTA2
                             3,//PTA12
                             3,//PTA13
                             3,//PTB0
                             3,//PTB1
                             5,//PTD6

        },
        .channel          = {FTM_CHANNELS_CH0,//PTE20
                             FTM_CHANNELS_CH1,//PTE21
                             FTM_CHANNELS_CH1,//PTA1
                             FTM_CHANNELS_CH0,//PTA2
                             FTM_CHANNELS_CH0,//PTA12
                             FTM_CHANNELS_CH1,//PTA13
                             FTM_CHANNELS_CH0,//PTB0
                             FTM_CHANNELS_CH1,//PTB1
                             FTM_CHANNELS_CH0,//PTD6

        },

        .faultPins        = {FTM_FAULTPINS_PTA19,

        },
        .faultChannel     = {FTM_FAULTCHANNELS_0,

        },
        .faultPinsPtr    = {&PORTA_PCR19,

        },
        .faultMux        = {3,

        },

        .isr              = FTM1_IRQHandler,
        .isrNumber        = INTERRUPT_FTM1,

        .devInitialized   = 0,
};
Ftm_DeviceHandle OB_FTM1 = &ftm1;

static Ftm_Device ftm3 = {
        .regMap           = FTM3_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_FTM3_MASK,

        .pins             = {FTM_PINS_PTE5,
                             FTM_PINS_PTE6,
                             FTM_PINS_PTA18,
                             FTM_PINS_PTB18,
                             FTM_PINS_PTB19,
                             FTM_PINS_PTC8,
                             FTM_PINS_PTC9,
                             FTM_PINS_PTC10,
                             FTM_PINS_PTC11,
                             FTM_PINS_PTD0,
                             FTM_PINS_PTD1,
                             FTM_PINS_PTD2,
                             FTM_PINS_PTD3,

        },
        .pinsPtr          = {&PORTE_PCR5, //PRE5
                             &PORTE_PCR6, //PTE6
                             &PORTA_PCR18,//PTA18
                             &PORTB_PCR18,//PTB18
                             &PORTB_PCR19,//PTB19
                             &PORTC_PCR8, //PTC8
                             &PORTC_PCR9, //PTC9
                             &PORTC_PCR10,//PTC10
                             &PORTC_PCR11,//PTC11
                             &PORTD_PCR0, //PTD0
                             &PORTD_PCR1, //PTD1
                             &PORTD_PCR2, //PTD2
                             &PORTD_PCR3, //PTD3

        },
        .pinMux           = {6,//PTE5
                             6,//PTE6
                             6,//PTA18
                             4,//PTB18
                             4,//PTB19
                             3,//PTC8
                             3,//PTC9
                             3,//PTC10
                             3,//PTC11
                             4,//PTD0
                             4,//PTD1
                             4,//PTD2
                             4,//PTD3

        },
        .channel          = {FTM_CHANNELS_CH0,//PTE5
                             FTM_CHANNELS_CH1,//PTE6
                             FTM_CHANNELS_CH2,//PTA18
                             FTM_CHANNELS_CH2,//PTB18
                             FTM_CHANNELS_CH3,//PTB19
                             FTM_CHANNELS_CH4,//PTC8
                             FTM_CHANNELS_CH5,//PTC9
                             FTM_CHANNELS_CH6,//PTC10
                             FTM_CHANNELS_CH7,//PTC11
                             FTM_CHANNELS_CH0,//PTD0
                             FTM_CHANNELS_CH1,//PTD1
                             FTM_CHANNELS_CH2,//PTD2
                             FTM_CHANNELS_CH3,//PTD3


        },

        .faultPins        = {FTM_FAULTPINS_PTC3,
                             FTM_FAULTPINS_PTC12,
        },
        .faultChannel     = {FTM_FAULTCHANNELS_0,
                             FTM_FAULTCHANNELS_0,
        },
        .faultPinsPtr    = {&PORTC_PCR3,
                            &PORTC_PCR12,
        },
        .faultMux        = {6,
                            6,
        },

        .isr              = FTM3_IRQHandler,
        .isrNumber        = INTERRUPT_FTM3,

        .devInitialized   = 0,
};
Ftm_DeviceHandle OB_FTM3 = &ftm3;

static void Ftm_callbackInterrupt (Ftm_DeviceHandle dev)
{
    switch (dev->mode)
    {
    case FTM_MODE_INPUT_CAPTURE:
        break;
    case FTM_MODE_OUTPUT_COMPARE:
        break;
    case FTM_MODE_QUADRATURE_DECODE:
        break;
    case FTM_MODE_PWM:
    case FTM_MODE_FREE:
        dev->callback();
        /* Reading SC register and clear TOF bit */
        FTM_SC_REG(dev->regMap) &= ~FTM_SC_TOF_MASK;

        break;
    case FTM_MODE_COMBINE:
        dev->callback();
        /* Reading SC register and clear TOF bit */
        FTM_SC_REG(dev->regMap) &= ~FTM_SC_TOF_MASK;
        break;
    default:
        assert(0);
        break;
    }
}

static System_Errors Ftm_addFaultPin (Ftm_DeviceHandle dev, Ftm_FaultPins pin, Ftm_FaultChannels *channel)
{
    uint8_t devPinIndex = 0;

    if (dev->devInitialized == 0)
        return ERRORS_FTM_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < FTM_MAX_FAULT_PINS; ++devPinIndex)
    {
        if (dev->faultPins[devPinIndex] == pin)
        {
            *(dev->faultPinsPtr[devPinIndex]) =
                    PORT_PCR_MUX(dev->faultMux[devPinIndex]) | PORT_PCR_IRQC(0);
            *channel = dev->faultChannel[devPinIndex];
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_FTM_FAULT_PIN_WRONG;
}

void FTM0_IRQHandler (void)
{
    Ftm_callbackInterrupt(OB_FTM0);
}

void FTM1_IRQHandler (void)
{
    Ftm_callbackInterrupt(OB_FTM1);
}


void FTM3_IRQHandler (void)
{
    Ftm_callbackInterrupt(OB_FTM3);
}


static uint8_t Ftm_computeDeadTimeModulo (uint32_t deadTime, Ftm_DeadTimePrescaler prescaler)
{
    uint32_t clock = Clock_getFrequency(CLOCK_FAST_PERIPHERALS);
    uint32_t modulo;

    switch (prescaler)
    {
    case FTM_DEADTIMEPRESCALER_1:
        modulo = (uint8_t) ((clock * deadTime)/1000000);
        break;
    case FTM_DEADTIMEPRESCALER_2:
        modulo = (uint8_t) ((clock * deadTime)/1000000) >> 1;
        break;
    case FTM_DEADTIMEPRESCALER_4:
        modulo = (uint8_t) ((clock * deadTime)/1000000) >> 2;
        break;
    case FTM_DEADTIMEPRESCALER_8:
        modulo = (uint8_t) ((clock * deadTime)/1000000) >> 3;
        break;
    }

    assert(modulo < 0xFF);
    return (uint8_t) modulo;
}

static Ftm_DeadTimePrescaler Ftm_computeDeadTimePrescale (uint32_t deadTime)
{
    uint32_t clock = Clock_getFrequency(CLOCK_FAST_PERIPHERALS);
    uint32_t prescaler = (uint32_t) ((deadTime * clock / 1000000) >> 6) + 1;

    if (prescaler > 4)
        return FTM_DEADTIMEPRESCALER_8;
    else if (prescaler >= 2)
        return FTM_DEADTIMEPRESCALER_4;
    else if (prescaler >= 1)
        return FTM_DEADTIMEPRESCALER_2;
    else
        return FTM_DEADTIMEPRESCALER_1;
}

static Ftm_Prescaler Ftm_computeFrequencyPrescale (uint32_t timerFrequency)
{
    uint32_t clock = Clock_getFrequency(CLOCK_FAST_PERIPHERALS);
    uint8_t prescaler = (clock / timerFrequency) / 65536;

    if (prescaler > 64)
        return FTM_PRESCALER_128;
    else if (prescaler >= 32)
        return FTM_PRESCALER_64;
    else if (prescaler >= 16)
        return FTM_PRESCALER_32;
    else if (prescaler >= 8)
        return FTM_PRESCALER_16;
    else if (prescaler >= 4)
        return FTM_PRESCALER_8;
    else if (prescaler >= 2)
        return FTM_PRESCALER_4;
    else if (prescaler >= 1)
        return FTM_PRESCALER_2;
    else
        return FTM_PRESCALER_1;
}

static uint16_t Ftm_computeModulo (uint32_t timerFrequency, Ftm_Prescaler prescaler)
{
    uint32_t clock = Clock_getFrequency(CLOCK_FAST_PERIPHERALS);
    uint32_t modulo;

    switch (prescaler)
    {
    case FTM_PRESCALER_1:
        modulo = (uint16_t) (clock / timerFrequency);
        break;
    case FTM_PRESCALER_2:
        modulo = (uint16_t) (clock / (timerFrequency * 2));
        break;
    case FTM_PRESCALER_4:
        modulo = (uint16_t) (clock / (timerFrequency * 4));
        break;
    case FTM_PRESCALER_8:
        modulo = (uint16_t) (clock / (timerFrequency * 8));
        break;
    case FTM_PRESCALER_16:
        modulo = (uint16_t) (clock / (timerFrequency * 16));
        break;
    case FTM_PRESCALER_32:
        modulo = (uint16_t) (clock / (timerFrequency * 32));
        break;
    case FTM_PRESCALER_64:
        modulo = (uint16_t) (clock / (timerFrequency * 64));
        break;
    case FTM_PRESCALER_128:
        modulo = (uint16_t) (clock / (timerFrequency * 128));
        break;
    }

    assert(modulo < 0xFFFF);
    return (uint16_t) modulo;
}

static uint16_t Ftm_computeDutyValue (uint16_t dutyScaled, uint16_t modulo)
{
    if (dutyScaled > 32768)
    {
        return 32768;
    }
    else
    {
        return (modulo * dutyScaled) / 32768;
    }
}

static volatile uint32_t* Ftm_getCnVRegister (Ftm_DeviceHandle dev, Ftm_Channels channel)
{
    switch (channel)
    {
    case FTM_CHANNELS_CH0:
        return &FTM_CnV_REG(dev->regMap,0);
    case FTM_CHANNELS_CH1:
        return &FTM_CnV_REG(dev->regMap,1);
    case FTM_CHANNELS_CH2:
        return &FTM_CnV_REG(dev->regMap,2);
    case FTM_CHANNELS_CH3:
        return &FTM_CnV_REG(dev->regMap,3);
    case FTM_CHANNELS_CH4:
        return &FTM_CnV_REG(dev->regMap,4);
    case FTM_CHANNELS_CH5:
        return &FTM_CnV_REG(dev->regMap,5);
    case FTM_CHANNELS_CH6:
        return &FTM_CnV_REG(dev->regMap,6);
    case FTM_CHANNELS_CH7:
        return &FTM_CnV_REG(dev->regMap,7);
    default:
        assert(0);
        return 0;
    }
}

static volatile uint32_t* Ftm_getCnSCRegister (Ftm_DeviceHandle dev, Ftm_Channels channel)
{
    switch (channel)
    {
    case FTM_CHANNELS_CH0:
        return &FTM_CnSC_REG(dev->regMap,0);
    case FTM_CHANNELS_CH1:
        return &FTM_CnSC_REG(dev->regMap,1);
    case FTM_CHANNELS_CH2:
        return &FTM_CnSC_REG(dev->regMap,2);
    case FTM_CHANNELS_CH3:
        return &FTM_CnSC_REG(dev->regMap,3);
    case FTM_CHANNELS_CH4:
        return &FTM_CnSC_REG(dev->regMap,4);
    case FTM_CHANNELS_CH5:
        return &FTM_CnSC_REG(dev->regMap,5);
    case FTM_CHANNELS_CH6:
        return &FTM_CnSC_REG(dev->regMap,6);
    case FTM_CHANNELS_CH7:
        return &FTM_CnSC_REG(dev->regMap,7);
    default:
        assert(0);
        return 0;
    }
}

void Ftm_setPwm (Ftm_DeviceHandle dev, Ftm_Channels channel, uint16_t dutyScaled)
{
    volatile uint32_t* regCVPtr = Ftm_getCnVRegister(dev,channel);

    if (regCVPtr)
    {
        *regCVPtr = Ftm_computeDutyValue(dutyScaled,FTM_MOD_REG(dev->regMap));
    }
}

void Ftm_init (Ftm_DeviceHandle dev, void *callback, Ftm_Config *config)
{
    Ftm_Prescaler prescaler;
    uint16_t modulo;
    int16_t initCounter;

    Ftm_DeadTimePrescaler deadPrescaler;
    uint8_t deadModulo;

    uint8_t configPinIndex;
    uint8_t devPinIndex;

    /* Fault channel temp variable */
    Ftm_FaultChannels faultCh;

    /* For combine mode */
    uint8_t i = 0;
    uint8_t channelNum = 0;
    uint8_t channelIndex;
    uint8_t configurationMask;
    System_Errors error = ERRORS_NO_ERROR;

    /* Enable the clock to the selected FTM */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* If call back exist save it */
    if (callback)
    {
        dev->callback = callback;
        /* Enable interrupt */
        Interrupt_enable(dev->isrNumber);
        FTM_SC_REG(dev->regMap) |= FTM_SC_TOIE_MASK;
    }

    dev->mode = config->mode;

    dev->devInitialized = 1;

    switch (dev->mode)
    {
    case FTM_MODE_INPUT_CAPTURE:
        prescaler = Ftm_computeFrequencyPrescale(config->timerFrequency);

        dev->configurationBits = config->configurationBits;
        FTM_SC_REG(dev->regMap) &=  ~FTM_SC_CPWMS_MASK;

        /* Initialize every selected channels */
        for (configPinIndex = 0; configPinIndex < FTM_MAX_CHANNEL; ++configPinIndex)
        {
            Ftm_Pins pin = config->pins[configPinIndex];

            if (pin == FTM_PINS_STOP)
                break;

            Ftm_addInputCapturePin(dev,pin,dev->configurationBits);
        }

        FTM_SC_REG(dev->regMap) = FTM_SC_CLKS(1) | FTM_SC_PS(prescaler) | 0;
        break;
    case FTM_MODE_OUTPUT_COMPARE:
        break;
    case FTM_MODE_QUADRATURE_DECODE:
        break;
    case FTM_MODE_PWM:
        /* Compute prescale factor */
        prescaler = Ftm_computeFrequencyPrescale(config->timerFrequency);

        if (config->configurationBits & FTM_CONFIG_PWM_CENTER_ALIGNED)
        {
            dev->configurationBits = FTM_CONFIG_PWM_CENTER_ALIGNED;
            FTM_SC_REG(dev->regMap) |= FTM_SC_CPWMS_MASK;
        }
        else
        {
            dev->configurationBits = FTM_CONFIG_PWM_EDGE_ALIGNED;
            FTM_SC_REG(dev->regMap) &= ~FTM_SC_CPWMS_MASK;
        }

        /* Compute timer modulo and set it */
        modulo = Ftm_computeModulo(config->timerFrequency,prescaler);
        if (dev->configurationBits & FTM_CONFIG_PWM_CENTER_ALIGNED)
        {
            modulo = (modulo / 2) - 1;
            initCounter = 0;
        }
        else
        {
            modulo -= 1;
            initCounter = 0;
        }
        FTM_CNTIN_REG(dev->regMap) = FTM_CNTIN_INIT(initCounter);
        FTM_MOD_REG(dev->regMap) = modulo;

        /* Initialize every selected channels */
        for (configPinIndex = 0; configPinIndex < FTM_MAX_CHANNEL; ++configPinIndex)
        {
            Ftm_Pins pin = config->pins[configPinIndex];

            if (pin == FTM_PINS_STOP)
                break;

            Ftm_addPwmPin(dev,pin,config->duty[configPinIndex]);
        }

        FTM_SC_REG(dev->regMap) |= FTM_SC_CLKS(1) | FTM_SC_PS(prescaler) | 0;
        break;
    case FTM_MODE_FREE:

        /* Compute prescale factor */
        prescaler = Ftm_computeFrequencyPrescale(config->timerFrequency);

        /* Compute timer modulo */
        modulo = Ftm_computeModulo(config->timerFrequency,prescaler);

        FTM_CNTIN_REG(dev->regMap) = FTM_CNTIN_INIT(config->initCounter);
        FTM_MOD_REG(dev->regMap) = modulo - 1;
        FTM_SC_REG(dev->regMap) |= FTM_SC_CLKS(1) |
                                   FTM_SC_PS(prescaler) | 0;
        break;

    case FTM_MODE_COMBINE:

        /* Mask interrupt */
        Interrupt_disable(dev->isrNumber);

        /* Calculate prescaler */
        prescaler = Ftm_computeFrequencyPrescale(config->timerFrequency);

        /*Calculate modulo*/
        modulo = Ftm_computeModulo(config->timerFrequency,prescaler);

        /* Configure pin associate to channel */
        for (configPinIndex = 0; configPinIndex < FTM_MAX_CHANNEL; ++configPinIndex)
        {
            Ftm_Pins pin = config->pins[configPinIndex];
            if (pin == FTM_PINS_STOP)  break;
            Ftm_addPwmPin(dev,pin,config->duty[configPinIndex]);
        }

        /* Calculate the number of channel */
        channelNum = configPinIndex >> 1;

        /* Disable write protection  */
        FTM_MODE_REG(dev->regMap) |= FTM_MODE_WPDIS_MASK;
        /* Enable special function register */
        FTM_MODE_REG(dev->regMap) |= FTM_MODE_FTMEN_MASK;

        /* Asymmetrical or Symmetrical */
        if (config->symmetrical)
        {
            /* Load MOD and CNTIN so enable central aligned mode */
            FTM_MOD_REG(dev->regMap) = modulo/2-1;
            FTM_CNTIN_REG(dev->regMap) = -modulo/2;
        }
        else
        {
            FTM_MOD_REG(dev->regMap) = 0;
            FTM_CNTIN_REG(dev->regMap) = modulo-1;
        }



        FTM_SC_REG(dev->regMap) |= FTM_SC_CLKS(1) |
                                   FTM_SC_PS(prescaler);

        /* Enables the loading of the MOD, CNTIN, and CV registers with the values of their write buffers*/
        FTM_PWMLOAD_REG(dev->regMap) |= FTM_PWMLOAD_LDOK_MASK;

        /* For all channels set align type loading type and.....*/
        for (channelIndex=0; channelIndex < channelNum; channelIndex++)
        {
            configurationMask = 0x00;

            if (config->channelPair[channelIndex].align)
            {
                /* ELSnB:ELSnA = 1:0 Set channel mode to generate positive PWM */
                FTM_CnSC_REG(dev->regMap,config->channelPair[channelIndex].pair*2) |= FTM_CnSC_ELSB_MASK;
                FTM_CnSC_REG(dev->regMap,config->channelPair[channelIndex].pair*2+1) |= FTM_CnSC_ELSB_MASK;
            }
            else
            {
                /* ELSnB:ELSnA = X:1 Set channel mode to generate negative PWM */
                FTM_CnSC_REG(dev->regMap,config->channelPair[channelIndex].pair*2) |= FTM_CnSC_ELSA_MASK;
                FTM_CnSC_REG(dev->regMap,config->channelPair[channelIndex].pair*2+1) |= FTM_CnSC_ELSA_MASK;
            }

            /*
             * Include CnV or C(n+1)V in comparison process to evaluate reload
             * of register pag 1002 datasheet
             */
            if (config->channelPair[channelIndex].reload)
            {
                FTM_PWMLOAD_REG(dev->regMap) |=
                        config->channelPair[channelIndex].reload <<
                        config->channelPair[channelIndex].pair*2;
            }

            /* Combine channel and set the other configuration for the selected channel */
            configurationMask |= (
                    SHIFT_LEFT(config->channelPair[channelIndex].enableFaultInterrupt,6) |
                    SHIFT_LEFT(config->channelPair[channelIndex].enableSynchronization,5)|
                    SHIFT_LEFT(config->channelPair[channelIndex].enableDeadTime,4)       |
                    SHIFT_LEFT(config->channelPair[channelIndex].enableComplementary,1)  |
                    SHIFT_LEFT(1,0)) & 0xFF;

            /* Upload COMBINE REG */
            FTM_COMBINE_REG(dev->regMap) |= (configurationMask <<
                    (8 * config->channelPair[channelIndex].pair));
            /* Enable output mask */
            FTM_OUTMASK_REG(dev->regMap) |=
                    0x3 << (config->channelPair[channelIndex].pair * 2);
        }

        /* Calculate DeadTime parameter */
        if (config->deadTime)
        {
            /* Calculate dead time prescaler */
            deadPrescaler = Ftm_computeDeadTimePrescale(config->deadTime);
            /* Calculate dead time module */
            deadModulo = Ftm_computeDeadTimeModulo(config->deadTime, deadPrescaler);
            /* Set prescaler */
            FTM_DEADTIME_REG(dev->regMap) |= FTM_DEADTIME_DTPS(deadPrescaler);
            /* Set dead time module */
            FTM_DEADTIME_REG(dev->regMap) |= FTM_DEADTIME_DTVAL(deadModulo);
        }

        /* Synchronization */
        /* TODO: outside?? */
        FTM_SYNC_REG(dev->regMap) = config->syncEvent;

        /* Drive the output with the value reported in  OUTINIT register */
        FTM_MODE_REG(dev->regMap) |= FTM_MODE_INIT_MASK;

        /* Enable output mask */
        FTM_OUTMASK_REG(dev->regMap)=0xF;

        break;
    }

    /* Enable the fault interrupt */
    if (config->interruptEnableFault)
        FTM_MOD_REG(dev->regMap) |= FTM_MODE_FAULTIE_MASK;

    /* Configure fault pin */
    i = 0;
    while (config->fault[i].pin != FTM_FAULTPINS_STOP)
    {
        /* Enable all fault control */
        FTM_MODE_REG(dev->regMap) |= FTM_MODE_FAULTM(config->faultMode);
        /* Enable pin to fault function */
        error = Ftm_addFaultPin(dev,config->fault[i].pin,&faultCh);
        if (error) break;
        FTM_FLTCTRL_REG(dev->regMap) |= 1 << faultCh;

        if (config->fault[i].enableFilter)
        {
            FTM_FLTCTRL_REG(dev->regMap) |= 1 << (faultCh + 4);
            FTM_FLTCTRL_REG(dev->regMap) |=
                    FTM_FLTCTRL_FFVAL(config->faultFilterValue);
        }
        FTM_FLTPOL_REG(dev->regMap)|= config->fault[i].polarity << i;
        i++;
    }

    /* Trigger channel */
    if (config->triggerChannel != FTM_TRIGGER_NOCH)
    {
        /* Enables the generation of the trigger when
         * the FTM counter is equal to the CNTIN register
         */
        FTM_EXTTRIG_REG(dev->regMap) |= (config->triggerChannel & 0x3F) |
                (config->enableInitTrigger << FTM_EXTTRIG_INITTRIGEN_SHIFT);

    }

}

void Ftm_resetCounter (Ftm_DeviceHandle dev)
{
    FTM_CNT_REG(dev->regMap) = 0;
}

void Ftm_enableInterrupt (Ftm_DeviceHandle dev)
{
    /* disable interrupt */
    FTM_SC_REG(dev->regMap) &= ~FTM_SC_TOIE_MASK;
    FTM_SC_REG(dev->regMap) &= ~FTM_SC_TOF_MASK;
    /* set to zero cont */
    FTM_CNT_REG(dev->regMap) = 0;
    /* enable interrupt */
    FTM_SC_REG(dev->regMap) |= FTM_SC_TOIE_MASK;
}

void Ftm_disableInterrupt (Ftm_DeviceHandle dev)
{
    /* disable interrupt */
    FTM_SC_REG(dev->regMap) &= ~FTM_SC_TOIE_MASK;
}

void Ftm_stopCount(Ftm_DeviceHandle dev)
{
    FTM_SC_REG(dev->regMap) &= FTM_SC_CLKS_MASK;
    FTM_SC_REG(dev->regMap) |= FTM_SC_CLKS(0);
}

void Ftm_startCount(Ftm_DeviceHandle dev)
{
    FTM_SC_REG(dev->regMap) &= FTM_SC_CLKS_MASK;
    FTM_SC_REG(dev->regMap) |= FTM_SC_CLKS(1);
}

System_Errors Ftm_addPwmPin (Ftm_DeviceHandle dev, Ftm_Pins pin, uint16_t dutyScaled)
{
    uint8_t devPinIndex;

    volatile uint32_t* regCSCPtr;
    volatile uint32_t* regCVPtr;

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
    /* Select the right register */
    regCSCPtr = Ftm_getCnSCRegister(dev,dev->channel[devPinIndex]);
    regCVPtr  = Ftm_getCnVRegister(dev,dev->channel[devPinIndex]);

    /* Enable channel and set PWM value */
    if (regCSCPtr && regCVPtr)
    {
        if (dev->configurationBits & FTM_CONFIG_PWM_CENTER_ALIGNED)
        {
            *regCSCPtr = FTM_CnSC_ELSB_MASK;
        }
        else
        {
            *regCSCPtr = FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK;
        }

        Ftm_setPwm (dev,dev->channel[devPinIndex],dutyScaled);
    }
    else
    {
        return ERRORS_FTM_CHANNEL_NOT_FOUND;
    }

    return ERRORS_FTM_OK;
}

void Ftm_enableChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel)
{
    volatile uint32_t* regCSCPtr = Ftm_getCnSCRegister(dev,channel);

    if (regCSCPtr)
    {
        *regCSCPtr |= FTM_CnSC_CHIE_MASK;
    }
}

void Ftm_disableChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel)
{
    volatile uint32_t* regCSCPtr = Ftm_getCnSCRegister(dev,channel);

    if (regCSCPtr)
    {
        *regCSCPtr &= ~FTM_CnSC_CHIE_MASK;
    }
}

bool Ftm_isChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel)
{
    volatile uint32_t* regCSCPtr = Ftm_getCnSCRegister(dev,channel);

    if (regCSCPtr && (*regCSCPtr & FTM_CnSC_CHF_MASK))
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
        *regCSCPtr |= FTM_CnSC_CHF_MASK;
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
        *regCSCPtr &=  ~FTM_CnSC_MSA_MASK;
        *regCSCPtr &=  ~FTM_CnSC_MSB_MASK;

        *regCSCPtr &= ~(FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);

        if (configurations & FTM_CONFIG_INPUT_RISING_EDGE)
        {
            *regCSCPtr |= FTM_CnSC_ELSA_MASK;
        }
        else if (configurations & FTM_CONFIG_INPUT_FALLING_EDGE)
        {
            *regCSCPtr |= FTM_CnSC_ELSB_MASK;
        }
        else
        {
            *regCSCPtr |= (FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK);
        }

        /* Enable Selected Channel Interrupt */
        *regCSCPtr |= FTM_CnSC_CHIE_MASK;
    }
    else
    {
        return ERRORS_FTM_CHANNEL_NOT_FOUND;
    }

    return ERRORS_FTM_OK;
}

uint8_t Ftm_ResetFault (Ftm_DeviceHandle dev)
{
    FTM_FMS_REG(dev->regMap) &= ~FTM_FMS_FAULTF_MASK;
    return ((FTM_FMS_REG(dev->regMap) & FTM_FMS_FAULTF_MASK) >> FTM_FMS_FAULTIN_SHIFT);

}

uint16_t Ftm_getModule(Ftm_DeviceHandle dev)
{
  return  FTM_MOD_REG(dev->regMap)-FTM_CNTIN_REG(dev->regMap)+1;
}

#endif /* LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F */

#endif // LIBOHIBOARD_FTM


