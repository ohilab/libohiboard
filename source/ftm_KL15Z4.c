/* Copyright (C) 2014-2017 A. C. Open Hardware Ideas Lab
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
 ******************************************************************************/

/**
 * @file libohiboard/source/ftm_K15Z4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Ftm implementations for KL15Z4.
 */

#ifdef LIBOHIBOARD_FTM

#include "platforms.h"

#include "ftm.h"
#include "interrupt.h"
#include "clock.h"

#if defined (LIBOHIBOARD_KL15Z4)

#define FTM_MAX_PINS                     21

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

typedef struct Ftm_Device
{
    TPM_MemMapPtr regMap;                          /**< Device memory pointer */

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Ftm_Pins pins[FTM_MAX_PINS];    /**< List of the pin for the FTM channel. */
    volatile uint32_t* pinsPtr[FTM_MAX_PINS];
    Ftm_Channels channel[FTM_MAX_PINS];
    uint8_t pinMux[FTM_MAX_PINS];     /**< Mux of the pin of the FTM channel. */

    void (*isr)(void);                     /**< The function pointer for ISR. */
    void (*callback)(void);      /**< The function pointer for user callback. */
    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Ftm_Mode mode;                                  /**< Modes of operations. */

    uint8_t configurationBits;       /**< A useful variable to configure FTM. */

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Ftm_Device;

static Ftm_Device ftm0 = {
        .regMap           = TPM0_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_TPM0_MASK,

        .pins             = {FTM_PINS_PTA0,
                             FTM_PINS_PTA3,
                             FTM_PINS_PTA4,
                             FTM_PINS_PTA5,
                             FTM_PINS_PTC1,
                             FTM_PINS_PTC2,
                             FTM_PINS_PTC3,
                             FTM_PINS_PTC4,
                             FTM_PINS_PTC8,
                             FTM_PINS_PTC9,
                             FTM_PINS_PTD0,
                             FTM_PINS_PTD1,
                             FTM_PINS_PTD2,
                             FTM_PINS_PTD3,
                             FTM_PINS_PTD4,
                             FTM_PINS_PTD5,
                             FTM_PINS_PTE24,
                             FTM_PINS_PTE25,
                             FTM_PINS_PTE29,
                             FTM_PINS_PTE30,
                             FTM_PINS_PTE31,
        },
        .pinsPtr          = {&PORTA_PCR0,
                             &PORTA_PCR3,
                             &PORTA_PCR4,
                             &PORTA_PCR5,
                             &PORTC_PCR1,
                             &PORTC_PCR2,
                             &PORTC_PCR3,
                             &PORTC_PCR4,
                             &PORTC_PCR8,
                             &PORTC_PCR9,
                             &PORTD_PCR0,
                             &PORTD_PCR1,
                             &PORTD_PCR2,
                             &PORTD_PCR3,
                             &PORTD_PCR4,
                             &PORTD_PCR5,
                             &PORTE_PCR24,
                             &PORTE_PCR25,
                             &PORTE_PCR29,
                             &PORTE_PCR30,
                             &PORTE_PCR31,
        },
        .pinMux           = {3,
                             3,
                             3,
                             3,
                             4,
                             4,
                             4,
                             4,
                             3,
                             3,
                             4,
                             4,
                             4,
                             4,
                             4,
                             4,
                             3,
                             3,
                             3,
                             3,
                             3,
        },
        .channel          = {FTM_CHANNELS_CH5,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH2,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH2,
                             FTM_CHANNELS_CH3,
                             FTM_CHANNELS_CH4,
                             FTM_CHANNELS_CH5,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH2,
                             FTM_CHANNELS_CH3,
                             FTM_CHANNELS_CH4,
                             FTM_CHANNELS_CH5,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH2,
                             FTM_CHANNELS_CH3,
                             FTM_CHANNELS_CH4,
        },

        .isr              = TPM0_IRQHandler,
        .isrNumber        = INTERRUPT_TPM0,

        .devInitialized   = 0,
};
Ftm_DeviceHandle OB_FTM0 = &ftm0;

static Ftm_Device ftm1 = {
        .regMap           = TPM1_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_TPM1_MASK,

        .pins             = {FTM_PINS_PTA12,
                             FTM_PINS_PTA13,
                             FTM_PINS_PTB0,
                             FTM_PINS_PTB1,
                             FTM_PINS_PTE20,
                             FTM_PINS_PTE21,
        },
        .pinsPtr          = {&PORTA_PCR12,
                             &PORTA_PCR13,
                             &PORTB_PCR0,
                             &PORTB_PCR1,
                             &PORTE_PCR20,
                             &PORTE_PCR21,
        },
        .pinMux           = {3,
                             3,
                             3,
                             3,
                             3,
                             3,
        },
        .channel          = {FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
        },

        .isr              = TPM1_IRQHandler,
        .isrNumber        = INTERRUPT_TPM1,

        .devInitialized   = 0,
};
Ftm_DeviceHandle OB_FTM1 = &ftm1;

static Ftm_Device ftm2 = {
        .regMap           = TPM2_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_TPM2_MASK,

        .pins             = {FTM_PINS_PTA1,
                             FTM_PINS_PTA2,
                             FTM_PINS_PTB2,
                             FTM_PINS_PTB3,
                             FTM_PINS_PTB18,
                             FTM_PINS_PTB19,
                             FTM_PINS_PTE22,
                             FTM_PINS_PTE23,
        },
        .pinsPtr          = {&PORTA_PCR1,
                             &PORTA_PCR2,
                             &PORTB_PCR2,
                             &PORTB_PCR3,
                             &PORTB_PCR18,
                             &PORTB_PCR19,
                             &PORTE_PCR22,
                             &PORTE_PCR23,
        },
        .pinMux           = {3,
                             3,
                             3,
                             3,
                             3,
                             3,
                             3,
                             3,
        },
        .channel          = {FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
        },

        .isr              = TPM2_IRQHandler,
        .isrNumber        = INTERRUPT_TPM2,

        .devInitialized   = 0,
};
Ftm_DeviceHandle OB_FTM2 = &ftm2;

static void Ftm_callbackInterrupt (Ftm_DeviceHandle dev)
{
    switch (dev->mode)
    {
    case FTM_MODE_INPUT_CAPTURE:
        dev->callback();
        break;
    case FTM_MODE_OUTPUT_COMPARE:
        break;
    case FTM_MODE_QUADRATURE_DECODE:
        break;
    case FTM_MODE_PWM:
    case FTM_MODE_FREE:
        /* Reading SC register and clear TOF bit */
        TPM_SC_REG(dev->regMap) |= TPM_SC_TOF_MASK;
        dev->callback();
        break;
    default:
        assert(0);
        break;
    }
}

void TPM0_IRQHandler (void)
{
    Ftm_callbackInterrupt(OB_FTM0);
}

void TPM1_IRQHandler (void)
{
    Ftm_callbackInterrupt(OB_FTM1);
}

void TPM2_IRQHandler (void)
{
    Ftm_callbackInterrupt(OB_FTM2);
}

static Ftm_Prescaler Ftm_computeFrequencyPrescale (Ftm_DeviceHandle dev, uint32_t timerFrequency)
{
    uint32_t clock;
    uint8_t prescaler;

    switch(Clock_getCurrentState())
    {
    case CLOCK_FBE:
    case CLOCK_FBI:
    case CLOCK_FEI:
    case CLOCK_FEE:
        clock = Clock_getFrequency(CLOCK_SYSTEM);
        break;

    case CLOCK_PBE:
    case CLOCK_PEE:
        clock = Clock_getFrequency(CLOCK_SYSTEM)/2;
        break;
    }

    if (dev->mode == FTM_MODE_INPUT_CAPTURE)
        prescaler = (clock / timerFrequency);
    else
        prescaler = (clock / timerFrequency) / 65536;

    if (prescaler > 64)
        return FTM_PRESCALER_128;
    else if (prescaler > 32)
        return FTM_PRESCALER_64;
    else if (prescaler > 16)
        return FTM_PRESCALER_32;
    else if (prescaler > 8)
        return FTM_PRESCALER_16;
    else if (prescaler > 4)
        return FTM_PRESCALER_8;
    else if (prescaler > 2)
        return FTM_PRESCALER_4;
    else if (prescaler > 1)
        return FTM_PRESCALER_2;
    else
        return FTM_PRESCALER_1;
}

static uint16_t Ftm_computeModulo (uint32_t timerFrequency, Ftm_Prescaler prescaler)
{
    uint32_t clock;
    uint32_t modulo;

    switch(Clock_getCurrentState())
    {
    case CLOCK_FBE:
    case CLOCK_FBI:
    case CLOCK_FEI:
    case CLOCK_FEE:
        clock = Clock_getFrequency(CLOCK_SYSTEM);
        break;

    case CLOCK_PBE:
    case CLOCK_PEE:
        clock = Clock_getFrequency(CLOCK_SYSTEM)/2;
        break;
    }

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
        return &TPM_CnV_REG(dev->regMap,0);
    case FTM_CHANNELS_CH1:
        return &TPM_CnV_REG(dev->regMap,1);
    case FTM_CHANNELS_CH2:
        return &TPM_CnV_REG(dev->regMap,2);
    case FTM_CHANNELS_CH3:
        return &TPM_CnV_REG(dev->regMap,3);
    case FTM_CHANNELS_CH4:
        return &TPM_CnV_REG(dev->regMap,4);
    case FTM_CHANNELS_CH5:
        return &TPM_CnV_REG(dev->regMap,5);
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
        return &TPM_CnSC_REG(dev->regMap,0);
    case FTM_CHANNELS_CH1:
        return &TPM_CnSC_REG(dev->regMap,1);
    case FTM_CHANNELS_CH2:
        return &TPM_CnSC_REG(dev->regMap,2);
    case FTM_CHANNELS_CH3:
        return &TPM_CnSC_REG(dev->regMap,3);
    case FTM_CHANNELS_CH4:
        return &TPM_CnSC_REG(dev->regMap,4);
    case FTM_CHANNELS_CH5:
        return &TPM_CnSC_REG(dev->regMap,5);
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
        *regCVPtr = Ftm_computeDutyValue(dutyScaled,TPM_MOD_REG(dev->regMap));
    }
}

void Ftm_init (Ftm_DeviceHandle dev, void *callback, Ftm_Config *config)
{
    Ftm_Prescaler prescaler;
    uint16_t modulo;

    uint8_t configPinIndex;
    uint8_t devPinIndex;

    /* Enable the clock to the selected FTM/TPM */
    *dev->simScgcPtr |= dev->simScgcBitEnable;
    SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);

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
    case FTM_MODE_PWM:
        /* Compute prescale factor */
        prescaler = Ftm_computeFrequencyPrescale(dev,config->timerFrequency);

        if (config->configurationBits & FTM_CONFIG_PWM_CENTER_ALIGNED)
        {
            dev->configurationBits = FTM_CONFIG_PWM_CENTER_ALIGNED;
            TPM_SC_REG(dev->regMap) |= TPM_SC_CPWMS_MASK;
        }
        else
        {
            dev->configurationBits = FTM_CONFIG_PWM_EDGE_ALIGNED;
            TPM_SC_REG(dev->regMap) &= ~TPM_SC_CPWMS_MASK;
        }

        /* Compute timer modulo and set it */
        modulo = Ftm_computeModulo(config->timerFrequency,prescaler);
        if (dev->configurationBits & FTM_CONFIG_PWM_CENTER_ALIGNED)
        {
            modulo = (modulo / 2) - 1;
        }
        else
        {
            modulo -= 1;
        }
        TPM_MOD_REG(dev->regMap) = modulo;

        /* Initialize every selected channels */
        for (configPinIndex = 0; configPinIndex < FTM_MAX_CHANNEL; ++configPinIndex)
        {
            Ftm_Pins pin = config->pins[configPinIndex];

            if (pin == FTM_PINS_STOP)
                break;

            Ftm_addPwmPin(dev,pin,config->duty[configPinIndex]);
        }

        TPM_SC_REG(dev->regMap) = TPM_SC_CMOD(1) | TPM_SC_PS(prescaler) | 0;

        break;
    case FTM_MODE_FREE:

        /* Compute prescale factor */
        prescaler = Ftm_computeFrequencyPrescale(dev,config->timerFrequency);

        /* Compute timer modulo */
        modulo = Ftm_computeModulo(config->timerFrequency,prescaler);

        TPM_CNT_REG(dev->regMap) = 0;
        TPM_MOD_REG(dev->regMap) = modulo - 1;
        TPM_SC_REG(dev->regMap) = TPM_SC_TOIE_MASK | TPM_SC_CMOD(1) |
                                  TPM_SC_PS(prescaler) | 0;
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

void Ftm_stopCount(Ftm_DeviceHandle dev)
{
    TPM_SC_REG(dev->regMap) &= TPM_SC_CMOD_MASK;
    TPM_SC_REG(dev->regMap) |= TPM_SC_CMOD(0);
}

void Ftm_startCount(Ftm_DeviceHandle dev)
{
    TPM_SC_REG(dev->regMap) &= TPM_SC_CMOD_MASK;
    TPM_SC_REG(dev->regMap) |= TPM_SC_CMOD(1);
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
    regCSCPtr = Ftm_getCnSCRegister(dev,dev->channel[devPinIndex]);
    regCVPtr = Ftm_getCnVRegister(dev,dev->channel[devPinIndex]);

    /* Enable channel and set PWM value */
    if (regCSCPtr && regCVPtr)
    {
        if (dev->configurationBits & FTM_CONFIG_PWM_CENTER_ALIGNED)
        {
            *regCSCPtr = TPM_CnSC_ELSB_MASK;
        }
        else
        {
            *regCSCPtr = TPM_CnSC_ELSB_MASK | TPM_CnSC_MSB_MASK;
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

#endif // LIBOHIBOARD_KL15Z4

#endif //  LIBOHIBOARD_FTM
