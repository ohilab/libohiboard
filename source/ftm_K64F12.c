/* Copyright (C) 2014-2015 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Alessio Paolucci <a.paolucci89@gmail.com>
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
 * @file libohiboard/source/ftm_K64F12.c
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @brief Ftm implementations for FRDMK64.
 */

#ifdef LIBOHIBOARD_FTM

#include "platforms.h"

#include "ftm.h"
#include "interrupt.h"
#include "clock.h"

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

#define FTM_MAX_PINS                     31

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
    FTM_MemMapPtr regMap;                          /**< Device memory pointer */
    
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


void Ftm_isrFtm0 (void);
void Ftm_isrFtm1 (void);
void Ftm_isrFtm2 (void);
void Ftm_isrFtm3 (void);

static Ftm_Device ftm0 = {
        .regMap           = FTM0_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_FTM0_MASK,

        .pins             = {FTM_PINS_PTA0,
                             FTM_PINS_PTA1,
                             FTM_PINS_PTA2,
                             FTM_PINS_PTA3,
                             FTM_PINS_PTA4,
                             FTM_PINS_PTA5,
                             FTM_PINS_PTC1,
                             FTM_PINS_PTC2,
                             FTM_PINS_PTC3,
                             FTM_PINS_PTC4,
                             FTM_PINS_PTC5,
                             FTM_PINS_PTD4,
                             FTM_PINS_PTD5,
                             FTM_PINS_PTD6,
                             FTM_PINS_PTD7,
        },
        .pinsPtr          = {&PORTA_PCR0,
                             &PORTA_PCR1,
                             &PORTA_PCR2,
                             &PORTA_PCR3,
                             &PORTA_PCR4,
                             &PORTA_PCR5,
                             &PORTC_PCR1,
                             &PORTC_PCR2,
                             &PORTC_PCR3,
                             &PORTC_PCR4,
                             &PORTC_PCR5,
                             &PORTD_PCR4,
                             &PORTD_PCR5,
                             &PORTD_PCR6,
                             &PORTD_PCR7,
        },
        .pinMux           = {3,
                             3,
                             3,
                             3,
                             3,
                             3,
                             4,
                             4,
                             4,
                             4,
                             5,
                             4,
                             4,
                             4,
                             4,
        },
        .channel          = {FTM_CHANNELS_CH5,
                             FTM_CHANNELS_CH6,
                             FTM_CHANNELS_CH7,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH2,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH2,
                             FTM_CHANNELS_CH3,
                             FTM_CHANNELS_CH2,
                             FTM_CHANNELS_CH4,
                             FTM_CHANNELS_CH5,
                             FTM_CHANNELS_CH6,
                             FTM_CHANNELS_CH7,
        },

        .isr              = Ftm_isrFtm0,
        .isrNumber        = INTERRUPT_FTM0,
        
        .devInitialized   = 0,
};
Ftm_DeviceHandle FTM0 = &ftm0; 

static Ftm_Device ftm1 = {
        .regMap           = FTM1_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_FTM1_MASK,

        .pins             = {FTM_PINS_PTA12,
                             FTM_PINS_PTA13,
                             FTM_PINS_PTB0,
                             FTM_PINS_PTB1,
        },
        .pinsPtr          = {&PORTA_PCR12,
                             &PORTA_PCR13,
                             &PORTB_PCR0,
                             &PORTB_PCR1,
        },
        .pinMux           = {3,
                             3,
                             3,
                             3,
        },
        .channel          = {FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
        },
        
        .isr              = Ftm_isrFtm1,
        .isrNumber        = INTERRUPT_FTM1,
        
        .devInitialized   = 0,
};
Ftm_DeviceHandle FTM1 = &ftm1;

static Ftm_Device ftm2 = {
        .regMap           = FTM2_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC3,
        .simScgcBitEnable = SIM_SCGC3_FTM2_MASK,

        .pins             = {FTM_PINS_PTB18,
                             FTM_PINS_PTB19,
        },
        .pinsPtr          = {&PORTB_PCR18,
                             &PORTB_PCR19,
        },
        .pinMux           = {3,
                             3,
        },
        .channel          = {FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
        },
        
        .isr              = Ftm_isrFtm2,
        .isrNumber        = INTERRUPT_FTM2,
        
        .devInitialized   = 0,
};
Ftm_DeviceHandle FTM2 = &ftm2;

static Ftm_Device ftm3 = {
        .regMap           = FTM3_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC3,
        .simScgcBitEnable = SIM_SCGC3_FTM3_MASK,

        .pins             = {FTM_PINS_PTE5,
        					 FTM_PINS_PTE6,
        					 FTM_PINS_PTC8,
        					 FTM_PINS_PTC9,
        					 FTM_PINS_PTC10,
        					 FTM_PINS_PTC11,
        					 FTM_PINS_PTD0,
        					 FTM_PINS_PTD1,
        					 FTM_PINS_PTD2,
        					 FTM_PINS_PTD3,
        },
        .pinsPtr          = {&PORTE_PCR5,
                             &PORTE_PCR6,
                             &PORTC_PCR8,
                             &PORTC_PCR9,
                             &PORTC_PCR10,
                             &PORTC_PCR11,
                             &PORTD_PCR0,
                             &PORTD_PCR1,
                             &PORTD_PCR2,
                             &PORTD_PCR3,
        },
        .pinMux           = {6,
                             6,
                             3,
                             3,
                             3,
                             3,
                             4,
                             4,
                             4,
                             4,
        },
        .channel          = {FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH4,
                             FTM_CHANNELS_CH5,
                             FTM_CHANNELS_CH6,
                             FTM_CHANNELS_CH7,
                             FTM_CHANNELS_CH0,
                             FTM_CHANNELS_CH1,
                             FTM_CHANNELS_CH2,
                             FTM_CHANNELS_CH3,
        },
        
        .isr              = Ftm_isrFtm3,
        .isrNumber        = INTERRUPT_FTM3,
        
        .devInitialized   = 0,
};
Ftm_DeviceHandle FTM3 = &ftm3;

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
        /* Reading SC register and clear TOF bit */
        FTM_SC_REG(dev->regMap) &= ~FTM_SC_TOF_MASK;
        dev->callback();
        break;
    default:
        assert(0);
        break;
    }
}

void Ftm_isrFtm0 (void)
{
    Ftm_callbackInterrupt(FTM0);
}

void Ftm_isrFtm1 (void)
{
    Ftm_callbackInterrupt(FTM1);    
}

void Ftm_isrFtm2 (void)
{
    Ftm_callbackInterrupt(FTM2);
}

void Ftm_isrFtm3 (void)
{
    Ftm_callbackInterrupt(FTM3);
}

static Ftm_Prescaler Ftm_computeFrequencyPrescale (uint32_t timerFrequency)
{
    uint32_t clock = Clock_getFrequency(CLOCK_BUS);
    uint8_t prescaler = (clock / timerFrequency) / 65536;
    
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
    uint32_t clock = Clock_getFrequency(CLOCK_BUS);
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

void Ftm_setPwm (Ftm_DeviceHandle dev, Ftm_Channels channel, uint16_t dutyScaled)
{
    volatile uint32_t* regCVPtr;
    
    switch (channel)
    {
    case FTM_CHANNELS_CH0:
        regCVPtr = &FTM_CnV_REG(dev->regMap,0);
        break;
    case FTM_CHANNELS_CH1:
        regCVPtr = &FTM_CnV_REG(dev->regMap,1);
        break;
    case FTM_CHANNELS_CH2:
        regCVPtr = &FTM_CnV_REG(dev->regMap,2);
        break;
    case FTM_CHANNELS_CH3:
        regCVPtr = &FTM_CnV_REG(dev->regMap,3);
        break;
    case FTM_CHANNELS_CH4:
        regCVPtr = &FTM_CnV_REG(dev->regMap,4);
        break;
    case FTM_CHANNELS_CH5:
        regCVPtr = &FTM_CnV_REG(dev->regMap,5);
        break;
    case FTM_CHANNELS_CH6:
        regCVPtr = &FTM_CnV_REG(dev->regMap,6);
        break;
    case FTM_CHANNELS_CH7:
        regCVPtr = &FTM_CnV_REG(dev->regMap,7);
        break;
    default:
        assert(0);
        regCVPtr = 0;
        break;
    }
    
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
    
    uint8_t configPinIndex;
    uint8_t devPinIndex;
    
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
        FTM_SC_REG(dev->regMap) = FTM_SC_TOIE_MASK | FTM_SC_CLKS(1) | 
                                  FTM_SC_PS(prescaler) | 0;
        break;
    }
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
    switch (dev->channel[devPinIndex])
    {
    case FTM_CHANNELS_CH0:
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,0);
        regCVPtr = &FTM_CnV_REG(dev->regMap,0);          
        break;
    case FTM_CHANNELS_CH1:
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,1);
        regCVPtr = &FTM_CnV_REG(dev->regMap,1);
        break;
    case FTM_CHANNELS_CH2:
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,2);
        regCVPtr = &FTM_CnV_REG(dev->regMap,2);
        break;
    case FTM_CHANNELS_CH3:
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,3);
        regCVPtr = &FTM_CnV_REG(dev->regMap,3);  
        break;
    case FTM_CHANNELS_CH4:
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,4);
        regCVPtr = &FTM_CnV_REG(dev->regMap,4);  
        break;
    case FTM_CHANNELS_CH5:
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,5);
        regCVPtr = &FTM_CnV_REG(dev->regMap,5);
        break;
    case FTM_CHANNELS_CH6:
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,6);
        regCVPtr = &FTM_CnV_REG(dev->regMap,6);
        break;
    case FTM_CHANNELS_CH7:
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,7);
        regCVPtr = &FTM_CnV_REG(dev->regMap,7);
        break;
    default:
        assert(0);
        regCSCPtr = 0;
        regCVPtr = 0;
        break;
    }

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
 
#endif /* LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F */

#endif // LIBOHIBOARD_FTM
