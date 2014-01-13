/******************************************************************************
 * Copyright (C) 2014 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  
 * Project: libohiboard
 * Package: FTM
 * Version: 0.0
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>
 ******************************************************************************/

/**
 * @file libohiboard/source/ftm.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief FTM implementations.
 */

#include "ftm.h"

#include "interrupt.h"
#include "clock.h"

#include "board.h"

#define FTM_MAX_PINS                     20

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

#if defined(MK60DZ10)


#endif

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
    void (*callback)(void);  /**< The function pointer for user callback. */
    Interrupt_Vector isrNumber;                       /**< ISR vector number. */
    
    Ftm_Mode mode;                                  /**< Modes of operations. */
    
    uint8_t configurationBits;        /**< A useful variable to configure FTM */
} Ftm_Device;

#if defined(MK60DZ10)

void Ftm_isrFtm0 (void);
void Ftm_isrFtm1 (void);
void Ftm_isrFtm2 (void);

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
                             FTM_CHANNELS_CH4,
                             FTM_CHANNELS_CH5,
                             FTM_CHANNELS_CH6,
                             FTM_CHANNELS_CH7,
        },

        .isr              = Ftm_isrFtm0,
        .isrNumber        = INTERRUPT_FTM0,
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
};
Ftm_DeviceHandle FTM2 = &ftm2;

#endif

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
        break;
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

void Ftm_init (Ftm_DeviceHandle dev, void *callback, Ftm_Config *config)
{
    Ftm_Prescaler prescaler;
    uint16_t modulo;
    int16_t initCounter;
    
    uint8_t channelIndex;
    uint8_t pinIndex;
    
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
        
        /* Compute timer modulo */
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
        
        for (channelIndex = 0; channelIndex < FTM_MAX_CHANNEL; ++channelIndex)
        {
            volatile uint32_t* regCCSPtr;
            volatile uint32_t* regCVPtr;
            
            if (config->pins[channelIndex] == FTM_PINS_STOP)
                break;
            
            for (pinIndex = 0; pinIndex < FTM_MAX_PINS; ++pinIndex)
            {
                if (dev->pins[pinIndex] == config->pins[channelIndex])
                {
                    *(dev->pinsPtr[pinIndex]) = 
                       PORT_PCR_MUX(dev->pinMux[pinIndex]) | PORT_PCR_IRQC(0);
                    break;
                }
            }
            
            switch (dev->channel[channelIndex])
            {
            case FTM_CHANNELS_CH0:
                regCCSPtr = &FTM_CnSC_REG(dev->regMap,0);
                regCVPtr = &FTM_CnV_REG(dev->regMap,0);
                break;
            case FTM_CHANNELS_CH1:
                regCCSPtr = &FTM_CnSC_REG(dev->regMap,1);
                regCVPtr = &FTM_CnV_REG(dev->regMap,1);
                break;
            case FTM_CHANNELS_CH2:
                regCCSPtr = &FTM_CnSC_REG(dev->regMap,2);
                regCVPtr = &FTM_CnV_REG(dev->regMap,2);
                break;
            case FTM_CHANNELS_CH3:
                regCCSPtr = &FTM_CnSC_REG(dev->regMap,3);
                regCVPtr = &FTM_CnV_REG(dev->regMap,3);
                break;
            case FTM_CHANNELS_CH4:
                regCCSPtr = &FTM_CnSC_REG(dev->regMap,4);
                regCVPtr = &FTM_CnV_REG(dev->regMap,4);
                break;
            case FTM_CHANNELS_CH5:
                regCCSPtr = &FTM_CnSC_REG(dev->regMap,5);
                regCVPtr = &FTM_CnV_REG(dev->regMap,5);
                break;
            case FTM_CHANNELS_CH6:
                regCCSPtr = &FTM_CnSC_REG(dev->regMap,6);
                regCVPtr = &FTM_CnV_REG(dev->regMap,6);
                break;
            case FTM_CHANNELS_CH7:
                regCCSPtr = &FTM_CnSC_REG(dev->regMap,7);
                regCVPtr = &FTM_CnV_REG(dev->regMap,7);
                break;
            default:
                assert(0);
                break;
            }

        }
        
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
