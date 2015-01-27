/******************************************************************************
 * Copyright (C) 2014 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/ftm.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief FTM implementations.
 */

/**
 * TODO:
 * -> Aggiungere enable del clock dei pin direttamente da qui per i canali 
 *    che fanno il PWM!
 */

#ifdef LIBOHIBOARD_FTM

#include "ftm.h"

#if defined (FRDMKL25Z) || defined(MKL15Z4) || defined(FRDMKL05Z) || \
	defined (FRDMKL02Z) || defined(MKL02Z4) ||                       \
	defined (FRDMKL03Z) || defined(MKL03Z4) || defined (MK60DZ10) || \
	defined (MK10DZ10) || defined(MK10D10)

#include "interrupt.h"
#include "clock.h"

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

typedef struct Ftm_Device
{
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
    FTM_MemMapPtr regMap;                          /**< Device memory pointer */
#elif defined (FRDMKL25Z)
    TPM_MemMapPtr regMap;                          /**< Device memory pointer */
#endif
    
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

#elif defined (OHIBOARD_R1)

void Ftm_isrFtm0 (void);
void Ftm_isrFtm1 (void);
void Ftm_isrFtm2 (void);

#elif defined (FRDMKL25Z)

void Ftm_isrFtm0 (void);
void Ftm_isrFtm1 (void);
void Ftm_isrFtm2 (void);

static Ftm_Device ftm0 = {
        .regMap           = TPM0_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_TPM0_MASK,

        .pins             = {FTM_PINS_PTA4,
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
                             FTM_PINS_PTE29,
                             FTM_PINS_PTE30,
                             FTM_PINS_PTE31,
        },
        .pinsPtr          = {&PORTA_PCR4,
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
                             &PORTE_PCR29,
                             &PORTE_PCR30,
                             &PORTE_PCR31,
        },
        .pinMux           = {3,
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
        },
        .channel          = {FTM_CHANNELS_CH1,
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
                             FTM_CHANNELS_CH2,
                             FTM_CHANNELS_CH3,
                             FTM_CHANNELS_CH4,
        },

        .isr              = Ftm_isrFtm0,
        .isrNumber        = INTERRUPT_TPM0,
        
        .devInitialized   = 0,
};
Ftm_DeviceHandle FTM0 = &ftm0; 

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

        .isr              = Ftm_isrFtm1,
        .isrNumber        = INTERRUPT_TPM1,
        
        .devInitialized   = 0,
};
Ftm_DeviceHandle FTM1 = &ftm1; 

static Ftm_Device ftm2 = {
        .regMap           = TPM2_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_TPM2_MASK,

        .pins             = {FTM_PINS_PTA1,
                             FTM_PINS_PTA2,
                             FTM_PINS_PTB0,
                             FTM_PINS_PTB1,
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

        .isr              = Ftm_isrFtm2,
        .isrNumber        = INTERRUPT_TPM2,
        
        .devInitialized   = 0,
};
Ftm_DeviceHandle FTM2 = &ftm2; 

#elif defined(MK10D10)

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
                             FTM_PINS_PTA6,
                             FTM_PINS_PTA7,                             
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
                             &PORTA_PCR6,
                             &PORTA_PCR7,                            
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
                             FTM_CHANNELS_CH3,
                             FTM_CHANNELS_CH4,
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
        
        .devInitialized   = 0,
};
Ftm_DeviceHandle FTM0 = &ftm0; 

static Ftm_Device ftm1 = {
        .regMap           = FTM1_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_FTM1_MASK,

        .pins             = {FTM_PINS_PTA8,
                             FTM_PINS_PTA9,
                             FTM_PINS_PTA12,
                             FTM_PINS_PTA13,
                             FTM_PINS_PTB0,
                             FTM_PINS_PTB1,
        },
        .pinsPtr          = {&PORTA_PCR8,
                             &PORTA_PCR9,
                             &PORTA_PCR12,
                             &PORTA_PCR13,
                             &PORTB_PCR0,
                             &PORTB_PCR1,
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
        
        .isr              = Ftm_isrFtm1,
        .isrNumber        = INTERRUPT_FTM1,
        
        .devInitialized   = 0,
};
Ftm_DeviceHandle FTM1 = &ftm1;

static Ftm_Device ftm2 = {
        .regMap           = FTM2_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC3,
        .simScgcBitEnable = SIM_SCGC3_FTM2_MASK,

        .pins             = {FTM_PINS_PTA10,
                             FTM_PINS_PTA11,
                             FTM_PINS_PTB18,
                             FTM_PINS_PTB19,
        },
        .pinsPtr          = {&PORTA_PCR10,
                             &PORTA_PCR11,
                             &PORTB_PCR18,
                             &PORTB_PCR19,
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
        
        .isr              = Ftm_isrFtm2,
        .isrNumber        = INTERRUPT_FTM2,
        
        .devInitialized   = 0,
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
    case FTM_MODE_FREE:
        /* Reading SC register and clear TOF bit */
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        FTM_SC_REG(dev->regMap) &= ~FTM_SC_TOF_MASK;
#elif defined (FRDMKL25Z)
        TPM_SC_REG(dev->regMap) |= TPM_SC_TOF_MASK;
#endif
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
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCVPtr = &FTM_CnV_REG(dev->regMap,0);
#elif defined (FRDMKL25Z)
        regCVPtr = &TPM_CnV_REG(dev->regMap,0);
#endif
        break;
    case FTM_CHANNELS_CH1:
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCVPtr = &FTM_CnV_REG(dev->regMap,1);
#elif defined (FRDMKL25Z)
        regCVPtr = &TPM_CnV_REG(dev->regMap,1);
#endif
        break;
    case FTM_CHANNELS_CH2:
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCVPtr = &FTM_CnV_REG(dev->regMap,2);
#elif defined (FRDMKL25Z)
        regCVPtr = &TPM_CnV_REG(dev->regMap,2);
#endif
        break;
    case FTM_CHANNELS_CH3:
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCVPtr = &FTM_CnV_REG(dev->regMap,3);
#elif defined (FRDMKL25Z)
        regCVPtr = &TPM_CnV_REG(dev->regMap,3);
#endif
        break;
    case FTM_CHANNELS_CH4:
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCVPtr = &FTM_CnV_REG(dev->regMap,4);
#elif defined (FRDMKL25Z)
        regCVPtr = &TPM_CnV_REG(dev->regMap,4);
#endif
        break;
    case FTM_CHANNELS_CH5:
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCVPtr = &FTM_CnV_REG(dev->regMap,5);
#elif defined (FRDMKL25Z)
        regCVPtr = &TPM_CnV_REG(dev->regMap,5);
#endif
        break;
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
    case FTM_CHANNELS_CH6:
        regCVPtr = &FTM_CnV_REG(dev->regMap,6);
        break;
    case FTM_CHANNELS_CH7:
        regCVPtr = &FTM_CnV_REG(dev->regMap,7);
        break;
#endif
    default:
        assert(0);
        regCVPtr = 0;
        break;
    }
    
    if (regCVPtr)
    {
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        *regCVPtr = Ftm_computeDutyValue(dutyScaled,FTM_MOD_REG(dev->regMap));
#elif defined (FRDMKL25Z)
        *regCVPtr = Ftm_computeDutyValue(dutyScaled,TPM_MOD_REG(dev->regMap));        
#endif
    }
}

void Ftm_init (Ftm_DeviceHandle dev, void *callback, Ftm_Config *config)
{
    Ftm_Prescaler prescaler;
    uint16_t modulo;
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
    int16_t initCounter;
#endif
    
    uint8_t configPinIndex;
    uint8_t devPinIndex;
    
    /* Enable the clock to the selected FTM/TPM */ 
    *dev->simScgcPtr |= dev->simScgcBitEnable;
#if defined (FRDMKL25Z)
    SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);
#endif
    
    /* If call back exist save it */
    if (callback)
    {
        dev->callback = callback;
        /* Enable interrupt */
        Interrupt_enable(dev->isrNumber);
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        FTM_SC_REG(dev->regMap) |= FTM_SC_TOIE_MASK;
#elif defined (FRDMKL25Z)
        TPM_SC_REG(dev->regMap) |= TPM_SC_TOIE_MASK;
#endif
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
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
            FTM_SC_REG(dev->regMap) |= FTM_SC_CPWMS_MASK;
#elif defined (FRDMKL25Z)
            TPM_SC_REG(dev->regMap) |= TPM_SC_CPWMS_MASK;
#endif
        }
        else
        {
            dev->configurationBits = FTM_CONFIG_PWM_EDGE_ALIGNED;
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
            FTM_SC_REG(dev->regMap) &= ~FTM_SC_CPWMS_MASK;  
#elif defined (FRDMKL25Z)
            TPM_SC_REG(dev->regMap) &= ~TPM_SC_CPWMS_MASK;  
#endif
        }
        
        /* Compute timer modulo and set it */
        modulo = Ftm_computeModulo(config->timerFrequency,prescaler);
        if (dev->configurationBits & FTM_CONFIG_PWM_CENTER_ALIGNED)
        {
            modulo = (modulo / 2) - 1;
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
            initCounter = 0;
#endif
        }
        else
        {
            modulo -= 1;
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
            initCounter = 0;
#endif
        }
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        FTM_CNTIN_REG(dev->regMap) = FTM_CNTIN_INIT(initCounter);
        FTM_MOD_REG(dev->regMap) = modulo;
#elif defined (FRDMKL25Z)
        TPM_MOD_REG(dev->regMap) = modulo;
#endif

        /* Initialize every selected channels */
        for (configPinIndex = 0; configPinIndex < FTM_MAX_CHANNEL; ++configPinIndex)
        {
            Ftm_Pins pin = config->pins[configPinIndex];
            
            if (pin == FTM_PINS_STOP)
                break;
            
            Ftm_addPwmPin(dev,pin,config->duty[configPinIndex]);
        }

#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        FTM_SC_REG(dev->regMap) = FTM_SC_CLKS(1) | FTM_SC_PS(prescaler) | 0;
#elif defined (FRDMKL25Z)
        TPM_SC_REG(dev->regMap) = TPM_SC_CMOD(1) | TPM_SC_PS(prescaler) | 0;
#endif

        break;
    case FTM_MODE_FREE:
        
        /* Compute prescale factor */
        prescaler = Ftm_computeFrequencyPrescale(config->timerFrequency);
        
        /* Compute timer modulo */
        modulo = Ftm_computeModulo(config->timerFrequency,prescaler);
        
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        FTM_CNTIN_REG(dev->regMap) = FTM_CNTIN_INIT(config->initCounter);
        FTM_MOD_REG(dev->regMap) = modulo - 1;
        FTM_SC_REG(dev->regMap) = FTM_SC_TOIE_MASK | FTM_SC_CLKS(1) | 
                                  FTM_SC_PS(prescaler) | 0;
#elif defined(FRDMKL25Z)
        TPM_CNT_REG(dev->regMap) = 0;
        TPM_MOD_REG(dev->regMap) = modulo - 1;
        TPM_SC_REG(dev->regMap) = TPM_SC_TOIE_MASK | TPM_SC_CMOD(1) | 
                                  TPM_SC_PS(prescaler) | 0;
#endif
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
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,0);
        regCVPtr = &FTM_CnV_REG(dev->regMap,0);
#elif defined (FRDMKL25Z)
        regCSCPtr = &TPM_CnSC_REG(dev->regMap,0);
        regCVPtr = &TPM_CnV_REG(dev->regMap,0);
#endif                
        break;
    case FTM_CHANNELS_CH1:
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,1);
        regCVPtr = &FTM_CnV_REG(dev->regMap,1);
#elif defined (FRDMKL25Z)
        regCSCPtr = &TPM_CnSC_REG(dev->regMap,1);
        regCVPtr = &TPM_CnV_REG(dev->regMap,1);
#endif   
        break;
    case FTM_CHANNELS_CH2:
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,2);
        regCVPtr = &FTM_CnV_REG(dev->regMap,2);
#elif defined (FRDMKL25Z)
        regCSCPtr = &TPM_CnSC_REG(dev->regMap,2);
        regCVPtr = &TPM_CnV_REG(dev->regMap,2);
#endif   
        break;
    case FTM_CHANNELS_CH3:
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,3);
        regCVPtr = &FTM_CnV_REG(dev->regMap,3);
#elif defined (FRDMKL25Z)
        regCSCPtr = &TPM_CnSC_REG(dev->regMap,3);
        regCVPtr = &TPM_CnV_REG(dev->regMap,3);
#endif   
        break;
    case FTM_CHANNELS_CH4:
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,4);
        regCVPtr = &FTM_CnV_REG(dev->regMap,4);
#elif defined (FRDMKL25Z)
        regCSCPtr = &TPM_CnSC_REG(dev->regMap,4);
        regCVPtr = &TPM_CnV_REG(dev->regMap,4);
#endif   
        break;
    case FTM_CHANNELS_CH5:
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,5);
        regCVPtr = &FTM_CnV_REG(dev->regMap,5);
#elif defined (FRDMKL25Z)
        regCSCPtr = &TPM_CnSC_REG(dev->regMap,5);
        regCVPtr = &TPM_CnV_REG(dev->regMap,5);
#endif   
        break;
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
    case FTM_CHANNELS_CH6:
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,6);
        regCVPtr = &FTM_CnV_REG(dev->regMap,6);
        break;
    case FTM_CHANNELS_CH7:
        regCSCPtr = &FTM_CnSC_REG(dev->regMap,7);
        regCVPtr = &FTM_CnV_REG(dev->regMap,7);
        break;
#endif
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
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
            *regCSCPtr = FTM_CnSC_ELSB_MASK;
#elif defined (FRDMKL25Z)
            *regCSCPtr = TPM_CnSC_ELSB_MASK;
#endif
        }
        else
        {
#if defined (MK60DZ10) || defined (OHIBOARD_R1) || defined (MK10D10)
            *regCSCPtr = FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK;
#elif defined (FRDMKL25Z)
            *regCSCPtr = TPM_CnSC_ELSB_MASK | TPM_CnSC_MSB_MASK;
#endif                    
        }
        
        Ftm_setPwm (dev,dev->channel[devPinIndex],dutyScaled);
    }
    else
    {
        return ERRORS_FTM_CHANNEL_NOT_FOUND;
    }

    return ERRORS_FTM_OK;
}

#endif

#endif /*LIBOHIBOARD_FTM*/
