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

    uint32_t* channelPin[8]; /**< List of the pin number for the FTM channel. */
    uint8_t channelPinMux[8];         /**< Mux of the pin of the FTM channel. */
     
    void (*isr)(void);                     /**< The function pointer for ISR. */
    void (*callback)(uint32_t);  /**< The function pointer for user callback. */
    Interrupt_Vector isrNumber;                       /**< ISR vector number. */
    
    Ftm_Mode mode;                                  /**< Modes of operations. */
} Ftm_Device;

#if defined(MK60DZ10)

void Ftm_isrFtm0 (void)
{
    
}

void Ftm_isrFtm1 (void)
{
    
}

void Ftm_isrFtm2 (void)
{
    
}

static Ftm_Device ftm0 = {
        .regMap           = FTM0_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_FTM0_MASK,

        .isr              = Ftm_isrFtm0,
        .isrNumber        = INTERRUPT_FTM0,
};
Ftm_DeviceHandle FTM0 = &ftm0; 

static Ftm_Device ftm1 = {
        .regMap           = FTM1_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_FTM1_MASK,

        .isr              = Ftm_isrFtm1,
        .isrNumber        = INTERRUPT_FTM1,
};
Ftm_DeviceHandle FTM1 = &ftm1;

static Ftm_Device ftm2 = {
        .regMap           = FTM2_BASE_PTR,
        
        .simScgcPtr       = &SIM_SCGC3,
        .simScgcBitEnable = SIM_SCGC3_FTM2_MASK,

        .isr              = Ftm_isrFtm2,
        .isrNumber        = INTERRUPT_FTM2,
};
Ftm_DeviceHandle FTM2 = &ftm2;

#endif


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
    
}

void Ftm_init (Ftm_DeviceHandle dev, void *callback, Ftm_Config *config)
{
    Ftm_Prescaler prescaler;
    uint16_t modulo;
    
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
        break;
    case FTM_MODE_FREE:
        
        /* Compute prescale factor */
        prescaler = Ftm_computeFrequencyPrescale(config->timerFrequency);
        
        /* Compute timer modulo */
        modulo = Ftm_computeModulo(config->timerFrequency,prescaler);
        break;
    }
}
