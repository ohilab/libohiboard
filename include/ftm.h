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
 * @file libohiboard/include/ftm.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief FTM definitions and prototypes.
 */

#ifndef __FTM_H
#define __FTM_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum 
{
    FTM_MODE_INPUT_CAPTURE,
    FTM_MODE_QUADRATURE_DECODE,
    FTM_MODE_OUTPUT_COMPARE,
    FTM_MODE_PWM,
    FTM_MODE_FREE
} Ftm_Mode;

typedef struct Ftm_Device* Ftm_DeviceHandle;

#if defined(MK60DZ10)

#define FTM_MAX_CHANNEL                  8

/* Configuration bits */
#define FTM_CONFIG_PWM_EDGE_ALIGNED      0x00
#define FTM_CONFIG_PWM_CENTER_ALIGNED    0x01

typedef enum
{
    FTM_PINS_PTA0,
    FTM_PINS_PTA1,
    FTM_PINS_PTA2,
    FTM_PINS_PTA3,
    FTM_PINS_PTA4,
    FTM_PINS_PTA5,
    FTM_PINS_PTA12,
    FTM_PINS_PTA13,
    FTM_PINS_PTB0,
    FTM_PINS_PTB1,
    FTM_PINS_PTB18,
    FTM_PINS_PTB19,
    FTM_PINS_PTC1,
    FTM_PINS_PTC2,
    FTM_PINS_PTC3,
    FTM_PINS_PTC4,
    FTM_PINS_PTD4,
    FTM_PINS_PTD5,
    FTM_PINS_PTD6,
    FTM_PINS_PTD7,
    FTM_PINS_STOP,
} Ftm_Pins;

typedef enum
{
    FTM_CHANNELS_CH0,
    FTM_CHANNELS_CH1,
    FTM_CHANNELS_CH2,
    FTM_CHANNELS_CH3,
    FTM_CHANNELS_CH4,
    FTM_CHANNELS_CH5,
    FTM_CHANNELS_CH6,
    FTM_CHANNELS_CH7,
} Ftm_Channels;


void Ftm_isrFtm0 (void);
void Ftm_isrFtm1 (void);
void Ftm_isrFtm2 (void);

extern Ftm_DeviceHandle FTM0;
extern Ftm_DeviceHandle FTM1;
extern Ftm_DeviceHandle FTM2;

#endif

typedef struct Ftm_Config
{
    Ftm_Mode mode;                                  /**< Modes of operations. */
    
    uint16_t modulo;             /**< The modulo value for the timer counter. */
    uint16_t initCounter;

    uint32_t timerFrequency;                            /**< Timer frequency. */
    
    Ftm_Pins pins[FTM_MAX_CHANNEL + 1];
    uint16_t duty[FTM_MAX_CHANNEL + 1];
    
    uint8_t configurationBits;        /**< A useful variable to configure FTM */
    
} Ftm_Config;

void Ftm_init (Ftm_DeviceHandle dev, void *callback, Ftm_Config *config);


#endif /* __FTM_H */
