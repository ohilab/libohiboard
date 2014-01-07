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

typedef struct Ftm_Config
{
    Ftm_Mode mode;                                  /**< Modes of operations. */
    
    uint16_t modulo;             /**< The modulo value for the timer counter. */
    uint16_t initCounter;

    uint32_t timerFrequency;                            /**< Timer frequency. */
} Ftm_Config;

#if defined(MK60DZ10)

extern Ftm_DeviceHandle FTM0;
extern Ftm_DeviceHandle FTM1;
extern Ftm_DeviceHandle FTM2;

#endif

void Ftm_init (Ftm_DeviceHandle dev, void *callback, Ftm_Config *config);


#endif /* __FTM_H */
