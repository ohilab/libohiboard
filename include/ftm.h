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
 * @file libohiboard/include/ftm.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief FTM definitions and prototypes.
 */

#ifdef LIBOHIBOARD_FTM

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

/* Configuration bits */
#define FTM_CONFIG_PWM_EDGE_ALIGNED      0x00
#define FTM_CONFIG_PWM_CENTER_ALIGNED    0x01
#define FTM_CONFIG_PWM_POLARITY_HIGH     0x00
#define FTM_CONFIG_PWM_POLARITY_LOW      0x02

#if defined(MK60DZ10)

#define FTM_MAX_CHANNEL                  8

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

#elif defined (OHIBOARD_R1)

/* Da definire in base all'ohiboard!! */

#define FTM_MAX_CHANNEL                  8

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

#elif defined (FRDMKL25Z)

#define FTM_MAX_CHANNEL                  6

typedef enum
{
    //FTM_PINS_PTA0, Not connected on FRDM-KL25
    FTM_PINS_PTA1,
    FTM_PINS_PTA2,
    //FTM_PINS_PTA3, Not connected on FRDM-KL25
    FTM_PINS_PTA4,
    FTM_PINS_PTA5,
    FTM_PINS_PTA12,
    FTM_PINS_PTA13,

    FTM_PINS_PTB0,
    FTM_PINS_PTB1,
    FTM_PINS_PTB2,
    FTM_PINS_PTB3,
    FTM_PINS_PTB18,
    FTM_PINS_PTB19,
    
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
    
    FTM_PINS_PTE20,
    FTM_PINS_PTE21,
    FTM_PINS_PTE22,
    FTM_PINS_PTE23,
    //FTM_PINS_PTE24, Not connected on FRDM-KL25
    //FTM_PINS_PTE25, Not connected on FRDM-KL25
    FTM_PINS_PTE29,
    FTM_PINS_PTE30,
    FTM_PINS_PTE31,

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

void Ftm_setPwm (Ftm_DeviceHandle dev, Ftm_Channels channel, uint16_t dutyScaled);

#endif /* __FTM_H */

#endif /* LIBOHIBOARD_FTM */
