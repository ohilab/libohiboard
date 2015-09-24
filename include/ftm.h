/******************************************************************************
 * Copyright (C) 2014-2015 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/include/ftm.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Matteo Civale <matteo.civale@gmail.com>
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

typedef enum
{
	FTM_CONFIGINPUTCAPTURE_RISING_EDGE,
	FTM_CONFIGINPUTCAPTURE_FALLING_EDGE,
	FTM_CONFIGINPUTCAPTURE_BOTH_EDGE,
} Ftm_ConfigInputCapture;

typedef struct Ftm_Device* Ftm_DeviceHandle;

/* Configuration bits */
#define FTM_CONFIG_PWM_EDGE_ALIGNED      0x00
#define FTM_CONFIG_PWM_CENTER_ALIGNED    0x01
#define FTM_CONFIG_PWM_POLARITY_HIGH     0x00
#define FTM_CONFIG_PWM_POLARITY_LOW      0x02

/* Configuration bits for Input Capture mode */
#define FTM_CONFIG_INPUT_RISING_EDGE	 0x04
#define FTM_CONFIG_INPUT_FALLING_EDGE	 0x08
#define FTM_CONFIG_INPUT_BOTH_EDGES		 0x10

#if defined (LIBOHIBOARD_KL15Z4)

#define FTM_MAX_CHANNEL   6

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
    FTM_PINS_PTE24,
    FTM_PINS_PTE25,
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

#elif defined (LIBOHIBOARD_KL25Z4)     || \
	defined (LIBOHIBOARD_FRDMKL25Z)

/* FIXME: Enable the KL25 into device on .c file! */

#define FTM_MAX_CHANNEL   6

typedef enum
{
#if defined (LIBOHIBOARD_KL25Z4)
    FTM_PINS_PTA0,
#endif
    FTM_PINS_PTA1,
    FTM_PINS_PTA2,
#if defined (LIBOHIBOARD_KL25Z4)
    FTM_PINS_PTA3,
#endif
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
#if defined (LIBOHIBOARD_KL25Z4)
    FTM_PINS_PTE24,
    FTM_PINS_PTE25,
#endif
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

#elif defined(LIBOHIBOARD_K10D10)

#define FTM_MAX_CHANNEL                  8

typedef enum
{
    FTM_PINS_PTA0,
    FTM_PINS_PTA1,
    FTM_PINS_PTA2,
    FTM_PINS_PTA3,
    FTM_PINS_PTA4,
    FTM_PINS_PTA5,
    FTM_PINS_PTA6,
    FTM_PINS_PTA7,
    FTM_PINS_PTA8,
    FTM_PINS_PTA9,
    FTM_PINS_PTA10,
    FTM_PINS_PTA11,
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

#elif defined (LIBOHIBOARD_K60DZ10) || \
	  defined (LIBOHIBOARD_OHIBOARD_R1)

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

#elif defined (LIBOHIBOARD_K64F12)     || \
	  defined (LIBOHIBOARD_FRDMK64F)

#define FTM_MAX_CHANNEL                  8

typedef enum
{
	FTM_PINS_PTE5,
	FTM_PINS_PTE6,
	
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
    FTM_PINS_PTC5,
    FTM_PINS_PTC8,
    FTM_PINS_PTC9,
    FTM_PINS_PTC10,
    FTM_PINS_PTC11,
    
    FTM_PINS_PTD0,
    FTM_PINS_PTD1,
    FTM_PINS_PTD2,
    FTM_PINS_PTD3,    
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
void Ftm_isrFtm3 (void);

extern Ftm_DeviceHandle FTM0;
extern Ftm_DeviceHandle FTM1;
extern Ftm_DeviceHandle FTM2;
extern Ftm_DeviceHandle FTM3;

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

void Ftm_resetCounter (Ftm_DeviceHandle dev);

/* Valid only in free counter mode */
void Ftm_startInterrupt (Ftm_DeviceHandle dev);
void Ftm_stopInterrupt (Ftm_DeviceHandle dev);

void Ftm_startCount(Ftm_DeviceHandle dev);
void Ftm_stopCount(Ftm_DeviceHandle dev);

/* Set PWM */
System_Errors Ftm_addPwmPin (Ftm_DeviceHandle dev, Ftm_Pins pin, uint16_t dutyScaled);
void Ftm_setPwm (Ftm_DeviceHandle dev, Ftm_Channels channel, uint16_t dutyScaled);

/* Set Input Capture */
System_Errors Ftm_addInputCapturePin (Ftm_DeviceHandle dev, Ftm_Pins pin);

#endif /* __FTM_H */

#endif /* LIBOHIBOARD_FTM */
