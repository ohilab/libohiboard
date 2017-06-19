/******************************************************************************
 * Copyright (C) 2014-2017 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Matteo Civale <matteo.civale@gmail.com>
 *  Simone Giacomucci <simone.giacomucci@gmail.com>
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
 * @author Simone Giacomucci <simone.giacomucci@gmail.com>
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

#if defined (LIBOHIBOARD_FRDMK64F) || \
    defined (LIBOHIBOARD_K64F12)   || \
    defined (LIBOHIBOARD_KV31F12)  || \
    defined (LIBOHIBOARD_KV46F)    || \
    defined (LIBOHIBOARD_TRWKV46F)

    FTM_MODE_COMBINE,
#endif

    FTM_MODE_PWM,
    FTM_MODE_FREE
} Ftm_Mode;

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

void TPM0_IRQHandler (void);
void TPM1_IRQHandler (void);
void TPM2_IRQHandler (void);

extern Ftm_DeviceHandle OB_FTM0;
extern Ftm_DeviceHandle OB_FTM1;
extern Ftm_DeviceHandle OB_FTM2;

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

    FTM_CHANNELS_NONE,
} Ftm_Channels;

void TPM0_IRQHandler (void);
void TPM1_IRQHandler (void);
void TPM2_IRQHandler (void);

extern Ftm_DeviceHandle OB_FTM0;
extern Ftm_DeviceHandle OB_FTM1;
extern Ftm_DeviceHandle OB_FTM2;

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


void FTM0_IRQHandler (void);
void FTM1_IRQHandler (void);
void FTM2_IRQHandler (void);

extern Ftm_DeviceHandle OB_FTM0;
extern Ftm_DeviceHandle OB_FTM1;
extern Ftm_DeviceHandle OB_FTM2;

#elif defined(LIBOHIBOARD_K12D5)

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
    FTM_PINS_PTB12,
    FTM_PINS_PTB13,
    FTM_PINS_PTB18,
    FTM_PINS_PTB19,

    FTM_PINS_PTC1,
    FTM_PINS_PTC2,
    FTM_PINS_PTC3,
    FTM_PINS_PTC4,
    FTM_PINS_PTC5,

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


void FTM0_IRQHandler (void);
void FTM1_IRQHandler (void);
void FTM2_IRQHandler (void);

extern Ftm_DeviceHandle OB_FTM0;
extern Ftm_DeviceHandle OB_FTM1;
extern Ftm_DeviceHandle OB_FTM2;

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
	  defined (LIBOHIBOARD_FRDMK64F)   || \
	  defined(LIBOHIBOARD_KV31F12)     || \
	  defined (LIBOHIBOARD_KV46F)      || \
	  defined (LIBOHIBOARD_TRWKV46F)

#if defined (LIBOHIBOARD_K64F12)     || \
	defined (LIBOHIBOARD_FRDMK64F)

#define FTM_MAX_CHANNEL                  8
#define FTM_MAX_FAULT_CHANNEL            4

#elif defined (LIBOHIBOARD_KV31F12)

#define FTM_MAX_CHANNEL                  8
#define FTM_MAX_FAULT_CHANNEL            4

#elif  defined (LIBOHIBOARD_KV46F)      || \
       defined  (LIBOHIBOARD_TRWKV46F)

#define FTM_MAX_CHANNEL                  8
#define FTM_MAX_FAULT_CHANNEL            4

#endif

#if defined(LIBOHIBOARD_KV31F12)

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

    FTM_PINS_PTE5,
    FTM_PINS_PTE6,
    FTM_PINS_PTE24,
    FTM_PINS_PTE25,

    FTM_PINS_STOP,
} Ftm_Pins;

typedef enum
{
    FTM_FAULTPINS_PTA4,
    FTM_FAULTPINS_PTA18,
    FTM_FAULTPINS_PTA19,

    FTM_FAULTPINS_PTB1,
    FTM_FAULTPINS_PTB2,
    FTM_FAULTPINS_PTB3,
    FTM_FAULTPINS_PTB10,
    FTM_FAULTPINS_PTB11,

    FTM_FAULTPINS_PTC0,
    FTM_FAULTPINS_PTC9,
    FTM_FAULTPINS_PTC12,

    FTM_FAULTPINS_PTD6,
    FTM_FAULTPINS_PTD7,

    FTM_FAULTPINS_PTE16,

    FTM_FAULTPINS_STOP,
} Ftm_FaultPins;


#else

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
    FTM_PINS_PTA18,

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

    FTM_PINS_PTE20,
    FTM_PINS_PTE21,
    FTM_PINS_PTE24,
    FTM_PINS_PTE25,
    FTM_PINS_PTE29,
    FTM_PINS_PTE30,

    FTM_PINS_STOP,
} Ftm_Pins;

typedef enum
{
    FTM_FAULTPINS_PTA4,
    FTM_FAULTPINS_PTA18,
    FTM_FAULTPINS_PTA19,

    FTM_FAULTPINS_PTB1,
    FTM_FAULTPINS_PTB2,
    FTM_FAULTPINS_PTB3,
    FTM_FAULTPINS_PTB4,
    FTM_FAULTPINS_PTB5,
    FTM_FAULTPINS_PTB10,
    FTM_FAULTPINS_PTB11,

    FTM_FAULTPINS_PTC0,
    FTM_FAULTPINS_PTC3,
#if defined (LIBOHIBOARD_K64F12)
    FTM_FAULTPINS_PTC9,
#endif
    FTM_FAULTPINS_PTC12,

    FTM_FAULTPINS_PTD6,
    FTM_FAULTPINS_PTD7,
    FTM_FAULTPINS_PTD12,

    FTM_FAULTPINS_PTE16,

    FTM_FAULTPINS_STOP,
} Ftm_FaultPins;

#endif

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


typedef enum
{
    FTM_TRIGGER_CH0  = 4,
    FTM_TRIGGER_CH1  = 5,
    FTM_TRIGGER_CH2  = 0,
    FTM_TRIGGER_CH3  = 1,
    FTM_TRIGGER_CH4  = 2,
    FTM_TRIGGER_CH5  = 3,
    FTM_TRIGGER_NOCH = 6,

} Ftm_TriggerChannel;

typedef enum
{
    FTM_FAULTCHANNELS_0    = 0,
    FTM_FAULTCHANNELS_1    = 1,
    FTM_FAULTCHANNELS_2    = 2,
    FTM_FAULTCHANNELS_3    = 3,
    FTM_FAULTCHANNELS_NONE = 4,

} Ftm_FaultChannels;

typedef enum
{
    FTM_FAULTMODE_DISABLE     = 0,
    FTM_FAULTMODE_ONLYEVEN    = 1,
    FTM_FAULTMODE_MANUALCLEAR = 2,
    FTM_FAULTMODE_AUTOCLEAR   = 3,

} Ftm_FaultMode;

typedef enum
{
    FTM_FAULTPOLARITY_HIGH = 0,
    FTM_FAULTPOLARITY_LOW  = 1,

} Ftm_FaultPolarity;

/**
 * Synchronization Type (see RM at page 972)
 */
typedef enum
{
    FTM_SYNCEVENT_COUNT_MIN   = 0x01,
    FTM_SYNCEVENT_COUNT_MAX   = 0x02,
    FTM_SYNCEVENT_RE_INIT     = 0x04,
    FTM_SYNCEVENT_OUTPUT_MASK = 0x08,
    FTM_SYNCEVENT_TRIG0       = 0x10,
    FTM_SYNCEVENT_TRIG1       = 0x20,
    FTM_SYNCEVENT_TRIG2       = 0x40,
} Ftm_SyncEvent;

void FTM0_IRQHandler (void);
void FTM1_IRQHandler (void);
void FTM2_IRQHandler (void);
void FTM3_IRQHandler (void);

extern Ftm_DeviceHandle OB_FTM0;
extern Ftm_DeviceHandle OB_FTM1;
extern Ftm_DeviceHandle OB_FTM2;
extern Ftm_DeviceHandle OB_FTM3;

typedef enum
{
    FTM_COMBINECHANNELALIGN_NEGATIVE = 0,
    FTM_COMBINECHANNELALIGN_POSITIVE = 1,
} Ftm_CombineChannelAlign;

typedef enum
{
    FTM_COMBINECHANNELPAIR_0_1 = 0,
    FTM_COMBINECHANNELPAIR_2_3 = 1,
    FTM_COMBINECHANNELPAIR_4_5 = 2,
    FTM_COMBINECHANNELPAIR_6_7 = 3,
} Ftm_CombineChannelPair;

typedef enum
{
    FTM_COMBINERELOAD_NONE    = 0,
    FTM_COMBINERELOAD_CH_LOW  = 1,
    FTM_COMBINERELOAD_CH_HIGH = 2,
    FTM_COMBINERELOAD_BOTH    = 3,
} Ftm_CombineReload;

typedef struct _Ftm_CombineChannelConfig
{
    Ftm_CombineChannelPair pair;
    Ftm_CombineChannelAlign align;

    Ftm_CombineReload reload;

    bool enableDeadTime;
    bool enableSynchronization;
    bool enableComplementary;
    bool enableFaultInterrupt;

} Ftm_CombineChannelConfig;

typedef struct _Ftm_FaultConfig
{
    Ftm_FaultPins pin;
    bool enableFilter;
    Ftm_FaultPolarity polarity;

} Ftm_FaultConfig;

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

#if defined (LIBOHIBOARD_FRDMK64F) || \
    defined (LIBOHIBOARD_K64F12)   || \
    defined (LIBOHIBOARD_KV31F12)  || \
	defined (LIBOHIBOARD_KV46F)    || \
	defined (LIBOHIBOARD_TRWKV46F)

    /* Fault configurations */
    Ftm_FaultConfig fault[FTM_MAX_FAULT_CHANNEL];
    uint8_t faultFilterValue;
    Ftm_FaultMode faultMode;
    bool interruptEnableFault;

    Ftm_TriggerChannel triggerChannel;
    bool enableInitTrigger;
    Ftm_SyncEvent syncEvent;

    
    /* For Combine mode */
    Ftm_CombineChannelConfig channelPair[FTM_MAX_CHANNEL>>1];
    uint32_t deadTime;  /**< Only valid in combine or complementary mode [ns] */
    bool symmetrical;

#endif

} Ftm_Config;

void Ftm_init (Ftm_DeviceHandle dev, void *callback, Ftm_Config *config);

void Ftm_resetCounter (Ftm_DeviceHandle dev);

/* Valid only in free counter mode */
void Ftm_enableInterrupt (Ftm_DeviceHandle dev);
void Ftm_disableInterrupt (Ftm_DeviceHandle dev);

void Ftm_startCount(Ftm_DeviceHandle dev);
void Ftm_stopCount(Ftm_DeviceHandle dev);
uint16_t Ftm_getModule(Ftm_DeviceHandle dev);

/* Set PWM */
System_Errors Ftm_addPwmPin (Ftm_DeviceHandle dev, Ftm_Pins pin, uint16_t dutyScaled);
void Ftm_setPwm (Ftm_DeviceHandle dev, Ftm_Channels channel, uint16_t dutyScaled);

/* Set Input Capture */
System_Errors Ftm_addInputCapturePin (Ftm_DeviceHandle dev, Ftm_Pins pin, uint16_t configurations);

/* Channel function */
void Ftm_enableChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel);
void Ftm_disableChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel);
bool Ftm_isChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel);
void Ftm_clearChannelFlagInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel);
uint16_t Ftm_getChannelCount (Ftm_DeviceHandle dev, Ftm_Channels channel);

uint8_t Ftm_ResetFault (Ftm_DeviceHandle dev);

#endif /* __FTM_H */

#endif /* LIBOHIBOARD_FTM */
