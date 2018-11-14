/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2014-2018 A. C. Open Hardware Ideas Lab
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
 */

/**
 * @file libohiboard/include/timer.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @author Simone Giacomucci <simone.giacomucci@gmail.com>
 * @brief Timer definitions and prototypes.
 */

#ifdef LIBOHIBOARD_TIMER

#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "system.h"
#include "gpio.h"

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Timer_DeviceState
{
    TIMER_DEVICESTATE_RESET,
    TIMER_DEVICESTATE_READY,
    TIMER_DEVICESTATE_BUSY,
    TIMER_DEVICESTATE_ERROR,

} Timer_DeviceState;

typedef enum _Timer_Mode
{
    TIMER_MODE_FREE,
    TIMER_MODE_PWM,
    TIMER_MODE_INPUT_CAPTURE,
    TIMER_MODE_OUTPUT_COMPARE,

#if defined (LIBOHIBOARD_NXP_KINETIS)

    FTM_MODE_QUADRATURE_DECODE,

#if defined (LIBOHIBOARD_FRDMK64F) || \
    defined (LIBOHIBOARD_K64F12)   || \
    defined (LIBOHIBOARD_KV31F12)  || \
    defined (LIBOHIBOARD_KV46F)    || \
    defined (LIBOHIBOARD_TRWKV46F)

    FTM_MODE_COMBINE,

#endif

// ENDIF: LIBOHIBOARD_NXP_KINETIS
#endif

} Timer_Mode;

#if defined (LIBOHIBOARD_STM32L4)

typedef enum _Timer_ClockSource
{
    TIMER_CLOCKSOURCE_INTERNAL,          /**< Internal clock selection CK_INT */
    TIMER_CLOCKSOURCE_INTERNAL_ITR0,           /**< Internal trigger input #0 */
    TIMER_CLOCKSOURCE_INTERNAL_ITR1,           /**< Internal trigger input #1 */
    TIMER_CLOCKSOURCE_INTERNAL_ITR2,           /**< Internal trigger input #2 */
    TIMER_CLOCKSOURCE_INTERNAL_ITR3,           /**< Internal trigger input #3 */
    TIMER_CLOCKSOURCE_EXTERNAL_MODE_1,                /**< External input pin */
    TIMER_CLOCKSOURCE_EXTERNAL_MODE_2,            /**< External trigger input */

} Timer_ClockSource;

typedef enum _Timer_ClockPolarity
{
    /** Inverted polarity for ETR clock source */
    TIMER_CLOCKPOLARITY_INVERTED,
    /** Non-inverted polarity for ETR clock source */
    TIMER_CLOCKPOLARITY_NON_INVERTED,
    TIMER_CLOCKPOLARITY_RISING,
    TIMER_CLOCKPOLARITY_FALLING,
    TIMER_CLOCKPOLARITY_BOTH_EDGE,

} Timer_ClockPolarity;


typedef enum _Timer_ClockPrescaler
{
    TIMER_CLOCKPRESCALER_1,
    TIMER_CLOCKPRESCALER_2,
    TIMER_CLOCKPRESCALER_4,
    TIMER_CLOCKPRESCALER_8,

} Timer_ClockPrescaler;

#endif

typedef enum _Timer_CounterMode
{
    TIMER_COUNTERMODE_UP,
    TIMER_COUNTERMODE_DOWN,

#if defined (LIBOHIBOARD_STM32L4)
    TIMER_COUNTERMODE_CENTER_ALIGNED_1,
    TIMER_COUNTERMODE_CENTER_ALIGNED_2,
    TIMER_COUNTERMODE_CENTER_ALIGNED_3,
#elif defined (LIBOHIBOARD_MKL)
    TIMER_COUTERMODE_CENTER_ALIGNED,
#endif

} Timer_CounterMode;

/**
 *
 */
typedef struct _Timer_Device* Timer_DeviceHandle;

/* Configuration bits */

#define FTM_CONFIG_PWM_POLARITY_HIGH     0x00
#define FTM_CONFIG_PWM_POLARITY_LOW      0x02

/* Configuration bits for Input Capture mode */
#define FTM_CONFIG_INPUT_RISING_EDGE	 0x04
#define FTM_CONFIG_INPUT_FALLING_EDGE	 0x08
#define FTM_CONFIG_INPUT_BOTH_EDGES		 0x10


#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/timer_STM32L4.h"

#else

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

#endif

typedef struct _Timer_Config
{
    Timer_Mode mode;                                /**< Modes of operations. */
    
    uint32_t modulo;             /**< The modulo value for the timer counter. */
    uint32_t prescaler;       /**< The prescaler value for the timer counter. */

    uint32_t timerFrequency;                            /**< Timer frequency. */
    
    uint8_t configurationBits;        /**< A useful variable to configure FTM */

    Timer_ClockSource clockSource;                 /**< Selected clock source */
#if defined (LIBOHIBOARD_STM32L4)
    Timer_ClockPolarity clockPolarity;             /**< Clock source polarity */
    Timer_ClockPrescaler clockPrescaler;          /**< Clock source prescaler */
    /**< Clock source input filter: must be between 0x0 and 0xF */
    uint32_t clockFilter;

    /**
     * During counter operation this causes the counter to be loaded from its
     * auto-reload register only at the next update event.
     */
    bool autoreload;
#endif

    void (* freeCounterCallback)(struct _Timer_Device *dev);

    /**< Define the counter type for a specific operational mode */
    Timer_CounterMode counterMode;

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

} Timer_Config;

/** @name Configuration functions
 *  Functions to initialize and de-initialize a Timer peripheral.
 */
///@{

/**
 * @brief Initialize the Timer according to the specified parameters
 * in the @ref Timer_Config and initialize the associated handle.
 *
 * @param[in] dev Timer device handle
 * @param[in] config Configuration parameters list.
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_init (Timer_DeviceHandle dev, Timer_Config *config);

/**
 * This function de-initialize the selected peripheral.
 *
 * @param[in] dev Timer device handle
 */
System_Errors Timer_deInit (Timer_DeviceHandle dev);

///@}

/** @name Free-Counter functions
 *  Functions to manage free counting of Timer peripheral.
 */
///@{

/**
 * This functions start free-counting. In case callback was defined,
 * enable relative interrupt.
 *
 * @param[in] dev Timer device handle
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_start (Timer_DeviceHandle dev);

/**
 * This functions stop free-counting and disable relative interrupt.
 *
 * @param[in] dev Timer device handle
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_stop (Timer_DeviceHandle dev);

///@}

/** @name PWM functions
 *  Functions to manage PWM pins of a Timer peripheral.
 */
///@{

typedef enum _Timer_OutputCompareMode
{
#if defined (LIBOHIBOARD_ST_STM32)

    TIMER_OUTPUTCOMPAREMODE_COUNTING,
    TIMER_OUTPUTCOMPAREMODE_ACTIVE_ON_MATCH,
    TIMER_OUTPUTCOMPAREMODE_INACTIVE_ON_MATCH,
    TIMER_OUTPUTCOMPAREMODE_TOGGLE,
    TIMER_OUTPUTCOMPAREMODE_FORCED_INACTIVE,
    TIMER_OUTPUTCOMPAREMODE_FORCED_ACTIVE,
    TIMER_OUTPUTCOMPAREMODE_PWM1,
    TIMER_OUTPUTCOMPAREMODE_PWM2,
    TIMER_OUTPUTCOMPAREMODE_RETRIGGERABLE_OPM1,
    TIMER_OUTPUTCOMPAREMODE_RETRIGGERABLE_OPM2,
    TIMER_OUTPUTCOMPAREMODE_COMBINED_PWM1,
    TIMER_OUTPUTCOMPAREMODE_COMBINED_PWM2,
    TIMER_OUTPUTCOMPAREMODE_ASYMMETRIC_PWM1,
    TIMER_OUTPUTCOMPAREMODE_ASYMMETRIC_PWM2,

#endif

} Timer_OutputCompareMode;

typedef struct _Timer_OutputCompareConfig
{
    Timer_OutputCompareMode mode;

    uint8_t duty;

    /**
     * This flag must be used to choice if accelerate the effect of an event
     * on the trigger in input on the CC output or not.
     * This flag acts only if the channel is configured in PWM1 or PWM2 mode.
     */
    bool fastMode;

    Gpio_Level polarity;
    Gpio_Level nPolarity;

    Gpio_Level idleState;
    Gpio_Level nIdleState;

} Timer_OutputCompareConfig;

/**
 * This function configure the selected pin to generate a PWM signal
 * with the frequency chose during initialization procedure with
 * @ref Timer_init and as duty-cicly the value into configuration list.
 *
 * @param[in] dev Timer device handle
 * @param[in] config Output compare configuration parameters list.
 * @param[in] pin Selected pin for the PWM output
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_configPwmPin (Timer_DeviceHandle dev,
                                  Timer_OutputCompareConfig* config,
                                  Timer_Pins pin);

/**
 * This function modify on-fly the duty cycle of PWM signal generated into
 * selected channel.
 *
 * @param[in] dev Timer device handle
 * @param[in] channel The output channel
 * @param[in] duty A number 0 and 100 for duty cycle
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_setPwmDuty (Timer_DeviceHandle dev,
                                Timer_Channels channel,
                                uint8_t duty);

/**
 * This function start PWM generation.
 *
 * @param[in] dev Timer device handle
 * @param[in] channel The output channel
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_startPwm (Timer_DeviceHandle dev, Timer_Channels channel);

/**
 * This function stop PWM generation.
 *
 * @param[in] dev Timer device handle
 * @param[in] channel The output channel
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_stopPwm (Timer_DeviceHandle dev, Timer_Channels channel);

///@}


//void Ftm_resetCounter (Ftm_DeviceHandle dev);
//
///* Valid only in free counter mode */
//void Ftm_enableInterrupt (Ftm_DeviceHandle dev);
//void Ftm_disableInterrupt (Ftm_DeviceHandle dev);
//
//void Ftm_startCount(Ftm_DeviceHandle dev);
//void Ftm_stopCount(Ftm_DeviceHandle dev);
//uint16_t Ftm_getModule(Ftm_DeviceHandle dev);
//
///* Set PWM */
//System_Errors Ftm_addPwmPin (Ftm_DeviceHandle dev, Ftm_Pins pin, uint16_t dutyScaled);
//void Ftm_setPwm (Ftm_DeviceHandle dev, Ftm_Channels channel, uint16_t dutyScaled);
//
///* Set Input Capture */
//System_Errors Ftm_addInputCapturePin (Ftm_DeviceHandle dev, Ftm_Pins pin, uint16_t configurations);
//
///* Channel function */
//void Ftm_enableChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel);
//void Ftm_disableChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel);
//bool Ftm_isChannelInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel);
//void Ftm_clearChannelFlagInterrupt (Ftm_DeviceHandle dev, Ftm_Channels channel);
//uint16_t Ftm_getChannelCount (Ftm_DeviceHandle dev, Ftm_Channels channel);
//
//uint8_t Ftm_ResetFault (Ftm_DeviceHandle dev);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_H

#endif // LIBOHIBOARD_TIMER
