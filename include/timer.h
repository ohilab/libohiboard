/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2014-2020 A. C. Open Hardware Ideas Lab
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

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_TIMER

/**
 * @defgroup TIMER TIMER
 * @brief TIMER HAL driver
 * @{
 */

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
 * @defgroup TIMER_Configuration_Types Timer configuration types
 * @brief Types for Timer configuration.
 * @{
 */

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

/**
 *
 */
typedef enum _Timer_Mode
{
    TIMER_MODE_FREE,
    TIMER_MODE_PWM,
    TIMER_MODE_INPUT_CAPTURE,
    TIMER_MODE_OUTPUT_COMPARE,

#if defined (LIBOHIBOARD_NXP_KINETIS)

    TIMER_MODE_QUADRATURE_DECODE,

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

/**
 * Clock source selection for Timer peripheral.
 *
 * @note The value is MCU-specific.
 */
typedef enum _Timer_ClockSource
{
#if defined (LIBOHIBOARD_STM32L0) || \
    defined (LIBOHIBOARD_STM32L4) || \
    defined (LIBOHIBOARD_STM32WB)

    TIMER_CLOCKSOURCE_INTERNAL,          /**< Internal clock selection CK_INT */
    TIMER_CLOCKSOURCE_INTERNAL_ITR0,           /**< Internal trigger input #0 */
    TIMER_CLOCKSOURCE_INTERNAL_ITR1,           /**< Internal trigger input #1 */
    TIMER_CLOCKSOURCE_INTERNAL_ITR2,           /**< Internal trigger input #2 */
    TIMER_CLOCKSOURCE_INTERNAL_ITR3,           /**< Internal trigger input #3 */
    TIMER_CLOCKSOURCE_EXTERNAL_MODE_1,                /**< External input pin */
    TIMER_CLOCKSOURCE_EXTERNAL_MODE_2,            /**< External trigger input */

#endif

#if defined (LIBOHIBOARD_NXP_KINETIS)
    TIMER_CLOCKSOURCE_INTERNAL,                 /**< Internal clock selection */
    TIMER_CLOCKSOURCE_MCG,
    TIMER_CLOCKSOURCE_OSCILLATOR,
#endif

#if defined (LIBOHIBOARD_PIC24FJ)

    /** Internal clock selection F_OSC/2. */
    TIMER_CLOCKSOURCE_INTERNAL = 0x0000,
    /** SOSC clock selection. */
    TIMER_CLOCKSOURCE_SOSC     = _T2CON_TCS_MASK,
    /** TyCK external clock input. */
    TIMER_CLOCKSOURCE_TyCK     = _T2CON_TCS_MASK | _T2CON_TECS0_MASK,
    /** LPRC oscillator clock selection. */
    TIMER_CLOCKSOURCE_LPRC_OSC = _T2CON_TCS_MASK | _T2CON_TECS1_MASK,
    /** TxCK generic timer external input. */
    TIMER_CLOCKSOURCE_TxCK     = _T2CON_TCS_MASK | _T2CON_TECS_MASK,

#endif

} Timer_ClockSource;

typedef enum _Timer_ClockPrescaler
{

#if defined (LIBOHIBOARD_NXP_KINETIS)
    TIMER_CLOCKPRESCALER_1   = 0,
    TIMER_CLOCKPRESCALER_2   = 1,
    TIMER_CLOCKPRESCALER_4   = 2,
    TIMER_CLOCKPRESCALER_8   = 3,
    TIMER_CLOCKPRESCALER_16  = 4,
    TIMER_CLOCKPRESCALER_32  = 5,
    TIMER_CLOCKPRESCALER_64  = 6,
    TIMER_CLOCKPRESCALER_128 = 7,
#endif

#if defined (LIBOHIBOARD_PIC24FJ)
    TIMER_CLOCKPRESCALER_1   = 0x0000,
    TIMER_CLOCKPRESCALER_8   = _T2CON_TCKPS0_MASK,
    TIMER_CLOCKPRESCALER_64  = _T2CON_TCKPS1_MASK,
    TIMER_CLOCKPRESCALER_256 = _T2CON_TCKPS0_MASK | _T2CON_TCKPS1_MASK,
#endif

    TIMER_CLOCKPRESCALER_NUMBER,

} Timer_ClockPrescaler;

#if defined (LIBOHIBOARD_STM32L0) || \
    defined (LIBOHIBOARD_STM32L4) || \
    defined (LIBOHIBOARD_STM32WB)

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

#endif

typedef enum _Timer_CounterMode
{
    TIMER_COUNTERMODE_UP,

#if !defined (LIBOHIBOARD_PIC24FJ)
    TIMER_COUNTERMODE_DOWN,
#endif

#if defined (LIBOHIBOARD_STM32L0) || \
    defined (LIBOHIBOARD_STM32L4) || \
    defined (LIBOHIBOARD_STM32WB)
    TIMER_COUNTERMODE_CENTER_ALIGNED_1,
    TIMER_COUNTERMODE_CENTER_ALIGNED_2,
    TIMER_COUNTERMODE_CENTER_ALIGNED_3,
#elif defined (LIBOHIBOARD_MKL)
    TIMER_COUNTERMODE_CENTER_ALIGNED,
    TIMER_COUNTERMODE_EDGE_ALIGNED,
#endif

} Timer_CounterMode;

/**
 * Useful enumeration used to detect the current channel that
 * generate an interrupt.
 */
typedef enum _Timer_ActiveChannels
{
    TIMER_ACTIVECHANNELS_NONE = 0x00u,

    TIMER_ACTIVECHANNELS_CH1  = 0x01u,
    TIMER_ACTIVECHANNELS_CH2  = 0x02u,
    TIMER_ACTIVECHANNELS_CH3  = 0x04u,
    TIMER_ACTIVECHANNELS_CH4  = 0x08u,
    TIMER_ACTIVECHANNELS_CH5  = 0x10u,
    TIMER_ACTIVECHANNELS_CH6  = 0x20u,
#if defined (LIBOHIBOARD_NXP_KINETIS)
    TIMER_ACTIVECHANNELS_CH7  = 0x40u,
    TIMER_ACTIVECHANNELS_CH8  = 0x80u,
#endif

} Timer_ActiveChannels;

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
#define FTM_CONFIG_INPUT_BOTH_EDGES      0x10


#if defined (LIBOHIBOARD_STM32L0)

#include "hardware/STM32L0/timer_STM32L0.h"

#elif defined (LIBOHIBOARD_STM32L4) || defined (LIBOHIBOARD_STM32WB)

#include "hardware/STM32L4/timer_STM32L4.h"

#elif defined (LIBOHIBOARD_MKL)

#include "hardware/NXPMKL/timer_MKL.h"

#elif defined (LIBOHIBOARD_PIC24FJ)

#include "hardware/PIC24FJ/timer_PIC24FJ.h"

#else

typedef enum _Timer_Channels
{
    TIMER_CHANNELS_NONE,
} Timer_Channels;

typedef enum _Timer_Pins
{
    TIMER_PINS_NONE,
} Timer_Pins;

#endif

typedef struct _Timer_Config
{
    Timer_Mode mode;                                /**< Modes of operations. */

    uint32_t modulo;             /**< The modulo value for the timer counter. */

#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    /**< A fixed prescaler value for the timer counter. */
    Timer_ClockPrescaler prescaler;
#else
    uint32_t prescaler;       /**< The prescaler value for the timer counter. */
#endif

    uint32_t timerFrequency;                            /**< Timer frequency. */

    Timer_ClockSource clockSource;                 /**< Selected clock source */

#if defined (LIBOHIBOARD_STM32L0) || \
    defined (LIBOHIBOARD_STM32L4) || \
    defined (LIBOHIBOARD_STM32WB)
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
    void (* pwmPulseFinishedCallback)(struct _Timer_Device *dev);
    void (* outputCompareCallback)(struct _Timer_Device *dev);
    void (* inputCaptureCallback)(struct _Timer_Device *dev);

    uint8_t isrPriority;

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

/**
 * @}
 */

/**
 * @defgroup TIMER_Configuration_Functions Timer configuration functions
 * @brief Functions to initialize and de-initialize a Timer peripheral.
 * @{
 */

/**
 * Initialize the Timer according to the specified parameters
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

/**
 * @}
 */

/**
 * @defgroup TIMER_Free_Counter TIMER Free Counter functions
 * @brief Functions to manage free counting of a Timer peripheral.
 * @{
 */

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

/**
 * Return the status of the selected timer.
 *
 * @param[in] dev Timer device handle
 * @return TRUE if timer is active and running, FALSE otherwise.
 */
bool Timer_isRunning (Timer_DeviceHandle dev);

/**
 * This function return the current timer counter.
 *
 * @param[in] dev Timer device handle
 * @return The current timer counter.
 */
uint32_t Timer_getCurrentCounter (Timer_DeviceHandle dev);

/**
 * TODO
 */
uint32_t Timer_getClockInputValue (Timer_DeviceHandle dev);

/**
 * This function change the prescaler value used to divide the internal
 * source clock.
 *
 * @warning This function doesn't use a value of @ref System_Errors for speed
 *          reason. So, no check on input parameters will be done!
 *
 * @param[in] dev Timer device handle
 * @param[in] prescaler The new clock prescaler for internal clock.
 */
void Timer_setPrescaler (Timer_DeviceHandle dev,
                         uint32_t prescaler);

void Timer_setCounter (Timer_DeviceHandle dev,
                       uint32_t counter);

/**
 * @}
 */

/**
 * @defgroup TIMER_PWM_Output_Compare TIMER PWM and Output Compare types and functions
 * @brief Functions to manage PWM and Output Compare pins of a Timer peripheral.
 * @{
 */

typedef enum _Timer_OutputCompareMode
{
#if defined (LIBOHIBOARD_ST_STM32)

    TIMER_OUTPUTCOMPAREMODE_COUNTING           = 0x00000000u,
    TIMER_OUTPUTCOMPAREMODE_ACTIVE_ON_MATCH    = TIM_CCMR1_OC1M_0,
    TIMER_OUTPUTCOMPAREMODE_INACTIVE_ON_MATCH  = TIM_CCMR1_OC1M_1,
    TIMER_OUTPUTCOMPAREMODE_TOGGLE             = (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1),
    TIMER_OUTPUTCOMPAREMODE_FORCED_INACTIVE    = TIM_CCMR1_OC1M_2,
    TIMER_OUTPUTCOMPAREMODE_FORCED_ACTIVE      = (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2),
    TIMER_OUTPUTCOMPAREMODE_PWM1               = (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2),
    TIMER_OUTPUTCOMPAREMODE_PWM2               = (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2),
#if defined (LIBOHIBOARD_STM32L4) || \
    defined (LIBOHIBOARD_STM32WB)
    TIMER_OUTPUTCOMPAREMODE_RETRIGGERABLE_OPM1 = TIM_CCMR1_OC1M_3,
    TIMER_OUTPUTCOMPAREMODE_RETRIGGERABLE_OPM2 = (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_3),
    TIMER_OUTPUTCOMPAREMODE_COMBINED_PWM1      = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_3),
    TIMER_OUTPUTCOMPAREMODE_COMBINED_PWM2      = (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_3),
    TIMER_OUTPUTCOMPAREMODE_ASYMMETRIC_PWM1    = (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_3),
    TIMER_OUTPUTCOMPAREMODE_ASYMMETRIC_PWM2    = TIM_CCMR1_OC1M,
#endif

#elif defined (LIBOHIBOARD_MICROCHIP_PIC) 
    
    TIMER_OUTPUTCOMPAREMODE_PWM,

#endif

    TIMER_OUTPUTCOMPAREMODE_NONE,

} Timer_OutputCompareMode;

typedef struct _Timer_OutputCompareConfig
{
    Timer_OutputCompareMode mode;
    
    /**
     * This field is used only in some architecture because there isn't a direct 
     * correlation between pin and channel.
     */
    Timer_Channels channel;

    /**
     * The duty-cycle usable in PWM mode.
     * Must be between 0 and 100.
     */
    uint8_t duty;
    
    /**
     * This value is frequency of the output signal.
     * 
     * @note It is used only into Microchip architecture. 
     */
    uint32_t frequency;

    /**
     * This value is the compare value in OC or OPM.
     * Must be between 0x0000 and 0xFFFF.
     */
    uint32_t pulse;

    /**
     * This flag must be used to choice if accelerate the effect of an event
     * on the trigger in input on the CC output or not.
     * 
     * @note This flag acts only if the channel is configured in PWM1 or PWM2 mode.
     * @note It is used only into STM32 architecture. 
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

/**
 * This function configure the selected pin to work in Output Compare mode.
 *
 * @param[in] dev Timer device handle
 * @param[in] config Output compare configuration parameters list.
 * @param[in] pin Selected pin for the OC output
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_configOutputComparePin (Timer_DeviceHandle dev,
                                            Timer_OutputCompareConfig* config,
                                            Timer_Pins pin);

/**
 * This function start Output Compare on selected channel.
 *
 * @param[in] dev Timer device handle
 * @param[in] channel The output channel
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_startOutputCompare (Timer_DeviceHandle dev, Timer_Channels channel);

/**
 * This function stop Output Compare on selected channel.
 *
 * @param[in] dev Timer device handle
 * @param[in] channel The output channel
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_stopOutputCompare (Timer_DeviceHandle dev, Timer_Channels channel);

/**
 * @}
 */ 

/**
 * @defgroup TIMER_Input_Capture TIMER Input Capture types and functions
 * @brief Functions to manage Input Capture pins of a Timer peripheral.
 * @{
 */

typedef enum _Timer_InputCapturePolarity
{
#if defined (LIBOHIBOARD_ST_STM32)

    TIMER_INPUTCAPTUREPOLARITY_RISING  = 0x00000000u,
    TIMER_INPUTCAPTUREPOLARITY_FALLING = TIM_CCER_CC1P,
    TIMER_INPUTCAPTUREPOLARITY_BOTH    = (TIM_CCER_CC1P | TIM_CCER_CC1NP),

#endif

    TIMER_INPUTCAPTUREPOLARITY_NONE,

} Timer_InputCapturePolarity;

typedef enum _Timer_InputCapturePrescaler
{
#if defined (LIBOHIBOARD_ST_STM32)

    TIMER_INPUTCAPTUREPRESCALER_DIV1  = 0x00000000u,
    TIMER_INPUTCAPTUREPRESCALER_DIV2  = TIM_CCMR1_IC1PSC_0,
    TIMER_INPUTCAPTUREPRESCALER_DIV4  = TIM_CCMR1_IC1PSC_1,
    TIMER_INPUTCAPTUREPRESCALER_DIV8  = TIM_CCMR1_IC1PSC,
#endif

    TIMER_INPUTCAPTUREPRESCALER_NONE,

} Timer_InputCapturePrescaler;

typedef enum _Timer_InputCaptureSelection
{
#if defined (LIBOHIBOARD_ST_STM32)

    TIMER_INPUTCAPTURESELECTION_DIRECT   = TIM_CCMR1_CC1S_0,
    TIMER_INPUTCAPTURESELECTION_INDIRECT = TIM_CCMR1_CC1S_1,
    TIMER_INPUTCAPTURESELECTION_TRC      = TIM_CCMR1_CC1S,
#endif

    TIMER_INPUTCAPTURESELECTION_NONE,

} Timer_InputCaptureSelection;


typedef struct _Timer_InputCaptureConfig
{
    Timer_InputCapturePolarity polarity;

    Timer_InputCapturePrescaler prescaler;

    Timer_InputCaptureSelection selection;

    /**
     * The input capture filter. The number must be between 0x0 and 0xF.
     *   - 0000: No filter, sampling is done at fDTS
     *   - 0001: fSAMPLING=fCK_INT, N=2
     *   - 0010: fSAMPLING=fCK_INT, N=4
     *   - 0011: fSAMPLING=fCK_INT, N=8
     *   - 0100: fSAMPLING=fDTS/2, N=6
     *   - 0101: fSAMPLING=fDTS/2, N=8
     *   - 0110: fSAMPLING=fDTS/4, N=6
     *   - 0111: fSAMPLING=fDTS/4, N=8
     *   - 1000: fSAMPLING=fDTS/8, N=6
     *   - 1001: fSAMPLING=fDTS/8, N=8
     *   - 1010: fSAMPLING=fDTS/16, N=5
     *   - 1011: fSAMPLING=fDTS/16, N=6
     *   - 1100: fSAMPLING=fDTS/16, N=8
     *   - 1101: fSAMPLING=fDTS/32, N=5
     *   - 1110: fSAMPLING=fDTS/32, N=6
     *   - 1111: fSAMPLING=fDTS/32, N=8
     */
    uint8_t filter;

} Timer_InputCaptureConfig;

/**
 * This function configure the selected pin to work in Input Capture mode.
 *
 * @param[in] dev Timer device handle
 * @param[in] config Input capture configuration parameters list.
 * @param[in] pin Selected pin for the IC input
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_configInputCapturePin (Timer_DeviceHandle dev,
                                           Timer_InputCaptureConfig* config,
                                           Timer_Pins pin);

/**
 * This function start Input Capture on selected channel.
 *
 * @param[in] dev Timer device handle
 * @param[in] channel The input channel
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_startInputCapture (Timer_DeviceHandle dev, Timer_Channels channel);

/**
 * This function stop Input Capture on selected channel.
 *
 * @param[in] dev Timer device handle
 * @param[in] channel The input channel
 * @return ERRORS_NO_ERROR The initialization is ok.
 */
System_Errors Timer_stopInputCapture (Timer_DeviceHandle dev, Timer_Channels channel);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __TIMER_H

/**
 * @}
 */

#endif // LIBOHIBOARD_TIMER

/**
 * @}
 */
