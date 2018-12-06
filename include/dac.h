/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2014-2018 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Francesco Piunti <francesco.piunti89@gmail.com>
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
 */

/**
 * @file libohiboard/include/dac.h
 * @author Francesco Piunti <francesco.piunti89@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief DAC definitions and prototypes.
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_DAC

/**
 * @defgroup DAC DAC
 * @brief DAC HAL driver
 * @{
 */

#ifndef __DAC_H
#define __DAC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "utility.h"

#ifdef LIBOHIBOARD_DMA
#include "dma.h"
#endif

/**
 * @defgroup DAC_Configuration_Params DAC configuration types
 * @{
 */

/**
 * Device handle for DAC peripheral.
 */
typedef struct _Dac_Device* Dac_DeviceHandle;

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Dac_DeviceState
{
    DAC_DEVICESTATE_RESET,
    DAC_DEVICESTATE_READY,
    DAC_DEVICESTATE_BUSY,
    DAC_DEVICESTATE_ERROR,

} Dac_DeviceState;

#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/dac_STM32L4.h"

#else

typedef enum {
    DAC_VOLTAGEREF_VDDA,
    DAC_VOLTAGEREF_VOUT
} Dac_VoltageRef;

typedef enum 
{
    DAC_TRIGGER_HARDWARE = 0x00,
    DAC_TRIGGER_SOFTWARE = 0x01,

} Dac_TriggerSelect;

typedef enum 
{
    DAC_POWERMODE_LOW,
    DAC_POWERMODE_HIGH
} Dac_PowerMode;

typedef enum 
{
    DAC_BUFFERMODE_NORMAL  = 0x0,
    DAC_BUFFERMODE_SWING   = 0x1,
    DAC_BUFFERMODE_ONETIME = 0x2,

#if defined (LIBOHIBOARD_KV31F12)  || \
    defined (LIBOHIBOARD_KV46F)    || \
    defined (LIBOHIBOARD_TRWKV46F) || \
    defined (LIBOHIBOARD_K64F12)   || \
    defined (LIBOHIBOARD_FRDMK64F)

    DAC_BUFFERMODE_FIFO    = 0x3,
#endif
    DAC_BUFFERMODE_OFF     = 0x4,

} Dac_BufferMode;

#if defined (LIBOHIBOARD_KV46F) || \
    defined (LIBOHIBOARD_TRWKV46F)

typedef struct
{
    uint8_t intTopEn      :1;
    uint8_t intBottmEn    :1;
    uint8_t intWaterMark  :1;
    uint8_t watermarkVale :2;

} Dac_InterruptEvent;

#else

typedef enum
{
	DAC_INTERRUPTEVENT_NO_EVENT,
	DAC_INTERRUPTEVENT_TOP,
	DAC_INTERRUPTEVENT_BOTTOM,
	DAC_INTERRUPTEVENT_BOOTH,

} Dac_InterruptEvent;
#endif

typedef struct Dac_Device* Dac_DeviceHandle;

#if defined (LIBOHIBOARD_KL25Z4)     || \
    defined (LIBOHIBOARD_FRDMKL25Z)

extern Dac_DeviceHandle OB_DAC0;

#elif defined (LIBOHIBOARD_K10D10)

extern Dac_DeviceHandle DAC0;
extern Dac_DeviceHandle DAC1;

#elif defined (LIBOHIBOARD_K12D5)

extern Dac_DeviceHandle OB_DAC0;

#elif defined (LIBOHIBOARD_KV31F12)

extern Dac_DeviceHandle OB_DAC0;
extern Dac_DeviceHandle OB_DAC1;

#elif defined (LIBOHIBOARD_KV46F) || \
    defined (LIBOHIBOARD_TRWKV46F)

extern Dac_DeviceHandle OB_DAC0;

typedef enum{
    DAC_HARDSYNC_XBARA      = 0x0,
    DAC_HARDSYNC_PDB_BOOTH  = 0x1,
    DAC_HARDSYNC_PDB0       = 0x2,
    DAC_HARDSYNC_PDB1       = 0x3,
}Dac_HardSyncSelect;

#elif defined (LIBOHIBOARD_K60DZ10) || \
      defined (LIBOHIBOARD_OHIBOARD_R1)

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

extern Dac_DeviceHandle OB_DAC0;
extern Dac_DeviceHandle OB_DAC1;

#endif

#endif

/**
 * The list of possible DAC resolution.
 */
typedef enum _Dac_Resolution
{
#if defined (LIBOHIBOARD_MKL)     || \
    defined (LIBOHIBOARD_MK)

    // TODO

#endif
#if defined (LIBOHIBOARD_STM32L4)
    DAC_RESOLUTION_12BIT = 0x00000000u,
    DAC_RESOLUTION_8BIT  = ADC_CFGR_RES_1,
#endif
} Dac_Resolution;

#if defined (LIBOHIBOARD_ST_STM32)

/**
 * List of possible alignment of the data to be written into registers.
 */
typedef enum _Dac_DataAlign
{
    DAC_DATAALIGN_12BIT_RIGHT = 0x00000000u,
    DAC_DATAALIGN_12BIT_LEFT  = 0x00000004u,
    DAC_DATAALIGN_8BIT_RIGHT  = 0x00000008u,
} Dac_DataAlign;

/**
 * List of possible events to trigger conversion.
 */
typedef enum _Dac_Trigger
{
    /** Conversion start automatically after value has been loaded. */
    DAC_TRIGGER_NONE         = (0x00000000u),

#if defined (LIBOHIBOARD_STM32L431) || \
    defined (LIBOHIBOARD_STM32L432) || \
    defined (LIBOHIBOARD_STM32L433) || \
    defined (LIBOHIBOARD_STM32L442) || \
    defined (LIBOHIBOARD_STM32L443)

    // TODO

#elif defined (LIBOHIBOARD_STM32L451) || \
	  defined (LIBOHIBOARD_STM32L452) || \
	  defined (LIBOHIBOARD_STM32L462)

    // TODO

#elif defined(LIBOHIBOARD_STM32L471) || \
      defined(LIBOHIBOARD_STM32L475) || \
      defined(LIBOHIBOARD_STM32L476) || \
      defined(LIBOHIBOARD_STM32L485) || \
      defined(LIBOHIBOARD_STM32L486) || \
      defined(LIBOHIBOARD_STM32L496) || \
      defined(LIBOHIBOARD_STM32L4A6)

    DAC_TRIGGER_TIM6_TRGO    = (DAC_CR_TEN1),
    DAC_TRIGGER_TIM8_TRGO    = (DAC_CR_TEN1 | DAC_CR_TSEL1_0),
    DAC_TRIGGER_TIM7_TRGO    = (DAC_CR_TEN1 | DAC_CR_TSEL1_1),
    DAC_TRIGGER_TIM5_TRGO    = (DAC_CR_TEN1 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0),
    DAC_TRIGGER_TIM2_TRGO    = (DAC_CR_TEN1 | DAC_CR_TSEL1_2),
    DAC_TRIGGER_TIM4_TRGO    = (DAC_CR_TEN1 | DAC_CR_TSEL1_2 | DAC_CR_TSEL1_0),
    DAC_TRIGGER_EXTI_9       = (DAC_CR_TEN1 | DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1),
    DAC_TRIGGER_SOFTWARE     = (DAC_CR_TEN1 | DAC_CR_TSEL1),

#elif defined (LIBOHIBOARD_STM32L4R5) || \
      defined (LIBOHIBOARD_STM32L4R7) || \
      defined (LIBOHIBOARD_STM32L4R9) || \
      defined (LIBOHIBOARD_STM32L4S5) || \
      defined (LIBOHIBOARD_STM32L4S7) || \
      defined (LIBOHIBOARD_STM32L4S9)

    // TODO

#endif

} Dac_Trigger;

#endif

/**
 *
 */
typedef struct _Dac_Config
{
#if defined (LIBOHIBOARD_NXP_KINETIS)
    Dac_VoltageRef ref;
    
    Dac_PowerMode powerMode;
    
    Dac_TriggerSelect trigger;
    Dac_BufferMode buffer;

    bool dmaEnable;
    Dac_InterruptEvent interruptEvent;

#if defined (LIBOHIBOARD_KV46F) || \
    defined (LIBOHIBOARD_TRWKV46F)

    Dac_HardSyncSelect hardSyncSel;

#endif

    void (*intisr)(void);

#elif defined (LIBOHIBOARD_ST_STM32)

    uint32_t empty;    /**< Do NOT use this parameter! Only for compatibility */

#endif
} Dac_Config;

/**
 * @}
 */

/**
 * @defgroup DAC_Configuration_Functions DAC configuration functions
 * @brief Functions to initialize and de-initialize a DAC peripheral.
 * @{
 */

/**
 * This function initialize the Dac device and setup operational mode according
 * to the specified parameters in the @ref Dac_Config
 *
 * @note STM32: do not use the config parameter because every channel was configured into
 *       @ref Dac_configPin with specific parameters. Put 0 into the field or an
 *       empty element of @ref Dac_Config. Anyway, in this case, the parameter will
 *       be ignored.
 *
 * @param[in] dev Dac device handle
 * @param[in] config A pointer to configuration object
 * @return A System_Errors elements that indicate the status of initialization.
 */
System_Errors Dac_init (Dac_DeviceHandle dev, Dac_Config *config);

/**
 * This function de-initialize the selected peripheral.
 *
 * @param[in] dev Dac device handle
 */
System_Errors Dac_deInit (Dac_DeviceHandle dev);

/**
 * @}
 */

/**
 * @defgroup DAC_Write_Functions DAC write functions
 * @brief Functions to configure and write data to pins.
 * @{
 */

/**
 * Configuration parameter list for a specific DAC channel.
 */
typedef struct _Dac_ChannelConfig
{
    Dac_Resolution resolution;                           /**< DAC resolutions */

    /**
     * Select the software/external event source used to trigger DAC conversion.
     */
    Dac_Trigger trigger;

    /**
     * Specifies the conversion mode.
     * @ref UTILITY_STATE_DISABLE normal mode, otherwise with
     * @ref UTILITY_STATE_ENABLE enable the sample-and-hold mode.
     */
    Utility_State sampleAndHold;

    /**
     * Specifies whether the output buffer is enabled (@ref UTILITY_STATE_ENABLE) or not.
     */
    Utility_State outputBuffer;

    /**
     * Specifies whether the output is connected or not to DAC peripheral.
     */
    Utility_State internalConnect;

    /**
     * Specifies the data alignment used to write data.
     */
    Dac_DataAlign align;

} Dac_ChannelConfig;

/**
 * Configure pin to be used as Dac output.
 *
 * @param[in] dev Dac device handle
 * @param[in] config Configuration list for selected pin
 * @param[in] pin Selected microcontroller pin
 * @return ERRORS_NO_ERROR for configuration without problems, otherwise a specific error.
 */
System_Errors Dac_configPin (Dac_DeviceHandle dev, Dac_ChannelConfig* config, Dac_Pins pin);

/**
 * Enable Dac and start conversion.
 *
 * @param[in] dev Dac device handle
 * @param[in] channel The selected channel
 * @return ERRORS_NO_ERROR for start conversion without problems, otherwise a specific error.
 */
System_Errors Dac_start (Dac_DeviceHandle dev, Dac_Channels channel);

/**
 * Stop Dac conversion and disable peripheral.
 *
 * @param[in] dev Dac device handle
 * @param[in] channel The selected channel
 * @return ERRORS_NO_ERROR for stop conversion without problems, otherwise a specific error.
 */
System_Errors Dac_stop (Dac_DeviceHandle dev, Dac_Channels channel);

/**
 * Set the specified value to be converted in the selected Dac channel.
 *
 * @param[in] dev Dac device handle
 * @param[in] channel The selected channel
 * @param[in] value The value to be converted
 * @return ERRORS_NO_ERROR for start conversion without problems, otherwise a specific error.
 */
System_Errors Dac_write (Dac_DeviceHandle dev, Dac_Channels channel, uint16_t value);

/**
 * @}
 */

#ifdef LIBOHIBOARD_DMA
    uint8_t Dac_enableDmaTrigger (Dac_DeviceHandle dev, Dma_RequestSource request);
#endif

#ifdef __cplusplus
}
#endif

#endif // __DAC_H

/**
 * @}
 */

#endif // LIBOHIBOARD_DAC

/**
 * @}
 */
