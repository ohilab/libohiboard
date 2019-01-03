/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2018 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Alessio Paolucci
 *  Leonardo Morichelli
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
 * @file libohiboard/include/clock.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Alessio Paolucci
 * @author Leonardo Morichelli
 * @brief Clock definitions and prototypes.
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

/**
 * @defgroup CLOCK CLOCK
 * @brief Clock HAL driver
 * @{
 */

#ifndef __CLOCK_H
#define __CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum _Clock_Source
{
#if defined (LIBOHIBOARD_NXP_KINETIS)

    CLOCK_BUS,
    CLOCK_SYSTEM,
    CLOCK_FLEXBUS,
    CLOCK_FLASH,

// END IF: LIBOHIBOARD_NXP_KINETIS
#elif defined (LIBOHIBOARD_ST_STM32)

    CLOCK_BUS,
    CLOCK_SYSTEM,

// END IF: LIBOHIBOARD_ST_STM32
#endif

} Clock_Source;

typedef enum _Clock_Origin
{

    CLOCK_NO_SOURCE            = 0x0000,
    CLOCK_EXTERNAL             = 0x0001,
    CLOCK_CRYSTAL              = 0x0002,

#if defined (LIBOHIBOARD_NXP_KINETIS)

	CLOCK_INTERNAL_LIRC        = 0x0004,
	CLOCK_INTERNAL_HIRC        = 0x0008,

// END IF: LIBOHIBOARD_NXP_KINETIS
#elif defined (LIBOHIBOARD_ST_STM32)

#if defined (LIBOHIBOARD_STM32L4)

    CLOCK_INTERNAL_LSI         = 0x0004,
    CLOCK_INTERNAL_HSI         = 0x0008,
    CLOCK_INTERNAL_MSI         = 0x0010,
    CLOCK_INTERNAL_PLL         = 0x0020,
    CLOCK_EXTERNAL_LSE_CRYSTAL = 0x0040,

#endif // LIBOHIBOARD_STM32L4

// END IF: LIBOHIBOARD_ST_STM32
#endif

} Clock_Origin;

#if defined (LIBOHIBOARD_NXP_KINETIS)

typedef enum _Clock_State
{

    CLOCK_STATE_FEI  = 0u,
    CLOCK_STATE_FBI  = 1u,
    CLOCK_STATE_BLPI = 2u,
    CLOCK_STATE_FEE  = 3u,
    CLOCK_STATE_FBE  = 4u,
    CLOCK_STATE_BLPE = 5u,
    CLOCK_STATE_PBE  = 6u,
    CLOCK_STATE_PEE  = 7u,

    /**
     * It is not a correct Clock mode, but it is used to report an error.
     */
    CLOCK_STATE_NONE = 0xFF,

} Clock_State;

/**
 * This function return the current MCG mode of operation.
 *
 * @return The current mode.
 */
Clock_State Clock_getCurrentState (void);

#endif

typedef enum _Clock_Output
{
    CLOCK_OUTPUT_SYSCLK   = 0x0001,

#if defined (LIBOHIBOARD_ST_STM32)
    CLOCK_OUTPUT_HCLK     = 0x0002,
    CLOCK_OUTPUT_PCLK1    = 0x0004,
    CLOCK_OUTPUT_PCLK2    = 0x0008,
    CLOCK_OUTPUT_PLLR     = 0x0010,
    CLOCK_OUTPUT_PLLQ     = 0x0020,
    CLOCK_OUTPUT_PLLP     = 0x0040,
    CLOCK_OUTPUT_PLLSAI1R = 0x0080,
    CLOCK_OUTPUT_PLLSAI1Q = 0x0100,
    CLOCK_OUTPUT_PLLSAI1P = 0x0200,
    CLOCK_OUTPUT_PLLSAI2R = 0x0400,
    CLOCK_OUTPUT_PLLSAI2P = 0x0800,
#endif

} Clock_Output;


#if defined (LIBOHIBOARD_ST_STM32)

typedef enum _Clock_OscillatorState
{
    CLOCK_OSCILLATORSTATE_OFF,
    CLOCK_OSCILLATORSTATE_ON,
} Clock_OscillatorState;

typedef enum _Clock_SystemSource
{
    CLOCK_SYSTEMSOURCE_HSE = 0x0001,
    CLOCK_SYSTEMSOURCE_MSI = 0x0002,
    CLOCK_SYSTEMSOURCE_HSI = 0x0004,
    CLOCK_SYSTEMSOURCE_PLL = 0x0008,
} Clock_SystemSource;

/**
 * System source clock.
 *
 * @note The values are the same of register field
 *       (for writing value in register must be operate
 *       the correct shift).
 */
typedef enum _Clock_SystemSourceSws
{
    CLOCK_SYSTEMSOURCESWS_MSI = 0b00,
    CLOCK_SYSTEMSOURCESWS_HSI = 0b01,
    CLOCK_SYSTEMSOURCESWS_HSE = 0b10,
    CLOCK_SYSTEMSOURCESWS_PLL = 0b11,
} Clock_SystemSourceSws;

/**
 * PLL source clock
 *
 * @note The values are the same of register field
 *       (for writing value in register must be operate
 *       the correct shift).
 */
typedef enum _Clock_PllSource
{
    CLOCK_PLLSOURCE_NONE = 0x00000000u,
    CLOCK_PLLSOURCE_MSI  = (RCC_PLLCFGR_PLLSRC_MSI),
    CLOCK_PLLSOURCE_HSI  = (RCC_PLLCFGR_PLLSRC_HSI),
    CLOCK_PLLSOURCE_HSE  = (RCC_PLLCFGR_PLLSRC_HSE),
} Clock_PllSource;

typedef enum _Clock_AHBDivider
{
    CLOCK_AHBDIVIDER_1    = 0,
    CLOCK_AHBDIVIDER_2    = 1,
    CLOCK_AHBDIVIDER_4    = 2,
    CLOCK_AHBDIVIDER_8    = 3,
    CLOCK_AHBDIVIDER_16   = 4,
    CLOCK_AHBDIVIDER_64   = 5,
    CLOCK_AHBDIVIDER_128  = 6,
    CLOCK_AHBDIVIDER_256  = 7,
    CLOCK_AHBDIVIDER_512  = 8,

} Clock_AHBDivider;

typedef enum _Clock_APBDivider
{
    CLOCK_APBDIVIDER_1    = 0,
    CLOCK_APBDIVIDER_2    = 1,
    CLOCK_APBDIVIDER_4    = 2,
    CLOCK_APBDIVIDER_8    = 3,
    CLOCK_APBDIVIDER_16   = 4,

} Clock_APBDivider;

typedef enum _Clock_MSIRange
{
    CLOCK_MSIRANGE_100KHz  = (RCC_CR_MSIRANGE_0),
    CLOCK_MSIRANGE_200KHz  = (RCC_CR_MSIRANGE_1),
    CLOCK_MSIRANGE_400KHz  = (RCC_CR_MSIRANGE_2),
    CLOCK_MSIRANGE_800KHz  = (RCC_CR_MSIRANGE_3),
    CLOCK_MSIRANGE_1MHz    = (RCC_CR_MSIRANGE_4),
    CLOCK_MSIRANGE_2MHz    = (RCC_CR_MSIRANGE_5),
    CLOCK_MSIRANGE_4MHz    = (RCC_CR_MSIRANGE_6),
    CLOCK_MSIRANGE_8MHz    = (RCC_CR_MSIRANGE_7),
    CLOCK_MSIRANGE_16MHz   = (RCC_CR_MSIRANGE_8),
    CLOCK_MSIRANGE_24MHz   = (RCC_CR_MSIRANGE_9),
    CLOCK_MSIRANGE_32MHz   = (RCC_CR_MSIRANGE_10),
    CLOCK_MSIRANGE_48MHz   = (RCC_CR_MSIRANGE_11),

} Clock_MSIRange;

typedef enum _Clock_PLLPrescaler
{
    CLOCK_PLLPRESCALER_1  = 0,
    CLOCK_PLLPRESCALER_2  = 1,
    CLOCK_PLLPRESCALER_3  = 2,
    CLOCK_PLLPRESCALER_4  = 3,
    CLOCK_PLLPRESCALER_5  = 4,
    CLOCK_PLLPRESCALER_6  = 5,
    CLOCK_PLLPRESCALER_7  = 6,
    CLOCK_PLLPRESCALER_8  = 7,

} Clock_PLLPrescaler;

typedef enum _Clock_PLLMultiplier
{
    CLOCK_PLLMULTIPLIER_8  = 8,
    CLOCK_PLLMULTIPLIER_9  = 9,
    CLOCK_PLLMULTIPLIER_10 = 10,
    CLOCK_PLLMULTIPLIER_11 = 11,
    CLOCK_PLLMULTIPLIER_12 = 12,
    CLOCK_PLLMULTIPLIER_13 = 13,
    CLOCK_PLLMULTIPLIER_14 = 14,
    CLOCK_PLLMULTIPLIER_15 = 15,
    CLOCK_PLLMULTIPLIER_16 = 16,
    CLOCK_PLLMULTIPLIER_17 = 17,
    CLOCK_PLLMULTIPLIER_18 = 18,
    CLOCK_PLLMULTIPLIER_19 = 19,
    CLOCK_PLLMULTIPLIER_20 = 20,
    CLOCK_PLLMULTIPLIER_21 = 21,
    CLOCK_PLLMULTIPLIER_22 = 22,
    CLOCK_PLLMULTIPLIER_23 = 23,
    CLOCK_PLLMULTIPLIER_24 = 24,
    CLOCK_PLLMULTIPLIER_25 = 25,
    CLOCK_PLLMULTIPLIER_26 = 26,
    CLOCK_PLLMULTIPLIER_27 = 27,
    CLOCK_PLLMULTIPLIER_28 = 28,
    CLOCK_PLLMULTIPLIER_29 = 29,
    CLOCK_PLLMULTIPLIER_30 = 30,
    CLOCK_PLLMULTIPLIER_31 = 31,
    CLOCK_PLLMULTIPLIER_32 = 32,
    CLOCK_PLLMULTIPLIER_33 = 33,
    CLOCK_PLLMULTIPLIER_34 = 34,
    CLOCK_PLLMULTIPLIER_35 = 35,
    CLOCK_PLLMULTIPLIER_36 = 36,
    CLOCK_PLLMULTIPLIER_37 = 37,
    CLOCK_PLLMULTIPLIER_38 = 38,
    CLOCK_PLLMULTIPLIER_39 = 39,
    CLOCK_PLLMULTIPLIER_40 = 40,
    CLOCK_PLLMULTIPLIER_41 = 41,
    CLOCK_PLLMULTIPLIER_42 = 42,
    CLOCK_PLLMULTIPLIER_43 = 43,
    CLOCK_PLLMULTIPLIER_44 = 44,
    CLOCK_PLLMULTIPLIER_45 = 45,
    CLOCK_PLLMULTIPLIER_46 = 46,
    CLOCK_PLLMULTIPLIER_47 = 47,
    CLOCK_PLLMULTIPLIER_48 = 48,
    CLOCK_PLLMULTIPLIER_49 = 49,
    CLOCK_PLLMULTIPLIER_50 = 50,
    CLOCK_PLLMULTIPLIER_51 = 51,
    CLOCK_PLLMULTIPLIER_52 = 52,
    CLOCK_PLLMULTIPLIER_53 = 53,
    CLOCK_PLLMULTIPLIER_54 = 54,
    CLOCK_PLLMULTIPLIER_55 = 55,
    CLOCK_PLLMULTIPLIER_56 = 56,
    CLOCK_PLLMULTIPLIER_57 = 57,
    CLOCK_PLLMULTIPLIER_58 = 58,
    CLOCK_PLLMULTIPLIER_59 = 59,
    CLOCK_PLLMULTIPLIER_60 = 60,
    CLOCK_PLLMULTIPLIER_61 = 61,
    CLOCK_PLLMULTIPLIER_62 = 62,
    CLOCK_PLLMULTIPLIER_63 = 63,
    CLOCK_PLLMULTIPLIER_64 = 64,
    CLOCK_PLLMULTIPLIER_65 = 65,
    CLOCK_PLLMULTIPLIER_66 = 66,
    CLOCK_PLLMULTIPLIER_67 = 67,
    CLOCK_PLLMULTIPLIER_68 = 68,
    CLOCK_PLLMULTIPLIER_69 = 69,
    CLOCK_PLLMULTIPLIER_70 = 70,
    CLOCK_PLLMULTIPLIER_71 = 71,
    CLOCK_PLLMULTIPLIER_72 = 72,
    CLOCK_PLLMULTIPLIER_73 = 73,
    CLOCK_PLLMULTIPLIER_74 = 74,
    CLOCK_PLLMULTIPLIER_75 = 75,
    CLOCK_PLLMULTIPLIER_76 = 76,
    CLOCK_PLLMULTIPLIER_77 = 77,
    CLOCK_PLLMULTIPLIER_78 = 78,
    CLOCK_PLLMULTIPLIER_79 = 79,
    CLOCK_PLLMULTIPLIER_80 = 80,
    CLOCK_PLLMULTIPLIER_81 = 81,
    CLOCK_PLLMULTIPLIER_82 = 82,
    CLOCK_PLLMULTIPLIER_83 = 83,
    CLOCK_PLLMULTIPLIER_84 = 84,
    CLOCK_PLLMULTIPLIER_85 = 85,
    CLOCK_PLLMULTIPLIER_86 = 86,

} Clock_PLLMultiplier;

typedef enum _Clock_PLLDividerR
{
    CLOCK_PLLDIVIDER_R_2        = 0,
    CLOCK_PLLDIVIDER_R_4        = 1,
    CLOCK_PLLDIVIDER_R_6        = 2,
    CLOCK_PLLDIVIDER_R_8        = 3,
    CLOCK_PLLDIVIDER_R_DISABLED = 0xFF,

} Clock_PLLDividerR;

typedef enum _Clock_PLLDividerQ
{
    CLOCK_PLLDIVIDER_Q_2        = 0,
    CLOCK_PLLDIVIDER_Q_4        = 1,
    CLOCK_PLLDIVIDER_Q_6        = 2,
    CLOCK_PLLDIVIDER_Q_8        = 3,
    CLOCK_PLLDIVIDER_Q_DISABLED = 0xFF,

} Clock_PLLDividerQ;

typedef enum _Clock_PLLDividerP
{
    CLOCK_PLLDIVIDER_P_7        = 0,
    CLOCK_PLLDIVIDER_P_17       = 1,
    CLOCK_PLLDIVIDER_P_DISABLED = 0xFF,

} Clock_PLLDividerP;

typedef struct _Clock_PLLConfig
{
    Clock_PLLMultiplier multiplier;
    Clock_PLLDividerR dividerR;
    Clock_PLLDividerQ dividerQ;
    Clock_PLLDividerP dividerP;

} Clock_PLLConfig;

typedef enum _Clock_McoDivider
{
    CLOCK_MCODIVIDER_1  = 0,
    CLOCK_MCODIVIDER_2  = 1,
    CLOCK_MCODIVIDER_4  = 2,
    CLOCK_MCODIVIDER_8  = 3,
    CLOCK_MCODIVIDER_16 = 4,
} Clock_McoDivider;

typedef enum _Clock_McoSource
{
    CLOCK_MCOSOURCE_DISABLED = 0,
    CLOCK_MCOSOURCE_SYSCLK   = 1,
    CLOCK_MCOSOURCE_MSI      = 2,
    CLOCK_MCOSOURCE_HSI      = 3,
    CLOCK_MCOSOURCE_HSE      = 4,
    CLOCK_MCOSOURCE_PLL      = 5,
    CLOCK_MCOSOURCE_LSI      = 6,
    CLOCK_MCOSOURCE_LSE      = 7,
    //CLOCK_MCOSOURCE_HSI48 = 8,
} Clock_McoSource;

#endif

#if defined (LIBOHIBOARD_NXP_KINETIS)

/**
 * List of possible bus divider value.
 */
typedef enum _Clock_BusDivider
{
    CLOCK_BUSDIVIDER_1 = 0,
    CLOCK_BUSDIVIDER_2 = 1,
    CLOCK_BUSDIVIDER_3 = 2,
    CLOCK_BUSDIVIDER_4 = 3,
    CLOCK_BUSDIVIDER_5 = 4,
    CLOCK_BUSDIVIDER_6 = 5,
    CLOCK_BUSDIVIDER_7 = 6,
    CLOCK_BUSDIVIDER_8 = 7,

} Clock_BusDivider;


#endif

/**
 *
 */
typedef struct _Clock_Config
{
    Clock_Origin source;

    uint32_t fext;
    uint32_t foutSys;

#if defined (LIBOHIBOARD_NXP_KINETIS)

#if defined(LIBOHIBOARD_KV46F)   || \
    defined(LIBOHIBOARD_TWRKV46F)

    uint8_t coreDivider;
    uint8_t fastPerDivider;
    uint8_t flashDivider;
    bool enableHGO;

#else

    /**
     * The divider for bus clock.
     *
     * @note into MKL microcontroller, this divider is used both for
     *       bus and flash clock.
     */
    Clock_BusDivider busDivider;

#endif

// END IF: LIBOHIBOARD_NXP_KINETIS
#elif defined (LIBOHIBOARD_ST_STM32)

    Clock_OscillatorState hseState;
    Clock_OscillatorState hsiState;
    Clock_OscillatorState msiState;
    Clock_OscillatorState lsiState;
    Clock_OscillatorState lseState;
    Clock_OscillatorState pllState;

    Clock_SystemSource sysSource;
    Clock_PllSource pllSource;

    Clock_Output output;

    Clock_AHBDivider ahbDivider;
    Clock_APBDivider apb1Divider;
    Clock_APBDivider apb2Divider;
    Clock_MSIRange msiRange;

    Clock_PLLPrescaler pllPrescaler;
    Clock_PLLConfig pll;
    Clock_PLLConfig pllSai1;
    Clock_PLLConfig pllSai2;

    Clock_McoDivider mcoPrescaler;
    Clock_McoSource mcoSource;

// END IF: LIBOHIBOARD_ST_STM32
#endif

} Clock_Config;

// Useful define
#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/clock_STM32L4.h"

#elif defined (LIBOHIBOARD_MKL)

#include "hardware/clock_MKL.h"

#endif

/**
 * Initiliaze and configure clock peripheral.
 *
 * @param[in] config Configuration list parameter
 * @return ERRORS_NO_ERROR without problems
 */
System_Errors Clock_init (Clock_Config *config);

/**
 * Return the selected output clock.
 *
 * @param[in] output The selected output clock
 * @return The clock frequency in Hz
 */
uint32_t Clock_getOutputValue (Clock_Output output);

#if defined (LIBOHIBOARD_NXP_KINETIS)

System_Errors Clock_setDividers (uint8_t busDivider, uint8_t flexbusDivider, uint8_t flashDivider);

#elif defined (LIBOHIBOARD_ST_STM32)

System_Errors Clock_setDividers (uint32_t ahbDivider, uint32_t apb1Divider, uint32_t apb2Divider);

/**
 * Set the MSI_RANGE value used for clock switching during clock switch.
 *
 * @param[in] msi Value of MSI range (It must be minor than range 10).
 * @note This value influences the time of switching.
 *       Using RANGE_6 (4 MHz) the time of switch is about 1 ms,
 *       at RANGE_9 (24 MHz) is 200 us.
 */
void Clock_setMsiRangeSwitching (Clock_MSIRange msi);

/**
 * Configure the Clock configuration for having
 * a SYSCLK at frequency specified
 *
 * @param[in] frequency: value of SYSCLK
 * @return ERRORS_NO_ERROR without problems
 */
System_Errors Clock_setFrequency (uint32_t frequency);

#endif

/**
 * Return the selected oscillator clock value.
 *
 * @return The clock frequency in Hz
 */
uint32_t Clock_getOscillatorValue (void);

/**
 * Return the system clock set in configuration struct.
 *
 * @param[in] config configuration struct.
 * @return value of SYSCLK clock.
 */
uint32_t Clock_getConfigOscillatorValue (Clock_Config *config);

#if defined (LIBOHIBOARD_K10D10)       || \
    defined (LIBOHIBOARD_K10D7)        || \
    defined (LIBOHIBOARD_K12D5)        || \
    defined (LIBOHIBOARD_K60DZ10)      || \
    defined (LIBOHIBOARD_K64F12)       || \
    defined (LIBOHIBOARD_FRDMK64F)     || \
    defined (LIBOHIBOARD_KV31F12)      || \
    defined (LIBOHIBOARD_KV46F)        || \
    defined (LIBOHIBOARD_TWRKV46F)     || \
    defined (LIBOHIBOARD_OHIBOARD_R1)

uint8_t Clock_getCoreDivider();

#endif

#ifdef __cplusplus
}
#endif

#endif // __CLOCK_H

/**
 * @}
 */

/**
 * @}
 */
