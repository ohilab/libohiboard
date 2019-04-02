/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/source/PIC24FJ/lowpower-timer_PIC24FJ.c
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief Low-Power Timer implementations for PIC24FJ
 */

#include "platforms.h"

#if defined (LIBOHIBOARD_LOWPOWER)

#ifdef __cplusplus
extern "C" {
#endif

#include "lowpower.h"

#if defined (LIBOHIBOARD_PIC24FJ)

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

/**
 * @addtogroup LOWPOWER
 * @{
 */

/**
 * @defgroup LOWPOWER_Private Low Power private functions and types.
 * @{
 */

/**
 *
 */
#define LOWPOWER_IS_FREQUENCY_RIGHT(FREQ1, FREQ2)  ((FREQ1 == FREQ2)?(TRUE):(FALSE))

/**
 *
 */
#define LOWPOWER_IS_VALID_CLOCK(FREQ)      ((FREQ <= 2000000u)?(TRUE):(FALSE))

/**
 *
 */
typedef struct _LowPower_Device
{
    RESET_TypeDef* regmapRESET;
    CPU_TypeDef* regmapCPU;

    LowPower_Mode currentMode;
    LowPower_ResetControl resetControl;

} LowPower_Device;

static LowPower_Device lpd =
{
		.regmapRESET  = RESET,
        .regmapCPU = CPU,

		.currentMode  = LOWPOWER_MODE_RUN,
		.resetControl =
		{
            .value = 0,
		},
};

/**
 *
 */
static void LowPower_readResetStatus(void)
{
    // Read Reset flags
	lpd.resetControl.value = lpd.regmapRESET->RCON;
	
	// Clear Reset flags
	lpd.regmapRESET->RCON = 0;
}

/**
 *
 */
static void LowPower_enterIdleMode (void)
{
    ClrWdt();
    UTILITY_MODIFY_REGISTER(lpd.regmapCPU->SR, _SR_IPL_MASK, (0u << _SR_IPL_POSITION));
    __builtin_disi(0x0001);
	Idle();
}

/**
 *
 */
static void LowPower_enterSleepMode (void)
{
    ClrWdt();
    UTILITY_MODIFY_REGISTER(lpd.regmapCPU->SR, _SR_IPL_MASK, (0u << _SR_IPL_POSITION));
    __builtin_disi(0x0001);
	Sleep();
}


/**
 * Enter into new power mode.
 *
 * @note The return value is not used!
 *
 * @param[in] mode The new low power mode
 */
System_Errors LowPower_enterMode (LowPower_Mode mode)
{
    lpd.currentMode = mode;
    
	switch (mode)
	{
	default:
	case LOWPOWER_MODE_RUN:
		
		break;
	case LOWPOWER_MODE_IDLE:
		LowPower_enterIdleMode();
		break;
	case LOWPOWER_MODE_SLEEP:
		LowPower_enterSleepMode();
		break;
	}

	lpd.currentMode = LOWPOWER_MODE_RUN;
	return ERRORS_NO_ERROR;
}

/**
 * @}
 */

void LowPower_init (void)
{
    LowPower_readResetStatus();
    lpd.currentMode = LOWPOWER_MODE_RUN;
}

LowPower_ResetControl LowPower_getResetStatus (void)
{
    return lpd.resetControl;
}

System_Errors LowPower_setModeByFrequency(uint32_t frequency, LowPower_Mode mode)
{
	System_Errors err = ERRORS_NO_ERROR;
    Clock_Config tClockConfig = 
    {
#if defined (LIBOHIBOARD_PIC24FJ)
        .source = CLOCK_INTERNAL_FRCPLL | CLOCK_EXTERNAL_SOSC,
#endif
    };
    if(frequency >= 32000000)
    {
        tClockConfig.source = CLOCK_INTERNAL_FRCPLL | CLOCK_EXTERNAL_SOSC;
    }
    else
    {
        tClockConfig.source = CLOCK_INTERNAL_FRC | CLOCK_EXTERNAL_SOSC;
    }
	err = Clock_init(&tClockConfig);
	err = LowPower_enterMode(mode);
	return err;
}

System_Errors LowPower_setModeByConfiguration (Clock_Config* config, LowPower_Mode mode)
{
	System_Errors err = ERRORS_NO_ERROR;
	err = Clock_init(config);
	err = LowPower_enterMode(mode);
	return err;
}

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_LOWPOWER
