/*
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Leonardo Morichelli <leonardo.morichelli@live.com>
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
 * @file libohiboard/source/lowpower_STM32L4.c
 * @author Leonardo Morichelli <leonardo.morichelli@live.com>
 * @brief Low Power implementations for STM32L4 Series
 */

#ifdef LIBOHIBOARD_LOWPOWER

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L4)

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

/**
 * @addtogroup LOWPOWER
 * @{
 */

#include "lowpower.h"

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
    PWR_TypeDef* regmapPWR;
    SCB_Type* regmapSCB;
    RCC_TypeDef* regmapRCC;

    LowPower_Mode currentMode;

    LowPower_ResetControl resetControl;

} LowPower_Device;

static LowPower_Device lpd =
{
		.regmapPWR    = PWR,
		.regmapSCB    = SCB,
		.regmapRCC    = RCC,

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
	uint32_t regPWR_SR1 = lpd.regmapPWR->SR1;
	uint32_t regRCC_CSR = lpd.regmapRCC->CSR;

	lpd.resetControl.value = 0;

	lpd.resetControl.flags.pwrWufi    = ((regPWR_SR1 & PWR_SR1_WUFI) != 0)?(1):(0);
	lpd.resetControl.flags.pwrStandby = ((regPWR_SR1 & PWR_SR1_SBF) != 0)?(1):(0);

	lpd.resetControl.flags.rccLowPowerReset            = ((regRCC_CSR & RCC_CSR_LPWRRSTF) != 0)?(1):(0);
	lpd.resetControl.flags.rccWatchdogReset            = ((regRCC_CSR & RCC_CSR_WWDGRSTF) != 0)?(1):(0);
	lpd.resetControl.flags.rccIndependentWatchdogReset = ((regRCC_CSR & RCC_CSR_IWDGRSTF) != 0)?(1):(0);
	lpd.resetControl.flags.rccSoftwareReset            = ((regRCC_CSR & RCC_CSR_SFTRSTF) != 0)?(1):(0);
	lpd.resetControl.flags.rccBOR                      = ((regRCC_CSR & RCC_CSR_BORRSTF) != 0)?(1):(0);
	lpd.resetControl.flags.rccPinReset                 = ((regRCC_CSR & RCC_CSR_PINRSTF) != 0)?(1):(0);
	lpd.resetControl.flags.rccOptionbyteLoaderReset    = ((regRCC_CSR & RCC_CSR_OBLRSTF) != 0)?(1):(0);
	lpd.resetControl.flags.rccFirewallReset            = ((regRCC_CSR & RCC_CSR_FWRSTF) != 0)?(1):(0);

	// Clear Power flags
	UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->SCR, PWR_SCR_CSBF);
	if (lpd.resetControl.flags.pwrWufi)
	{
		LowPower_clearWakeUpflags(LOWPOWER_WAKEUPPINS_PIN1 |
		                          LOWPOWER_WAKEUPPINS_PIN2 |
		                          LOWPOWER_WAKEUPPINS_PIN3 |
		                          LOWPOWER_WAKEUPPINS_PIN4 |
		                          LOWPOWER_WAKEUPPINS_PIN5);
	}
	// Clear Reset flags
	UTILITY_SET_REGISTER_BIT(lpd.regmapRCC->CSR, RCC_CSR_RMVF);
}

/**
 * Return if system is configured in Low Power mode.
 *
 * @return TRUE in case of Low-Power Run mode, FALSE otherwise.
 */
static bool LowPower_isLowPowerRunMode (void)
{
	return (UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->CR1, PWR_CR1_LPR) == 0)?(FALSE):(TRUE);
}

/**
 * Enter Low-Power Run (LPRUN) mode.
 */
static void LowPower_enableLowPowerRunMode (void)
{
	UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR1, PWR_CR1_LPR);
}

/**
 * Exit Low-power Run mode.
 *
 * @return ERRORS_NO_ERROR whether no error, ERRORS_LOWPOWER_TIMEOUT otherwise.
 */
static System_Errors LowPower_disableLowPowerRunMode (void)
{
	System_Errors err = ERRORS_NO_ERROR;
	uint32_t tickstart = 0;

	UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR1, PWR_CR1_LPR);

	tickstart = System_currentTick();
	while (UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->SR2, PWR_SR2_REGLPF) == 1)
	{
		if ((System_currentTick() - tickstart) > 5000u)
			return ERRORS_LOWPOWER_TIMEOUT;
	}

	return err;
}

/**
 *
 */
static System_Errors LowPower_setVoltageScaling (LowPower_VoltageScaling voltageScaling)
{
	System_Errors err = ERRORS_NO_ERROR;
	UTILITY_MODIFY_REGISTER(lpd.regmapPWR->CR1, PWR_CR1_VOS_Msk, ((uint32_t)voltageScaling << PWR_CR1_VOS_Pos));
	if (voltageScaling == LOWPOWER_VOLTAGESCALING_SCALE1)
	{
		uint32_t tickstart = 0;
		tickstart = System_currentTick();
		while (UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->SR2, PWR_SR2_VOSF) == 1)
		{
			if ((System_currentTick() - tickstart) > 5000u)
				return ERRORS_LOWPOWER_TIMEOUT;
		}
	}
	return err;
}

/**
 * Wait for Interrupt or Event
 * param[in] entry Select the type of wake-up.
 * @music Metallica::Loard.UntilItSleeps(); // _|m/
 */
static void LowPower_untilItSleeps (LowPower_WaitFor entry)
{
	if (entry == LOWPOWER_WAITFOR_INTERRUPT)
	{
	    /* Request Wait For Interrupt */
	    __WFI();
	}
	else
	{
	    /* Request Wait For Event */
	    __SEV();
	    __WFE();
	    __WFE();
	}
}

/**
 *
 */
static void LowPower_enterSleepMode (LowPower_Regulator regulator, LowPower_WaitFor sleepEntry)
{
	switch (regulator)
	{
	case LOWPOWER_REGULATOR_MAIN:
		if (LowPower_isLowPowerRunMode() == TRUE)
		{
			LowPower_disableLowPowerRunMode();
		}
		break;

	case LOWPOWER_REGULATOR_LOW:
		if (LowPower_isLowPowerRunMode() == FALSE)
		{
			LowPower_enableLowPowerRunMode();
		}
		break;
	}

	/* Clear SLEEPDEEP bit of Cortex System Control Register */
	UTILITY_CLEAR_REGISTER_BIT(lpd.regmapSCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
	LowPower_untilItSleeps(sleepEntry);
}

/**
 *
 */
static void LowPower_enterStopMode (uint32_t level, LowPower_WaitFor stopEntry)
{
	uint32_t pwrCr1Lpms = 0;
	switch (level)
	{
	default:
	case 0:
		pwrCr1Lpms = PWR_CR1_LPMS_STOP0;
		break;
	case 1:
		pwrCr1Lpms = PWR_CR1_LPMS_STOP1;
		break;
	case 2:
		pwrCr1Lpms = PWR_CR1_LPMS_STOP2;
		break;
	}

	UTILITY_MODIFY_REGISTER(lpd.regmapPWR->CR1, PWR_CR1_LPMS, pwrCr1Lpms);
	UTILITY_SET_REGISTER_BIT(lpd.regmapSCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
	LowPower_untilItSleeps(stopEntry);
	UTILITY_CLEAR_REGISTER_BIT(lpd.regmapSCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
}

/**
 *
 */
static void LowPower_enterStandbyMode (void)
{
	UTILITY_MODIFY_REGISTER(lpd.regmapPWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STANDBY);
	UTILITY_SET_REGISTER_BIT(lpd.regmapSCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
	LowPower_untilItSleeps(LOWPOWER_WAITFOR_INTERRUPT);
}

/**
 *
 */
static void LowPower_enterShutdownMode (void)
{
	UTILITY_MODIFY_REGISTER(lpd.regmapPWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_SHUTDOWN);
	UTILITY_SET_REGISTER_BIT(lpd.regmapSCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
	LowPower_untilItSleeps(LOWPOWER_WAITFOR_INTERRUPT);
}

/**
 *
 */
static LowPower_Mode LowPower_modeSwitchingRoute[LOWPOWER_MODE_NUMBER][LOWPOWER_MODE_NUMBER] =
{
	/*                          {LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_SLEEP,	LOWPOWER_MODE_LPSLEEP, LOWPOWER_MODE_STOP0, LOWPOWER_MODE_STOP1, LOWPOWER_MODE_STOP2, LOWPOWER_MODE_STANDBY, LOWPOWER_MODE_SHUTDOWN}, */
	/* LOWPOWER_MODE_RUN      */{LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_SLEEP, LOWPOWER_MODE_LPRUN,   LOWPOWER_MODE_STOP0, LOWPOWER_MODE_STOP1, LOWPOWER_MODE_STOP2, LOWPOWER_MODE_STANDBY, LOWPOWER_MODE_SHUTDOWN},
	/* LOWPOWER_MODE_LPRUN    */{LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPSLEEP, LOWPOWER_MODE_RUN,   LOWPOWER_MODE_STOP1, LOWPOWER_MODE_RUN,   LOWPOWER_MODE_STANDBY, LOWPOWER_MODE_SHUTDOWN},
	/* LOWPOWER_MODE_SLEEP    */{LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_SLEEP, LOWPOWER_MODE_RUN,     LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,     LOWPOWER_MODE_RUN},
	/* LOWPOWER_MODE_LPSLEEP  */{LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_LPSLEEP, LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_LPRUN,   LOWPOWER_MODE_LPRUN},
	/* LOWPOWER_MODE_STOP0    */{LOWPOWER_MODE_RUN,	  LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,     LOWPOWER_MODE_STOP0, LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,     LOWPOWER_MODE_RUN},
	/* LOWPOWER_MODE_STOP1    */{LOWPOWER_MODE_RUN,	  LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,     LOWPOWER_MODE_RUN,   LOWPOWER_MODE_STOP1, LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,     LOWPOWER_MODE_RUN},
	/* LOWPOWER_MODE_STOP2    */{LOWPOWER_MODE_RUN,	  LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,     LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_STOP2, LOWPOWER_MODE_RUN,     LOWPOWER_MODE_RUN},
	/* LOWPOWER_MODE_STANDBY  */{LOWPOWER_MODE_RUN,	  LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,     LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_STANDBY, LOWPOWER_MODE_RUN},
	/* LOWPOWER_MODE_SHUTDOWN */{LOWPOWER_MODE_RUN,	  LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,     LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,   LOWPOWER_MODE_RUN,     LOWPOWER_MODE_SHUTDOWN},
};

/**
 * Enter into new power mode.
 *
 * @note The return value is not used!
 *
 * @param[in] mode The new low power mode
 */
static System_Errors LowPower_enterMode (LowPower_Mode mode)
{
	switch (mode)
	{
	default:
	case LOWPOWER_MODE_RUN:
		LowPower_disableLowPowerRunMode();
		break;
	case LOWPOWER_MODE_LPRUN:
		LowPower_enableLowPowerRunMode();
		break;
	case LOWPOWER_MODE_SLEEP:
		System_suspendTick();
		LowPower_enterSleepMode(LOWPOWER_REGULATOR_MAIN, LOWPOWER_WAITFOR_INTERRUPT);
		System_resumeTick();
		break;
	case LOWPOWER_MODE_LPSLEEP:
		LowPower_enableLowPowerRunMode();
		System_suspendTick();
		LowPower_enterSleepMode(LOWPOWER_REGULATOR_LOW, LOWPOWER_WAITFOR_INTERRUPT);
		System_resumeTick();
		break;
	case LOWPOWER_MODE_STOP0:
		LowPower_enterStopMode(0, LOWPOWER_WAITFOR_INTERRUPT);
		break;
	case LOWPOWER_MODE_STOP1:
		LowPower_enableLowPowerRunMode();
		LowPower_enterStopMode(1, LOWPOWER_WAITFOR_INTERRUPT);
		break;
	case LOWPOWER_MODE_STOP2:
		LowPower_enableLowPowerRunMode();
		LowPower_enterStopMode(2, LOWPOWER_WAITFOR_INTERRUPT);
		break;
	case LOWPOWER_MODE_STANDBY:
		LowPower_enableLowPowerRunMode();
		LowPower_enterStandbyMode();
		break;
	case LOWPOWER_MODE_SHUTDOWN:
		LowPower_enableLowPowerRunMode();
		LowPower_enterShutdownMode();
		break;
	}

	lpd.currentMode = mode;
	return ERRORS_NO_ERROR;
}

/**
 * Switch to another power mode.
 *
 * @param[in] mode the new power mode
 */
static System_Errors LowPower_switchMode (LowPower_Mode mode)
{
	System_Errors err = ERRORS_NO_ERROR;

	LowPower_Mode currentMode = LowPower_getMode();
	while ((currentMode != mode) && (err == ERRORS_NO_ERROR))
	{
		err = LowPower_enterMode(LowPower_modeSwitchingRoute[currentMode][mode]);
		currentMode = LowPower_getMode();
	}
	return err;
}

/**
 * @}
 */

void LowPower_init (void)
{
    LowPower_readResetStatus();
    CLOCK_ENABLE_PWR();
    CLOCK_ENABLE_SYSCFG();
}

LowPower_ResetControl LowPower_getResetStatus (void)
{
    return lpd.resetControl;
}

#define PWR_CR4_WP_Msk (PWR_CR4_WP5_Msk|PWR_CR4_WP4_Msk|PWR_CR4_WP3_Msk|PWR_CR4_WP2_Msk|PWR_CR4_WP1_Msk)

void LowPower_enableWakeUpPin (LowPower_WakeUpPins pins, LowPower_WakeUpEdge polarity)
{
	UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR3, PWR_CR3_EIWUL_Msk);
	UTILITY_MODIFY_REGISTER(lpd.regmapPWR->CR3, PWR_CR3_EWUP_Msk, pins);
	UTILITY_MODIFY_REGISTER(lpd.regmapPWR->CR4, PWR_CR4_WP_Msk, polarity);
}

void LowPower_disableWakeUpPin (LowPower_WakeUpPins pins)
{
	if (pins & LOWPOWER_WAKEUPPINS_PIN5)
	{
		UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR3, PWR_CR3_EWUP5);
	}
	if (pins & LOWPOWER_WAKEUPPINS_PIN4)
	{
		UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR3, PWR_CR3_EWUP4);
	}
	if (pins & LOWPOWER_WAKEUPPINS_PIN3)
	{
		UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR3, PWR_CR3_EWUP3);
	}
	if (pins & LOWPOWER_WAKEUPPINS_PIN2)
	{
		UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR3, PWR_CR3_EWUP2);
	}
	if (pins & LOWPOWER_WAKEUPPINS_PIN1)
	{
		UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR3, PWR_CR3_EWUP1);
	}
	if (pins == 0)
	{
		UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR3, PWR_CR3_EIWF_Msk);
	}
}

uint32_t LowPower_getWakeUpflags (void)
{
	return UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->SR1, PWR_CR3_EWUP_Msk);
}

void LowPower_clearWakeUpflags (uint32_t flags)
{
	UTILITY_MODIFY_REGISTER(lpd.regmapPWR->SCR, PWR_SCR_CWUF_Msk, (flags));
}

System_Errors LowPower_setModeByFrequency(uint32_t frequency, LowPower_Mode mode)
{
	System_Errors err = ERRORS_NO_ERROR;
	if (((mode == LOWPOWER_MODE_LPRUN) || (mode == LOWPOWER_MODE_LPSLEEP)) && (frequency > 2000000u))
	{
		frequency = 2000000u;
	}
	if ((mode != LOWPOWER_MODE_LPRUN) && (mode != LOWPOWER_MODE_LPSLEEP))
	{
		LowPower_disableLowPowerRunMode();
	}
	if (frequency > 26000000)
	{
		LowPower_setVoltageScaling(LOWPOWER_VOLTAGESCALING_SCALE1);
	}
	err = Clock_setFrequency(frequency);
	if (frequency <= 26000000)
	{
		LowPower_setVoltageScaling(LOWPOWER_VOLTAGESCALING_SCALE2);
	}
	ohiassert(LOWPOWER_IS_FREQUENCY_RIGHT(frequency, Clock_getOscillatorValue()));
	if(err != ERRORS_NO_ERROR)
	{
		return err;
	}
	err = LowPower_switchMode(mode);
	return err;
}

System_Errors LowPower_setModeByConfiguration (Clock_Config* config, LowPower_Mode mode)
{
	System_Errors err = ERRORS_NO_ERROR;
	if ((mode != LOWPOWER_MODE_LPRUN) && (mode != LOWPOWER_MODE_LPSLEEP))
	{
		LowPower_disableLowPowerRunMode();
	}
	if (Clock_getConfigOscillatorValue(config) > 26000000)
	{
		LowPower_setVoltageScaling(LOWPOWER_VOLTAGESCALING_SCALE1);
	}
	err = Clock_init(config);
	if (Clock_getOscillatorValue() <= 26000000)
	{
		LowPower_setVoltageScaling(LOWPOWER_VOLTAGESCALING_SCALE2);
	}
	if (((mode == LOWPOWER_MODE_LPRUN) || (mode == LOWPOWER_MODE_LPSLEEP)))
	{
		ohiassert(LOWPOWER_IS_VALID_CLOCK(config->foutSys));
	}
	if (err != ERRORS_NO_ERROR)
	{
		return err;
	}
	err = LowPower_switchMode(mode);
	return err;
}

LowPower_Mode LowPower_getMode(void)
{
	return lpd.currentMode;
}

#endif // LIBOHIBOARD_STM32L4

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_LOWPOWER

