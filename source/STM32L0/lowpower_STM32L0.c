/*
 * Copyright (C) 2021 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Niccolò Paolinelli <ai03@hotmail.it>
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
 * @file libohiboard/source/STM32L0/lowpower_STM32L0.c
 * @author Niccolò Paolinelli <ai03@hotmail.it>
 * @brief Low Power implementations for STM32L0 Series
 */

#ifdef LIBOHIBOARD_LOWPOWER

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L0)

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
    SYSCFG_TypeDef* regmapSYSCFG;

    LowPower_Mode currentMode;

    LowPower_ResetControl resetControl;

} LowPower_Device;

static LowPower_Device lpd =
{
        .regmapPWR    = PWR,
        .regmapSCB    = SCB,
        .regmapRCC    = RCC,
        .regmapSYSCFG = SYSCFG,

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
    uint32_t regPWR_CSR = lpd.regmapPWR->CSR;
    uint32_t regRCC_CSR = lpd.regmapRCC->CSR;

    lpd.resetControl.value = 0;

    lpd.resetControl.flags.pwrWuf    = ((regPWR_CSR & PWR_CSR_WUF) != 0)?(1):(0);
    lpd.resetControl.flags.pwrStandby = ((regPWR_CSR & PWR_CSR_SBF) != 0)?(1):(0);

    lpd.resetControl.flags.rccLowPowerReset            = ((regRCC_CSR & RCC_CSR_LPWRRSTF) != 0)?(1):(0);
    lpd.resetControl.flags.rccWatchdogReset            = ((regRCC_CSR & RCC_CSR_WWDGRSTF) != 0)?(1):(0);
    lpd.resetControl.flags.rccIndependentWatchdogReset = ((regRCC_CSR & RCC_CSR_IWDGRSTF) != 0)?(1):(0);
    lpd.resetControl.flags.rccSoftwareReset            = ((regRCC_CSR & RCC_CSR_SFTRSTF) != 0)?(1):(0);
    lpd.resetControl.flags.rccPOR                      = ((regRCC_CSR & RCC_CSR_PORRSTF) != 0)?(1):(0);
    lpd.resetControl.flags.rccPinReset                 = ((regRCC_CSR & RCC_CSR_PINRSTF) != 0)?(1):(0);
    lpd.resetControl.flags.rccOptionbyteLoaderReset    = ((regRCC_CSR & RCC_CSR_OBLRSTF) != 0)?(1):(0);
    lpd.resetControl.flags.rccFirewallReset            = ((regRCC_CSR & RCC_CSR_FWRSTF) != 0)?(1):(0);

    // Clear Power flags
    UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_CSBF);
    if (lpd.resetControl.flags.pwrWuf)
    {
        LowPower_clearWakeUpflags(LOWPOWER_WAKEUPPINS_PIN1 |
                                  LOWPOWER_WAKEUPPINS_PIN2 |
                                  LOWPOWER_WAKEUPPINS_PIN3 );
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
    return (UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_LPRUN) == 0)?(FALSE):(TRUE);
}

/**
 * Enter Low-Power Run (LPRUN) mode.
 */
static void LowPower_enableLowPowerRunMode (void)
{
    UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_LPSDSR);
    UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_LPRUN);
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

    UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_LPRUN);
    UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_LPSDSR);

    tickstart = System_currentTick();
    while (UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->CSR, PWR_CSR_REGLPF) == 1)
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
    UTILITY_MODIFY_REGISTER(lpd.regmapPWR->CR, PWR_CR_VOS_Msk, ((uint32_t)voltageScaling << PWR_CR_VOS_Pos));
    if (voltageScaling == LOWPOWER_VOLTAGESCALING_SCALE1)
    {
        uint32_t tickstart = 0;
        tickstart = System_currentTick();
        while (UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->CSR, PWR_CSR_VOSF) == 1)
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
//        __WFE(); //perchè due? Si bloccava qui
    }
}

/**
 * TODO: Check function
 */
static void LowPower_enterSleepMode (LowPower_Regulator regulator, LowPower_WaitFor sleepEntry)
{
    /* It is forbidden to configure both EN_VREFINT=1 and ULP=1 if the device is
       in Stop mode or in Sleep/Low-power sleep mode */
    if((UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_ULP) != 0) &&
            (UTILITY_READ_REGISTER_BIT(lpd.regmapSYSCFG->CFGR3, SYSCFG_CFGR3_EN_VREFINT) != 0))
    {
      UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_ULP);
    }

    /* Clear PDDS and LPDS bits */
   UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR, (PWR_CR_PDDS | PWR_CR_LPSDSR));

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

    if((UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_ULP) != 0) &&
            (UTILITY_READ_REGISTER_BIT(lpd.regmapSYSCFG->CFGR3, SYSCFG_CFGR3_EN_VREFINT) != 0))
    {
        UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_ULP);
    }

    /* Additional NOP to ensure all pending instructions are flushed before entering low power mode */
    __NOP();

}

/**
 * TODO: Check function
 */
static void LowPower_enterStopMode (LowPower_Regulator regulator, LowPower_WaitFor stopEntry)
{
    /* It is forbidden to configure both EN_VREFINT=1 and ULP=1 if the device is
       in Stop mode or in Sleep/Low-power sleep mode */
    if((UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_ULP) != 0) &&
            (UTILITY_READ_REGISTER_BIT(lpd.regmapSYSCFG->CFGR3, SYSCFG_CFGR3_EN_VREFINT) != 0))
    {
      UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_ULP);
    }

    UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR, (PWR_CR_PDDS | PWR_CR_LPSDSR));

    switch (regulator)
    {
    case LOWPOWER_REGULATOR_MAIN:
        if (LowPower_isLowPowerRunMode() == TRUE)
            LowPower_disableLowPowerRunMode();
        break;
    case LOWPOWER_REGULATOR_LOW:
        if (LowPower_isLowPowerRunMode() == FALSE)
            LowPower_enableLowPowerRunMode();
        break;
    }
    UTILITY_SET_REGISTER_BIT(lpd.regmapSCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
    LowPower_untilItSleeps(stopEntry);
    UTILITY_CLEAR_REGISTER_BIT(lpd.regmapSCB->SCR, SCB_SCR_SLEEPDEEP_Msk);

    if((UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_ULP) != 0) &&
            (UTILITY_READ_REGISTER_BIT(lpd.regmapSYSCFG->CFGR3, SYSCFG_CFGR3_EN_VREFINT) != 0))
    {
        UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_ULP);
    }
}

/**
 * TODO: Check function
 */
static void LowPower_enterStandbyMode (void)
{
    UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_PDDS);
    UTILITY_SET_REGISTER_BIT(lpd.regmapSCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
    LowPower_untilItSleeps(LOWPOWER_WAITFOR_INTERRUPT);
}

/**
 *
 */
static LowPower_Mode LowPower_modeSwitchingRoute[LOWPOWER_MODE_NUMBER][LOWPOWER_MODE_NUMBER] =
{
    // There is not matrix transaction between each mode.
    /*                          {LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_SLEEP, LOWPOWER_MODE_LPSLEEP, LOWPOWER_MODE_STOP,   LOWPOWER_MODE_STANDBY}, */
    /* LOWPOWER_MODE_RUN      */{LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_SLEEP, LOWPOWER_MODE_LPSLEEP, LOWPOWER_MODE_STOP,   LOWPOWER_MODE_STANDBY},
    /* LOWPOWER_MODE_LPRUN    */{LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_SLEEP, LOWPOWER_MODE_LPSLEEP, LOWPOWER_MODE_STOP,   LOWPOWER_MODE_STANDBY},
    /* LOWPOWER_MODE_SLEEP    */{LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_SLEEP, LOWPOWER_MODE_LPSLEEP, LOWPOWER_MODE_STOP,   LOWPOWER_MODE_STANDBY},
    /* LOWPOWER_MODE_LPSLEEP  */{LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_SLEEP, LOWPOWER_MODE_LPSLEEP, LOWPOWER_MODE_STOP,   LOWPOWER_MODE_STANDBY},
    /* LOWPOWER_MODE_STOP     */{LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_SLEEP, LOWPOWER_MODE_LPSLEEP, LOWPOWER_MODE_STOP,   LOWPOWER_MODE_STANDBY},
    /* LOWPOWER_MODE_STANDBY  */{LOWPOWER_MODE_RUN,   LOWPOWER_MODE_LPRUN, LOWPOWER_MODE_SLEEP, LOWPOWER_MODE_LPSLEEP, LOWPOWER_MODE_STOP,   LOWPOWER_MODE_STANDBY},
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
        LowPower_enterSleepMode(LOWPOWER_REGULATOR_LOW, LOWPOWER_WAITFOR_INTERRUPT); //modificato io in EVENT ma era INTERRUPT
        System_resumeTick();
        break;
    case LOWPOWER_MODE_STOP:
        System_suspendTick();
        LowPower_enterStopMode(LOWPOWER_REGULATOR_LOW, LOWPOWER_WAITFOR_INTERRUPT);
        System_resumeTick();
        break;
    case LOWPOWER_MODE_STANDBY:
        System_suspendTick();
        LowPower_enableLowPowerRunMode();
        LowPower_enterStandbyMode();
        System_resumeTick();
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

/*
 * FIXME: must to be check
 */
void LowPower_enableWakeUpPin (LowPower_WakeUpPins pins, LowPower_WakeUpEdge polarity)
{
    (void)polarity;
    UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CSR, pins);
}

void LowPower_disableWakeUpPin (LowPower_WakeUpPins pins)
{
    if (pins & LOWPOWER_WAKEUPPINS_PIN3)
    {
        UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CSR, PWR_CSR_EWUP3);
    }
    if (pins & LOWPOWER_WAKEUPPINS_PIN2)
    {
        UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CSR, PWR_CSR_EWUP2);
    }
    if (pins & LOWPOWER_WAKEUPPINS_PIN1)
    {
        UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CSR, PWR_CSR_EWUP1);
    }
    if (pins == 0)
    {
        UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CSR, PWR_CSR_EWUP3);
        UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CSR, PWR_CSR_EWUP2);
        UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CSR, PWR_CSR_EWUP1);
    }
}

uint32_t LowPower_getWakeUpflags (void)
{
    return UTILITY_READ_REGISTER_BIT(lpd.regmapPWR->CSR, PWR_CSR_WUF);
}

void LowPower_clearWakeUpflags (uint32_t flags)
{
    UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_CWUF);
}

System_Errors LowPower_setModeByFrequency (uint32_t frequency, LowPower_Mode mode)
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
    if ((frequency <= 26000000) && (frequency > 4200000))
    {
        LowPower_setVoltageScaling(LOWPOWER_VOLTAGESCALING_SCALE2);
    }
    if (frequency <= 4200000)
    {
        LowPower_setVoltageScaling(LOWPOWER_VOLTAGESCALING_SCALE3);
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
        ohiassert(LOWPOWER_IS_VALID_CLOCK( Clock_getOutputValue(CLOCK_OUTPUT_HCLK) )  );
    }
    if (err != ERRORS_NO_ERROR)
    {
        return err;
    }
    err = LowPower_switchMode(mode);
    return err;
}

LowPower_Mode LowPower_getMode (void)
{
    return lpd.currentMode;
}

void LowPower_enableUltraLowPower (void)
{
    UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_ULP);
}

void LowPower_disableUltraLowPower (void)
{
    UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_ULP);
}

void LowPower_enableFastWakeUp (void)
{
    UTILITY_SET_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_FWU);
}

void LowPower_disableFastWakeUp (void)
{
    UTILITY_CLEAR_REGISTER_BIT(lpd.regmapPWR->CR, PWR_CR_FWU);
}

void LowPower_wakeupClockConfig (LowPower_WakeUpClock clock)
{
    if (clock == LOWPOWER_WAKEUPCLOCK_MSI)
        UTILITY_CLEAR_REGISTER_BIT(lpd.regmapRCC->CFGR, RCC_CFGR_STOPWUCK);
    else
        UTILITY_SET_REGISTER_BIT(lpd.regmapRCC->CFGR, RCC_CFGR_STOPWUCK);
}

#endif // LIBOHIBOARD_STM32L0

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


