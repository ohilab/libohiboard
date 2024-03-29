/*
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
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
 */

/**
 * @file libohiboard/source/STM32L0/clock_STM32L0.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Clock implementations for STM32L0 Series
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "clock.h"
#include "utility.h"
#include "system.h"

#if defined (LIBOHIBOARD_STM32L0)


#define CLOCK_IS_VALID_OSCILLATOR(OSC)  (OSC != CLOCK_NO_SOURCE)                                            && \
                                        (((OSC & CLOCK_EXTERNAL) == CLOCK_EXTERNAL)                         || \
                                         ((OSC & CLOCK_CRYSTAL) == CLOCK_CRYSTAL)                           || \
                                         ((OSC & CLOCK_INTERNAL_LSI) == CLOCK_INTERNAL_LSI)                 || \
                                         ((OSC & CLOCK_INTERNAL_HSI) == CLOCK_INTERNAL_HSI)                 || \
                                         ((OSC & CLOCK_INTERNAL_MSI) == CLOCK_INTERNAL_MSI)                 || \
                                         ((OSC & CLOCK_EXTERNAL_LSE_CRYSTAL) == CLOCK_EXTERNAL_LSE_CRYSTAL))

#define CLOCK_IS_VALID_PLL_SOURCE(PLL_SOURCE) ((PLL_SOURCE == CLOCK_PLLSOURCE_HSI) || \
                                               (PLL_SOURCE == CLOCK_PLLSOURCE_HSE) || \
                                               (PLL_SOURCE == CLOCK_PLLSOURCE_NONE))

#define CLOCK_IS_VALID_MSI_RANGE(RANGE) ((RANGE == CLOCK_MSIRANGE_65536Hz)  || \
                                         (RANGE == CLOCK_MSIRANGE_131072Hz) || \
                                         (RANGE == CLOCK_MSIRANGE_262144Hz) || \
                                         (RANGE == CLOCK_MSIRANGE_524288Hz) || \
                                         (RANGE == CLOCK_MSIRANGE_1048kHz)  || \
                                         (RANGE == CLOCK_MSIRANGE_2097kHz)  || \
                                         (RANGE == CLOCK_MSIRANGE_4194kHz))

#define CLOCK_IS_VALID_HSE_STATE(HSESTATE) (((HSESTATE) == CLOCK_OSCILLATORSTATE_OFF) || \
                                            ((HSESTATE) == CLOCK_OSCILLATORSTATE_ON))

#define CLOCK_IS_VALID_HSI_STATE(HSISTATE) (((HSISTATE) == CLOCK_OSCILLATORSTATE_OFF) || \
                                            ((HSISTATE) == CLOCK_OSCILLATORSTATE_ON))

#define CLOCK_IS_VALID_LSI_STATE(LSISTATE) (((LSISTATE) == CLOCK_OSCILLATORSTATE_OFF) || \
                                            ((LSISTATE) == CLOCK_OSCILLATORSTATE_ON))

#define CLOCK_IS_VALID_LSE_STATE(LSESTATE) (((LSESTATE) == CLOCK_OSCILLATORSTATE_OFF) || \
                                            ((LSESTATE) == CLOCK_OSCILLATORSTATE_ON))

#define CLOCK_IS_VALID_MSI_STATE(MSISTATE) (((MSISTATE) == CLOCK_OSCILLATORSTATE_OFF) || \
                                            ((MSISTATE) == CLOCK_OSCILLATORSTATE_ON))

#define CLOCK_IS_VALID_OUTPUT(OUTPUT) (((OUTPUT & CLOCK_OUTPUT_SYSCLK) == CLOCK_OUTPUT_SYSCLK) || \
                                       ((OUTPUT & CLOCK_OUTPUT_HCLK)   == CLOCK_OUTPUT_HCLK)   || \
                                       ((OUTPUT & CLOCK_OUTPUT_PCLK1)  == CLOCK_OUTPUT_PCLK1)  || \
                                       ((OUTPUT & CLOCK_OUTPUT_PCLK2)  == CLOCK_OUTPUT_PCLK2))

#define CLOCK_IS_VALID_AHB_DIVIDER(DIVIDER) (((DIVIDER) == CLOCK_AHBDIVIDER_1)   || \
                                             ((DIVIDER) == CLOCK_AHBDIVIDER_2)   || \
                                             ((DIVIDER) == CLOCK_AHBDIVIDER_4)   || \
                                             ((DIVIDER) == CLOCK_AHBDIVIDER_8)   || \
                                             ((DIVIDER) == CLOCK_AHBDIVIDER_16)  || \
                                             ((DIVIDER) == CLOCK_AHBDIVIDER_64)  || \
                                             ((DIVIDER) == CLOCK_AHBDIVIDER_128) || \
                                             ((DIVIDER) == CLOCK_AHBDIVIDER_256) || \
                                             ((DIVIDER) == CLOCK_AHBDIVIDER_512))

#define CLOCK_IS_VALID_APB_DIVIDER(DIVIDER) (((DIVIDER) == CLOCK_APBDIVIDER_1)   || \
                                             ((DIVIDER) == CLOCK_APBDIVIDER_2)   || \
                                             ((DIVIDER) == CLOCK_APBDIVIDER_4)   || \
                                             ((DIVIDER) == CLOCK_APBDIVIDER_8)   || \
                                             ((DIVIDER) == CLOCK_APBDIVIDER_16))

#define CLOCK_IS_VALID_HSI_DIVIDER(DIVIDER) (((DIVIDER) == CLOCK_HSIDIVIDER_1) || \
                                             ((DIVIDER) == CLOCK_HSIDIVIDER_4))

#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU) || \
    defined (LIBOHIBOARD_STM32L0x2)    || \
    defined (LIBOHIBOARD_STM32L0x3)
#define CLOCK_IS_VALID_EXTERNAL_RANGE(VALUE) ((VALUE >= CLOCK_MIN_FREQ_HSE) && (VALUE <= CLOCK_MAX_FREQ_HSE))

#define CLOCK_IS_VALID_SYSSOURCE(SYSSOURCE) (((SYSSOURCE & CLOCK_SYSTEMSOURCE_HSI) == CLOCK_SYSTEMSOURCE_HSI) || \
                                             ((SYSSOURCE & CLOCK_SYSTEMSOURCE_HSE) == CLOCK_SYSTEMSOURCE_HSE) || \
                                             ((SYSSOURCE & CLOCK_SYSTEMSOURCE_PLL) == CLOCK_SYSTEMSOURCE_PLL) || \
                                             ((SYSSOURCE & CLOCK_SYSTEMSOURCE_MSI) == CLOCK_SYSTEMSOURCE_MSI))
#else

#define CLOCK_IS_VALID_SYSSOURCE(SYSSOURCE) (((SYSSOURCE & CLOCK_SYSTEMSOURCE_HSI) == CLOCK_SYSTEMSOURCE_HSI) || \
                                             ((SYSSOURCE & CLOCK_SYSTEMSOURCE_PLL) == CLOCK_SYSTEMSOURCE_PLL) || \
                                             ((SYSSOURCE & CLOCK_SYSTEMSOURCE_MSI) == CLOCK_SYSTEMSOURCE_MSI))

#endif

static const uint32_t CLOCK_AHB_PRESCALE_REGISTER_TABLE[9] =
{
    RCC_CFGR_HPRE_DIV1,
    RCC_CFGR_HPRE_DIV2,
    RCC_CFGR_HPRE_DIV4,
    RCC_CFGR_HPRE_DIV8,
    RCC_CFGR_HPRE_DIV16,
    RCC_CFGR_HPRE_DIV64,
    RCC_CFGR_HPRE_DIV128,
    RCC_CFGR_HPRE_DIV256,
    RCC_CFGR_HPRE_DIV512,
};

static const uint8_t CLOCK_AHB_PRESCALE_SHIFT_TABLE[16] =
{
    0U, // DIV1 - NOT USED
    0U, // DIV1 - NOT USED
    0U, // DIV1 - NOT USED
    0U, // DIV1 - NOT USED
    0U, // DIV1 - NOT USED
    0U, // DIV1 - NOT USED
    0U, // DIV1 - NOT USED
    0U, // DIV1 - NOT USED
    1U, // DIV2
    2U, // DIV4
    3U, // DIV8
    4U, // DIV16
    6U, // DIV64
    7U, // DIV128
    8U, // DIV256
    9U, // DIV512
};

static const uint32_t CLOCK_APB1_PRESCALE_REGISTER_TABLE[5] =
{
    RCC_CFGR_PPRE1_DIV1,
    RCC_CFGR_PPRE1_DIV2,
    RCC_CFGR_PPRE1_DIV4,
    RCC_CFGR_PPRE1_DIV8,
    RCC_CFGR_PPRE1_DIV16
};

static const uint32_t CLOCK_APB2_PRESCALE_REGISTER_TABLE[5] =
{
    RCC_CFGR_PPRE2_DIV1,
    RCC_CFGR_PPRE2_DIV2,
    RCC_CFGR_PPRE2_DIV4,
    RCC_CFGR_PPRE2_DIV8,
    RCC_CFGR_PPRE2_DIV16
};

static const uint8_t CLOCK_APB_PRESCALE_SHIFT_TABLE[8] =
{
    0U, // DIV1 - NOT USED
    0U, // DIV1 - NOT USED
    0U, // DIV1 - NOT USED
    0U, // DIV1 - NOT USED
    1U, // DIV2
    2U, // DIV4
    3U, // DIV8
    4U, // DIV16
};

/**
 * Useful constant to define default value of MSI oscillator in hertz.
 */
static const uint32_t CLOCK_MSI_RANGE[]  =
{
    65536,
    131072,
    262144,
    524288,
    1048000,
    2097000,
    4194000,
};


typedef struct _Clock_Device
{
    RCC_TypeDef*   regmap;
    FLASH_TypeDef* regmapFlash;
    PWR_TypeDef*   regmapPwr;

    uint32_t systemCoreClock; /**< Value that store current system core clock */
    uint32_t hclkClock;
    uint32_t pclk1Clock;
    uint32_t pclk2Clock;

    uint32_t pllClock;

    uint32_t externalClock;           /**< Oscillator or external clock value */

    uint8_t devLSIInitialized;
    uint8_t devLSEInitialized;
    uint8_t devHSEInitialized;
    uint8_t devHSIInitialized;
    uint8_t devPLLInitialized;

    Clock_MSIRange deInitRange;

    System_Errors rccError;

} Clock_Device;


static Clock_Device clk =
{
    .regmap            = RCC,
    .regmapFlash       = FLASH,
	.regmapPwr         = PWR,

    .systemCoreClock   = CLOCK_STARTUP_FREQ,
    .hclkClock         = 0U,
    .pclk1Clock        = 0U,
    .pclk2Clock        = 0U,

    .externalClock     = 0U,

    .devLSIInitialized = 0,
    .devLSEInitialized = 0,
    .devHSIInitialized = 0,
    .devHSEInitialized = 0,
    .devPLLInitialized = 0,

    .deInitRange       = CLOCK_MSIRANGE_2097kHz,

    .rccError          = ERRORS_NO_ERROR,
};

/**
 * Deinitialize Clock configuration registers.
 *
 * @return ERRORS_NO_ERROR without problems
 * @note This function is used before set a new clock configuration.
 */
static System_Errors Clock_deInit (void);

/**
 * Return the source clock of PLL.
 *
 * @return value of PLL source clock.
 */
//static uint32_t Clock_getActualPllInputValue (void);

/**
 * Return the source clock of PLL set in configuration struct.
 *
 * @param[in] config configuration struct.
 *
 * @return value of source clock of PLL.
 */
//static uint32_t Clock_getConfigPllValue (Clock_Config *config, Clock_PLLConfig *pllConfig);

/**
 * Return the clock set for MSI in configuration struct.
 *
 * @param[in] config configuration struct.
 *
 * @return value of MSI clock
 */
static uint32_t Clock_getConfigMsiValue (Clock_Config* config)
{
    return CLOCK_MSI_RANGE[(config->msiRange >> RCC_ICSCR_MSIRANGE_Pos)];
}

/**
 * Return the clock provided by MSI set in register configuration.
 *
 * @return value of MSI clock.
 */
static uint32_t Clock_getActualMsiValue (void)
{
    uint32_t msiRange = (UTILITY_READ_REGISTER_BIT(clk.regmap->ICSCR, RCC_ICSCR_MSIRANGE) >> RCC_ICSCR_MSIRANGE_Pos);
    return CLOCK_MSI_RANGE[msiRange];
}

/**
 * Return the system clock set in register configuration.
 *
 * @return value of SYSCLK clock.
 */
static uint32_t Clock_getActualSystemValue (void)
{
    uint32_t systemClock = 0;
    Clock_SystemSourceSws sysclkSource = (Clock_SystemSourceSws)(UTILITY_READ_REGISTER_BIT(clk.regmap->CFGR, RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos);

    switch (sysclkSource)
    {
    default:
    case CLOCK_SYSTEMSOURCESWS_MSI:
        systemClock = Clock_getActualMsiValue();
        break;
    case CLOCK_SYSTEMSOURCESWS_HSI:
        if (UTILITY_READ_REGISTER_BIT(clk.regmap->CR,RCC_CR_HSIDIVEN) == 0)
            systemClock = CLOCK_FREQ_HSI;
        else
            systemClock = (CLOCK_FREQ_HSI / 4);
        break;
//    case CLOCK_SYSTEMSOURCESWS_HSE:
//        systemClock = clk0.externalClock;
//        break;
//    case CLOCK_SYSTEMSOURCESWS_PLL:
//    {
//        uint32_t pllmClock = Clock_getActualPllInputValue();
//        uint32_t pllnReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
//        uint32_t pllrReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos;
//        systemClock = (((pllmClock) * pllnReg) / ((pllrReg + 1) * 2));
//    }
//        break;
    }

    return systemClock;
}

/**
 * Return the current HCLK clock set in register configuration.
 *
 * @return value of HCLK clock.
 */
static uint32_t Clock_getActualHclkValue (void)
{
    uint32_t hclkClock = Clock_getActualSystemValue();
    uint32_t cfgr = clk.regmap->CFGR;
    uint32_t shifter = UTILITY_READ_REGISTER_BIT(cfgr,RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
    hclkClock = (hclkClock >> CLOCK_AHB_PRESCALE_SHIFT_TABLE[shifter]);

    return hclkClock;
}

/**
 * Return the clock set for HCLK in configuration struct.
 *
 * @param[in] config configuration struct.
 *
 * @return value of HCLK clock
 */
static uint32_t Clock_getConfigHclkValue (Clock_Config* config)
{
    uint32_t hclkClock = Clock_getConfigOscillatorValue(config);
    hclkClock = (hclkClock >> config->ahbDivider);
    return hclkClock;
}

static void Clock_updateOutputValue (void)
{
    clk.systemCoreClock = Clock_getActualSystemValue();
//    uint32_t pllmClock = Clock_getActualPllInputValue();

    // Compute HCLK, PCLK1 and PCLK2
    uint32_t cfgr = clk.regmap->CFGR;
    uint32_t shifter = UTILITY_READ_REGISTER_BIT(cfgr,RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
    clk.hclkClock = (clk.systemCoreClock >> CLOCK_AHB_PRESCALE_SHIFT_TABLE[shifter]);

    shifter = UTILITY_READ_REGISTER_BIT(cfgr,RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    clk.pclk1Clock = (clk.hclkClock >> CLOCK_APB_PRESCALE_SHIFT_TABLE[shifter]);

    shifter = UTILITY_READ_REGISTER_BIT(cfgr,RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    clk.pclk2Clock = (clk.hclkClock >> CLOCK_APB_PRESCALE_SHIFT_TABLE[shifter]);
}

static System_Errors Clock_oscillatorConfig (Clock_Config* config)
{
    uint32_t tickstart = 0;

    if ((config->source & CLOCK_INTERNAL_MSI) == CLOCK_INTERNAL_MSI)
    {
        ohiassert(CLOCK_IS_VALID_MSI_RANGE(config->msiRange));
        ohiassert(CLOCK_IS_VALID_MSI_STATE(config->msiState));

        // Check if MSI is just used as SYSCLK
        // In this case we can't disable it
        if ((clk.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_MSI)
        {
            if ((UTILITY_READ_REGISTER_BIT(clk.regmap->CR,RCC_CR_MSIRDY) != 0) &&
                (config->msiState == CLOCK_OSCILLATORSTATE_OFF))
            {
                return ERRORS_CLOCK_WRONG_CONFIGURATION;
            }
            else
            {
                UTILITY_MODIFY_REGISTER(clk.regmap->ICSCR, RCC_ICSCR_MSIRANGE, config->msiRange);
            }
        }
        else
        {
            if (config->msiState != CLOCK_OSCILLATORSTATE_OFF)
            {
                // Enable MSI
                UTILITY_SET_REGISTER_BIT(clk.regmap->CR, RCC_CR_MSION);

                // Wait until MSI is ready
                tickstart = System_currentTick();
                while (UTILITY_READ_REGISTER_BIT(clk.regmap->CR, RCC_CR_MSION) == 0u)
                {
                    if ((System_currentTick() - tickstart) > CLOCK_MSI_TIMEOUT_VALUE)
                        return ERRORS_CLOCK_TIMEOUT;
                }

                // Set MSI value
                UTILITY_MODIFY_REGISTER(clk.regmap->ICSCR, RCC_ICSCR_MSIRANGE, config->msiRange);
            }
            else
            {
                // Disable MSI
                UTILITY_CLEAR_REGISTER_BIT(clk.regmap->CR, RCC_CR_MSION);

                // Wait until MSI is ready
                tickstart = System_currentTick();
                while (UTILITY_READ_REGISTER_BIT(clk.regmap->CR, RCC_CR_MSION) != 0u)
                {
                    if ((System_currentTick() - tickstart) > CLOCK_MSI_TIMEOUT_VALUE)
                        return ERRORS_CLOCK_TIMEOUT;
                }
            }
        }
    }

    // HSI Configuration
    if ((config->source & CLOCK_INTERNAL_HSI) == CLOCK_INTERNAL_HSI)
    {
        // Check the HSI state value, and divider
        ohiassert(CLOCK_IS_VALID_HSI_STATE(config->hsiState));
        ohiassert(CLOCK_IS_VALID_HSI_DIVIDER(config->hsiDivider));

        // Check if HSI is just used as SYSCLK or as PLL source
        // In this case we can't disable it
        if (((clk.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI) ||
           (((clk.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) &&
            ((clk.regmap->CFGR & RCC_CFGR_PLLSRC) == CLOCK_PLLSOURCE_HSI)))
        {
            // When HSI is used as system clock it will not disabled
            if ((UTILITY_READ_REGISTER_BIT(clk.regmap->CR,RCC_CR_HSIRDY) != 0) &&
                (config->hsiState == CLOCK_OSCILLATORSTATE_OFF))
            {
                return ERRORS_CLOCK_WRONG_CONFIGURATION;
            }
            else
            {
                // Change only the divider...
                UTILITY_SET_REGISTER_BIT(clk.regmap->CR,config->hsiDivider);
            }
        }
        else
        {
            if (config->hsiState == CLOCK_OSCILLATORSTATE_OFF)
            {
                // Switch off the oscillator
                UTILITY_CLEAR_REGISTER_BIT(clk.regmap->CR,RCC_CR_HSION);

                // Wait until the HSERDY bit is cleared
                tickstart = System_currentTick();
                while ((clk.regmap->CR & RCC_CR_HSIRDY) > 0)
                {
                    if ((System_currentTick() - tickstart) > CLOCK_HSI_TIMEOUT_VALUE)
                        return ERRORS_CLOCK_TIMEOUT;
                }
            }
            else
            {
                // Switch on the oscillator
                UTILITY_SET_REGISTER_BIT(clk.regmap->CR,RCC_CR_HSION);
                UTILITY_SET_REGISTER_BIT(clk.regmap->CR,config->hsiDivider);

                // Wait until the HSERDY bit is set
                tickstart = System_currentTick();
                while ((clk.regmap->CR & RCC_CR_HSIRDY) == 0)
                {
                    if ((System_currentTick() - tickstart) > CLOCK_HSI_TIMEOUT_VALUE)
                        return ERRORS_CLOCK_TIMEOUT;
                }
            }
        }
    }

    // LSE Configuration
    // See page 143 of RM0367 for informations about backup domain control
    if ((config->source & CLOCK_EXTERNAL_LSE_CRYSTAL) == CLOCK_EXTERNAL_LSE_CRYSTAL)
    {
        bool isPwrChanged = FALSE;

        // Check the LSE state value
        ohiassert(CLOCK_IS_VALID_LSE_STATE(config->lseState));

        // LSE is write protected because is under backup domain
        // Unlock this register before write!
        // First operation is enable PWR driver
        if (CLOCK_IS_ENABLE_PWR() == FALSE)
        {
            CLOCK_ENABLE_PWR();
            isPwrChanged = TRUE;
        }

        if (CLOCK_BACKUP_IS_ENABLE_WRITE_ACCESS() == FALSE)
        {
        	// Enable write access
        	CLOCK_BACKUP_ENABLE_WRITE_ACCESS();
        	// Wait until write access was enabled
            tickstart = System_currentTick();
            while ((clk.regmapPwr->CR & PWR_CR_DBP) == 0)
            {
                if ((System_currentTick() - tickstart) > CLOCK_WRITE_ACCESS_TIMEOUT_VALUE)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }

        // TODO: LSE with BYPASS!
        if (config->lseState == CLOCK_OSCILLATORSTATE_OFF)
        {
            // Switch off the oscillator
            UTILITY_CLEAR_REGISTER_BIT(clk.regmap->CSR,RCC_CSR_LSEON);
            UTILITY_CLEAR_REGISTER_BIT(clk.regmap->CSR,RCC_CSR_LSEBYP);

            // Wait until LSE is disabled
            tickstart = System_currentTick();
            while ((clk.regmap->CSR & RCC_CSR_LSERDY) > 0)
            {
                if ((System_currentTick() - tickstart) > CLOCK_LSE_TIMEOUT_VALUE)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }
        else
        {
            // Switch on the oscillator
            UTILITY_SET_REGISTER_BIT(clk.regmap->CSR,RCC_CSR_LSEON);

            // Wait until LSE is ready
            tickstart = System_currentTick();
            while ((clk.regmap->CSR & RCC_CSR_LSERDY) == 0)
            {
                if ((System_currentTick() - tickstart) > CLOCK_LSE_TIMEOUT_VALUE)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }

        // Restore PWR Clock status
        if (isPwrChanged == TRUE)
        {
            CLOCK_DISABLE_PWR();
        }
    }

    if ((config->source & CLOCK_INTERNAL_LSI) == CLOCK_INTERNAL_LSI)
    {
        // Check the LSI state value
        ohiassert(CLOCK_IS_VALID_LSI_STATE(config->lsiState));

        if (config->lsiState == CLOCK_OSCILLATORSTATE_OFF)
        {
            // Switch off the oscillator
            UTILITY_CLEAR_REGISTER_BIT(clk.regmap->CSR,RCC_CSR_LSION);

            // Wait until LSI is disabled
            tickstart = System_currentTick();
            while ((clk.regmap->CSR & RCC_CSR_LSIRDY) > 0)
            {
                if ((System_currentTick() - tickstart) > 2u)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }
        else
        {
            // Switch on the oscillator
            UTILITY_SET_REGISTER_BIT(clk.regmap->CSR,RCC_CSR_LSION);

            // Wait until LSI is ready
            tickstart = System_currentTick();
            while ((clk.regmap->CSR & RCC_CSR_LSIRDY) == 0)
            {
                if ((System_currentTick() - tickstart) > 2u)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }
    }

//    UTILITY_MODIFY_REGISTER(clk.regmap->CFGR, RCC_CFGR_MCOSEL_Msk, (config->mcoSource << RCC_CFGR_MCOSEL_Pos));
//    UTILITY_MODIFY_REGISTER(clk.regmap->CFGR, RCC_CFGR_MCOPRE_Msk, (config->mcoPrescaler << RCC_CFGR_MCOPRE_Pos));

    return ERRORS_NO_ERROR;
}

static System_Errors Clock_outputConfig (Clock_Config* config)
{
    if ((config->output & CLOCK_OUTPUT_SYSCLK) == CLOCK_OUTPUT_SYSCLK)
    {
        // Check if SYSCLK source is valid
        ohiassert(CLOCK_IS_VALID_SYSSOURCE(config->sysSource));
        uint32_t cfgrSW = 0;

        // PLL is selected as sys clock
        if (config->sysSource == CLOCK_SYSTEMSOURCE_PLL)
        {
//            // Check if the source is ready
//            if (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR,RCC_CR_PLLRDY) == 0)
//            {
//                return ERRORS_CLOCK_PLL_NOT_READY;
//            }
//
//            cfgrSW = RCC_CFGR_SW_PLL;
        }
        else if (config->sysSource == CLOCK_SYSTEMSOURCE_HSI)
        {
            // Check if the source is ready
            if (UTILITY_READ_REGISTER_BIT(clk.regmap->CR,RCC_CR_HSIRDY) == 0)
            {
                return ERRORS_CLOCK_HSI_NOT_READY;
            }

            cfgrSW = RCC_CFGR_SW_HSI;
        }
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU) || \
    defined (LIBOHIBOARD_STM32L0x2)    || \
    defined (LIBOHIBOARD_STM32L0x3)
        else if (config->sysSource == CLOCK_SYSTEMSOURCE_HSE)
        {
//            // Check if the source is ready
//            if (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSERDY) == 0)
//            {
//                return ERRORS_CLOCK_HSE_NOT_READY;
//            }
//
//            // Save chose value for CFGR register
//            cfgrSW = RCC_CFGR_SW_HSE;
        }
#endif
        else if (config->sysSource == CLOCK_SYSTEMSOURCE_MSI)
        {
            // Check if the source is ready
            if (UTILITY_READ_REGISTER_BIT(clk.regmap->CR,RCC_CR_MSIRDY) == 0)
            {
                return ERRORS_CLOCK_MSI_NOT_READY;
            }

            cfgrSW = RCC_CFGR_SW_MSI;
        }

        // Clear SW part of CFGR register and updated informations
        clk.regmap->CFGR &= ~(RCC_CFGR_SW);
        clk.regmap->CFGR |= cfgrSW;

        // Check if new value was setted
        uint32_t tickstart = System_currentTick();
        while (UTILITY_READ_REGISTER_BIT(clk.regmap->CFGR,RCC_CFGR_SWS) != (cfgrSW << RCC_CFGR_SWS_Pos))
        {
            if ((System_currentTick() - tickstart) > CLOCK_SOURCE_SWITCH_TIMEOUT_VALUE)
                return ERRORS_CLOCK_TIMEOUT;
        }
    }

    if ((config->output & CLOCK_OUTPUT_HCLK) == CLOCK_OUTPUT_HCLK)
    {
        // Check if divider is valid
        ohiassert(CLOCK_IS_VALID_AHB_DIVIDER(config->ahbDivider));

        // Clear HPRE part of CFGR register and updated informations
        clk.regmap->CFGR &= ~(RCC_CFGR_HPRE);
        clk.regmap->CFGR |= CLOCK_AHB_PRESCALE_REGISTER_TABLE[config->ahbDivider];
    }

    if ((config->output & CLOCK_OUTPUT_PCLK1) == CLOCK_OUTPUT_PCLK1)
    {
        // Check if divider is valid
        ohiassert(CLOCK_IS_VALID_APB_DIVIDER(config->apb1Divider));

        // Clear PPRE1 part of CFGR register and updated informations
        clk.regmap->CFGR &= ~(RCC_CFGR_PPRE1);
        clk.regmap->CFGR |= CLOCK_APB1_PRESCALE_REGISTER_TABLE[config->apb1Divider];
    }

    if ((config->output & CLOCK_OUTPUT_PCLK2) == CLOCK_OUTPUT_PCLK2)
    {
        // Check if divider is valid
        ohiassert(CLOCK_IS_VALID_APB_DIVIDER(config->apb2Divider));

        // Clear PPRE2 part of CFGR register and updated informations
        clk.regmap->CFGR &= ~(RCC_CFGR_PPRE2);
        clk.regmap->CFGR |= CLOCK_APB2_PRESCALE_REGISTER_TABLE[config->apb2Divider];
    }

    // Update system core clock informations...
    Clock_updateOutputValue();

    return ERRORS_NO_ERROR;
}

#if 0
#define CLOCK_IS_VALID_PLL_FREQUENCY(FREQ) ((FREQ >= CLOCK_MIN_FREQ_MSI && FREQ <= CLOCK_MAX_FREQ_PLL)?(TRUE):(FALSE))

#define CLOCK_IS_VALID_CONFIG_PLL_FREQUENCY(OSC_CONFIG, PLL_CONFIG) (Clock_getConfigPllValue(OSC_CONFIG, PLL_CONFIG) <= CLOCK_MAX_FREQ_PLL)

static System_Errors Clock_oscillatorConfig (Clock_Config* config)
{
    uint32_t tickstart = 0;

    // HSE with external clock configuration
    if ((config->source & CLOCK_EXTERNAL) == CLOCK_EXTERNAL)
    {
        // Check the HSE state value
        ohiassert(CLOCK_IS_VALID_HSE_STATE(config->hseState));

        // Check if HSE is just used as SYSCLK or as PLL source
        // In this case we can't disable it
        if (((clk0.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE) ||
           (((clk0.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) &&
            ((clk0.regmap->PLLCFGR & RCC_PLLCFGR_PLLSRC) == CLOCK_PLLSOURCE_HSE)))
        {
            return ERRORS_CLOCK_WRONG_CONFIGURATION;
        }
        else
        {
            if (config->hseState == CLOCK_OSCILLATORSTATE_OFF)
            {
                // Switch off this oscillator
                UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSEON);
                UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSEBYP);

                // Wait until the HSERDY bit is cleared
                tickstart = System_currentTick();
                while ((clk0.regmap->CR & RCC_CR_HSERDY) > 0)
                {
                    if ((System_currentTick() - tickstart) > 5000u)
                        return ERRORS_CLOCK_TIMEOUT;
                }
            }
            else
            {
                // Switch on this oscillator, and bypass functionality
                UTILITY_SET_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSEON);
                UTILITY_SET_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSEBYP);

                // Wait until the HSERDY bit is set
                tickstart = System_currentTick();
                while ((clk0.regmap->CR & RCC_CR_HSERDY) == 0)
                {
                    if ((System_currentTick() - tickstart) > 5000u)
                        return ERRORS_CLOCK_TIMEOUT;
                }
            }
        }
    }

    // HSE with crystal configuration
    if ((config->source & CLOCK_CRYSTAL) == CLOCK_CRYSTAL)
    {
        // Check the HSE state value
        ohiassert(CLOCK_IS_VALID_HSE_STATE(config->hseState));

        // Check if HSE is just used as SYSCLK or as PLL source
        // In this case we can't disable it
        if (((clk0.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE) ||
           (((clk0.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) &&
            ((clk0.regmap->PLLCFGR & RCC_PLLCFGR_PLLSRC) == CLOCK_PLLSOURCE_HSE)))
        {
            return ERRORS_CLOCK_WRONG_CONFIGURATION;
        }
        else
        {
            if (config->hseState == CLOCK_OSCILLATORSTATE_OFF)
            {
                // Switch off this oscillator
                UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSEON);
                UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSEBYP);

                // Wait until the HSERDY bit is cleared
                tickstart = System_currentTick();
                while ((clk0.regmap->CR & RCC_CR_HSERDY) > 0)
                {
                    if ((System_currentTick() - tickstart) > 5000u)
                        return ERRORS_CLOCK_TIMEOUT;
                }
            }
            else
            {
                // Switch on this oscillator, and disable bypass functionality just for safety
                UTILITY_SET_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSEON);
                UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSEBYP);

                // Wait until the HSERDY bit is set
                tickstart = System_currentTick();
                while ((clk0.regmap->CR & RCC_CR_HSERDY) == 0)
                {
                    if ((System_currentTick() - tickstart) > 5000u)
                        return ERRORS_CLOCK_TIMEOUT;
                }
            }
        }
    }

    // LSI Configuration
    if ((config->source & CLOCK_INTERNAL_LSI) == CLOCK_INTERNAL_LSI)
    {
        // Check the LSI state value
        ohiassert(CLOCK_IS_VALID_LSI_STATE(config->lsiState));

        if (config->lsiState == CLOCK_OSCILLATORSTATE_OFF)
        {
            // Switch off the oscillator
            UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CSR,RCC_CSR_LSION);

            // Wait until LSI is disabled
            tickstart = System_currentTick();
            while ((clk0.regmap->CSR & RCC_CSR_LSIRDY) > 0)
            {
                if ((System_currentTick() - tickstart) > 2u)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }
        else
        {
            // Switch on the oscillator
            UTILITY_SET_REGISTER_BIT(clk0.regmap->CSR,RCC_CSR_LSION);

            // Wait until LSI is ready
            tickstart = System_currentTick();
            while ((clk0.regmap->CSR & RCC_CSR_LSIRDY) == 0)
            {
                if ((System_currentTick() - tickstart) > 2u)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }
    }

    if (config->pllState == CLOCK_OSCILLATORSTATE_ON && config->sysSource == CLOCK_SYSTEMSOURCE_PLL)
    {
        //*** PLL ***/
        //1. Disable the PLL by setting PLLON to 0 in Clock control register (RCC_CR).
        UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR,RCC_CR_PLLON);
        //2. Wait until PLLRDY is cleared. The PLL is now fully stopped.
        tickstart = System_currentTick();
        while (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_PLLRDY) == 1)
        {
            if ((System_currentTick() - tickstart) > 5000u)
                return ERRORS_CLOCK_TIMEOUT;
        }
        //3. Change the desired parameter.
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLSRC_Msk, (config->pllSource << RCC_PLLCFGR_PLLSRC_Pos));
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, (config->pllPrescaler << RCC_PLLCFGR_PLLM_Pos)); // /->M
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLN_Msk, (config->pll.multiplier << RCC_PLLCFGR_PLLN_Pos)); // ->*N
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLR_Msk, (config->pll.dividerR << RCC_PLLCFGR_PLLR_Pos)); // /R->
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLQ_Msk, (config->pll.dividerQ << RCC_PLLCFGR_PLLQ_Pos)); // /Q->
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLP_Msk, (config->pll.dividerP << RCC_PLLCFGR_PLLP_Pos)); // /P->
        //4. Enable the PLL again by setting PLLON to 1.
        UTILITY_SET_REGISTER_BIT(clk0.regmap->CR,RCC_CR_PLLON);

        //5. Enable the desired PLL outputs by configuring PLLPEN, PLLQEN, PLLREN in PLL configuration register (RCC_PLLCFGR).
        if ((config->output & CLOCK_OUTPUT_PLLR) && (config->pll.dividerR != CLOCK_PLLDIVIDER_R_DISABLED))
        {
            UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLCFGR,RCC_PLLCFGR_PLLREN);
        }
        if ((config->output & CLOCK_OUTPUT_PLLQ) && (config->pll.dividerQ != CLOCK_PLLDIVIDER_Q_DISABLED))
        {
            UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLCFGR,RCC_PLLCFGR_PLLQEN);
        }
        if ((config->output & CLOCK_OUTPUT_PLLP) && (config->pll.dividerP != CLOCK_PLLDIVIDER_P_DISABLED))
        {
            UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLCFGR,RCC_PLLCFGR_PLLPEN);
        }
        //6. Wait until PLLRDY is set. The PLL is now running.
        if ((config->output & CLOCK_OUTPUT_PLLR) || (config->output & CLOCK_OUTPUT_PLLQ) || (config->output & CLOCK_OUTPUT_PLLP))
        {
            tickstart = System_currentTick();
            while (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_PLLRDY) == 0)
            {
                if ((System_currentTick() - tickstart) > 5000u)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }

        //*** PLLSAI1, PLLSAI2 ***/
        //1. Disable the PLLSAI1/PLLSAI2 by setting PLLSAI1ON/PLLSAI2ON to 0 in Clock control register (RCC_CR).
        UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR,RCC_CR_PLLSAI1ON);
        UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR,RCC_CR_PLLSAI2ON);
        //2. Wait until PLLSAI1RDY/PLLSAI2RDY is cleared. The PLLSAI1/PLLSAI2 is now fully stopped.
        tickstart = System_currentTick();
        while ((UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_PLLSAI1RDY) == 1) || (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_PLLSAI2RDY) == 1))
        {
            if ((System_currentTick() - tickstart) > 5000u)
                return ERRORS_CLOCK_TIMEOUT;
        }
        //3. Change the desired parameter.
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N_Msk, (config->pllSai1.multiplier << RCC_PLLSAI1CFGR_PLLSAI1N_Pos)); // ->*N
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R_Msk, (config->pllSai1.dividerR << RCC_PLLSAI1CFGR_PLLSAI1R_Pos)); // /R->
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q_Msk, (config->pllSai1.dividerQ << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos)); // /Q->
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P_Msk, (config->pllSai1.dividerP << RCC_PLLSAI1CFGR_PLLSAI1P_Pos)); // /P->

        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2N_Msk, (config->pllSai2.multiplier << RCC_PLLSAI2CFGR_PLLSAI2N_Pos)); // ->*N
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2R_Msk, (config->pllSai2.dividerR << RCC_PLLSAI2CFGR_PLLSAI2R_Pos)); // /R->
        UTILITY_MODIFY_REGISTER(clk0.regmap->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2P_Msk, (config->pllSai2.dividerP << RCC_PLLSAI2CFGR_PLLSAI2P_Pos)); // /P->
        //4. Enable the PLLSAI1/PLLSAI2 again by setting PLLSAI1ON/PLLSAI2ON to 1.
        if (config->output & CLOCK_OUTPUT_PLLSAI1R)
        {
            UTILITY_SET_REGISTER_BIT(clk0.regmap->CR,RCC_CR_PLLSAI1ON);
        }
        if (config->output & CLOCK_OUTPUT_PLLSAI2R)
        {
            UTILITY_SET_REGISTER_BIT(clk0.regmap->CR,RCC_CR_PLLSAI2ON);
        }
        //5. Enable the desired PLL outputs by configuring PLLSAI1PEN/PLLSAI2PEN, PLLSAI1QEN/PLLSAI2QEN, PLLSAI1REN/PLLSAI2REN in PLLSAI1 configuration register (RCC_PLLSAI1CFGR) and PLLSAI2 configuration register (RCC_PLLSAI2CFGR).
        if ((config->output & CLOCK_OUTPUT_PLLSAI1R) && (config->pllSai1.dividerR != CLOCK_PLLDIVIDER_R_DISABLED))
        {
            UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLSAI1CFGR,RCC_PLLSAI1CFGR_PLLSAI1REN);
        }
        if ((config->output & CLOCK_OUTPUT_PLLSAI1Q) && (config->pllSai1.dividerQ != CLOCK_PLLDIVIDER_Q_DISABLED))
        {
            UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLSAI1CFGR,RCC_PLLSAI1CFGR_PLLSAI1QEN);
        }
        if ((config->output & CLOCK_OUTPUT_PLLSAI1P) && (config->pllSai1.dividerP != CLOCK_PLLDIVIDER_P_DISABLED))
        {
            UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLSAI1CFGR,RCC_PLLSAI1CFGR_PLLSAI1PEN);
        }
        if ((config->output & CLOCK_OUTPUT_PLLSAI2R) && (config->pllSai2.dividerR != CLOCK_PLLDIVIDER_R_DISABLED))
        {
            UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLSAI2CFGR,RCC_PLLSAI2CFGR_PLLSAI2REN);
        }
        if ((config->output & CLOCK_OUTPUT_PLLSAI2P) && (config->pllSai2.dividerP != CLOCK_PLLDIVIDER_P_DISABLED))
        {
            UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLSAI2CFGR,RCC_PLLSAI2CFGR_PLLSAI2PEN);
        }
        //6a. Wait until PLLSAI1RDY is set. The PLLSAI1 is now running.
        if ((config->output & CLOCK_OUTPUT_PLLSAI1R) ||
            (config->output & CLOCK_OUTPUT_PLLSAI1Q) ||
            (config->output & CLOCK_OUTPUT_PLLSAI1P))
        {
            tickstart = System_currentTick();
            while ((UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_PLLSAI1RDY) == 0))
            {
                if ((System_currentTick() - tickstart) > 5000u)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }
        //6b. Wait until PLLSAI2RDY is set. The PLLSAI2 is now running.
        if ((config->output & CLOCK_OUTPUT_PLLSAI2R) || (config->output & CLOCK_OUTPUT_PLLSAI2P))
        {
            tickstart = System_currentTick();
            while ((UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_PLLSAI2RDY) == 0))
            {
                if ((System_currentTick() - tickstart) > 5000u)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }
    }

    UTILITY_MODIFY_REGISTER(clk0.regmap->CFGR, RCC_CFGR_MCOSEL_Msk, (config->mcoSource << RCC_CFGR_MCOSEL_Pos));
    UTILITY_MODIFY_REGISTER(clk0.regmap->CFGR, RCC_CFGR_MCOPRE_Msk, (config->mcoPrescaler << RCC_CFGR_MCOPRE_Pos));

    return ERRORS_NO_ERROR;
}
#endif

static Clock_Config clockConfig;

Clock_Config* Clock_getConfig(void)
{
    return &clockConfig;
}

System_Errors Clock_init (Clock_Config* config)
{
    // Check and save external clock value
    // update value of external source
    // (this value can be used after for calculate clock)
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU) || \
    defined (LIBOHIBOARD_STM32L0x2)    || \
    defined (LIBOHIBOARD_STM32L0x3)
    if ((config->source == CLOCK_CRYSTAL) || (config->source == CLOCK_EXTERNAL))
    {
        ohiassert(CLOCK_IS_VALID_EXTERNAL_RANGE(config->fext));
        clk.externalClock = config->fext;
    }
    else
#endif
    {
        clk.externalClock = 0;
    }

    //Reset Clock state
    Clock_deInit();

    //calculate value of clock HCLK
    uint32_t currentHclk = Clock_getActualHclkValue();
    uint32_t newHclk= Clock_getConfigHclkValue(config);

    System_Errors err = ERRORS_NO_ERROR;

    if (config == NULL)
    {
        return ERRORS_CLOCK_NO_CONFIG;
    }

    // Check if selected oscillator is valid
    ohiassert(CLOCK_IS_VALID_OSCILLATOR(config->source));
    // Check whether user choice only one type of external reference: clock or crystal
    if ((config->source & (CLOCK_EXTERNAL | CLOCK_CRYSTAL)) != 0)
    {
        ohiassert(((config->source & CLOCK_EXTERNAL) == CLOCK_EXTERNAL) ^
                  ((config->source & CLOCK_CRYSTAL) == CLOCK_CRYSTAL));
    }

    // TODO: Check PLL conditions
#if 0
    if ((config->source & CLOCK_INTERNAL_PLL) ==  CLOCK_INTERNAL_PLL)
    {
        // LSI and LSE are not valid; MSI, HSI and HSE are valid.
        ohiassert(CLOCK_IS_VALID_PLL_SOURCE(config->pllSource));
        // The PLL output frequency must not exceed 80 MHz.
        ohiassert(CLOCK_IS_VALID_CONFIG_PLL_FREQUENCY(config, &config->pll));
        // The PLL output frequency must not exceed 80 MHz.
        ohiassert(CLOCK_IS_VALID_CONFIG_PLL_FREQUENCY(config, &config->pllSai1));
        // The PLL output frequency must not exceed 80 MHz.
        ohiassert(CLOCK_IS_VALID_CONFIG_PLL_FREQUENCY(config, &config->pllSai2));
    }
#endif

    // Setup default value of internal clock
    Clock_updateOutputValue();
    // Initialize SysTick with default clock value
    System_systickInit(0);

    // If frequency is increasing
    if ((newHclk > currentHclk) &&
        (newHclk > CLOCK_VOLTAGERANGE1_MAX_FREQ_WO_WAIT))
    {
        UTILITY_SET_REGISTER_BIT(clk.regmapFlash->ACR,FLASH_ACR_LATENCY);

        // Check that the new number of wait states is taken into account to access
        // the Flash memory by reading the FLASH_ACR register
        if (UTILITY_READ_REGISTER_BIT(clk.regmapFlash->ACR,FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY)
        {
            return ERRORS_CLOCK_WRONG_CONFIGURATION;
        }
    }

    err = Clock_oscillatorConfig(config);

    // Check if selected valid clock output
    ohiassert(CLOCK_IS_VALID_OUTPUT(config->output));

    err = Clock_outputConfig(config);
    if (err != ERRORS_NO_ERROR)
    {
        Clock_deInit();
    }

    // If frequency is decreasing
    if ((newHclk < currentHclk) &&
        (newHclk < CLOCK_VOLTAGERANGE1_MAX_FREQ_WO_WAIT))
    {
        UTILITY_CLEAR_REGISTER_BIT(clk.regmapFlash->ACR,FLASH_ACR_LATENCY);

        // Check that the new number of wait states is taken into account to access
        // the Flash memory by reading the FLASH_ACR register
        if (UTILITY_READ_REGISTER_BIT(clk.regmapFlash->ACR,FLASH_ACR_LATENCY) != 0u)
        {
            return ERRORS_CLOCK_WRONG_CONFIGURATION;
        }
    }

    // Setup default value of internal clock
    Clock_updateOutputValue();

    // Save current configuration
    clockConfig = *config;

    // Initialize SysTick with new value of HCLK
    System_systickInit(0);

    return err;
}

void Clock_setMsiRangeSwitching (Clock_MSIRange msi)
{
    if (ohiassert(CLOCK_IS_VALID_MSI_RANGE(msi)) == ERRORS_NO_ERROR)
    {
        clk.deInitRange = msi;
    }
    else
    {
        clk.deInitRange = CLOCK_MSIRANGE_2097kHz;
    }
}

static bool Clock_isFrequencyInMsiRange (uint32_t frequency, Clock_MSIRange* msiRange)
{
	for (uint8_t i = 0; i < UTILITY_DIMOF(CLOCK_MSI_RANGE); i++)
	{
		if (CLOCK_MSI_RANGE[i] == frequency)
		{
			*msiRange = (Clock_MSIRange)(i << RCC_ICSCR_MSIRANGE_Pos);
			return TRUE;
		}
	}
	return FALSE;
}

#if 0
static bool Clock_isFrequencyInPllRange (uint32_t frequency, Clock_PllSource *pllSource, Clock_PLLPrescaler* pllPrescaler, Clock_PLLConfig *pllConfig)
{
	uint32_t pllSourceClk = 0, sysClk = 0;
	int mIdx = 0, nIdx = 0, rIdx = 0;
	uint16_t mSet[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	uint16_t nSet[78] = {0};
	uint16_t rSet[4] = {2, 4, 6, 8};

	for (nIdx = 0; nIdx < sizeof(nSet); nIdx++)
	{
		nSet[nIdx] = nIdx + 8;
	}

	// HSE
	if (clk0.externalClock != 0)
	{
		pllSourceClk = clk0.externalClock;
		for (mIdx = 0; mIdx < sizeof(mSet); mIdx++)
		{
			for (nIdx = 0; nIdx < sizeof(nSet); nIdx++)
			{
				for (rIdx = 0; rIdx < sizeof(rSet); rIdx++)
				{
					sysClk = pllSourceClk / mSet[mIdx] * nSet[nIdx] / rSet[rIdx];
					if (sysClk == frequency)
					{
						*pllSource = CLOCK_PLLSOURCE_HSE;
						*pllPrescaler = (Clock_PLLPrescaler)mIdx;
						pllConfig->multiplier = (Clock_PLLMultiplier)nSet[nIdx];
						pllConfig->dividerR = (Clock_PLLDividerR)rIdx;
						return TRUE;
					}
				}
			}
		}
	}

	// HSI
	pllSourceClk = CLOCK_FREQ_HSI;
	for (mIdx = 0; mIdx < sizeof(mSet); mIdx++)
	{
		for (nIdx = 0; nIdx < sizeof(nSet); nIdx++)
		{
			for (rIdx = 0; rIdx < sizeof(rSet); rIdx++)
			{
				sysClk = pllSourceClk / mSet[mIdx] * nSet[nIdx] / rSet[rIdx];
				if (sysClk == frequency)
				{
					*pllSource = CLOCK_PLLSOURCE_HSI;
					*pllPrescaler = (Clock_PLLPrescaler)mIdx;
					pllConfig->multiplier = (Clock_PLLMultiplier)nSet[nIdx];
					pllConfig->dividerR = (Clock_PLLDividerR)rIdx;
					return TRUE;
				}
			}
		}
	}

	// MSI
	uint16_t i = 0;
	for (i = 0; i < UTILITY_DIMOF(Clock_msiRange); i++)
	{
		pllSourceClk = Clock_msiRange[i];
		for (mIdx = 0; mIdx < sizeof(mSet); mIdx++)
		{
			for (nIdx = 0; nIdx < sizeof(nSet); nIdx++)
			{
				for (rIdx = 0; rIdx < sizeof(rSet); rIdx++)
				{
					sysClk = pllSourceClk / mSet[mIdx] * nSet[nIdx] / rSet[rIdx];
					if (sysClk == frequency)
					{
						*pllSource = CLOCK_PLLSOURCE_MSI;
						*pllPrescaler = (Clock_PLLPrescaler)mIdx;
						pllConfig->multiplier = (Clock_PLLMultiplier)nSet[nIdx];
						pllConfig->dividerR = (Clock_PLLDividerR)rIdx;
						return TRUE;
					}
				}
			}
		}
	}

	return FALSE;
}

#endif

static System_Errors Clock_deInit (void)
{
    uint32_t tickstart = 0;

    // Set internal default value
    UTILITY_MODIFY_REGISTER(clk.regmap->ICSCR, \
                           (RCC_ICSCR_MSITRIM | RCC_ICSCR_HSITRIM | RCC_ICSCR_MSIRANGE),\
                           (CLOCK_MSI_DEFAULT_TRIM_VALUE | CLOCK_HSI_DEFAULT_TRIM_VALUE | clk.deInitRange));

    // Set MSI ON
    UTILITY_SET_REGISTER_BIT(clk.regmap->CR, RCC_CR_MSION);

    // Wait until MSI is ready
    tickstart = System_currentTick();
    while (UTILITY_READ_REGISTER_BIT(clk.regmap->CR, RCC_CR_MSION) == 0)
    {
        if ((System_currentTick() - tickstart) > CLOCK_SOURCE_SWITCH_TIMEOUT_VALUE)
            return ERRORS_CLOCK_TIMEOUT;
    }

    // Clear RCC Configuration (MSI is selected as system clock)
    UTILITY_WRITE_REGISTER(clk.regmap->CFGR, 0);

    // Setup default value of internal clock
    Clock_updateOutputValue();
    // Initialize SysTick with default clock value (MSI @ selected range)
    System_systickInit(0);

    // Wait until system clock is ready
    tickstart = System_currentTick();
    while (UTILITY_READ_REGISTER_BIT(clk.regmap->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI)
    {
        if ((System_currentTick() - tickstart) > 5000u)
            return ERRORS_CLOCK_TIMEOUT;
    }

    // Clear/disable HSI, HSE, CSS and PLL
    UTILITY_CLEAR_REGISTER_BIT(clk.regmap->CR, RCC_CR_HSION | RCC_CR_HSIKERON | RCC_CR_HSIDIVEN | RCC_CR_HSIOUTEN | \
                                               RCC_CR_HSEON | RCC_CR_CSSHSEON | RCC_CR_PLLON );

    // Read register just like a nop
    (void)UTILITY_READ_REGISTER_BIT(clk.regmap->CR,RCC_CR_HSEON);

    // Reset HSEBYP
    UTILITY_CLEAR_REGISTER_BIT(clk.regmap->CR, RCC_CR_HSEBYP);

    // Wait until PLLs are reset
    tickstart = System_currentTick();
    while (UTILITY_READ_REGISTER_BIT(clk.regmap->CR, RCC_CR_PLLRDY) != 0)
    {
        if ((System_currentTick() - tickstart) > 5000u)
            return ERRORS_CLOCK_TIMEOUT;
    }


    // Disable all interrupts
    UTILITY_WRITE_REGISTER(clk.regmap->CIER, 0u);
    // Clear all interrupts
    UTILITY_WRITE_REGISTER(clk.regmap->CICR, 0xFFFFFFFFu);

    // Clear reset flags
    UTILITY_SET_REGISTER_BIT(clk.regmap->CSR, RCC_CSR_RMVF);

    // No wait states at default startup
    UTILITY_CLEAR_REGISTER_BIT(clk.regmapFlash->ACR,FLASH_ACR_LATENCY);

    // return NO_ERROR
    return ERRORS_NO_ERROR;
}

uint32_t Clock_getOutputValue (Clock_Output output)
{
    switch (output)
    {
    case CLOCK_OUTPUT_SYSCLK:
        return clk.systemCoreClock;
    case CLOCK_OUTPUT_HCLK:
        return clk.hclkClock;
    case CLOCK_OUTPUT_PCLK1:
        return clk.pclk1Clock;
    case CLOCK_OUTPUT_PCLK2:
        return clk.pclk2Clock;

    default:
        return 0;
    }
}

uint32_t Clock_getOscillatorValue (void)
{
    return clk.systemCoreClock;
}

uint32_t Clock_getConfigOscillatorValue (Clock_Config *config)
{
    uint32_t systemClock = 0;

    switch (config->sysSource)
    {
    default:
    case CLOCK_SYSTEMSOURCE_MSI:
        systemClock = Clock_getConfigMsiValue(config);
        break;
    case CLOCK_SYSTEMSOURCE_HSI:
        if (config->hsiDivider == CLOCK_HSIDIVIDER_4)
            systemClock = (CLOCK_FREQ_HSI / 4);
        else
            systemClock = CLOCK_FREQ_HSI;
        break;
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU) || \
    defined (LIBOHIBOARD_STM32L0x2)    || \
    defined (LIBOHIBOARD_STM32L0x3)
    case CLOCK_SYSTEMSOURCE_HSE:
//        systemClock = clk0.externalClock;
#endif
        break;
    case CLOCK_SYSTEMSOURCE_PLL:
//        systemClock = Clock_getConfigPllValue(config, &config->pll);
        break;
    }

    return systemClock;
}

#if 0
static uint32_t Clock_getActualPllInputValue(void)
{
    uint32_t baseClock = 0, pllmClock = 0;
    Clock_PllSource pllSource = (Clock_PllSource)(UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLSRC_Msk));
    uint32_t pllmReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos;

    switch (pllSource)
    {
    default:
    case CLOCK_PLLSOURCE_NONE:
        baseClock = 0;
        break;
    case CLOCK_PLLSOURCE_MSI:
        baseClock = Clock_getActualMsiValue();
        break;
    case CLOCK_PLLSOURCE_HSI:
        baseClock = CLOCK_FREQ_HSI;
        break;
    case CLOCK_PLLSOURCE_HSE:
        baseClock = clk0.externalClock;
        break;
    }

    pllmClock = ((baseClock) / (pllmReg + 1));
    return pllmClock;
}

static uint32_t Clock_getConfigPllValue (Clock_Config* config, Clock_PLLConfig *pllConfig)
{
    uint32_t base = 0, frequency = 0;

    switch (config->pllSource)
    {
    default:
    case CLOCK_PLLSOURCE_NONE:
        return 0;

    case CLOCK_PLLSOURCE_MSI:
        base = Clock_msiRange[config->msiRange];
        break;

    case CLOCK_PLLSOURCE_HSI:
        base = CLOCK_FREQ_HSI;
        break;

    case CLOCK_PLLSOURCE_HSE:
        base = clk0.externalClock;
        break;
    }

    frequency = base / (config->pllPrescaler + 1);
    frequency *= pllConfig->multiplier;

    switch (pllConfig->dividerR)
    {
    case CLOCK_PLLDIVIDER_R_2:
        frequency /= 2;
        break;

    case CLOCK_PLLDIVIDER_R_4:
        frequency /= 4;
        break;

    case CLOCK_PLLDIVIDER_R_6:
        frequency /= 6;
        break;

    default:
    case CLOCK_PLLDIVIDER_R_8:
        frequency /= 8;
        break;
    }

    return frequency;
}

#endif

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif
