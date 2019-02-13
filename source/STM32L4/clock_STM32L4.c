/*
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/source/STM32L4/clock_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Leonardo Morichelli
 * @brief Clock implementations for STM32L4 Series
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "clock.h"
#include "utility.h"
#include "system.h"

#if defined (LIBOHIBOARD_STM32L4)

#define CLOCK_IS_VALID_OSCILLATOR(OSC)  (OSC != CLOCK_NO_SOURCE)                                            && \
                                        (((OSC & CLOCK_EXTERNAL) == CLOCK_EXTERNAL)                         || \
                                         ((OSC & CLOCK_CRYSTAL) == CLOCK_CRYSTAL)                           || \
                                         ((OSC & CLOCK_INTERNAL_LSI) == CLOCK_INTERNAL_LSI)                 || \
                                         ((OSC & CLOCK_INTERNAL_HSI) == CLOCK_INTERNAL_HSI)                 || \
                                         ((OSC & CLOCK_INTERNAL_MSI) == CLOCK_INTERNAL_MSI)                 || \
                                         ((OSC & CLOCK_EXTERNAL_LSE_CRYSTAL) == CLOCK_EXTERNAL_LSE_CRYSTAL))

#define CLOCK_IS_VALID_PLL_SOURCE(PLL_SOURCE) ((PLL_SOURCE == CLOCK_PLLSOURCE_HSI) || \
                                               (PLL_SOURCE == CLOCK_PLLSOURCE_HSE) || \
                                               (PLL_SOURCE == CLOCK_PLLSOURCE_MSI) || \
                                               (PLL_SOURCE == CLOCK_PLLSOURCE_NONE))

#define CLOCK_IS_VALID_PLL_FREQUENCY(FREQ) ((FREQ >= CLOCK_MIN_FREQ_MSI && FREQ <= CLOCK_MAX_FREQ_PLL)?(TRUE):(FALSE))

#define CLOCK_IS_VALID_CONFIG_PLL_FREQUENCY(OSC_CONFIG, PLL_CONFIG) (Clock_getConfigPllValue(OSC_CONFIG, PLL_CONFIG) <= CLOCK_MAX_FREQ_PLL)

#define CLOCK_IS_VALID_HSE_STATE(HSESTATE) (((HSESTATE) == CLOCK_OSCILLATORSTATE_OFF) || \
                                            ((HSESTATE) == CLOCK_OSCILLATORSTATE_ON))

#define CLOCK_IS_VALID_HSI_STATE(HSISTATE) (((HSISTATE) == CLOCK_OSCILLATORSTATE_OFF) || \
                                            ((HSISTATE) == CLOCK_OSCILLATORSTATE_ON))

#define CLOCK_IS_VALID_LSI_STATE(LSISTATE) (((LSISTATE) == CLOCK_OSCILLATORSTATE_OFF) || \
                                            ((LSISTATE) == CLOCK_OSCILLATORSTATE_ON))

#define CLOCK_IS_VALID_LSE_STATE(LSESTATE) (((LSESTATE) == CLOCK_OSCILLATORSTATE_OFF) || \
                                            ((LSESTATE) == CLOCK_OSCILLATORSTATE_ON))

#define CLOCK_IS_VALID_EXTERNAL_RANGE(VALUE) ((VALUE >= CLOCK_MIN_FREQ_HSE) && (VALUE <= CLOCK_MAX_FREQ_HSE))

#define CLOCK_IS_VALID_OUTPUT(OUTPUT) (((OUTPUT & CLOCK_OUTPUT_SYSCLK) == CLOCK_OUTPUT_SYSCLK) || \
                                       ((OUTPUT & CLOCK_OUTPUT_HCLK)   == CLOCK_OUTPUT_HCLK)   || \
                                       ((OUTPUT & CLOCK_OUTPUT_PCLK1)  == CLOCK_OUTPUT_PCLK1)  || \
                                       ((OUTPUT & CLOCK_OUTPUT_PCLK2)  == CLOCK_OUTPUT_PCLK2))

#define CLOCK_IS_VALID_SYSSOURCE(SYSSOURCE) (((SYSSOURCE & CLOCK_SYSTEMSOURCE_HSI) == CLOCK_SYSTEMSOURCE_HSI) || \
                                             ((SYSSOURCE & CLOCK_SYSTEMSOURCE_HSE) == CLOCK_SYSTEMSOURCE_HSE) || \
                                             ((SYSSOURCE & CLOCK_SYSTEMSOURCE_PLL) == CLOCK_SYSTEMSOURCE_PLL) || \
                                             ((SYSSOURCE & CLOCK_SYSTEMSOURCE_MSI) == CLOCK_SYSTEMSOURCE_MSI))

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


typedef struct _Clock_Device
{
    RCC_TypeDef* regmap;
    FLASH_TypeDef* regmapFlash;

    uint32_t systemCoreClock; /**< Value that store current system core clock */
    uint32_t hclkClock;
    uint32_t pclk1Clock;
    uint32_t pclk2Clock;

    uint32_t pllrClock;
    uint32_t pllqClock;
    uint32_t pllpClock;
    uint32_t pllsai1rClock;
    uint32_t pllsai1qClock;
    uint32_t pllsai1pClock;
    uint32_t pllsai2rClock;
    uint32_t pllsai2pClock;

    uint32_t externalClock;           /**< Oscillator or external clock value */

    uint8_t devLSIInitialized;
    uint8_t devLSEInitialized;
    uint8_t devHSEInitialized;
    uint8_t devHSIInitialized;
    uint8_t devPLLInitialized;

    Clock_MSIRange deinitRange;

    System_Errors rccError;

} Clock_Device;

static Clock_Device clk0 =
{
    .regmap            = RCC,
    .regmapFlash       = FLASH,

    .systemCoreClock   = 4000000U,
    .hclkClock         = 0U,
    .pclk1Clock        = 0U,
    .pclk2Clock        = 0U,

    .externalClock     = 0U,

    .devLSIInitialized = 0,
    .devLSEInitialized = 0,
    .devHSIInitialized = 0,
    .devHSEInitialized = 0,
    .devPLLInitialized = 0,

    .deinitRange       = CLOCK_MSIRANGE_4MHz,

    .rccError          = ERRORS_NO_ERROR,
};

/**
 * Useful constant to define default value of MSI oscillator in hertz.
 */
static const uint32_t Clock_msiRange[]  =
{
    100000,
    200000,
    400000,
    800000,
    1000000,
    2000000,
    4000000,
    8000000,
    16000000,
    24000000,
    32000000,
    48000000,
};

/**
 * Useful constant to define default value of flash latency based on clock speed.
 */
static const uint32_t Clock_flashLatency[] =
{
    16000000,
    32000000,
    48000000,
    64000000,
    80000000,
};

/**
 * Deinitialize Clock configuration registers.
 *
 * @return ERRORS_NO_ERROR without problems
 * @note This function is used before set a new clock configuration.
 */
static System_Errors Clock_deInit (void);

/**
 * Return the clock provided by MSI set in register configuration.
 *
 * @return value of MSI clock.
 */
static uint32_t Clock_getActualMsiValue (void);

/**
 * Return the clock set for MSI in configuration struct.
 *
 * @param[in] config configuration struct.
 *
 * @return value of MSI clock
 */
static uint32_t Clock_getConfigMsiValue (Clock_Config *config);

/**
 * Return the source clock of PLL.
 *
 * @return value of PLL source clock.
 */
static uint32_t Clock_getActualPllInputValue (void);

/**
 * Return the source clock of PLL set in configuration struct.
 *
 * @param[in] config configuration struct.
 *
 * @return value of source clock of PLL.
 */
static uint32_t Clock_getConfigPllValue (Clock_Config *config, Clock_PLLConfig *pllConfig);

/**
 * Return the system clock set in register configuration.
 *
 * @return value of SYSCLK clock.
 */
static uint32_t Clock_getActualSystemValue (void);

/**
 * Set the proper flash latency for frequency specified.
 *
 * @param[in] frequency the new value of system clock.
 * @note This function is used in Clock_init(). It must be called
 *       before of change clock if new frequency is major then actual,
 *       after if otherwise.
 * @note The frequency latency
 */
static void Clock_setProperFlashLatency (uint32_t frequency)
{
    int latency = 0;
    while (latency < UTILITY_DIMOF(Clock_flashLatency))
    {
        if (frequency <= Clock_flashLatency[latency])
        {
            UTILITY_MODIFY_REGISTER(clk0.regmapFlash->ACR, FLASH_ACR_LATENCY_Msk, (latency << FLASH_ACR_LATENCY_Pos));
            break;
        }
        latency++;
    }
}

static void Clock_updateOutputValue (void)
{
    clk0.systemCoreClock = Clock_getActualSystemValue();
    uint32_t pllmClock = Clock_getActualPllInputValue();

    // Compute HCLK, PCLK1 and PCLK2
    uint32_t cfgr = clk0.regmap->CFGR;
    uint32_t shifter = UTILITY_READ_REGISTER_BIT(cfgr,RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
    clk0.hclkClock = (clk0.systemCoreClock >> CLOCK_AHB_PRESCALE_SHIFT_TABLE[shifter]);

    shifter = UTILITY_READ_REGISTER_BIT(cfgr,RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    clk0.pclk1Clock = (clk0.hclkClock >> CLOCK_APB_PRESCALE_SHIFT_TABLE[shifter]);

    shifter = UTILITY_READ_REGISTER_BIT(cfgr,RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    clk0.pclk2Clock = (clk0.hclkClock >> CLOCK_APB_PRESCALE_SHIFT_TABLE[shifter]);

    uint32_t pllnReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
    uint32_t pllrReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos;
    uint32_t pllqReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos;
    uint32_t pllpReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos;
    clk0.pllrClock = (((pllmClock) * pllnReg) / ((pllrReg + 1) * 2));
    clk0.pllqClock = (((pllmClock) * pllnReg) / ((pllqReg + 1) * 2));
    clk0.pllpClock = (((pllmClock) * pllnReg) / ((pllpReg == 0)?(7):(17)));

    uint32_t pllsai1nReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N_Msk) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos;
    uint32_t pllsai1rReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R_Msk) >> RCC_PLLSAI1CFGR_PLLSAI1R_Pos;
    uint32_t pllsai1qReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R_Msk) >> RCC_PLLSAI1CFGR_PLLSAI1R_Pos;
    uint32_t pllsai1pReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R_Msk) >> RCC_PLLSAI1CFGR_PLLSAI1R_Pos;
    clk0.pllsai1rClock = (((pllmClock) * pllsai1nReg) / ((pllsai1rReg + 1) * 2));
    clk0.pllsai1qClock = (((pllmClock) * pllsai1nReg) / ((pllsai1qReg + 1) * 2));
    clk0.pllsai1pClock = (((pllmClock) * pllsai1nReg) / ((pllsai1pReg == 0)?(7):(17)));

    uint32_t pllsai2nReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2N_Msk) >> RCC_PLLSAI2CFGR_PLLSAI2N_Pos;
    uint32_t pllsai2rReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2R_Msk) >> RCC_PLLSAI2CFGR_PLLSAI2R_Pos;
    uint32_t pllsai2pReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2R_Msk) >> RCC_PLLSAI2CFGR_PLLSAI2R_Pos;
    clk0.pllsai2rClock = (((pllmClock) * pllsai2nReg) / ((pllsai2rReg + 1) * 2));
    clk0.pllsai2pClock = (((pllmClock) * pllsai2nReg) / ((pllsai2pReg == 0)?(7):(17)));
}

static System_Errors Clock_oscillatorConfig (Clock_Config* config)
{
    uint32_t tickstart = 0;

    if ((config->source & CLOCK_INTERNAL_MSI) == CLOCK_INTERNAL_MSI)
    {
        // Set MSI ON
        UTILITY_SET_REGISTER_BIT(clk0.regmap->CR, RCC_CR_MSION);

        // Wait until MSI is ready
        tickstart = System_currentTick();
        while (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_MSION) == 0)
        {
            if ((System_currentTick() - tickstart) > 5000u)
                return ERRORS_CLOCK_TIMEOUT;
        }

        // Set MSIRANGE  default value
        UTILITY_SET_REGISTER_BIT(clk0.regmap->CR, RCC_CR_MSIRGSEL);
        UTILITY_MODIFY_REGISTER(clk0.regmap->CR, RCC_CR_MSIRANGE, config->msiRange);

        // Clear RCC Configuration (MSI is selected as system clock)
        UTILITY_WRITE_REGISTER(clk0.regmap->CFGR, 0);
    }

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

    // HSI Configuration
    if ((config->source & CLOCK_INTERNAL_HSI) == CLOCK_INTERNAL_HSI)
    {
        // Check the HSI state value
        ohiassert(CLOCK_IS_VALID_HSI_STATE(config->hsiState));

        // Check if HSI is just used as SYSCLK or as PLL source
        // In this case we can't disable it
        if (((clk0.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI) ||
           (((clk0.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) &&
            ((clk0.regmap->PLLCFGR & RCC_PLLCFGR_PLLSRC) == CLOCK_PLLSOURCE_HSI)))
        {
            return ERRORS_CLOCK_WRONG_CONFIGURATION;
        }
        else
        {
            if (config->hsiState == CLOCK_OSCILLATORSTATE_OFF)
            {
                // Switch off the oscillator
                UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSION);

                // Wait until the HSERDY bit is cleared
                tickstart = System_currentTick();
                while ((clk0.regmap->CR & RCC_CR_HSIRDY) > 0)
                {
                    if ((System_currentTick() - tickstart) > 5000u)
                        return ERRORS_CLOCK_TIMEOUT;
                }
            }
            else
            {
                // Switch on the oscillator
                UTILITY_SET_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSION);

                // Wait until the HSERDY bit is set
                tickstart = System_currentTick();
                while ((clk0.regmap->CR & RCC_CR_HSIRDY) == 0)
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

    // LSE Configuration
    // See page 272 of RM0351 for informations about backup domain control
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

        // Disable write protection
        CLOCK_BACKUP_DISABLE_WRITE_PROTECTION();
        // Wait some cycle...
        asm("NOP");
        asm("NOP");

        if (config->lseState == CLOCK_OSCILLATORSTATE_OFF)
        {
            // Switch off the oscillator
            UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->BDCR,RCC_BDCR_LSEON);

            // Wait until LSE is disabled
            tickstart = System_currentTick();
            while ((clk0.regmap->BDCR & RCC_BDCR_LSERDY) > 0)
            {
                if ((System_currentTick() - tickstart) > 5000u)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }
        else
        {
            // Switch on the oscillator
            UTILITY_SET_REGISTER_BIT(clk0.regmap->BDCR,RCC_BDCR_LSEON);

            // Wait until LSE is ready
            tickstart = System_currentTick();
            while ((clk0.regmap->BDCR & RCC_BDCR_LSERDY) == 0)
            {
                if ((System_currentTick() - tickstart) > 5000u)
                    return ERRORS_CLOCK_TIMEOUT;
            }
        }

        // Restore PWR status
        if (isPwrChanged == TRUE)
        {
            CLOCK_DISABLE_PWR();
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
            // Check if the source is ready
            if (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR,RCC_CR_PLLRDY) == 0)
            {
                return ERRORS_CLOCK_PLL_NOT_READY;
            }

            cfgrSW = RCC_CFGR_SW_PLL;
        }
        else if (config->sysSource == CLOCK_SYSTEMSOURCE_HSI)
        {
            // Check if the source is ready
            if (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSIRDY) == 0)
            {
                return ERRORS_CLOCK_HSI_NOT_READY;
            }

            cfgrSW = RCC_CFGR_SW_HSI;
        }
        else if (config->sysSource == CLOCK_SYSTEMSOURCE_HSE)
        {
            // Check if the source is ready
            if (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR,RCC_CR_HSERDY) == 0)
            {
                return ERRORS_CLOCK_HSE_NOT_READY;
            }

            // Save chose value for CFGR register
            cfgrSW = RCC_CFGR_SW_HSE;
        }
        else if (config->sysSource == CLOCK_SYSTEMSOURCE_MSI)
        {
            // Check if the source is ready
            if (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR,RCC_CR_MSIRDY) == 0)
            {
                return ERRORS_CLOCK_MSI_NOT_READY;
            }

            cfgrSW = RCC_CFGR_SW_MSI;
        }

        // Clear SW part of CFGR register and updated informations
        clk0.regmap->CFGR &= ~(RCC_CFGR_SW_Msk);
        clk0.regmap->CFGR |= cfgrSW;

        // Check if new value was setted
        uint32_t tickstart = System_currentTick();
        while (UTILITY_READ_REGISTER_BIT(clk0.regmap->CFGR,RCC_CFGR_SWS) != (cfgrSW << RCC_CFGR_SWS_Pos))
        {
            if ((System_currentTick() - tickstart) > 5000u)
                return ERRORS_CLOCK_TIMEOUT;
        }
    }

    if ((config->output & CLOCK_OUTPUT_HCLK) == CLOCK_OUTPUT_HCLK)
    {
        // Check if divider is valid
        ohiassert(CLOCK_IS_VALID_AHB_DIVIDER(config->ahbDivider));

        // Clear HPRE part of CFGR register and updated informations
        clk0.regmap->CFGR &= ~(RCC_CFGR_HPRE_Msk);
        clk0.regmap->CFGR |= CLOCK_AHB_PRESCALE_REGISTER_TABLE[config->ahbDivider];
    }

    if ((config->output & CLOCK_OUTPUT_PCLK1) == CLOCK_OUTPUT_PCLK1)
    {
        // Check if divider is valid
        ohiassert(CLOCK_IS_VALID_APB_DIVIDER(config->apb1Divider));

        // Clear PPRE1 part of CFGR register and updated informations
        clk0.regmap->CFGR &= ~(RCC_CFGR_PPRE1_Msk);
        clk0.regmap->CFGR |= CLOCK_APB1_PRESCALE_REGISTER_TABLE[config->apb1Divider];
    }

    if ((config->output & CLOCK_OUTPUT_PCLK2) == CLOCK_OUTPUT_PCLK2)
    {
        // Check if divider is valid
        ohiassert(CLOCK_IS_VALID_APB_DIVIDER(config->apb2Divider));

        // Clear PPRE2 part of CFGR register and updated informations
        clk0.regmap->CFGR &= ~(RCC_CFGR_PPRE2_Msk);
        clk0.regmap->CFGR |= CLOCK_APB2_PRESCALE_REGISTER_TABLE[config->apb2Divider];
    }

    // Update system core clock informations...
    Clock_updateOutputValue();

    return ERRORS_NO_ERROR;
}

System_Errors Clock_init (Clock_Config* config)
{
    // Check and save external clock value
    // update value of external source
    // (this value can be used after for calculate clock)
    if ((config->source == CLOCK_CRYSTAL) || (config->source == CLOCK_EXTERNAL))
    {
        ohiassert(CLOCK_IS_VALID_EXTERNAL_RANGE(config->fext));
        clk0.externalClock = config->fext;
    }
    else
    {
        clk0.externalClock = 0;
    }

    //Reset Clock state
    Clock_deInit();

    //calculate value of clock
    uint32_t currentSystemClock = Clock_getActualSystemValue();
    uint32_t newSystemClock = Clock_getConfigOscillatorValue(config);

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

    // Check PLL conditions
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

    // Setup default value of internal clock
    Clock_updateOutputValue();
    // Initialize SysTick with default clock value (MSI @ 4MHz)
    System_systickInit(0);

    // if frequency is increasing
    if (newSystemClock > currentSystemClock)
    {
        // set flash latency before to change clock
        Clock_setProperFlashLatency(newSystemClock);
    }

    err = Clock_oscillatorConfig(config);

    // Check if selected valid clock output
    ohiassert(CLOCK_IS_VALID_OUTPUT(config->output));

    err = Clock_outputConfig(config);
    if (err != ERRORS_NO_ERROR)
    {
        Clock_deInit();
    }

    // if frequency is decreasing
    if(newSystemClock < currentSystemClock)
    {
        // set flash latency after changed clock
        Clock_setProperFlashLatency(newSystemClock);
    }

    // Setup default value of internal clock
    Clock_updateOutputValue();
    // Initialize SysTick with new value of HCLK
    System_systickInit(0);

    return err;
}

void Clock_setMsiRangeSwitching (Clock_MSIRange msi)
{
    if ((msi >= CLOCK_MSIRANGE_100KHz) && (msi <= CLOCK_MSIRANGE_24MHz))
    {
        clk0.deinitRange = msi;
    }
}

static bool Clock_isFrequencyInMsiRange (uint32_t frequency, Clock_MSIRange* pMsirange)
{
	uint16_t i = 0;
	for (i = 0; i < UTILITY_DIMOF(Clock_msiRange); i++)
	{
		if (Clock_msiRange[i] == frequency)
		{
			*pMsirange = (Clock_MSIRange)(i << RCC_CR_MSIRANGE_Pos);
			return TRUE;
		}
	}
	return FALSE;
}

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

System_Errors Clock_setFrequency (uint32_t frequency)
{
	System_Errors err = ERRORS_NO_ERROR;

	Clock_Config clkConfig;
	memset(&clkConfig, 0, sizeof(clkConfig));
	clkConfig.source = CLOCK_INTERNAL_MSI;
	clkConfig.output = CLOCK_OUTPUT_SYSCLK | CLOCK_OUTPUT_HCLK | CLOCK_OUTPUT_PCLK1 | CLOCK_OUTPUT_PCLK2 | CLOCK_OUTPUT_PLLR;
	clkConfig.ahbDivider = CLOCK_AHBDIVIDER_1;
	clkConfig.apb1Divider = CLOCK_APBDIVIDER_1;
	clkConfig.apb2Divider = CLOCK_APBDIVIDER_1;
	clkConfig.msiRange = CLOCK_MSIRANGE_4MHz;

	clkConfig.pllSource = CLOCK_PLLSOURCE_HSI;
	clkConfig.pllPrescaler = CLOCK_PLLPRESCALER_1;
	clkConfig.pll.multiplier = CLOCK_PLLMULTIPLIER_10;
	clkConfig.pll.dividerR = CLOCK_PLLDIVIDER_R_2;
	clkConfig.pll.dividerQ = CLOCK_PLLDIVIDER_Q_DISABLED;
	clkConfig.pll.dividerP = CLOCK_PLLDIVIDER_P_DISABLED;

	clkConfig.pll.multiplier = CLOCK_PLLMULTIPLIER_10;
	clkConfig.pllSai1.multiplier = CLOCK_PLLMULTIPLIER_10;
	clkConfig.pllSai2.multiplier = CLOCK_PLLMULTIPLIER_10;
	clkConfig.mcoPrescaler = CLOCK_PLLPRESCALER_1;
	clkConfig.mcoSource = CLOCK_MCOSOURCE_DISABLED;

	// Check value of frequency, if it is out of range
	// normalize the value
	ohiassert(CLOCK_IS_VALID_PLL_FREQUENCY(frequency));
	if (frequency < CLOCK_MIN_FREQ_MSI)
	{
		frequency = CLOCK_MIN_FREQ_MSI;
	}

	if (frequency > CLOCK_MAX_FREQ_PLL)
	{
		frequency = CLOCK_MAX_FREQ_PLL;
	}

	if (frequency > 2000000)
	{
		Clock_setMsiRangeSwitching(CLOCK_MSIRANGE_24MHz);
	}
	else
	{
		Clock_setMsiRangeSwitching(CLOCK_MSIRANGE_4MHz);
	}

	if (clk0.externalClock != 0 && frequency == clk0.externalClock)
	{
		clkConfig.source |= CLOCK_EXTERNAL;
		clkConfig.sysSource = CLOCK_SYSTEMSOURCE_HSE;
		clkConfig.hseState = CLOCK_OSCILLATORSTATE_ON;

		err = Clock_init(&clkConfig);
	}
	else if (frequency == CLOCK_FREQ_HSI)
	{
		clkConfig.source |= CLOCK_INTERNAL_HSI;
		clkConfig.sysSource = CLOCK_SYSTEMSOURCE_HSI;
		clkConfig.hsiState = CLOCK_OSCILLATORSTATE_ON;

		err = Clock_init(&clkConfig);
	}
	else if (frequency == CLOCK_MAX_FREQ_PLL)
	{
		clkConfig.source |= (CLOCK_INTERNAL_PLL|CLOCK_INTERNAL_HSI);
		clkConfig.sysSource = CLOCK_SYSTEMSOURCE_PLL;
		clkConfig.pllState = CLOCK_OSCILLATORSTATE_ON;

		clkConfig.hsiState = CLOCK_OSCILLATORSTATE_ON;
		clkConfig.pllSource = CLOCK_PLLSOURCE_HSI;
		clkConfig.pllPrescaler = CLOCK_PLLPRESCALER_1;
		clkConfig.pll.multiplier = CLOCK_PLLMULTIPLIER_10;
		clkConfig.pll.dividerR = CLOCK_PLLDIVIDER_R_2;

		err = Clock_init(&clkConfig);
	}
	else if (Clock_isFrequencyInMsiRange(frequency, &clkConfig.msiRange))
	{
		clkConfig.source |= CLOCK_INTERNAL_MSI;
		clkConfig.sysSource = CLOCK_SYSTEMSOURCE_MSI;
		clkConfig.msiState = CLOCK_OSCILLATORSTATE_ON;

		err = Clock_init(&clkConfig);
	}
	else if (Clock_isFrequencyInPllRange(frequency, &clkConfig.pllSource, &clkConfig.pllPrescaler, &clkConfig.pll))
	{
		clkConfig.source |= (CLOCK_INTERNAL_PLL|CLOCK_INTERNAL_HSI);
		clkConfig.sysSource = CLOCK_SYSTEMSOURCE_PLL;
		clkConfig.pllState = CLOCK_OSCILLATORSTATE_ON;

		err = Clock_init(&clkConfig);
	}
	else
	{
		err = ERRORS_CLOCK_FREQ_OUT_OF_RANGE;
	}

	return err;
}

static System_Errors Clock_deInit (void)
{
    uint32_t tickstart = 0;

    // Set MSI ON
    UTILITY_SET_REGISTER_BIT(clk0.regmap->CR, RCC_CR_MSION);

    // Wait until MSI is ready
    tickstart = System_currentTick();
    while (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_MSION) == 0)
    {
        if ((System_currentTick() - tickstart) > 5000u)
            return ERRORS_CLOCK_TIMEOUT;
    }

    // Set MSIRANGE  default value
    UTILITY_SET_REGISTER_BIT(clk0.regmap->CR, RCC_CR_MSIRGSEL);
    UTILITY_MODIFY_REGISTER(clk0.regmap->CR, RCC_CR_MSIRANGE, clk0.deinitRange);

    // Clear RCC Configuration (MSI is selected as system clock)
    UTILITY_WRITE_REGISTER(clk0.regmap->CFGR, 0);

    // Setup default value of internal clock
    Clock_updateOutputValue();
    // Initialize SysTick with default clock value (MSI @ selected range)
    System_systickInit(0);

    // Wait until system clock is ready
    tickstart = System_currentTick();
    while (UTILITY_READ_REGISTER_BIT(clk0.regmap->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI)
    {
        if ((System_currentTick() - tickstart) > 5000u)
            return ERRORS_CLOCK_TIMEOUT;
    }

    // Clear HSI, HSE and PLL
    UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR, RCC_CR_HSEON | RCC_CR_HSION | RCC_CR_HSIKERON| RCC_CR_HSIASFS | RCC_CR_PLLON | RCC_CR_PLLSAI1ON | RCC_CR_PLLSAI2ON);

    // Wait until PLLs are reset
    tickstart = System_currentTick();
    while (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_PLLRDY | RCC_CR_PLLSAI1RDY | RCC_CR_PLLSAI2RDY) != 0)
    {
        if ((System_currentTick() - tickstart) > 5000u)
            return ERRORS_CLOCK_TIMEOUT;
    }

    // Reset PLL Configurations
    UTILITY_WRITE_REGISTER(clk0.regmap->PLLCFGR, 0);
    UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLN_4);

    UTILITY_WRITE_REGISTER(clk0.regmap->PLLSAI1CFGR, 0);
    UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N_4);

    UTILITY_WRITE_REGISTER(clk0.regmap->PLLSAI2CFGR, 0);
    UTILITY_SET_REGISTER_BIT(clk0.regmap->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2N_4);

    // Reset HSEBYP
    UTILITY_CLEAR_REGISTER_BIT(clk0.regmap->CR, RCC_CR_HSEBYP);

    // Disable all interrupts
    UTILITY_WRITE_REGISTER(clk0.regmap->CIER, 0U);

    // Clear all interrupts
    UTILITY_WRITE_REGISTER(clk0.regmap->CICR, 0xFFFFFFFFU);

    // Clear all reset flags
    UTILITY_SET_REGISTER_BIT(clk0.regmap->CSR, RCC_CSR_RMVF);

    Clock_setProperFlashLatency(Clock_getActualSystemValue());

    // return NO_ERROR
    return ERRORS_NO_ERROR;
}

uint32_t Clock_getOutputValue (Clock_Output output)
{
    switch (output)
    {
    case CLOCK_OUTPUT_SYSCLK:
        return clk0.systemCoreClock;
    case CLOCK_OUTPUT_HCLK:
        return clk0.hclkClock;
    case CLOCK_OUTPUT_PCLK1:
        return clk0.pclk1Clock;
    case CLOCK_OUTPUT_PCLK2:
        return clk0.pclk2Clock;

    case CLOCK_OUTPUT_PLLR:
        return clk0.pllrClock;
    case CLOCK_OUTPUT_PLLQ:
        return clk0.pllqClock;
    case CLOCK_OUTPUT_PLLP:
        return clk0.pllpClock;

    case CLOCK_OUTPUT_PLLSAI1R:
        return clk0.pllsai1rClock;
    case CLOCK_OUTPUT_PLLSAI1Q:
        return clk0.pllsai1qClock;
    case CLOCK_OUTPUT_PLLSAI1P:
        return clk0.pllsai1pClock;

    case CLOCK_OUTPUT_PLLSAI2R:
        return clk0.pllsai2rClock;
    case CLOCK_OUTPUT_PLLSAI2P:
        return clk0.pllsai2pClock;

    default:
        return 0;
    }
}

uint32_t Clock_getOscillatorValue (void)
{
    return clk0.systemCoreClock;
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
        systemClock = CLOCK_FREQ_HSI;
        break;
    case CLOCK_SYSTEMSOURCE_HSE:
        systemClock = clk0.externalClock;
        break;
    case CLOCK_SYSTEMSOURCE_PLL:
        systemClock = Clock_getConfigPllValue(config, &config->pll);
        break;
    }

    return systemClock;
}

static uint32_t Clock_getActualMsiValue (void)
{
    uint32_t msiClock = 0, msiRange = 0;
    if (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_MSIRGSEL_Msk) == 0)
    {
        msiRange = (UTILITY_READ_REGISTER_BIT(clk0.regmap->CSR, RCC_CSR_MSISRANGE_Msk) >> RCC_CSR_MSISRANGE_Pos);
        msiClock = Clock_msiRange[msiRange];
    }
    else
    {
        msiRange = (UTILITY_READ_REGISTER_BIT(clk0.regmap->CR, RCC_CR_MSIRANGE_Msk) >> RCC_CR_MSIRANGE_Pos);
        msiClock = Clock_msiRange[msiRange];
    }
    return msiClock;
}

static uint32_t Clock_getConfigMsiValue (Clock_Config* config)
{
    return Clock_msiRange[(config->msiRange >> RCC_CR_MSIRANGE_Pos)];
}

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

static uint32_t Clock_getActualSystemValue (void)
{
    uint32_t systemClock = 0;
    Clock_SystemSourceSws sysclkSource = (Clock_SystemSourceSws)(UTILITY_READ_REGISTER_BIT(clk0.regmap->CFGR, RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos);

    switch (sysclkSource)
    {
    default:
    case CLOCK_SYSTEMSOURCESWS_MSI:
        systemClock = Clock_getActualMsiValue();
        break;
    case CLOCK_SYSTEMSOURCESWS_HSI:
        systemClock = CLOCK_FREQ_HSI;
        break;
    case CLOCK_SYSTEMSOURCESWS_HSE:
        systemClock = clk0.externalClock;
        break;
    case CLOCK_SYSTEMSOURCESWS_PLL:
    {
        uint32_t pllmClock = Clock_getActualPllInputValue();
        uint32_t pllnReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
        uint32_t pllrReg = UTILITY_READ_REGISTER_BIT(clk0.regmap->PLLCFGR, RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos;
        systemClock = (((pllmClock) * pllnReg) / ((pllrReg + 1) * 2));
    }
        break;
    }

    return systemClock;
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif
