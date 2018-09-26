/*
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/clock_STM32L476.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Clock implementations for STM32L476 family
 */

#include "platforms.h"
#include "clock.h"
#include "utility.h"

#if defined (LIBOHIBOARD_STM32L476)

#define CLOCK_MIN_FREQ_HSE                     4000000
#define CLOCK_MAX_FREQ_HSE                    48000000

#define CLOCK_IS_VALID_OSCILLATOR(OSC)  (OSC != CLOCK_NO_SOURCE)                            && \
                                        (((OSC & CLOCK_EXTERNAL) == CLOCK_EXTERNAL)         || \
                                         ((OSC & CLOCK_CRYSTAL) == CLOCK_CRYSTAL)           || \
                                         ((OSC & CLOCK_INTERNAL_32K) == CLOCK_INTERNAL_32K) || \
                                         ((OSC & CLOCK_INTERNAL_16M) == CLOCK_INTERNAL_16M) || \
                                         ((OSC & CLOCK_INTERNAL_MSI) == CLOCK_INTERNAL_MSI))

#define CLOCK_IS_VALID_HSE_STATE(HSESTATE) (((HSESTATE) == CLOCK_OSCILLATORSTATE_OFF) || \
                                            ((HSESTATE) == CLOCK_OSCILLATORSTATE_ON))

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

static const uint32_t CLOCK_AHB_PRESCALE_REGISTER_TABLE[8] =
{
        RCC_CFGR_HPRE_DIV1,
        RCC_CFGR_HPRE_DIV2,
        RCC_CFGR_HPRE_DIV4,
        RCC_CFGR_HPRE_DIV8,
        RCC_CFGR_HPRE_DIV16,
        RCC_CFGR_HPRE_DIV64,
        RCC_CFGR_HPRE_DIV128,
        RCC_CFGR_HPRE_DIV256,
        RCC_CFGR_HPRE_DIV512
};

static const uint32_t CLOCK_AHB_PRESCALE_SHIFT_TABLE[16] =
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

static const uint32_t CLOCK_APB_PRESCALE_SHIFT_TABLE[8] =
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


typedef struct Clock_Device
{
    RCC_TypeDef* regmap;

    uint32_t systemCoreClock; /**< Value that store current system core clock */
    uint32_t hclkClock;
    uint32_t pclk1Clock;
    uint32_t pclk2Clock;

    uint32_t externalClock;           /**< Oscillator or external clock value */

    uint8_t devLSIInitialized;
    uint8_t devLSEInitialized;
    uint8_t devHSEInitialized;
    uint8_t devHSIInitialized;
    uint8_t devPLLInitialized;

    System_Errors rccError;

} Clock_Device;

static Clock_Device Clock_device =
{
    .regmap = RCC,

    .systemCoreClock = 4000000U,
    .hclkClock       = 0U,
    .pclk1Clock      = 0U,
    .pclk2Clock      = 0U,

    .externalClock   = 0U,

    .devLSIInitialized = 0,
    .devLSEInitialized = 0,
    .devHSIInitialized = 0,
    .devHSEInitialized = 0,
    .devPLLInitialized = 0,

    .rccError = ERRORS_NO_ERROR,
};


static void Clock_updateOutputClock (void)
{
    if (UTILITY_READ_REGISTER_BIT(Clock_device.regmap->CFGR,RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE)
    {
        Clock_device.systemCoreClock = Clock_device.externalClock;
    }
    else if (UTILITY_READ_REGISTER_BIT(Clock_device.regmap->CFGR,RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI)
    {
        // TODO
    }
    else if (UTILITY_READ_REGISTER_BIT(Clock_device.regmap->CFGR,RCC_CFGR_SWS) == RCC_CFGR_SWS_MSI)
    {
        // TODO
    }
    else if (UTILITY_READ_REGISTER_BIT(Clock_device.regmap->CFGR,RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
    {
        // TODO
    }

    // Compute HCLK, PCLK1 and PCLK2
    uint32_t cfgr = Clock_device.regmap->CFGR;
    Clock_device.hclkClock = Clock_device.systemCoreClock >>
            CLOCK_AHB_PRESCALE_SHIFT_TABLE[UTILITY_READ_REGISTER_BIT(cfgr,RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

    Clock_device.pclk1Clock = Clock_device.hclkClock >>
            CLOCK_APB_PRESCALE_SHIFT_TABLE[UTILITY_READ_REGISTER_BIT(cfgr,RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos];

    Clock_device.pclk2Clock = Clock_device.hclkClock >>
            CLOCK_APB_PRESCALE_SHIFT_TABLE[UTILITY_READ_REGISTER_BIT(cfgr,RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
}

static System_Errors Clock_oscillatorConfig (Clock_Config* config)
{
    if ((config->source & CLOCK_INTERNAL_MSI) == CLOCK_INTERNAL_MSI)
    {

    }
    else if ((config->source & CLOCK_EXTERNAL) == CLOCK_EXTERNAL)
    {
        // Check the HSE state value
        ohiassert(CLOCK_IS_VALID_HSE_STATE(config->hseState));

        // Check and save external clock value
        ohiassert(CLOCK_IS_VALID_EXTERNAL_RANGE(config->fext));
        Clock_device.externalClock = config->fext;

        // Check if HSE is just used as SYSCLK or as PLL source
        // In this case we can't disable it
        if (((Clock_device.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE) ||
            (((Clock_device.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) &&
             ((Clock_device.regmap->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)))
        {
            return ERRORS_CLOCK_WRONG_CONFIGURATION;
        }
        else
        {
            if (config->hseState == CLOCK_OSCILLATORSTATE_OFF)
            {
                // Switch off this oscillator
                UTILITY_CLEAR_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_HSEON);
                UTILITY_CLEAR_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_HSEBYP);

                // Wait until the HSERDY bit is cleared
                // FIXME: Add timeout...
                while ((Clock_device.regmap->CR & RCC_CR_HSERDY) > 0);
                return ERRORS_NO_ERROR;
            }
            else
            {
                // Switch on this oscillator, and bypass functionality
                UTILITY_SET_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_HSEON);
                UTILITY_SET_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_HSEBYP);

                // Wait until the HSERDY bit is set
                // FIXME: Add timeout...
                while ((Clock_device.regmap->CR & RCC_CR_HSERDY) == 0);
                return ERRORS_NO_ERROR;
            }
        }
    }
    else if ((config->source & CLOCK_CRYSTAL) == CLOCK_CRYSTAL)
    {
        // Check the HSE state value
        ohiassert(CLOCK_IS_VALID_HSE_STATE(config->hseState));

        // Check and save external clock value
        ohiassert(CLOCK_IS_VALID_EXTERNAL_RANGE(config->fext));
        Clock_device.externalClock = config->fext;

        // Check if HSE is just used as SYSCLK or as PLL source
        // In this case we can't disable it
        if (((Clock_device.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE) ||
            (((Clock_device.regmap->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) &&
             ((Clock_device.regmap->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)))
        {
            return ERRORS_CLOCK_WRONG_CONFIGURATION;
        }
        else
        {
            if (config->hseState == CLOCK_OSCILLATORSTATE_OFF)
            {
                // Switch off this oscillator
                UTILITY_CLEAR_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_HSEON);
                UTILITY_CLEAR_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_HSEBYP);

                // Wait until the HSERDY bit is cleared
                // FIXME: Add timeout...
                while ((Clock_device.regmap->CR & RCC_CR_HSERDY) > 0);
                return ERRORS_NO_ERROR;
            }
            else
            {
                // Switch on this oscillator, and disable bypass functionality just for safety
                UTILITY_SET_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_HSEON);
                UTILITY_CLEAR_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_HSEBYP);

                // Wait until the HSERDY bit is set
                // FIXME: Add timeout...
                while ((Clock_device.regmap->CR & RCC_CR_HSERDY) == 0);
                return ERRORS_NO_ERROR;
            }
        }
    }

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
            if (UTILITY_READ_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_PLLRDY) == 0)
            {
                return ERRORS_CLOCK_PLL_NOT_READY;
            }

            cfgrSW = RCC_CFGR_SW_PLL;
        }
        else if (config->sysSource == CLOCK_SYSTEMSOURCE_HSI)
        {
            // Check if the source is ready
            if (UTILITY_READ_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_HSIRDY) == 0)
            {
                return ERRORS_CLOCK_HSI_NOT_READY;
            }

            cfgrSW = RCC_CFGR_SW_HSI;
        }
        else if (config->sysSource == CLOCK_SYSTEMSOURCE_HSE)
        {
            // Check if the source is ready
            if (UTILITY_READ_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_HSERDY) == 0)
            {
                return ERRORS_CLOCK_HSE_NOT_READY;
            }

            // Save chose value for CFGR register
            cfgrSW = RCC_CFGR_SW_HSE;
        }
        else if (config->sysSource == CLOCK_SYSTEMSOURCE_MSI)
        {
            // Check if the source is ready
            if (UTILITY_READ_REGISTER_BIT(Clock_device.regmap->CR,RCC_CR_MSIRDY) == 0)
            {
                return ERRORS_CLOCK_MSI_NOT_READY;
            }

            cfgrSW = RCC_CFGR_SW_MSI;
        }

        // Clear SW part of CFGR register and updated informations
        Clock_device.regmap->CFGR &= ~(RCC_CFGR_SW_Msk);
        Clock_device.regmap->CFGR |= cfgrSW;

        // Check if new value was setted
        while (UTILITY_READ_REGISTER_BIT(Clock_device.regmap->CFGR,RCC_CFGR_SWS) != (cfgrSW << RCC_CFGR_SWS_Pos));

        // FIXME: add timeout!
    }
    else if ((config->output & CLOCK_OUTPUT_HCLK) == CLOCK_OUTPUT_HCLK)
    {
        // Check if divider is valid
        ohiassert(CLOCK_IS_VALID_AHB_DIVIDER(config->ahbDivider));

        // Clear HPRE part of CFGR register and updated informations
        Clock_device.regmap->CFGR &= ~(RCC_CFGR_HPRE_Msk);
        Clock_device.regmap->CFGR |= CLOCK_AHB_PRESCALE_REGISTER_TABLE[config->ahbDivider];
    }
    else if ((config->output & CLOCK_OUTPUT_PCLK1) == CLOCK_OUTPUT_PCLK1)
    {
        // Check if divider is valid
        ohiassert(CLOCK_IS_VALID_APB_DIVIDER(config->apb1Divider));

        // Clear PPRE1 part of CFGR register and updated informations
        Clock_device.regmap->CFGR &= ~(RCC_CFGR_PPRE1_Msk);
        Clock_device.regmap->CFGR |= CLOCK_APB1_PRESCALE_REGISTER_TABLE[config->apb1Divider];
    }
    else if ((config->output & CLOCK_OUTPUT_PCLK2) == CLOCK_OUTPUT_PCLK2)
    {
        // Check if divider is valid
        ohiassert(CLOCK_IS_VALID_APB_DIVIDER(config->apb2Divider));

        // Clear PPRE2 part of CFGR register and updated informations
        Clock_device.regmap->CFGR &= ~(RCC_CFGR_PPRE2_Msk);
        Clock_device.regmap->CFGR |= CLOCK_APB2_PRESCALE_REGISTER_TABLE[config->apb2Divider];
    }

    // Update system core clock informations...
    Clock_updateOutputClock();

    return ERRORS_NO_ERROR;
}

System_Errors Clock_init (Clock_Config* config)
{
    System_Errors err = ERRORS_NO_ERROR;

    if (config == NULL)
    {
        return ERRORS_CLOCK_NO_CONFIG;
    }

    // Check if selected oscillator is valid
    ohiassert(CLOCK_IS_VALID_OSCILLATOR(config->source));

    // Check if both EXTERN and CRYSTAL are selected
    // HSE support only one of these condition
    ohiassert(((config->source & CLOCK_EXTERNAL) == CLOCK_EXTERNAL) ^
              ((config->source & CLOCK_CRYSTAL) == CLOCK_CRYSTAL));

    err = Clock_oscillatorConfig(config);

    // Check if selected valid clock output
    ohiassert(CLOCK_IS_VALID_OUTPUT(config->output));

    err = Clock_outputConfig(config);
    if (err != ERRORS_NO_ERROR)
    {
        // TODO:
    }

    return err;
}

uint32_t Clock_getOutputClock (Clock_Output output)
{
    switch (output)
    {
    case CLOCK_OUTPUT_SYSCLK:
        return Clock_device.systemCoreClock;
    case CLOCK_OUTPUT_HCLK:
        return Clock_device.hclkClock;
    case CLOCK_OUTPUT_PCLK1:
        return Clock_device.pclk1Clock;
    case CLOCK_OUTPUT_PCLK2:
        return Clock_device.pclk2Clock;
    }
}

#endif // LIBOHIBOARD_STM32L476
