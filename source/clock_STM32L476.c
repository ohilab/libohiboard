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

typedef struct Clock_Device
{
    RCC_TypeDef* regmap;


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


    .devLSIInitialized = 0,
    .devLSEInitialized = 0,
    .devHSIInitialized = 0,
    .devHSEInitialized = 0,
    .devPLLInitialized = 0,

    .rccError = ERRORS_NO_ERROR,
};

static System_Errors Clock_oscillatorConfig (Clock_Config* config)
{
    if ((config->source & CLOCK_INTERNAL_MSI) == CLOCK_INTERNAL_MSI)
    {

    }
    else if ((config->source & CLOCK_EXTERNAL) == CLOCK_EXTERNAL)
    {
        // Check the HSE state value
        ohiassert(CLOCK_IS_VALID_HSE_STATE(config->hseState));

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
    ohiassert(((OSC & CLOCK_EXTERNAL) == CLOCK_EXTERNAL) ^ ((OSC & CLOCK_CRYSTAL) == CLOCK_CRYSTAL));

    err = Clock_oscillatorConfig(config);


    return err;
}

#endif // LIBOHIBOARD_STM32L476
