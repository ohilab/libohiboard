/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2014-2018 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
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
 * @file libohiboard/include/clock_MKL.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @Clock implementations for NXP MKL Series.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "clock.h"
#include "utility.h"
#include "system.h"

#if defined (LIBOHIBOARD_MKL)

#define CLOCK_CLEAR_CHANGE_PARAM(DEVICE) do {                                             \
                                             DEVICE.changeParam.range0 = 0;               \
                                             DEVICE.changeParam.frdiv = 0;                \
                                             DEVICE.changeParam.frdivValue = 0;           \
                                             DEVICE.changeParam.outdiv1 = 0;              \
                                             DEVICE.changeParam.erefs0 = 0;               \
                                             DEVICE.changeParam.fcrdiv = 0;               \
                                             DEVICE.changeParam.fcrdivValue = 0;          \
                                             DEVICE.changeParam.ircs = 0;                 \
                                             DEVICE.changeParam.dmx_drst = 0;             \
                                             DEVICE.changeParam.prdiv = 0;                \
                                             DEVICE.changeParam.vdiv = 0;                 \
                                             DEVICE.changeParam.foutMcg = 0;              \
                                             DEVICE.changeParam.final = CLOCK_STATE_NONE; \
                                         } while(0)

typedef struct _Clock_Device
{
    MCG_Type* regmap;
    SIM_Type* regmapSim;

    Clock_State  state;                                /**< Current MCG state */

    uint32_t systemCoreClock; /**< Value that store current system core clock */

    uint32_t externalClock;           /**< Oscillator or external clock value */

    /**
     * Configuration parameters for clock mode transaction.
     */
    struct
    {
        uint8_t range0;

        uint8_t frdiv;
        uint16_t frdivValue;

        uint16_t outdiv1;

        /**
         * External Reference Select: selects the source for the external
         * reference clock.
         */
        uint8_t erefs0;

        uint8_t fcrdiv;
        uint8_t fcrdivValue;

        /**
         * Internal Reference Clock Select: selects between the fast or slow
         * internal reference clock source.
         */
        uint8_t ircs;

        uint8_t dmx_drst;

        uint8_t prdiv;

        uint8_t vdiv;

        /**
         * The target mode that MCG must reach after configuration.
         */
        Clock_State final;

        /**
         * Final frequency on MCGOUTCLK.
         */
        uint32_t foutMcg;

    } changeParam;

} Clock_Device;

static Clock_Device clk =
{
    .regmap          = MCG,

    // FIXME!
    .systemCoreClock = 4000000u,

    .externalClock   = 0u,

    .state           = CLOCK_STATE_FEI,
};

/**
 * This function check if the external oscillator or clock is inside the
 * possible range.
 *
 * @param[in] freq The external clock frequency in Hz
 * @return The range number if @ref freq is into one possible range, 0xFF otherwise.
 */
static uint8_t Clock_getOscillatorRange (uint32_t freq)
{
    if ((CLOCK_FREQ_OSC_IN_RANGE0_MIN <= freq) &&
        (freq <= CLOCK_FREQ_OSC_IN_RANGE0_MAX))
    {
        return 0;
    }
    else if ((CLOCK_FREQ_OSC_IN_RANGE1_MIN <= freq) &&
             (freq <= CLOCK_FREQ_OSC_IN_RANGE1_MAX))
    {
        return 1;
    }
    else if ((CLOCK_FREQ_OSC_IN_RANGE2_MIN <= freq) &&
             (freq <= CLOCK_FREQ_OSC_IN_RANGE2_MAX))
    {
        return 2;
    }
    else
    {
        return 0xFF;
    }
}

/**
 * Useful array for map the FRDIV (C1 register) value.
 * This value is correct only when RANGE0 is different from 0.
 */
static const uint32_t CLOCK_FRDIV_TABLE[8] =
{
    32,
    64,
    128,
    256,
    512,
    1024,
    1280,
    1536
};

/**
 * Useful array for map the FCRDIV (SC register) value.
 */
static const uint32_t CLOCK_FCRDIV_TABLE[8] =
{
    1,
    2,
    4,
    8,
    16,
    32,
    64,
    128
};

/**
 * Useful array for map LIRC multiplier in FEI mode.
 */
static const uint32_t CLOCK_LIRC_MULTIPLIER_TABLE[3] =
{
    1,
    640,
    1280
};

static const uint32_t CLOCK_LIRC_MULTIPLIER_REGISTER[3] =
{
    0,
    0,
    MCG_C4_DRST_DRS(1)
};

/**
 * Useful array for map FLL multiplier in FEE mode.
 */
static const uint32_t CLOCK_FLL_MULTIPLIER_TABLE[4] =
{
    640,
    732,
    1280,
    1464
};

/**
 * Useful array for map FLL register value in FEE mode
 * based on multiplier value.
 */
static const uint8_t CLOCK_FLL_MULTIPLIER_REGISTER[4] =
{
    0,
    MCG_C4_DMX32_MASK,
    MCG_C4_DRST_DRS(1),
    MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(1),
};


/**
 * This function compute the FRDIV value that can be used in C1 register
 * configuration, based on @ref freq value.
 * The FRDIV must divide the input frequency to FLL, in order to obtain
 * a value smaller than @ref CLOCK_FREQ_FLL_INPUT_MAX
 *
 * @param[in] freq The external clock frequency in Hz
 * @return The FRDIV value, 0xFF otherwise
 */
static uint8_t Clock_getFllInputDivider (uint32_t freq)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (freq < (CLOCK_FREQ_FLL_INPUT_MAX * CLOCK_FRDIV_TABLE[i]))
        {
            return i;
        }
    }
    return 0xFF;
}

/**
 * The transaction status matrix. It defines the path for state switch, the row is for
 * current state and the column is target state.
 */
static const Clock_State Clock_mcgStateMatrix[8][8] =
{
    {CLOCK_STATE_FEI, CLOCK_STATE_FBI, CLOCK_STATE_FBI,  CLOCK_STATE_FEE, CLOCK_STATE_FBE, CLOCK_STATE_FBE,  CLOCK_STATE_FBE, CLOCK_STATE_FBE}, // CLOCK_STATE_FEI
    {CLOCK_STATE_FEI, CLOCK_STATE_FBI, CLOCK_STATE_BLPI, CLOCK_STATE_FEE, CLOCK_STATE_FBE, CLOCK_STATE_FBE,  CLOCK_STATE_FBE, CLOCK_STATE_FBE}, // CLOCK_STATE_FBI
    {CLOCK_STATE_FBI, CLOCK_STATE_FBI, CLOCK_STATE_BLPI, CLOCK_STATE_FBI, CLOCK_STATE_FBI, CLOCK_STATE_FBI,  CLOCK_STATE_FBI, CLOCK_STATE_FBI}, // CLOCK_STATE_BLPI
    {CLOCK_STATE_FEI, CLOCK_STATE_FBI, CLOCK_STATE_FBI,  CLOCK_STATE_FEE, CLOCK_STATE_FBE, CLOCK_STATE_FBE,  CLOCK_STATE_FBE, CLOCK_STATE_FBE}, // CLOCK_STATE_FEE
    {CLOCK_STATE_FEI, CLOCK_STATE_FBI, CLOCK_STATE_FBI,  CLOCK_STATE_FEE, CLOCK_STATE_FBE, CLOCK_STATE_BLPE, CLOCK_STATE_PBE, CLOCK_STATE_PBE}, // CLOCK_STATE_FBE
    {CLOCK_STATE_FBE, CLOCK_STATE_FBE, CLOCK_STATE_FBE,  CLOCK_STATE_FBE, CLOCK_STATE_FBE, CLOCK_STATE_BLPE, CLOCK_STATE_PBE, CLOCK_STATE_PBE}, // CLOCK_STATE_BLPE
    {CLOCK_STATE_FBE, CLOCK_STATE_FBE, CLOCK_STATE_FBE,  CLOCK_STATE_FBE, CLOCK_STATE_FBE, CLOCK_STATE_BLPE, CLOCK_STATE_PBE, CLOCK_STATE_PEE}, // CLOCK_STATE_PBE
    {CLOCK_STATE_PBE, CLOCK_STATE_PBE, CLOCK_STATE_PBE,  CLOCK_STATE_PBE, CLOCK_STATE_PBE, CLOCK_STATE_PBE,  CLOCK_STATE_PBE, CLOCK_STATE_PBE}  // CLOCK_STATE_PEE
//   CLOCK_STATE_FEI  CLOCK_STATE_FBI  CLOCK_STATE_BLPI  CLOCK_STATE_FEE  CLOCK_STATE_FBE  CLOCK_STATE_BLPE  CLOCK_STATE_PBE  CLOCK_STATE_PEE
};

/**
 * This function set the current mode of MCG module to FEE:
 * FLL Engaged External.
 *
 * @note Pay attention to ERRATA 7993 (e7993) at this link
 *       https://www.nxp.com/docs/en/errata/KINETIS_W_1N41U.pdf
 */
static System_Errors Clock_setFeeMode (void)
{
    ohiassert(clk.changeParam.final != CLOCK_STATE_NONE);

    bool changeDrs = FALSE;
    uint8_t tmpreg = 0;
    uint8_t tmpC4 = clk.regmap->C4;

    // e7993
    if ((clk.regmap->S & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK)
    {
        clk.regmap->C4 ^= (1u << MCG_C4_DRST_DRS_SHIFT);
        changeDrs = TRUE;
    }

    // Set MCG->C2
    tmpreg = clk.regmap->C2;
    tmpreg &= ~(MCG_C2_RANGE0_MASK);
    tmpreg |= MCG_C2_RANGE0(clk.changeParam.range0) | // Set frequency range
              MCG_C2_EREFS0(clk.changeParam.erefs0);  // Set oscillator/external
    clk.regmap->C2 = tmpreg;

    // Set MCG->C1
    tmpreg = clk.regmap->C1;
    tmpreg &= (~(MCG_C1_CLKS_MASK | MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK));
    tmpreg |= MCG_C1_CLKS(0)                      | // PLL/FLL select
              MCG_C1_FRDIV(clk.changeParam.frdiv) | // Set frdiv
              MCG_C1_IREFS(0);                      // External source select
    clk.regmap->C1 = tmpreg;

    // wait the refresh of the status register
    if ((clk.regmap->C2 & MCG_C2_EREFS0_MASK) == MCG_C2_EREFS0_MASK)
    {
        while ((clk.regmap->S & MCG_S_OSCINIT0_MASK) != MCG_S_OSCINIT0_MASK);
    }

    // Wait until reference Source of FLL is the external reference clock.
    while ((clk.regmap->S & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK);

    // Restore DRST and DMX32 after initialization for e7993
    if (changeDrs == TRUE)
    {
        clk.regmap->C4 = tmpC4;
    }

    // Set MCG->C4
    tmpC4 &= (~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK));
    tmpC4 |= clk.changeParam.dmx_drst;
    clk.regmap->C4 = tmpC4;

    //  Wait until output of the FLL is selected
    while ((clk.regmap->S & MCG_S_CLKST_MASK) != 0)

    return ERRORS_NO_ERROR;
}

/**
 * This function set the current mode of MCG module to FBE:
 * FLL Bypassed External.
 *
 */
static System_Errors Clock_setFbeMode (void)
{
    ohiassert(clk.changeParam.final != CLOCK_STATE_NONE);

    bool changeDrs = FALSE;
    uint8_t tmpreg = 0;
    uint8_t tmpC4 = clk.regmap->C4;

    // Change to FLL mode
    clk.regmap->C6 &= ~(MCG_C6_PLLS_MASK);
    // Wait until source of PLLS clock is FLL clock.
    while ((clk.regmap->S & MCG_S_PLLST_MASK) != 0);

    // Disable LP
    clk.regmap->C2 &= ~(MCG_C2_LP_MASK);

    // e7993
    if ((clk.regmap->S & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK)
    {
        clk.regmap->C4 ^= (1u << MCG_C4_DRST_DRS_SHIFT);
        changeDrs = TRUE;
    }

    // Set MCG->C2
    tmpreg = clk.regmap->C2;
    tmpreg &= ~(MCG_C2_RANGE0_MASK);
    tmpreg |= MCG_C2_RANGE0(clk.changeParam.range0) | // Set frequency range
              MCG_C2_EREFS0(clk.changeParam.erefs0);  // Set oscillator/external
    clk.regmap->C2 = tmpreg;

    // Set MCG->C1
    tmpreg = clk.regmap->C1;
    tmpreg &= (~(MCG_C1_CLKS_MASK | MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK));
    tmpreg |= MCG_C1_CLKS(2)                      | // External reference clock is selected
              MCG_C1_FRDIV(clk.changeParam.frdiv) | // Set frdiv
              MCG_C1_IREFS(0);                      // External source select
    clk.regmap->C1 = tmpreg;

    // wait the refresh of the status register
    if ((clk.regmap->C2 & MCG_C2_EREFS0_MASK) == MCG_C2_EREFS0_MASK)
    {
        while ((clk.regmap->S & MCG_S_OSCINIT0_MASK) != MCG_S_OSCINIT0_MASK);
    }

    // Wait until reference Source of FLL is the external reference clock.
    while ((clk.regmap->S & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK);

    // Restore DRST and DMX32 after initialization for e7993
    if (changeDrs == TRUE)
    {
        clk.regmap->C4 = tmpC4;
    }

    // Set MCG->C4
    tmpC4 &= (~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK));
    tmpC4 |= clk.changeParam.dmx_drst;
    clk.regmap->C4 = tmpC4;

    //  Wait until output of the FLL is selected
    while ((clk.regmap->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2));

    return ERRORS_NO_ERROR;
}

/**
 * This function set the current mode of MCG module to FBI:
 * FLL Bypassed Internal.
 *
 *
 * @note Pay attention to ERRATA 7993 (e7993) at this link
 *       https://www.nxp.com/docs/en/errata/KINETIS_W_1N41U.pdf
 */
static System_Errors Clock_setFbiMode (void)
{
    ohiassert(clk.changeParam.final != CLOCK_STATE_NONE);

    bool changeDrs = FALSE;
    uint8_t tmpreg = 0;
    uint8_t tmpC4 = clk.regmap->C4;

    // e7993
    if ((clk.regmap->S & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK)
    {
        clk.regmap->C4 ^= (1u << MCG_C4_DRST_DRS_SHIFT);
        changeDrs = TRUE;
    }

    // Set MCG->C1
    tmpreg = clk.regmap->C1;
    tmpreg &= (~(MCG_C1_CLKS_MASK |  MCG_C1_IREFS_MASK));
    tmpreg |= MCG_C1_CLKS(1)  | // Internal reference clock is selected
              MCG_C1_IREFS(1);  // The slow internal reference clock is selected
    clk.regmap->C1 = tmpreg;

    // Set MCG->C2
    // Disable LP and select internal source
    tmpreg = clk.regmap->C2;
    tmpreg &= (~(MCG_C2_LP_MASK | MCG_C2_IRCS_MASK));
    tmpreg |= clk.changeParam.ircs;  // Select slow/fast internal reference clock
    clk.regmap->C2 = tmpreg;

    // Wait until reference Source of FLL is the internal reference clock.
    while ((clk.regmap->S & MCG_S_IREFST_MASK) != MCG_S_IREFST_MASK);

    // Wait until internal type of source was select
    while ((clk.regmap->S & MCG_S_IRCST_MASK) != clk.changeParam.ircs);

    // Restore DRST and DMX32 after initialization for e7993
    if (changeDrs == TRUE)
    {
        clk.regmap->C4 = tmpC4;
    }

    // Wait until internal reference clock is selected.
    while ((clk.regmap->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(1));

    // Set MCG->C4
    tmpC4 &= (~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK));
    tmpC4 |= clk.changeParam.dmx_drst;
    clk.regmap->C4 = tmpC4;
}


static System_Errors Clock_stateTransition (void)
{
    Clock_State state = Clock_getCurrentState();
    System_Errors err = ERRORS_NO_ERROR;

    do
    {

        switch (state)
        {
        case CLOCK_STATE_FEI:
            break;
        case CLOCK_STATE_FBI:
            err = Clock_setFbiMode();
            break;
        case CLOCK_STATE_BLPI:
            break;
        case CLOCK_STATE_FEE:
            err = Clock_setFeeMode();
            break;
        case CLOCK_STATE_FBE:
            err = Clock_setFbeMode();
            break;
        case CLOCK_STATE_BLPE:
            break;
        case CLOCK_STATE_PBE:
            break;
        case CLOCK_STATE_PEE:
            break;
        default:
            ohiassert(0);
            break;
        }
        if (err != ERRORS_NO_ERROR)
        {
            return err;
        }
    }
    while (state != clk.changeParam.final);

    return ERRORS_NO_ERROR;
}

System_Errors Clock_init (Clock_Config* config)
{
    bool isConfigFound = FALSE;

    if (config == NULL)
    {
        return ERRORS_CLOCK_NO_CONFIG;
    }
    // Check current mode
    Clock_getCurrentState();

    // Check external range, and FLL input range
    if ((config->source == CLOCK_CRYSTAL) || (config->source == CLOCK_EXTERNAL))
    {
        if (config->fext > CLOCK_FREQ_EXT_MAX)
            return ERRORS_CLOCK_EXTERNAL_TOO_BIG;

        clk.changeParam.range0 = Clock_getOscillatorRange(config->fext);
        ohiassert(clk.changeParam.range0 != 0xFF);

        if (clk.changeParam.range0 == 0)
        {
            clk.changeParam.frdiv      = 0;
            clk.changeParam.frdivValue = 1;
        }
        else
        {
            clk.changeParam.frdiv = Clock_getFllInputDivider(config->fext);
            ohiassert(clk.changeParam.frdiv != 0xFF);
            clk.changeParam.frdivValue = CLOCK_FRDIV_TABLE[clk.changeParam.frdiv];
        }

        // Save crystal or external clock option
        if (config->source == CLOCK_CRYSTAL)
        {
            // Oscillator requested
            clk.changeParam.erefs0 = MCG_C2_EREFS0_MASK;
        }
        else
        {
            // External reference clock requested
            clk.changeParam.erefs0 = 0u;
        }

        // Save external clock/oscillator value
        clk.externalClock = config->fext;
    }

    // Compute the OUTDIV1 value based on source choose and desired output value
    uint32_t foutMcg = 0;
    for (uint8_t div = CLOCK_REG_OUTDIV1_MIN; div < (CLOCK_REG_OUTDIV1_MAX + 1); ++div)
    {
        foutMcg = config->foutSys * div;

        if ((config->source == CLOCK_CRYSTAL) || (config->source == CLOCK_EXTERNAL))
        {
            if (foutMcg == config->fext)
            {
                clk.changeParam.final = CLOCK_STATE_BLPE;
                clk.changeParam.outdiv1 = div - 1;
                clk.changeParam.foutMcg = foutMcg;
                isConfigFound = TRUE;
                // Exit from OUTDIV1 searching...
                goto clock_config_result;
            }

            // Try with FLL
            if ((foutMcg < (CLOCK_FREQ_FLL_LOWRANGE_MIN)) ||
               (((CLOCK_FREQ_FLL_LOWRANGE_MAX) < foutMcg) && (foutMcg < (CLOCK_FREQ_FLL_MIDRANGE_MIN))) ||
               (foutMcg > (CLOCK_FREQ_MCGPLL_MAX)))
            {
                // In case of out of range
                continue;
            }
            else
            {
                uint32_t fllfrequency = config->fext/clk.changeParam.frdivValue;
                for (uint8_t fllmul = 0; fllmul < 4; ++fllmul)
                {
                    if (foutMcg == (fllfrequency*CLOCK_FLL_MULTIPLIER_TABLE[fllmul]))
                    {
                        clk.changeParam.final = CLOCK_STATE_FEE;
                        clk.changeParam.outdiv1 = div - 1;
                        clk.changeParam.foutMcg = foutMcg;
                        clk.changeParam.dmx_drst = CLOCK_FLL_MULTIPLIER_REGISTER[fllmul];
                        isConfigFound = TRUE;
                        // Exit from OUTDIV1 searching...
                        goto clock_config_result;
                    }
                }
            }

            // Try with PLL
            uint32_t pllfrequency = 0, plloutfrequency = 0, plldiff = 200000000u;
            uint8_t prdiv, vdiv;
            for (prdiv = 1; prdiv < (CLOCK_REG_PRDIV_MAX + 1); ++prdiv)
            {
                pllfrequency = config->fext/prdiv;
                // Check PLL input range
                if ((pllfrequency >= CLOCK_FREQ_PLL_INPUT_MIN) &&
                    (pllfrequency <= CLOCK_FREQ_PLL_INPUT_MAX))
                {
                    for (vdiv = CLOCK_REG_VDIV_MIN; vdiv < (CLOCK_REG_VDIV_MAX + 1); ++vdiv)
                    {
                        plloutfrequency = pllfrequency * vdiv;
                        // Check the output limit
                        if (plloutfrequency <= CLOCK_FREQ_MCGPLL_MAX)
                        {
                            // Check the difference between desiderata and computed value
                            if (foutMcg > plloutfrequency)
                            {
                                // Update data
                                if (plldiff > (foutMcg - plloutfrequency))
                                {
                                    plldiff = foutMcg - plloutfrequency;
                                    clk.changeParam.prdiv = MCG_C5_PRDIV0(prdiv-1);
                                    clk.changeParam.vdiv  = MCG_C6_VDIV0(vdiv-CLOCK_REG_VDIV_MIN);
                                }
                            }
                            else
                            {
                                // Update data
                                if (plldiff > (plloutfrequency - foutMcg))
                                {
                                    plldiff = plloutfrequency -
                                            foutMcg;
                                    clk.changeParam.prdiv = MCG_C5_PRDIV0(prdiv-1);
                                    clk.changeParam.vdiv  = MCG_C6_VDIV0(vdiv-CLOCK_REG_VDIV_MIN);
                                }
                            }
                        }
                    }
                }
            }
            // Check whether the found value is usable or not (3%)
            if (plldiff > ((foutMcg * 3) / 100))
            {
                clk.changeParam.prdiv = 0;
                clk.changeParam.vdiv  = 0;
                continue;
            }
            else
            {
                clk.changeParam.final = CLOCK_STATE_PEE;
                clk.changeParam.foutMcg = ((config->fext/(prdiv)) * vdiv);
                clk.changeParam.outdiv1 = div - 1;
                isConfigFound = TRUE;
                // Exit from OUTDIV1 searching...
                goto clock_config_result;
            }
        }
        else if (config->source == CLOCK_INTERNAL_LIRC)
        {
            uint32_t lfrequency = 0;
            for (uint8_t mul = 0; mul < 3; ++mul)
            {
                lfrequency = CLOCK_FREQ_INTERNAL_LIRC*CLOCK_LIRC_MULTIPLIER_TABLE[mul];
                if (((lfrequency - lfrequency/100) >= foutMcg) &&
                    ((lfrequency + lfrequency/100) <= foutMcg))
                {
                    // Save clock
                    if (mul == 0)
                    {
                        clk.changeParam.final = CLOCK_STATE_BLPI;
                    }
                    else
                    {
                        clk.changeParam.final = CLOCK_STATE_FEI;
                        clk.changeParam.dmx_drst = CLOCK_LIRC_MULTIPLIER_REGISTER[mul];
                    }
                    // Slow internal reference clock selected
                    clk.changeParam.ircs = 0;
                    clk.changeParam.foutMcg = lfrequency;
                    clk.changeParam.outdiv1 = div - 1;
                    isConfigFound = TRUE;
                    // Exit from OUTDIV1 searching...
                    goto clock_config_result;
                }
            }
        }
        else if (config->source == CLOCK_INTERNAL_HIRC)
        {
            uint32_t hfrequency = 0;
            for (uint8_t fcrdiv = 0; fcrdiv < 8; ++fcrdiv)
            {
                hfrequency = CLOCK_FREQ_INTERNAL_HIRC/CLOCK_FCRDIV_TABLE[fcrdiv];
                if (((hfrequency - hfrequency/100) >= foutMcg) &&
                    ((hfrequency + hfrequency/100) <= foutMcg))
                {
                    // Save clock
                    clk.changeParam.final = CLOCK_STATE_BLPI;
                    clk.changeParam.foutMcg = hfrequency;
                    // Fast internal reference clock selected
                    clk.changeParam.ircs = MCG_C2_IRCS_MASK;
                    clk.changeParam.fcrdiv = MCG_SC_FCRDIV(fcrdiv);
                    clk.changeParam.fcrdivValue = CLOCK_FCRDIV_TABLE[fcrdiv];
                    clk.changeParam.outdiv1 = div - 1;
                    isConfigFound = TRUE;
                    // Exit from OUTDIV1 searching...
                    goto clock_config_result;
                }
            }
        }
    }

clock_config_result:

    if (isConfigFound == FALSE)
    {
        CLOCK_CLEAR_CHANGE_PARAM(clk);
        return ERRORS_CLOCK_WRONG_CONFIGURATION;
    }

    // Setup MCGCLKOUT to system clock divider
    uint32_t tempReg = clk.regmapSim->CLKDIV1;
    tempReg &= (~(SIM_CLKDIV1_OUTDIV1_MASK));
    tempReg |= (SIM_CLKDIV1_OUTDIV1(clk.changeParam.outdiv1));
    clk.regmapSim->CLKDIV1 = tempReg;

    // Change operation mode
    Clock_stateTransition();

    return ERRORS_NO_ERROR;
}

System_Errors Clock_deInit ()
{

}

Clock_State Clock_getCurrentState ()
{
    switch ((clk.regmap->C1 & MCG_C1_CLKS_MASK) >> MCG_C1_CLKS_SHIFT)
    {
    case 0:
        if ((clk.regmap->C6 & MCG_C6_PLLS_MASK) != 0)
        {
            clk.state = CLOCK_STATE_PEE;
        }
        else
        {
            if ((clk.regmap->C1 & MCG_C1_IREFS_MASK) == 0)
            {
                clk.state = CLOCK_STATE_FEE;
            }
            else
            {
                clk.state = CLOCK_STATE_FEI;
            }
        }
        break;
    case 1:
        if ((clk.regmap->C2 & MCG_C2_LP_MASK) == 0)
        {
            clk.state = CLOCK_STATE_FBI;
        }
        else
        {
            clk.state = CLOCK_STATE_BLPI;
        }
        break;
    case 2:
        if ((clk.regmap->C2 & MCG_C2_LP_MASK) != 0)
        {
            clk.state = CLOCK_STATE_BLPE;
        }
        else
        {
            if ((clk.regmap->C6 & MCG_C6_PLLS_MASK) != 0)
            {
                clk.state = CLOCK_STATE_PBE;
            }
            else
            {
                clk.state = CLOCK_STATE_FBE;
            }
        }
        break;
    default:
        ohiassert(0);
    }
    return clk.state;
}

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif


#if 0

#if defined(LIBOHIBOARD_KL15Z4)

#define CLOCK_INIT_DIFF                       200000000
#define CLOCK_MAX_FREQ_MCG                    100000000
#define CLOCK_MAX_FREQ_SYS                    48000000
#define CLOCK_MAX_FREQ_EXT                    48000000
#define CLOCK_MAX_FREQ_BUS					  24000000
#define CLOCK_FREQ_INTERNAL_SLOW              32000
#define CLOCK_FREQ_INTERNAL_FAST              4000000
#define CLOCK_MIN_FREQ_RANGE0_OSC_IN          32000
#define CLOCK_MAX_FREQ_RANGE0_OSC_IN          40000
#define CLOCK_MIN_FREQ_RANGE1_OSC_IN          3000000
#define CLOCK_MAX_FREQ_RANGE1_OSC_IN          8000000
#define CLOCK_MIN_FREQ_RANGE2_OSC_IN          8000000
#define CLOCK_MAX_FREQ_RANGE2_OSC_IN          32000000
#define CLOCK_MIN_FREQ_FLL_IN                 31250
#define CLOCK_MAX_FREQ_FLL_IN                 39062.5
#define CLOCK_MIN_FREQ_RANGE0_FLL_OUT         20000000
#define CLOCK_MAX_FREQ_RANGE0_FLL_OUT         25000000
#define CLOCK_CENTER_FREQ_RANGE0_FLL_OUT      24000000
#define CLOCK_MIN_FREQ_RANGE1_FLL_OUT         40000000
#define CLOCK_MAX_FREQ_RANGE1_FLL_OUT         50000000
#define CLOCK_CENTER_FREQ_RANGE1_FLL_OUT      48000000
#define CLOCK_MAX_FREQ_PLL_OUT                100000000       
#define CLOCK_MIN_FREQ_PLL_IN                 2000000
#define CLOCK_MAX_FREQ_PLL_IN                 4000000
#define CLOCK_INTERNAL_FREQ_SLOW_OUT          32000
#define CLOCK_INTERNAL_FREQ_FAST_OUT_1        4000000
#define CLOCK_INTERNAL_FREQ_FAST_OUT_2        2000000
#define CLOCK_INTERNAL_FREQ_FAST_OUT_3        1000000
#define CLOCK_INTERNAL_FREQ_FAST_OUT_4        500000
#define CLOCK_INTERNAL_FREQ_FAST_OUT_5        250000
#define CLOCK_INTERNAL_FREQ_FAST_OUT_6        125000
#define CLOCK_INTERNAL_FREQ_FAST_OUT_7        31250

typedef struct Clock_Device
{
	MCG_MemMapPtr regmap;

    uint32_t foutMcg;
    Clock_State mcgState;
    
    uint8_t devInitialized;
    
    System_Errors mcgError;
}Clock_Device; 

static Clock_Device Clock_device = {
	.regmap = MCG_BASE_PTR,

	.foutMcg = 0,
	.mcgState = CLOCK_FEI,

	.devInitialized = 0,

	.mcgError = ERRORS_NO_ERROR,
};

/* Functions of single state transition */

/**
 * @brief
 * 
 * @param fext external frequency reference
 * @param dmx32 [0 or 1]
 * @param drstDrs [0 or 1]
 * @param range0 [0,1 or 2]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fei2fee (uint32_t fext, uint8_t dmx32, uint8_t drstDrs, uint8_t range0, uint8_t frdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint32_t fllRefClock;
    uint8_t dmx32Tmp;
    uint8_t drstDrsTmp;
    uint8_t range0Tmp;
    uint8_t frdivTmp;
    uint8_t tempReg;
   
    /* The RANGE0 and FRDIV evaluation have to be done out of this function. */ 
    
    /* write RANGE0 and FRDIV on the MCU register */
    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_RANGE0_MASK);
    tempReg |= MCG_C2_RANGE0(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap) ;
    tempReg &= ~(MCG_C1_FRDIV_MASK);
    tempReg |= MCG_C1_FRDIV(frdiv); 
    MCG_C1_REG(regmap) = tempReg;
    
    tempReg = MCG_C4_REG(regmap); 
    tempReg &=  ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK); //Clear these field of C4_REG
    tempReg |= ((dmx32 << MCG_C4_DMX32_SHIFT) | (MCG_C4_DRST_DRS(drstDrs))); // Set the moltiplication factor for FLL 
    MCG_C4_REG(regmap) = tempReg;

    MCG_C1_REG(regmap) &= ~(MCG_C1_IREFS_MASK); //select the external reference for the FLL
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS0_MASK)  == (MCG_C2_EREFS0_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT0_MASK) != MCG_S_OSCINIT0_MASK); //esce quanto oscillatore � inizializzato
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); //esce quando seleziona il riferimento esterno
    
    //Now in FEE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
//    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
    range0Tmp = ((MCG_C2_REG(regmap) & MCG_C2_RANGE0_MASK) >> MCG_C2_RANGE0_SHIFT);
    frdivTmp = ((MCG_C1_REG(regmap) & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT);
    dmx32Tmp = ((MCG_C4_REG(regmap) & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT);
    drstDrsTmp = ((MCG_C4_REG(regmap) & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT);
            
    if(range0Tmp == 0)
    {
        if(frdivTmp == 0) fllRefClock = fext/1;
        else if(frdivTmp == 1) fllRefClock = fext/2;
        else if(frdivTmp == 2) fllRefClock = fext/4;
        else if(frdivTmp == 3) fllRefClock = fext/8;
        else if(frdivTmp == 4) fllRefClock = fext/16;
        else if(frdivTmp == 5) fllRefClock = fext/32;
        else if(frdivTmp == 6) fllRefClock = fext/64;
        else if(frdivTmp == 7) fllRefClock = fext/128;
    }
    else 
    {
        if(frdivTmp == 0) fllRefClock = fext/32;
        else if(frdivTmp == 1) fllRefClock = fext/64;
        else if(frdivTmp == 2) fllRefClock = fext/128;
        else if(frdivTmp == 3) fllRefClock = fext/256;
        else if(frdivTmp == 4) fllRefClock = fext/512;
        else if(frdivTmp == 5) fllRefClock = fext/1024;
        else if(frdivTmp == 6) fllRefClock = fext/1280;
        else if(frdivTmp == 7) fllRefClock = fext/1536;
    }
    
    if(dmx32Tmp == 0)
    {
        if(drstDrsTmp == 0) foutMcg = fllRefClock*640;
        else if(drstDrsTmp == 1) foutMcg = fllRefClock*1280;
        else if(drstDrsTmp == 2) foutMcg = fllRefClock*1920;
        else if(drstDrsTmp == 3) foutMcg = fllRefClock*2560;
    }
    else if(dmx32Tmp == 1)
    {
        if(drstDrsTmp == 0) foutMcg = fllRefClock*732;
        else if(drstDrsTmp == 1) foutMcg = fllRefClock*1464;
        else if(drstDrsTmp == 2) foutMcg = fllRefClock*2197;
        else if(drstDrsTmp == 3) foutMcg = fllRefClock*2929;
    }
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param dmx32 [0 or 1]
 * @param drstDrs [0 or 1]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fee2fei (uint8_t dmx32, uint8_t drstDrs)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t dmx32Tmp;
    uint8_t drstDrsTmp;
    uint8_t tempReg;
    
    /* The RANGE0 and FRDIV evaluation have to be done out of this function. */
    
    tempReg =MCG_C4_REG(regmap); 
    tempReg &=  ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK); //Clear these field of C4_REG
    tempReg |= ((dmx32 << MCG_C4_DMX32_SHIFT) | (MCG_C4_DRST_DRS(drstDrs))); 
    MCG_C4_REG(regmap) = tempReg; 

    MCG_C1_REG(regmap) |= MCG_C1_IREFS_MASK; //select the internal slow reference for the FLL (IREFS = 1)
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) != MCG_S_IREFST_MASK); //esce quando seleziona il riferimento interno IREFST = 1
    
    //Now in FEI
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
//    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
    dmx32Tmp = ((MCG_C4_REG(regmap) & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT);
    drstDrsTmp = ((MCG_C4_REG(regmap) & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT);
    
    if(dmx32Tmp == 0)
    {
        if(drstDrsTmp == 0) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*640;
        else if(drstDrsTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1280;
        else if(drstDrsTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1920;
        else if(drstDrsTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2560;
    }
    else if(dmx32Tmp == 1)
    {
        if(drstDrsTmp == 0) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*732;
        else if(drstDrsTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1464;
        else if(drstDrsTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2197;
        else if(drstDrsTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2929;
    }
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param ircs [0 or 1]
 * @param fcrdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fei2fbi (uint8_t ircs, uint8_t fcrdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t fcrdivTmp;
    uint8_t tempReg; 
    
    tempReg = MCG_SC_REG(regmap); 
    tempReg &= ~(MCG_SC_FCRDIV_MASK); //Clear the FCRDIV field of SC_REG
    tempReg |= MCG_SC_FCRDIV(fcrdiv);
    MCG_SC_REG(regmap) = tempReg;

    
    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
    tempReg |= (ircs << MCG_C2_IRCS_SHIFT);
    MCG_C2_REG(regmap) = tempReg;
    
    
    tempReg = MCG_C1_REG(regmap); 
    tempReg &= ~(MCG_C1_CLKS_MASK); //Clear the CLKS field of C1_REG
    tempReg |= MCG_C1_CLKS(1); //Set Internal Reference Clock as MCGOUTCLK
    MCG_C1_REG(regmap) = tempReg;
    
    //wait for the state update
    while(((MCG_S_REG(regmap) & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT) != ircs); 
    while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(1)));
    
    //Now on FBI
    
    MCG_C1 |= MCG_C1_IRCLKEN_MASK;
    
    fcrdivTmp = ((MCG_SC_REG(regmap) & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT);
    
    if(ircs == 1)
    {
        if(fcrdivTmp == 0)  foutMcg = CLOCK_FREQ_INTERNAL_FAST/1;
        else if(fcrdivTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
        else if(fcrdivTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_FAST/4;
        else if(fcrdivTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_FAST/8;
        else if(fcrdivTmp == 4) foutMcg = CLOCK_FREQ_INTERNAL_FAST/16;
        else if(fcrdivTmp == 5) foutMcg = CLOCK_FREQ_INTERNAL_FAST/32;
        else if(fcrdivTmp == 6) foutMcg = CLOCK_FREQ_INTERNAL_FAST/64;
        else if(fcrdivTmp == 7) foutMcg = CLOCK_FREQ_INTERNAL_FAST/128;
    }
    else foutMcg = CLOCK_FREQ_INTERNAL_SLOW;
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param dmx32 [0 or 1]
 * @param drstDrs [0 or 1]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fbi2fei (uint8_t dmx32, uint8_t drstDrs)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t dmx32Tmp;
    uint8_t drstDrsTmp;
    uint8_t tempReg;
    
    tempReg = MCG_C4_REG(regmap); 
    tempReg &=  ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK); //Clear these field of C4_REG
    tempReg |= ((dmx32 << MCG_C4_DMX32_SHIFT) | (MCG_C4_DRST_DRS(drstDrs))); //Set the multiplication factor for FLL
    MCG_C4_REG(regmap) = tempReg;

    tempReg = MCG_C1_REG(regmap); 
    tempReg &= ~(MCG_C1_IREFS_MASK | MCG_C1_CLKS_MASK); //Clear the IREFS and CLKS fields of C1_REG
    tempReg |= (MCG_C1_IREFS_MASK | MCG_C1_CLKS(0)); //Set Slow Internal Reference for FLL_IN and FLL_OUT as MCGOUTCLK
    MCG_C1_REG(regmap) = tempReg;
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) != MCG_S_IREFST_MASK); 
    while((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(0));
    
    //Now in FEI 
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
//    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
    dmx32Tmp = ((MCG_C4_REG(regmap) & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT);
    drstDrsTmp = ((MCG_C4_REG(regmap) & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT);
    
    if(dmx32Tmp == 0)
    {
        if(drstDrsTmp == 0) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*640;
        else if(drstDrsTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1280;
        else if(drstDrsTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1920;
        else if(drstDrsTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2560;
    }
    else if(dmx32Tmp == 1)
    {
        if(drstDrsTmp == 0) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*732;
        else if(drstDrsTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1464;
        else if(drstDrsTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2197;
        else if(drstDrsTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2929;
    }
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param fext external frequency reference
 * @param range0 [0,1 or 2]
 * @param frdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fei2fbe (uint32_t fext, uint8_t range0, uint8_t frdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t tempReg;
    
    /* The RANGE0 and FRDIV evaluation have to be done out of this function. */ 
    
    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_RANGE0_MASK | MCG_C2_LP_MASK);
    tempReg |= MCG_C2_RANGE0(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK | MCG_C1_CLKS_MASK);
    tempReg |= (MCG_C1_FRDIV(frdiv) | MCG_C1_CLKS(2));
    MCG_C1_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS0_MASK)  == (MCG_C2_EREFS0_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT0_MASK) != MCG_S_OSCINIT0_MASK);
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); 
    while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)));
    
    //Now in FBE
    
	MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param dmx32 [0 or 1]
 * @param drstDrs [0 or 1]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fbe2fei (uint8_t dmx32, uint8_t drstDrs)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t dmx32Tmp;
    uint8_t drstDrsTmp;
    uint8_t tempReg;
    
    tempReg = MCG_C4_REG(regmap); 
    tempReg &=  ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK); //Clear these field of C4_REG
    tempReg |= ((dmx32 << MCG_C4_DMX32_SHIFT) | (MCG_C4_DRST_DRS(drstDrs))); //Set the multiplication factor forFLL
    MCG_C4_REG(regmap) = tempReg;

    tempReg = MCG_C1_REG(regmap); 
    tempReg &= ~(MCG_C1_IREFS_MASK | MCG_C1_CLKS_MASK); //Clear the IREFS and CLKS fields of C1_REG
    tempReg |= (MCG_C1_IREFS_MASK | MCG_C1_CLKS(0)); //Set Slow Internal Reference for FLL_IN and FLL_OUT as MCGOUTCLK
    MCG_C1_REG(regmap) = tempReg;
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) != MCG_S_IREFST_MASK); 
    while((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(0));
    
    //Now in FEI 
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
//    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
    dmx32Tmp = ((MCG_C4_REG(regmap) & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT);
    drstDrsTmp = ((MCG_C4_REG(regmap) & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT);
    
    if(dmx32Tmp == 0)
    {
        if(drstDrsTmp == 0) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*640;
        else if(drstDrsTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1280;
        else if(drstDrsTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1920;
        else if(drstDrsTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2560;
    }
    else if(dmx32Tmp == 1)
    {
        if(drstDrsTmp == 0) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*732;
        else if(drstDrsTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1464;
        else if(drstDrsTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2197;
        else if(drstDrsTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2929;
    }
    
    return foutMcg;
    
}

/**
 * @brief
 * 
 * @param ircs [0 or 1]
 * @param fcrdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fee2fbi (uint8_t ircs, uint8_t fcrdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t fcrdivTmp;
    uint8_t tempReg; 
    
    tempReg = MCG_SC_REG(regmap); 
    tempReg &= ~(MCG_SC_FCRDIV_MASK); //Clear the FCRDIV field of SC_REG
    tempReg |= MCG_SC_FCRDIV(fcrdiv);
    MCG_SC_REG(regmap) = tempReg;

    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
    tempReg |= (ircs << MCG_C2_IRCS_SHIFT);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap); 
    tempReg &= ~(MCG_C1_IREFS_MASK | MCG_C1_CLKS_MASK); //Clear the IREFS and CLKS fields of C1_REG
    //Set Internal Reference Clock Slow as FLL_IN and Internal Reference Clock as MCGOUTCLK
    tempReg |= (MCG_C1_IREFS_MASK | MCG_C1_CLKS(1));
    MCG_C1_REG(regmap) = tempReg;
    
    while(((MCG_S_REG(regmap) & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT) != ircs); 
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) != MCG_S_IREFST_MASK); 
    while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(1)));
    
    //Now in FBI
    
    MCG_C1 |= MCG_C1_IRCLKEN_MASK;
    
    fcrdivTmp = ((MCG_SC_REG(regmap) & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT);
    
    if(ircs == 1)
    {
        if(fcrdivTmp == 0)  foutMcg = CLOCK_FREQ_INTERNAL_FAST/1;
        else if(fcrdivTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
        else if(fcrdivTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_FAST/4;
        else if(fcrdivTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_FAST/8;
        else if(fcrdivTmp == 4) foutMcg = CLOCK_FREQ_INTERNAL_FAST/16;
        else if(fcrdivTmp == 5) foutMcg = CLOCK_FREQ_INTERNAL_FAST/32;
        else if(fcrdivTmp == 6) foutMcg = CLOCK_FREQ_INTERNAL_FAST/64;
        else if(fcrdivTmp == 7) foutMcg = CLOCK_FREQ_INTERNAL_FAST/128;
    }
    else foutMcg = CLOCK_FREQ_INTERNAL_SLOW;
    
    return foutMcg;
}


/**
 * @brief
 * 
 * @param fext external frequency reference
 * @param dmx32 [0 or 1]
 * @param drstDrs [0 or 1]
 * @param range0 [0,1 or 2]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fbi2fee (uint32_t fext, uint8_t dmx32, uint8_t drstDrs, uint8_t range0, uint8_t frdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint32_t fllRefClock;
    uint8_t dmx32Tmp;
    uint8_t drstDrsTmp;
    uint8_t range0Tmp;
    uint8_t frdivTmp;
    uint8_t tempReg;
    
    /* The RANGE0 and FRDIV evaluation have to be done out of this function. */ 

    tempReg = MCG_C2_REG(regmap); 
    tempReg &= ~(MCG_C2_RANGE0_MASK);
    tempReg |= MCG_C2_RANGE0(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap); 
    tempReg &= ~(MCG_C1_FRDIV_MASK);
    tempReg |= MCG_C1_FRDIV(frdiv);
    MCG_C1_REG(regmap) = tempReg;
    
    tempReg = MCG_C4_REG(regmap); 
    tempReg &=  ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK); //Clear these field of C4_REG
    tempReg |= ((dmx32 << MCG_C4_DMX32_SHIFT) | (MCG_C4_DRST_DRS(drstDrs))); //Set the multiplication factor for FLL
    MCG_C4_REG(regmap) = tempReg;

    MCG_C1_REG(regmap) &= ~(MCG_C1_IREFS_MASK | MCG_C1_CLKS_MASK); //select the external reference for FLL and MCGCLKOUT = FLL_OUT
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS0_MASK)  == (MCG_C2_EREFS0_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT0_MASK) != MCG_S_OSCINIT0_MASK); //esce quanto oscillatore � inizializzato
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); //esce quando seleziona il riferimento esterno
    while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(0)));
    
    //Now in FEE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
//    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
    range0Tmp = ((MCG_C2_REG(regmap) & MCG_C2_RANGE0_MASK) >> MCG_C2_RANGE0_SHIFT);
    frdivTmp = ((MCG_C1_REG(regmap) & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT);
    dmx32Tmp = ((MCG_C4_REG(regmap) & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT);
    drstDrsTmp = ((MCG_C4_REG(regmap) & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT);
            
    if(range0Tmp == 0)
    {
        if(frdivTmp == 0) fllRefClock = fext/1;
        else if(frdivTmp == 1) fllRefClock = fext/2;
        else if(frdivTmp == 2) fllRefClock = fext/4;
        else if(frdivTmp == 3) fllRefClock = fext/8;
        else if(frdivTmp == 4) fllRefClock = fext/16;
        else if(frdivTmp == 5) fllRefClock = fext/32;
        else if(frdivTmp == 6) fllRefClock = fext/64;
        else if(frdivTmp == 7) fllRefClock = fext/128;
    }
    else 
    {
        if(frdivTmp == 0) fllRefClock = fext/32;
        else if(frdivTmp == 1) fllRefClock = fext/64;
        else if(frdivTmp == 2) fllRefClock = fext/128;
        else if(frdivTmp == 3) fllRefClock = fext/256;
        else if(frdivTmp == 4) fllRefClock = fext/512;
        else if(frdivTmp == 5) fllRefClock = fext/1024;
        else if(frdivTmp == 6) fllRefClock = fext/1280;
        else if(frdivTmp == 7) fllRefClock = fext/1536;
    }
    
    if(dmx32Tmp == 0)
    {
        if(drstDrsTmp == 0) foutMcg = fllRefClock*640;
        else if(drstDrsTmp == 1) foutMcg = fllRefClock*1280;
        else if(drstDrsTmp == 2) foutMcg = fllRefClock*1920;
        else if(drstDrsTmp == 3) foutMcg = fllRefClock*2560;
    }
    else if(dmx32Tmp == 1)
    {
        if(drstDrsTmp == 0) foutMcg = fllRefClock*732;
        else if(drstDrsTmp == 1) foutMcg = fllRefClock*1464;
        else if(drstDrsTmp == 2) foutMcg = fllRefClock*2197;
        else if(drstDrsTmp == 3) foutMcg = fllRefClock*2929;
    }
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param fext external frequency reference
 * @param range0 [0,1 or 2]
 * @param frdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fee2fbe (uint32_t fext, uint8_t range0, uint8_t frdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t tempReg;
    
    /* The RANGE0 and FRDIV evaluation have to be done out of this function. */ 
  
    tempReg = MCG_C2_REG(regmap); 
    tempReg &= ~(MCG_C2_RANGE0_MASK | MCG_C2_LP_MASK);
    tempReg |= MCG_C2_RANGE0(range0);
    MCG_C2_REG(regmap) = tempReg;

    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_CLKS_MASK);
    tempReg |= (MCG_C1_FRDIV(frdiv) | MCG_C1_CLKS(2));
    MCG_C1_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS0_MASK)  == (MCG_C2_EREFS0_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT0_MASK) != MCG_S_OSCINIT0_MASK); //esce quanto oscillatore � inizializzato
    }
    while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)));
    
    //Now in FBE
    
	MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param fext external frequency reference
 * @param dmx32 [0 or 1]
 * @param drstDrs [0 or 1]
 * @param range0: [0,1 or 2]
 * @param frdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fbe2fee (uint32_t fext, uint8_t dmx32, uint8_t drstDrs, uint8_t range0, uint8_t frdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint32_t fllRefClock;
    uint8_t dmx32Tmp;
    uint8_t drstDrsTmp;
    uint8_t range0Tmp;
    uint8_t frdivTmp;
    uint8_t tempReg;
    
    /* The RANGE0 and FRDIV evaluation have to be done out of this function. */ 
    
    tempReg = MCG_C2_REG(regmap); 
    tempReg &= ~(MCG_C2_RANGE0_MASK);
    tempReg |= MCG_C2_RANGE0(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_FRDIV_MASK);
    tempReg |= MCG_C1_FRDIV(frdiv);
    MCG_C1_REG(regmap) = tempReg;
    
    tempReg = MCG_C4_REG(regmap);
    tempReg &=  ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK); //Clear these field of C4_REG
    tempReg |= ((dmx32 << MCG_C4_DMX32_SHIFT) | (MCG_C4_DRST_DRS(drstDrs))); //Set the multiplication factor for FLL
    MCG_C4_REG(regmap) = tempReg;

    MCG_C1_REG(regmap) &= ~(MCG_C1_CLKS_MASK); //select FLL_OUT as MCGOUTCLK
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS0_MASK)  == (MCG_C2_EREFS0_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT0_MASK) != MCG_S_OSCINIT0_MASK); //esce quanto oscillatore � inizializzato
    }
    
    while((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(0)); 
    
    //Now in FEE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
//    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
    range0Tmp = ((MCG_C2_REG(regmap) & MCG_C2_RANGE0_MASK) >> MCG_C2_RANGE0_SHIFT);
    frdivTmp = ((MCG_C1_REG(regmap) & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT);
    dmx32Tmp = ((MCG_C4_REG(regmap) & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT);
    drstDrsTmp = ((MCG_C4_REG(regmap) & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT);
            
    if(range0Tmp == 0)
    {
        if(frdivTmp == 0) fllRefClock = fext/1;
        else if(frdivTmp == 1) fllRefClock = fext/2;
        else if(frdivTmp == 2) fllRefClock = fext/4;
        else if(frdivTmp == 3) fllRefClock = fext/8;
        else if(frdivTmp == 4) fllRefClock = fext/16;
        else if(frdivTmp == 5) fllRefClock = fext/32;
        else if(frdivTmp == 6) fllRefClock = fext/64;
        else if(frdivTmp == 7) fllRefClock = fext/128;
    }
    else 
    {
        if(frdivTmp == 0) fllRefClock = fext/32;
        else if(frdivTmp == 1) fllRefClock = fext/64;
        else if(frdivTmp == 2) fllRefClock = fext/128;
        else if(frdivTmp == 3) fllRefClock = fext/256;
        else if(frdivTmp == 4) fllRefClock = fext/512;
        else if(frdivTmp == 5) fllRefClock = fext/1024;
        else if(frdivTmp == 6) fllRefClock = fext/1280;
        else if(frdivTmp == 7) fllRefClock = fext/1536;
    }
    
    if(dmx32Tmp == 0)
    {
        if(drstDrsTmp == 0) foutMcg = fllRefClock*640;
        else if(drstDrsTmp == 1) foutMcg = fllRefClock*1280;
        else if(drstDrsTmp == 2) foutMcg = fllRefClock*1920;
        else if(drstDrsTmp == 3) foutMcg = fllRefClock*2560;
    }
    else if(dmx32Tmp == 1)
    {
        if(drstDrsTmp == 0) foutMcg = fllRefClock*732;
        else if(drstDrsTmp == 1) foutMcg = fllRefClock*1464;
        else if(drstDrsTmp == 2) foutMcg = fllRefClock*2197;
        else if(drstDrsTmp == 3) foutMcg = fllRefClock*2929;
    }
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param fext external frequency reference
 * @param range0 [0,1 or 2]
 * @param frdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fbi2fbe (uint32_t fext, uint8_t range0, uint8_t frdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t tempReg;
    
    /* The RANGE0 and FRDIV evaluation have to be done out of this function. */ 
    
    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_RANGE0_MASK | MCG_C2_LP_MASK);
    tempReg |= MCG_C2_RANGE0(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap); 
    tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK | MCG_C1_CLKS_MASK);
    tempReg |= (MCG_C1_FRDIV(frdiv) | MCG_C1_CLKS(2));
    MCG_C1_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS0_MASK)  == (MCG_C2_EREFS0_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT0_MASK) != MCG_S_OSCINIT0_MASK); 
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); 
    while((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(2));
    
    //Now in FBE
    
	MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param ircs [0 or 1]
 * @param fcrdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fbe2fbi (uint8_t ircs, uint8_t fcrdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t fcrdivTmp;
    uint8_t tempReg; 
    
    tempReg = MCG_SC_REG(regmap);
    tempReg &= ~(MCG_SC_FCRDIV_MASK); //Clear the FCRDIV field of SC_REG
    tempReg |= MCG_SC_FCRDIV(fcrdiv);
    MCG_SC_REG(regmap) = tempReg;

    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
    tempReg |= (ircs << MCG_C2_IRCS_SHIFT);
    MCG_C2_REG(regmap) = tempReg; 

    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_IREFS_MASK | MCG_C1_CLKS_MASK); //Clear the IREFS and CLKS fields of C1_REG
    //Set Internal Reference Clock Slow as FLL_IN and Internal Reference Clock as MCGOUTCLK
    tempReg |= (MCG_C1_IREFS_MASK | MCG_C1_CLKS(1)); 
    MCG_C1_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while(((MCG_S_REG(regmap) & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT) != ircs); 
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) != MCG_S_IREFST_MASK); 
    while((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(1));
        
    //Now in FBI
    
    MCG_C1 |= MCG_C1_IRCLKEN_MASK;
    
    fcrdivTmp = ((MCG_SC_REG(regmap) & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT);
    
    if(ircs == 1)
    {
        if(fcrdivTmp == 0)  foutMcg = CLOCK_FREQ_INTERNAL_FAST/1;
        else if(fcrdivTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
        else if(fcrdivTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_FAST/4;
        else if(fcrdivTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_FAST/8;
        else if(fcrdivTmp == 4) foutMcg = CLOCK_FREQ_INTERNAL_FAST/16;
        else if(fcrdivTmp == 5) foutMcg = CLOCK_FREQ_INTERNAL_FAST/32;
        else if(fcrdivTmp == 6) foutMcg = CLOCK_FREQ_INTERNAL_FAST/64;
        else if(fcrdivTmp == 7) foutMcg = CLOCK_FREQ_INTERNAL_FAST/128;
    }
    else foutMcg = CLOCK_FREQ_INTERNAL_SLOW;
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param prdiv [0:24]
 * @param vdiv [0:31]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fbe2pbe (uint32_t fext, uint8_t prdiv, uint8_t vdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t tempReg;
    
    tempReg = MCG_C5_REG(regmap);
    tempReg &= ~(MCG_C5_PRDIV0_MASK);
    tempReg |= MCG_C5_PRDIV0(prdiv);
    MCG_C5_REG(regmap) = tempReg;
    
    tempReg = MCG_C6_REG(regmap);
    tempReg &= ~(MCG_C6_VDIV0_MASK | MCG_C6_PLLS_MASK);
    tempReg |= (MCG_C6_VDIV0(vdiv) | MCG_C6_PLLS_MASK); //Set VDIV0 multiplier and PLL use (PLLS = 1, if PLLS = 0 FLL)
    MCG_C6_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) != MCG_S_PLLST_MASK);
    while((MCG_S_REG(regmap) & MCG_S_LOCK0_MASK) != MCG_S_LOCK0_MASK);
    
    //Now in PBE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param fext external frequency reference
 * @param range0 [0,1 or 2]
 * @param frdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_pbe2fbe (uint32_t fext, uint8_t range0, uint8_t frdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
	uint32_t foutMcg = 0;
    uint8_t tempReg;
    
    /* The RANGE0 and FRDIV evaluation have to be done out of this function. */ 
    
    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_RANGE0_MASK | MCG_C2_LP_MASK);
    tempReg |= MCG_C2_RANGE0(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK);
    tempReg |= (MCG_C1_FRDIV(frdiv));
    MCG_C1_REG(regmap) = tempReg;
    
    MCG_C6_REG(regmap) &= ~(MCG_C6_PLLS_MASK);
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS0_MASK)  == (MCG_C2_EREFS0_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT0_MASK) != MCG_S_OSCINIT0_MASK); 
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); 
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) == MCG_S_PLLST_MASK);
    
    //Now in FBE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param prdiv [0:24]
 * @param vdiv [0:31]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_pbe2pee (uint32_t fext, uint8_t prdiv, uint8_t vdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
	uint32_t foutMcg = 0;
    uint8_t prdivTmp;
    uint8_t vdivTmp;
    uint8_t tempReg;
    
    /* For changing foutMcg from the transition PBE to PEE I have to turn off the pll and so I go in BLPE */
    MCG_C2_REG(regmap) |=  MCG_C2_LP_MASK; //LP = 1 for Low Power Mode.
    MCG_C6_REG(regmap) &= ~(MCG_C6_PLLS_MASK);
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) == MCG_S_PLLST_MASK);
    
    //Now in BLPE
    
    MCG_C2_REG(regmap) &= ~(MCG_C2_LP_MASK);
    
    tempReg = MCG_C5_REG(regmap); 
    tempReg &= ~(MCG_C5_PRDIV0_MASK);
    tempReg |= MCG_C5_PRDIV0(prdiv); //Set PRDIV0 divider
    MCG_C5_REG(regmap) = tempReg;
    
    tempReg = MCG_C6_REG(regmap);
    tempReg &= ~(MCG_C6_VDIV0_MASK | MCG_C6_PLLS_MASK);
    tempReg |= (MCG_C6_VDIV0(vdiv) | MCG_C6_PLLS_MASK); //Set VDIV0 multiplier and PLL use (PLLS = 1, if PLLS = 0 FLL)
    MCG_C6_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) != MCG_S_PLLST_MASK);
    while((MCG_S_REG(regmap) & MCG_S_LOCK0_MASK) != MCG_S_LOCK0_MASK);
    
    //Now in PBE
    
    MCG_C1_REG(regmap) &= ~(MCG_C1_CLKS_MASK); //Set C1_CLKS = 0 for PLL_OUT/FLL_OUT as MCGCLKOUT
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(3));
    
    //Now in PEE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
//    SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
    
    prdivTmp = ((MCG_C5_REG(regmap) & MCG_C5_PRDIV0_MASK) >> MCG_C5_PRDIV0_SHIFT);
    vdivTmp = ((MCG_C6_REG(regmap) & MCG_C6_VDIV0_MASK) >> MCG_C6_VDIV0_SHIFT);
    
    foutMcg = (fext/(prdivTmp + 1));
    foutMcg *= (vdivTmp + 24);
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param prdiv [0:24]
 * @param vdiv [0:31]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_pee2pbe (uint32_t fext, uint8_t prdiv, uint8_t vdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t tempReg;
    
    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_CLKS_MASK);
    tempReg |= MCG_C1_CLKS(2);
    MCG_C1_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(2));
    
    //Now in PBE
    
    /* For changing foutMcg from the transition PBE to PEE I have to turn off the pll and so I go in BLPE */
    MCG_C2_REG(regmap) |=  MCG_C2_LP_MASK; //LP = 1 for Low Power Mode.
    MCG_C6_REG(regmap) &= ~(MCG_C6_PLLS_MASK);
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) == MCG_S_PLLST_MASK);
    
    //Now in BLPE
    
    MCG_C2_REG(regmap) &= ~(MCG_C2_LP_MASK);
   
    tempReg = MCG_C5_REG(regmap);
    tempReg &= ~(MCG_C5_PRDIV0_MASK);
    tempReg |= MCG_C5_PRDIV0(prdiv); //Set PRDIV0 divider
    MCG_C5_REG(regmap) = tempReg;
    
    tempReg = MCG_C6_REG(regmap);
    tempReg &= ~(MCG_C6_VDIV0_MASK | MCG_C6_PLLS_MASK);
    tempReg |= (MCG_C6_VDIV0(vdiv) | MCG_C6_PLLS_MASK); //Set VDIV0 multiplier and PLL use (PLLS = 1, if PLLS = 0 FLL)
    MCG_C6_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) != MCG_S_PLLST_MASK);
    while((MCG_S_REG(regmap) & MCG_S_LOCK0_MASK) != MCG_S_LOCK0_MASK);

    //Now in PBE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param ircs [0 or 1]
 * @param fcrdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fbi2blpi (uint8_t ircs, uint8_t fcrdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t fcrdivTmp;
    uint8_t tempReg;
    
    tempReg = MCG_SC_REG(regmap);
    tempReg &= ~(MCG_SC_FCRDIV_MASK); //Clear the FCRDIV field of SC_REG
    tempReg |= MCG_SC_FCRDIV(fcrdiv);
    MCG_SC_REG(regmap) = tempReg;

    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
    tempReg |= ((ircs << MCG_C2_IRCS_SHIFT) | MCG_C2_LP_MASK);
    MCG_C2_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while(((MCG_S_REG(regmap) & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT) != ircs); 
    
    //Now in BLPI
    
    MCG_C1 |= MCG_C1_IRCLKEN_MASK;
    
    fcrdivTmp = ((MCG_SC_REG(regmap) & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT);
    
    if(ircs == 1)
    {
        if(fcrdivTmp == 0)  foutMcg = CLOCK_FREQ_INTERNAL_FAST/1;
        else if(fcrdivTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
        else if(fcrdivTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_FAST/4;
        else if(fcrdivTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_FAST/8;
        else if(fcrdivTmp == 4) foutMcg = CLOCK_FREQ_INTERNAL_FAST/16;
        else if(fcrdivTmp == 5) foutMcg = CLOCK_FREQ_INTERNAL_FAST/32;
        else if(fcrdivTmp == 6) foutMcg = CLOCK_FREQ_INTERNAL_FAST/64;
        else if(fcrdivTmp == 7) foutMcg = CLOCK_FREQ_INTERNAL_FAST/128;
    }
    else foutMcg = CLOCK_FREQ_INTERNAL_SLOW;
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param ircs [0 or 1]
 * @param fcrdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_blpi2fbi (uint8_t ircs, uint8_t fcrdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t fcrdivTmp;
    uint8_t tempReg; 
    
    tempReg = MCG_SC_REG(regmap);
    tempReg &= ~(MCG_SC_FCRDIV_MASK); //Clear the FCRDIV field of SC_REG
    tempReg |= MCG_SC_FCRDIV(fcrdiv);
    MCG_SC_REG(regmap) = tempReg;

    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
    tempReg |= (ircs << MCG_C2_IRCS_SHIFT);
    MCG_C2_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while(((MCG_S_REG(regmap) & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT) != ircs);
    
    //Now in FBI
    
    MCG_C1 |= MCG_C1_IRCLKEN_MASK;
    
    fcrdivTmp = ((MCG_SC_REG(regmap) & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT);
    
    if(ircs == 1)
    {
        if(fcrdivTmp == 0)  foutMcg = CLOCK_FREQ_INTERNAL_FAST/1;
        else if(fcrdivTmp == 1) foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
        else if(fcrdivTmp == 2) foutMcg = CLOCK_FREQ_INTERNAL_FAST/4;
        else if(fcrdivTmp == 3) foutMcg = CLOCK_FREQ_INTERNAL_FAST/8;
        else if(fcrdivTmp == 4) foutMcg = CLOCK_FREQ_INTERNAL_FAST/16;
        else if(fcrdivTmp == 5) foutMcg = CLOCK_FREQ_INTERNAL_FAST/32;
        else if(fcrdivTmp == 6) foutMcg = CLOCK_FREQ_INTERNAL_FAST/64;
        else if(fcrdivTmp == 7) foutMcg = CLOCK_FREQ_INTERNAL_FAST/128;
    }
    else foutMcg = CLOCK_FREQ_INTERNAL_SLOW;
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param fext external frequency reference
 * @return output frequency of MCG module.
 */
static uint32_t Clock_fbe2blpe (uint32_t fext)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    
    MCG_C2_REG(regmap) |=  MCG_C2_LP_MASK; //LP = 1 for Low Power Mode.
    
    //Now in BLPE
    
	MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param fext external frequency reference
 * @param range0 [0,1 or 2]
 * @param frdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_blpe2fbe (uint32_t fext, uint8_t range0, uint8_t frdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t tempReg;
    
    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_RANGE0_MASK | MCG_C2_LP_MASK);
    tempReg |= MCG_C2_RANGE0(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK);
    tempReg |= (MCG_C1_FRDIV(frdiv));
    MCG_C1_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS0_MASK)  == (MCG_C2_EREFS0_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT0_MASK) != MCG_S_OSCINIT0_MASK); //esce quanto oscillatore � inizializzato
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); //esce quando seleziona il riferimento esterno
    
    //Now in FBE
    
	MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param fext external frequency reference
 * @return output frequency of MCG module.
 */
static uint32_t Clock_pbe2blpe (uint32_t fext)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    
    MCG_C2_REG(regmap) |=  MCG_C2_LP_MASK; //LP = 1 for Low Power Mode.
    
    MCG_C6_REG(regmap) &= ~(MCG_C6_PLLS_MASK);
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) == MCG_S_PLLST_MASK);
    
    //Now in BLPE
    
	MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param prdiv [0:24]
 * @param vdiv [0:31]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_blpe2pbe (uint32_t fext, uint8_t prdiv, uint8_t vdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    uint8_t tempReg;
    
    tempReg = MCG_C5_REG(regmap);
    tempReg &= ~(MCG_C5_PRDIV0_MASK);
    tempReg |= MCG_C5_PRDIV0(prdiv); //Set PRDIV0 divider
    MCG_C5_REG(regmap) = tempReg; 
    
    tempReg = MCG_C6_REG(regmap);
    tempReg &= ~(MCG_C6_VDIV0_MASK | MCG_C6_PLLS_MASK);
    tempReg |= (MCG_C6_VDIV0(vdiv) | MCG_C6_PLLS_MASK); //Set VDIV0 multiplier and PLL use (PLLS = 1, if PLLS = 0 FLL)
    MCG_C6_REG(regmap) = tempReg;
    
    MCG_C2_REG(regmap) &=  ~(MCG_C2_LP_MASK); //LP = 0
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) != MCG_S_PLLST_MASK);
    while((MCG_S_REG(regmap) & MCG_S_LOCK0_MASK) != MCG_S_LOCK0_MASK);
    
    //Now in PBE
    
	MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 *
 * @return Current state.
 */
Clock_State Clock_getCurrentState ()
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    if (((MCG_C1_REG(regmap) & MCG_C1_CLKS_MASK) >> MCG_C1_CLKS_SHIFT) == 0)
    {
        if (((MCG_C6_REG(regmap) & MCG_C6_PLLS_MASK) >> MCG_C6_PLLS_SHIFT) == 1)
        {
            return CLOCK_PEE;
        }
        else
        {
            if (((MCG_C1_REG(regmap) & MCG_C1_IREFS_MASK) >> MCG_C1_IREFS_SHIFT) == 0)
                return CLOCK_FEE;
            else
                return CLOCK_FEI;
        }
    }
    else if (((MCG_C1_REG(regmap) & MCG_C1_CLKS_MASK) >> MCG_C1_CLKS_SHIFT) == 1)
    {
        if (((MCG_C2_REG(regmap) & MCG_C2_LP_MASK) >> MCG_C2_LP_SHIFT) == 0)
            return CLOCK_FBI;
        else
            return CLOCK_BLPI;
    }
    else if (((MCG_C1_REG(regmap) & MCG_C1_CLKS_MASK) >> MCG_C1_CLKS_SHIFT) == 2)
    {
        if (((MCG_C2_REG(regmap) & MCG_C2_LP_MASK) >> MCG_C2_LP_SHIFT) == 1)
        {
            return CLOCK_BLPE;
        }
        else
        {
            if (((MCG_C6_REG(regmap) & MCG_C6_PLLS_MASK) >> MCG_C6_PLLS_SHIFT) == 1)
                return CLOCK_PBE;
            else
                return CLOCK_FBE;
        }
    }
}

/**
 * @brief
 *
 * @param fext external frequency reference
 * @param stateOut desired state
 * @param prdiv
 * @param vdiv
 * @param dmx32
 * @param drstDrs
 * @param range0
 * @param frdiv
 * @param ircs
 * @param fcrdiv
 * @return output frequency of MCG module.
 */
static uint32_t Clock_StateTransition (uint32_t fext, Clock_State stateOut, uint8_t prdiv, uint8_t vdiv, uint8_t dmx32,
                             uint8_t drstDrs, uint8_t range0, uint8_t frdiv, uint8_t ircs, uint8_t fcrdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
    uint32_t foutMcg = 0;
    Clock_State stateIn;
    uint8_t tempReg;
    uint32_t fllRefClock;

    stateIn = Clock_getCurrentState();

    if (stateIn == CLOCK_FEI)
    {
        if (stateOut == CLOCK_FEI)
        {
            tempReg = MCG_C4_REG(regmap);
            tempReg &= ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK);
            tempReg |= (dmx32 << MCG_C4_DMX32_SHIFT | MCG_C4_DRST_DRS(drstDrs));
            MCG_C4_REG(regmap) = tempReg;

            if(dmx32 == 0)
            {
                if(drstDrs == 0) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*640;
                else if(drstDrs == 1) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1280;
                else if(drstDrs == 2) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1920;
                else if(drstDrs == 3) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2560;
            }
            else if(dmx32 == 1)
            {
                if(drstDrs == 0) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*732;
                else if(drstDrs == 1) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*1464;
                else if(drstDrs == 2) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2197;
                else if(drstDrs == 3) foutMcg = CLOCK_FREQ_INTERNAL_SLOW*2929;
            }
        }
        else if (stateOut == CLOCK_FEE)
            foutMcg = Clock_fei2fee(fext, dmx32, drstDrs, range0, frdiv);
        else if (stateOut == CLOCK_FBI)
            foutMcg = Clock_fei2fbi(ircs, fcrdiv);
        else if (stateOut == CLOCK_FBE)
            foutMcg = Clock_fei2fbe(fext, range0, frdiv);
        else if (stateOut == CLOCK_PBE)
        {
            Clock_fei2fbe(fext, range0, frdiv);
            foutMcg = Clock_fbe2pbe(fext, prdiv, vdiv);
        }
        else if (stateOut == CLOCK_PEE)
        {
            Clock_fei2fbe(fext, range0, frdiv);
            Clock_fbe2pbe(fext, prdiv, vdiv);
            foutMcg = Clock_pbe2pee(fext, prdiv, vdiv);
        }
        else if (stateOut == CLOCK_BLPI)
        {
            Clock_fei2fbi (ircs, fcrdiv);
            foutMcg = Clock_fbi2blpi(ircs, fcrdiv);
        }
        else if (stateOut == CLOCK_BLPE)
        {
            Clock_fei2fbe(fext, range0, frdiv);
            foutMcg = Clock_fbe2blpe(fext);
        }
    }
    else if (stateIn == CLOCK_FEE)
    {
        if (stateOut == CLOCK_FEE)
        {
            tempReg = MCG_C4_REG(regmap);
            tempReg &= ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK);
            tempReg |= (dmx32 << MCG_C4_DMX32_SHIFT | MCG_C4_DRST_DRS(drstDrs));
            MCG_C4_REG(regmap) = tempReg;

            if(range0 == 0)
            {
                if(frdiv == 0) fllRefClock = fext/1;
                else if(frdiv == 1) fllRefClock = fext/2;
                else if(frdiv == 2) fllRefClock = fext/4;
                else if(frdiv == 3) fllRefClock = fext/8;
                else if(frdiv == 4) fllRefClock = fext/16;
                else if(frdiv == 5) fllRefClock = fext/32;
                else if(frdiv == 6) fllRefClock = fext/64;
                else if(frdiv == 7) fllRefClock = fext/128;
            }
            else
            {
                if(frdiv == 0) fllRefClock = fext/32;
                else if(frdiv == 1) fllRefClock = fext/64;
                else if(frdiv == 2) fllRefClock = fext/128;
                else if(frdiv == 3) fllRefClock = fext/256;
                else if(frdiv == 4) fllRefClock = fext/512;
                else if(frdiv == 5) fllRefClock = fext/1024;
                else if(frdiv == 6) fllRefClock = fext/1280;
                else if(frdiv == 7) fllRefClock = fext/1536;
            }

            if(dmx32 == 0)
            {
                if(drstDrs == 0) foutMcg = fllRefClock*640;
                else if(drstDrs == 1) foutMcg = fllRefClock*1280;
                else if(drstDrs == 2) foutMcg = fllRefClock*1920;
                else if(drstDrs == 3) foutMcg = fllRefClock*2560;
            }
            else if(dmx32 == 1)
            {
                if(drstDrs == 0) foutMcg = fllRefClock*732;
                else if(drstDrs == 1) foutMcg = fllRefClock*1464;
                else if(drstDrs == 2) foutMcg = fllRefClock*2197;
                else if(drstDrs == 3) foutMcg = fllRefClock*2929;
            }
        }
        else if (stateOut == CLOCK_FEI)
            foutMcg = Clock_fee2fei(dmx32, drstDrs);
        else if (stateOut == CLOCK_FBI)
            foutMcg = Clock_fee2fbi(ircs, fcrdiv);
        else if (stateOut == CLOCK_FBE)
            foutMcg = Clock_fee2fbe(fext, range0, frdiv);
        else if (stateOut == CLOCK_PBE)
        {
            Clock_fee2fbe(fext, range0, frdiv);
            foutMcg = Clock_fbe2pbe(fext, prdiv, vdiv);
        }
        else if (stateOut == CLOCK_PEE)
        {
            Clock_fee2fbe(fext, range0, frdiv);
            Clock_fbe2pbe(fext, prdiv, vdiv);
            foutMcg = Clock_pbe2pee(fext, prdiv, vdiv);
        }
        else if(stateOut == CLOCK_BLPI)
        {
            Clock_fee2fbi (ircs, fcrdiv);
            foutMcg = Clock_fbi2blpi (ircs, fcrdiv);
        }
        else if(stateOut == CLOCK_BLPE)
        {
            Clock_fee2fbe (fext, range0, frdiv);
            foutMcg = Clock_fbe2blpe (fext);
        }
    }
    else if(stateIn == CLOCK_FBI)
    {
        if(stateOut == CLOCK_FBI)
        {
            tempReg = MCG_SC_REG(regmap);
            tempReg &= ~(MCG_SC_FCRDIV_MASK); //Clear the FCRDIV field of SC_REG
            tempReg |= MCG_SC_FCRDIV(fcrdiv);
            MCG_SC_REG(regmap) = tempReg;

            tempReg = MCG_C2_REG(regmap);
            tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
            tempReg |= (ircs << MCG_C2_IRCS_SHIFT);
            MCG_C2_REG(regmap) = tempReg;

            if(ircs == 1)
            {
                if(fcrdiv == 0)  foutMcg = CLOCK_FREQ_INTERNAL_FAST/1;
                else if(fcrdiv == 1) foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
                else if(fcrdiv == 2) foutMcg = CLOCK_FREQ_INTERNAL_FAST/4;
                else if(fcrdiv == 3) foutMcg = CLOCK_FREQ_INTERNAL_FAST/8;
                else if(fcrdiv == 4) foutMcg = CLOCK_FREQ_INTERNAL_FAST/16;
                else if(fcrdiv == 5) foutMcg = CLOCK_FREQ_INTERNAL_FAST/32;
                else if(fcrdiv == 6) foutMcg = CLOCK_FREQ_INTERNAL_FAST/64;
                else if(fcrdiv == 7) foutMcg = CLOCK_FREQ_INTERNAL_FAST/128;
            }
            else foutMcg = CLOCK_FREQ_INTERNAL_SLOW;
        }
        else if (stateOut == CLOCK_FEI)
            foutMcg = Clock_fbi2fei(dmx32, drstDrs);
        else if (stateOut == CLOCK_FEE)
            foutMcg = Clock_fbi2fee(fext, dmx32, drstDrs, range0, frdiv);
        else if (stateOut == CLOCK_FBE)
            foutMcg = Clock_fbi2fbe(fext, range0, frdiv);
        else if (stateOut == CLOCK_PBE)
        {
            Clock_fbi2fbe(fext, range0, frdiv);
            foutMcg = Clock_fbe2pbe(fext, prdiv, vdiv);
        }
        else if (stateOut == CLOCK_PEE)
        {
            Clock_fbi2fbe(fext, range0, frdiv);
            Clock_fbe2pbe(fext, prdiv, vdiv);
            foutMcg = Clock_pbe2pee(fext, prdiv, vdiv);
        }
        else if (stateOut == CLOCK_BLPI)
            foutMcg = Clock_fbi2blpi(ircs, fcrdiv);
        else if (stateOut == CLOCK_BLPE)
        {
            Clock_fbi2fbe(fext, range0, frdiv);
            foutMcg = Clock_fbe2blpe(fext);
        }
    }
    else if (stateIn == CLOCK_FBE)
    {
        if (stateOut == CLOCK_FBE)
        {
            tempReg = MCG_C2_REG(regmap);
            tempReg &= ~(MCG_C2_RANGE0_MASK | MCG_C2_LP_MASK);
            tempReg |= MCG_C2_RANGE0(range0);
            MCG_C2_REG(regmap) = tempReg;

            tempReg = MCG_C1_REG(regmap);
            tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK | MCG_C1_CLKS_MASK);
            tempReg |= (MCG_C1_FRDIV(frdiv) | MCG_C1_CLKS(2));
            MCG_C1_REG(regmap) = tempReg;

            foutMcg = fext;
        }
        else if (stateOut == CLOCK_FEI)
        {
            foutMcg = Clock_fbe2fei(dmx32, drstDrs);
        }
        else if (stateOut == CLOCK_FEE)
        {
        	foutMcg = Clock_fbe2fee(fext, dmx32, drstDrs, range0, frdiv);
        }
        else if (stateOut == CLOCK_FBI)
        {
        	foutMcg = Clock_fbe2fbi(ircs, fcrdiv);
        }
        else if (stateOut == CLOCK_PBE)
        {
        	foutMcg = Clock_fbe2pbe(fext, prdiv, vdiv);
        }
        else if (stateOut == CLOCK_PEE)
        {
        	Clock_fbe2pbe(fext, prdiv, vdiv);
        	foutMcg = Clock_pbe2pee(fext, prdiv, vdiv);
        }
        else if (stateOut == CLOCK_BLPI)
        {
        	Clock_fbe2fbi(ircs, fcrdiv);
        	foutMcg = Clock_fbi2blpi(ircs, fcrdiv);
        }
        else if (stateOut == CLOCK_BLPE)
        {
        	foutMcg = Clock_fbe2blpe(fext);
        }
    }
    else if (stateIn == CLOCK_PBE)
    {
    	if (stateOut == CLOCK_PBE)
    	{
    		Clock_pbe2blpe(fext);
    		foutMcg = Clock_blpe2pbe(fext, prdiv, vdiv);
    	}
    	else if (stateOut == CLOCK_PEE)
    	{
    		foutMcg = Clock_pbe2pee(fext, prdiv, vdiv);
    	}
    	else if (stateOut == CLOCK_FBE)
    	{
    		foutMcg = Clock_pbe2fbe(fext, range0, frdiv);
    	}
    	else if (stateOut == CLOCK_FEE)
    	{
    		Clock_pbe2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2fee(fext, dmx32, drstDrs, range0, frdiv);
    	}
    	else if (stateOut == CLOCK_FEI)
    	{
    		Clock_pbe2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2fei(dmx32, drstDrs);
    	}
    	else if (stateOut == CLOCK_FBI)
    	{
    		Clock_pbe2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2fbi(ircs, fcrdiv);
    	}
    	else if (stateOut == CLOCK_BLPI)
    	{
       		Clock_pbe2fbe(fext, range0, frdiv);
       		Clock_fbe2fbi(ircs, fcrdiv);
       		foutMcg = Clock_fbi2blpi(ircs, fcrdiv);
    	}
    	else if (stateOut == CLOCK_BLPE)
    	{
    		foutMcg = Clock_pbe2blpe(fext);
    	}
    }
    else if (stateIn == CLOCK_PEE)
    {
    	if (stateOut == CLOCK_PEE)
    	{
    		Clock_pee2pbe(fext, prdiv, vdiv);
    	    foutMcg = Clock_pbe2pee(fext, prdiv, vdiv);
    	}
    	else if (stateOut == CLOCK_PBE)
    	{
    		foutMcg = Clock_pee2pbe(fext, prdiv, vdiv);
    	}
    	else if (stateOut == CLOCK_FBE)
    	{
    		Clock_pee2pbe(fext, prdiv, vdiv);
    		foutMcg = Clock_pbe2fbe(fext, range0, frdiv);
    	}
    	else if (stateOut == CLOCK_FEE)
    	{
    		Clock_pee2pbe(fext, prdiv, vdiv);
    		Clock_pbe2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2fee(fext, dmx32, drstDrs, range0, frdiv);
    	}
    	else if (stateOut == CLOCK_FEI)
    	{
    		Clock_pee2pbe(fext, prdiv, vdiv);
    		Clock_pbe2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2fei(dmx32, drstDrs);
    	}
    	else if (stateOut == CLOCK_FBI)
    	{
    		Clock_pee2pbe(fext, prdiv, vdiv);
    		Clock_pbe2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2fbi(ircs, fcrdiv);
    	}
    	else if (stateOut == CLOCK_BLPI)
    	{
    		Clock_pee2pbe(fext, prdiv, vdiv);
    		Clock_pbe2fbe(fext, range0, frdiv);
    		Clock_fbe2fbi(ircs, fcrdiv);
    		foutMcg = Clock_fbi2blpi(ircs, fcrdiv);
    	}
    	else if (stateOut == CLOCK_BLPE)
    	{
    		Clock_pee2pbe(fext, prdiv, vdiv);
    		foutMcg = Clock_pbe2blpe(fext);
    	}
    }
    else if (stateIn == CLOCK_BLPI)
    {
    	if (stateOut == CLOCK_BLPI)
    	{
    	    tempReg = MCG_SC_REG(regmap);
    	    tempReg &= ~(MCG_SC_FCRDIV_MASK); //Clear the FCRDIV field of SC_REG
    	    tempReg |= MCG_SC_FCRDIV(fcrdiv);
    	    MCG_SC_REG(regmap) = tempReg;

    	    tempReg = MCG_C2_REG(regmap);
    	    tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
    	    tempReg |= ((ircs << MCG_C2_IRCS_SHIFT) | MCG_C2_LP_MASK);
    	    MCG_C2_REG(regmap) = tempReg;

    	    if(ircs == 1)
    	    {
    	        if(fcrdiv == 0)  foutMcg = CLOCK_FREQ_INTERNAL_FAST/1;
    	        else if(fcrdiv == 1) foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
    	        else if(fcrdiv == 2) foutMcg = CLOCK_FREQ_INTERNAL_FAST/4;
    	        else if(fcrdiv == 3) foutMcg = CLOCK_FREQ_INTERNAL_FAST/8;
    	        else if(fcrdiv == 4) foutMcg = CLOCK_FREQ_INTERNAL_FAST/16;
    	        else if(fcrdiv == 5) foutMcg = CLOCK_FREQ_INTERNAL_FAST/32;
    	        else if(fcrdiv == 6) foutMcg = CLOCK_FREQ_INTERNAL_FAST/64;
    	        else if(fcrdiv == 7) foutMcg = CLOCK_FREQ_INTERNAL_FAST/128;
    	    }
    	    else foutMcg = CLOCK_FREQ_INTERNAL_SLOW;
    	}
    	else if (stateOut == CLOCK_FBI)
    	{
    		foutMcg = Clock_blpi2fbi(ircs, fcrdiv);
    	}
    	else if (stateOut == CLOCK_FEI)
    	{
    		Clock_blpi2fbi(ircs, fcrdiv);
    		foutMcg = Clock_fbi2fei(dmx32, drstDrs);
    	}
    	else if (stateOut == CLOCK_FEE)
    	{
    		Clock_blpi2fbi(ircs, fcrdiv);
    		foutMcg = Clock_fbi2fee(fext, dmx32, drstDrs, range0, frdiv);
    	}
    	else if (stateOut == CLOCK_FBE)
    	{
    		Clock_blpi2fbi(ircs, fcrdiv);
    		foutMcg = Clock_fbi2fbe(fext, range0, frdiv);
    	}
    	else if (stateOut == CLOCK_PBE)
    	{
    		Clock_blpi2fbi(ircs, fcrdiv);
    		Clock_fbi2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2pbe(fext, prdiv, vdiv);
    	}
    	else if (stateOut == CLOCK_PEE)
    	{
    		Clock_blpi2fbi(ircs, fcrdiv);
    		Clock_fbi2fbe(fext, range0, frdiv);
    		Clock_fbe2pbe(fext, prdiv, vdiv);
    		foutMcg = Clock_pbe2pee(fext, prdiv, vdiv);
    	}
    	else if (stateOut == CLOCK_BLPE)
    	{
    		Clock_blpi2fbi(ircs, fcrdiv);
    		Clock_fbi2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2blpe(fext);
    	}
    }
    else if (stateIn == CLOCK_BLPE)
    {
    	if (stateOut == CLOCK_BLPE)
    	{
    		foutMcg = fext;
    	}
    	else if (stateOut == CLOCK_PBE)
    	{
    		foutMcg = Clock_blpe2pbe(fext, prdiv, vdiv);
    	}
    	else if (stateOut == CLOCK_PEE)
    	{
    		Clock_blpe2pbe(fext, prdiv, vdiv);
    		foutMcg = Clock_pbe2pee(fext, prdiv, vdiv);
    	}
    	else if (stateOut == CLOCK_FBE)
    	{
    		foutMcg = Clock_blpe2fbe(fext, range0, frdiv);
    	}
    	else if (stateOut == CLOCK_FEI)
    	{
    		Clock_blpe2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2fei(dmx32, drstDrs);
    	}
    	else if (stateOut == CLOCK_FEE)
    	{
    		Clock_blpe2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2fee(fext, dmx32, drstDrs, range0, frdiv);
    	}
    	else if (stateOut == CLOCK_FBI)
    	{
    		Clock_blpe2fbe(fext, range0, frdiv);
    		foutMcg = Clock_fbe2fbi(ircs, fcrdiv);
    	}
    	else if (stateOut == CLOCK_BLPI)
    	{
    		Clock_blpe2fbe(fext, range0, frdiv);
    		Clock_fbe2fbi(ircs, fcrdiv);
    		foutMcg = Clock_fbi2blpi(ircs, fcrdiv);
    	}
    }

    return foutMcg;
}


/**
 * @brief
 *
 * @param source: CLOCK_SYSTEM or CLOCK_BUS
 * @return Source frequency.
 */
uint32_t Clock_getFrequency (Clock_Source source)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
	uint8_t cpuDiv;
	uint8_t busDiv;

	cpuDiv = ((SIM_CLKDIV1_REG(SIM_BASE_PTR) & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT);
	busDiv = ((SIM_CLKDIV1_REG(SIM_BASE_PTR) & SIM_CLKDIV1_OUTDIV4_MASK) >> SIM_CLKDIV1_OUTDIV4_SHIFT);

	if (Clock_device.devInitialized == 1)
	{
		switch (source)
		{
		    case CLOCK_BUS:
		    	return (Clock_device.foutMcg/(cpuDiv + 1))/(busDiv + 1);
		    case CLOCK_SYSTEM:
		    	return Clock_device.foutMcg/(cpuDiv + 1);
		}
	}

	return 0; //return 0 if unknown source or MCG not initialized
}


/**
 * @brief
 *
 * @param busDivider: value of bus_clock divider
 * @param flexbusDivider: value of flexbus_clock divider
 * @param flashDivider: value of flash_clock divider
 * @return 1 if dividers are setted.
 */
System_Errors Clock_setDividers(uint8_t busDivider, uint8_t flexbusDivider, uint8_t flashDivider)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
	uint32_t cpuFreq = Clock_getFrequency(CLOCK_BUS);
	uint32_t tempReg;

	if ((cpuFreq/busDivider) > CLOCK_MAX_FREQ_BUS)
	{
		return ERRORS_MCG_OUT_OF_RANGE;
	}
	else
	{
	    tempReg = SIM_CLKDIV1_REG(SIM_BASE_PTR);
	    tempReg &= ~(SIM_CLKDIV1_OUTDIV4_MASK);
	    tempReg |= (SIM_CLKDIV1_OUTDIV4(busDivider-1));
	    SIM_CLKDIV1_REG(SIM_BASE_PTR) = tempReg;
	    return ERRORS_NO_ERROR;
	}
}

/**
 * @brief
 *
 * @param source: INTERNAL, EXTERNAL or CRYSTAL
 * @param fext: external frequency reference
 * @param foutSys: desired output frequency
 * @param busDivider: value of bus_clock divider
 * @return Error code.
 */
System_Errors Clock_init (Clock_Config *config)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
	uint32_t fext = config->fext;
	Clock_Origin source = config->source;
	uint32_t foutSys = config->foutSys;

	uint8_t busDivider = config->busDivider;
	uint8_t flexbusDivider = config->flexbusDivider;
	uint8_t flashDivider = config->flashDivider;

    uint32_t fdiff = CLOCK_INIT_DIFF; //impongo all'inizio un valore di fdiff pi� alto del massimo possibile (in questo caso 200MHz)
    uint8_t outdiv1 = 1; //divisore dell'mcgout per ottenere il clock di sistema
    uint32_t foutMcg = 0;
    Clock_State stateOutTmp; //stato in cui deve andare il sistema, utilizzata nelle operazioni di confronto
    System_Errors error;
    uint32_t f; //frequenza di uscita dello stato trovato
    uint32_t tempReg;

    Clock_State stateOut; //stato finale in cui deve andare il sistema

    //variabili utilizzate nel confronto per calcolare prdiv e vdiv se si utilizza il pll
    uint32_t diff;
    uint32_t f1; //f1=fext/prdiv
    uint32_t f2; //f2=f1*vdiv
    //dichiaro i puntatori
    uint8_t i;
    uint8_t j;
    uint8_t k;

    // variabili rappresentanti i campi dei registri dell'mcg utilizzate nelle operazioni di calcolo
    uint8_t prdivTmp = (MCG_C5_REG(regmap) & MCG_C5_PRDIV0_MASK) >> MCG_C5_PRDIV0_SHIFT;
    uint8_t vdivTmp = (MCG_C6_REG(regmap) & MCG_C6_VDIV0_MASK) >> MCG_C6_VDIV0_SHIFT;
    uint8_t dmx32Tmp = (MCG_C4_REG(regmap) & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT;
    uint8_t drstDrsTmp = (MCG_C4_REG(regmap) & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT;
    uint8_t ircsTmp = (MCG_C2_REG(regmap) & MCG_C2_IRCS_MASK) >> MCG_C2_IRCS_SHIFT;
    uint8_t fcrdivTmp = (MCG_SC_REG(regmap) & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT;
    uint8_t frdivTmp = (MCG_C1_REG(regmap) & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT;;
    uint8_t range0Tmp = (MCG_C2_REG(regmap) & MCG_C2_RANGE0_MASK) >> MCG_C2_RANGE0_SHIFT;;
    uint16_t fll_r;

    //variabili rappresentanti i campi dei registri dell'mcg finali
    uint8_t prdiv;
    uint8_t vdiv;
    uint8_t dmx32;
    uint8_t drstDrs;
    uint8_t ircs;
    uint8_t fcrdiv;
    uint8_t frdiv;
    uint8_t range0;

    if (Clock_device.devInitialized ==1)
    {
    	Clock_device.foutMcg = foutMcg;
    	Clock_device.mcgState = Clock_getCurrentState();
    	Clock_device.mcgError = ERRORS_MCG_JUST_INIT;
    	Clock_device.devInitialized = 0;
    	return error = ERRORS_MCG_JUST_INIT;
    }

    if(foutSys < 100000)
    {
    	Clock_device.foutMcg = foutMcg;
    	Clock_device.mcgState = Clock_getCurrentState();
    	Clock_device.mcgError = ERRORS_MCG_UNDER_100khz;
    	Clock_device.devInitialized = 0;
    	return error = ERRORS_MCG_UNDER_100khz;
    }

    /* calculation of foutMcg from foutSys */

    //MCG_C2_REG(regmap) |= MCG_C2_HGO0_MASK;

    /* calculation of RANGE 0 and frdivider */
    if((source == CLOCK_EXTERNAL) || (source == CLOCK_CRYSTAL))
    {

        range0Tmp = 0;
        if((CLOCK_MIN_FREQ_RANGE0_OSC_IN <= fext) && (fext <= CLOCK_MAX_FREQ_RANGE0_OSC_IN))
        {
            range0Tmp = 0;
        }
        else if(((CLOCK_MIN_FREQ_RANGE1_OSC_IN) <= fext) && (fext <= (CLOCK_MAX_FREQ_RANGE1_OSC_IN)))
        {
            range0Tmp = 1;
        }
        else if(((CLOCK_MIN_FREQ_RANGE2_OSC_IN) < fext) && (fext <= (CLOCK_MAX_FREQ_RANGE2_OSC_IN)))
        {
            range0Tmp = 2;
        }
        else
        {
        	Clock_device.foutMcg = foutMcg;
        	Clock_device.mcgState = Clock_getCurrentState();
        	Clock_device.mcgError = ERRORS_MCG_EXTERNAL_REFERENCE_OUT_OF_RANGE;
        	Clock_device.devInitialized = 0;
        	return error = ERRORS_MCG_EXTERNAL_REFERENCE_OUT_OF_RANGE;
        }


        fll_r = 0;
        if(range0Tmp == 0)
        {
            fll_r = 1;
            frdivTmp = 0;
        }
        else
        {
            if(fext <= CLOCK_MAX_FREQ_FLL_IN*32)  //39062.5*32
            {
                fll_r = 32;
                frdivTmp = 0;
            }
            else if(fext <= CLOCK_MAX_FREQ_FLL_IN*64) //39062.5*64
            {
                fll_r = 64;
                frdivTmp = 1;
            }
            else if(fext <= CLOCK_MAX_FREQ_FLL_IN*128) //39062.5*128
            {
                fll_r = 128;
                frdivTmp = 2;
            }
            else if(fext <= CLOCK_MAX_FREQ_FLL_IN*256) //39062.5*256
            {
                fll_r = 256;
                frdivTmp = 3;
            }
            else if(fext <= CLOCK_MAX_FREQ_FLL_IN*512) //39062.5*512
            {
                fll_r = 512;
                frdivTmp = 4;
            }
            else
            {
                fll_r = 1024;
                frdivTmp = 5;
            }
        }

        if(source == CLOCK_EXTERNAL)
        {
            MCG_C2_REG(regmap) &=  ~(MCG_C2_EREFS0_MASK); // select the external reference
            //  OSC0_CR |= OSC_CR_ERCLKEN_MASK;
        }
        else //if source == CLOCK_CRYSTAL
        {
            MCG_C2_REG(regmap) |= MCG_C2_EREFS0_MASK; // select the oscillator as reference
            //  OSC0_CR &= ~(OSC_CR_ERCLKEN_MASK);
        }
    }

    if(source == CLOCK_INTERNAL) error = ERRORS_MCG_NO_FREQUENCY;

    for(i = 1; i < 17; i++)
    {
        f = 0;
        foutMcg = foutSys*i;
        if(foutMcg <= (CLOCK_MAX_FREQ_MCG))
        {
            if((source == CLOCK_EXTERNAL) || (source == CLOCK_CRYSTAL))
            {
                if(fext > (CLOCK_MAX_FREQ_EXT)) //maximum value that can be externally bypassing the oscillator
                {
                    error = ERRORS_MCG_EXTERNAL_REFERENCE_OUT_OF_RANGE;;
                    continue;
                 }
                if(foutMcg == fext)
                {
                    stateOutTmp = CLOCK_BLPE;
                    f = fext;
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg < (CLOCK_MIN_FREQ_RANGE0_FLL_OUT)) || (((CLOCK_MAX_FREQ_RANGE0_FLL_OUT) < foutMcg) && (foutMcg < (CLOCK_MIN_FREQ_RANGE1_FLL_OUT))) || (foutMcg > (CLOCK_MAX_FREQ_PLL_OUT)))
                {
                    error = ERRORS_MCG_OUT_OF_RANGE;
                    continue;
                }
                else
                {

                    if(foutMcg == (fext/fll_r)*640)
                    {
                        stateOutTmp = CLOCK_FEE;
                        dmx32Tmp = 0;
                        drstDrsTmp = 0;
                        f = foutMcg;
                        error = ERRORS_NO_ERROR;
                    }
                    else if(foutMcg == (fext/fll_r)*732)
                    {
                        stateOutTmp = CLOCK_FEE;
                        dmx32Tmp = 1;
                        drstDrsTmp = 0;
                        f = foutMcg;
                        error = ERRORS_NO_ERROR;
                    }
                    else if(foutMcg == (fext/fll_r)*1280)
                    {
                        stateOutTmp = CLOCK_FEE;
                        dmx32Tmp = 0;
                        drstDrsTmp = 1;
                        f = foutMcg;
                        error = ERRORS_NO_ERROR;
                    }
                    else if(foutMcg == (fext/fll_r)*1464)
                    {
                        stateOutTmp = CLOCK_FEE;
                        dmx32Tmp = 1;
                        drstDrsTmp = 1;
                        f = foutMcg;
                        error = ERRORS_NO_ERROR;
                    }
                    else
                    {
                        // I will try with pll so calculate prdivTmp and vdivTmp
                        diff = CLOCK_INIT_DIFF;
                        for(j=1; j<26; j++)
                        {
                            f1 = fext/j;
                            if((f1 >= (CLOCK_MIN_FREQ_PLL_IN)) && (f1 <= (CLOCK_MAX_FREQ_PLL_IN)))
                            {
                                for(k=24; k<56; k++)
                                {
                                    f2 = f1*k;
                                    if(f2 <= (CLOCK_MAX_FREQ_PLL_OUT))
                                    {
                                        if(foutMcg > f2)
                                        {
                                            if(diff > (foutMcg - f2))
                                            {
                                                diff = foutMcg - f2;
                                                prdivTmp = j-1;
                                                vdivTmp = k-24;
                                            }
                                        }
                                        else
                                        {
                                            if(diff > (f2 - foutMcg))
                                            {
                                                diff = f2 - foutMcg;
                                                prdivTmp = j-1;
                                                vdivTmp = k-24;
                                            }
                                        }
                                    }

                                }
                            }
                        }
                        if(diff > (foutMcg*3)/100)
                        {
                            error = ERRORS_MCG_NO_FREQUENCY;
                            continue;
                        }
                        stateOutTmp = CLOCK_PEE;
                        f = (fext/(prdivTmp+1));
                        f = f*(vdivTmp+24);
                        error = ERRORS_NO_ERROR;
                    }
                }
            }
            else if(source == CLOCK_INTERNAL)
            {
                if((foutMcg >= ((CLOCK_FREQ_INTERNAL_SLOW*640)-((CLOCK_FREQ_INTERNAL_SLOW*640)/100))) && (foutMcg <= ((CLOCK_FREQ_INTERNAL_SLOW*640)+((CLOCK_FREQ_INTERNAL_SLOW*640)/100))))
                {
                	stateOutTmp = CLOCK_FEI;
                    dmx32Tmp = 0;
                    drstDrsTmp = 0;
                    f = CLOCK_FREQ_INTERNAL_SLOW*640; //F = 640
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_SLOW*1280)-((CLOCK_FREQ_INTERNAL_SLOW*1280)/100))) && (foutMcg <= ((CLOCK_FREQ_INTERNAL_SLOW*1280)+((CLOCK_FREQ_INTERNAL_SLOW*1280)/100))))
                {
                	stateOutTmp = CLOCK_FEI;
                    dmx32Tmp = 0;
                    drstDrsTmp = 1;
                    f = CLOCK_FREQ_INTERNAL_SLOW*1280; //F = 1280
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_SLOW)-((CLOCK_FREQ_INTERNAL_SLOW)/100))) && ((foutMcg <= ((CLOCK_FREQ_INTERNAL_SLOW)+((CLOCK_FREQ_INTERNAL_SLOW)/100)))))
                {
                	stateOutTmp = CLOCK_BLPI;
                    ircsTmp = 0;
                    f = CLOCK_FREQ_INTERNAL_SLOW;
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_FAST)-((CLOCK_FREQ_INTERNAL_FAST)/100))) && ((foutMcg <= ((CLOCK_FREQ_INTERNAL_FAST)+((CLOCK_FREQ_INTERNAL_FAST)/100)))))
                {
                	stateOutTmp = CLOCK_BLPI;
                    ircsTmp = 1;
                    fcrdivTmp = 0;
                    f = CLOCK_FREQ_INTERNAL_FAST/1;
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_FAST/2)-((CLOCK_FREQ_INTERNAL_FAST/2)/100))) && ((foutMcg <= ((CLOCK_FREQ_INTERNAL_FAST/2)+((CLOCK_FREQ_INTERNAL_FAST/2)/100)))))
                {
                	stateOutTmp = CLOCK_BLPI;
                    ircsTmp = 1;
                    fcrdivTmp = 1;
                    f = CLOCK_FREQ_INTERNAL_FAST/2;
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_FAST/4)-((CLOCK_FREQ_INTERNAL_FAST/4)/100))) && ((foutMcg <= ((CLOCK_FREQ_INTERNAL_FAST/4)+((CLOCK_FREQ_INTERNAL_FAST/4)/100)))))
                {
                	stateOutTmp = CLOCK_BLPI;
                    ircsTmp = 1;
                    fcrdivTmp = 2;
                    f = CLOCK_FREQ_INTERNAL_FAST/4;
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_FAST/8)-((CLOCK_FREQ_INTERNAL_FAST/8)/100))) && ((foutMcg <= ((CLOCK_FREQ_INTERNAL_FAST/8)+((CLOCK_FREQ_INTERNAL_FAST/8)/100)))))
                {
                	stateOutTmp = CLOCK_BLPI;
                    ircsTmp = 1;
                    fcrdivTmp = 3;
                    f = CLOCK_FREQ_INTERNAL_FAST/8;
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_FAST/16)-((CLOCK_FREQ_INTERNAL_FAST/16)/100))) && ((foutMcg <= ((CLOCK_FREQ_INTERNAL_FAST/16)+((CLOCK_FREQ_INTERNAL_FAST/16)/100)))))
                {
                	stateOutTmp = CLOCK_BLPI;
                    ircsTmp = 1;
                    fcrdivTmp = 4;
                    f = CLOCK_FREQ_INTERNAL_FAST/16;
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_FAST/32)-((CLOCK_FREQ_INTERNAL_FAST/32)/100))) && ((foutMcg <= ((CLOCK_FREQ_INTERNAL_FAST/32)+((CLOCK_FREQ_INTERNAL_FAST/32)/100)))))
                {
                	stateOutTmp = CLOCK_BLPI;
                    ircsTmp = 1;
                    fcrdivTmp = 5;
                    f = CLOCK_FREQ_INTERNAL_FAST/32;
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_FAST/64)-((CLOCK_FREQ_INTERNAL_FAST/64)/100))) && ((foutMcg <= ((CLOCK_FREQ_INTERNAL_FAST/64)+((CLOCK_FREQ_INTERNAL_FAST/64)/100)))))
                {
                	stateOutTmp = CLOCK_BLPI;
                	ircsTmp = 1;
                    fcrdivTmp = 6;
                    f = CLOCK_FREQ_INTERNAL_FAST/64;
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_FAST/128)-((CLOCK_FREQ_INTERNAL_FAST/128)/100))) && ((foutMcg <= ((CLOCK_FREQ_INTERNAL_FAST/128)+((CLOCK_FREQ_INTERNAL_FAST/128)/100)))))
                {
                	stateOutTmp = CLOCK_BLPI;
                    ircsTmp = 1;
                    fcrdivTmp = 7;
                    f = CLOCK_FREQ_INTERNAL_FAST/128;
                    error = ERRORS_NO_ERROR;
                }
                else
                {
                    continue;
                }
            }

            if(f == foutMcg)
            {
                outdiv1 = i;
                stateOut = stateOutTmp;
                prdiv = prdivTmp;
                vdiv = vdivTmp;
                dmx32 = dmx32Tmp;
                drstDrs = drstDrsTmp;
                ircs = ircsTmp;
                fcrdiv = fcrdivTmp;
                frdiv = frdivTmp;
                range0 = range0Tmp;
                break;
            }
            else if(f > foutMcg)
            {
                if(fdiff > f-foutMcg)
                {
                    fdiff = f-foutMcg;
                    outdiv1 = i;
                    stateOut = stateOutTmp;
                    prdiv = prdivTmp;
                    vdiv = vdivTmp;
                    dmx32 = dmx32Tmp;
                    drstDrs = drstDrsTmp;
                    ircs = ircsTmp;
                    fcrdiv = fcrdivTmp;
                    frdiv = frdivTmp;
                    range0 = range0Tmp;
                }
            }
            else
            {
                if(fdiff > foutMcg-f)
                {
                    fdiff = foutMcg-f;
                    outdiv1 = i;
                    stateOut = stateOutTmp;
                    prdiv = prdivTmp;
                    vdiv = vdivTmp;
                    dmx32 = dmx32Tmp;
                    drstDrs = drstDrsTmp;
                    ircs = ircsTmp;
                    fcrdiv = fcrdivTmp;
                    frdiv = frdivTmp;
                    range0 = range0Tmp;
                }
            }
        }
        else
        {
        	error = ERRORS_MCG_OUT_OF_RANGE;
        }
        foutMcg = foutSys*outdiv1;
        if(fdiff > (foutMcg*3)/100) error = ERRORS_MCG_NO_FREQUENCY;
        else error = ERRORS_NO_ERROR;
    }

    if(error != ERRORS_NO_ERROR)
    {
    	Clock_device.foutMcg = 0;
    	Clock_device.mcgState = Clock_getCurrentState();
    	Clock_device.mcgError = error;
    	Clock_device.devInitialized = 0;
    	return error;
    }

    /* select system_clock divider and bus_clock divider */
    tempReg = SIM_CLKDIV1_REG(SIM_BASE_PTR);
    tempReg &= ~(SIM_CLKDIV1_OUTDIV1_MASK);
    tempReg |= (SIM_CLKDIV1_OUTDIV1(outdiv1-1));
    SIM_CLKDIV1_REG(SIM_BASE_PTR) = tempReg;

    /* state transition */
    foutMcg = Clock_StateTransition(fext, stateOut, prdiv, vdiv, dmx32, drstDrs, range0, frdiv, ircs, fcrdiv);
    if(foutMcg == 0)
    {	Clock_device.foutMcg = 0;
        Clock_device.mcgState = Clock_getCurrentState();
        Clock_device.mcgError = ERRORS_MCG_NO_STATE;
        Clock_device.devInitialized = 0;
    	return error = ERRORS_MCG_NO_STATE;
    }
    else
    {
    	error = Clock_setDividers(busDivider, flexbusDivider, flashDivider);
        Clock_device.foutMcg = foutMcg;
        Clock_device.mcgState = Clock_getCurrentState();
        Clock_device.mcgError = error;
        Clock_device.devInitialized = 1;
        return error;
    }

    return error;
}

#endif // LIBOHIBOARD_KL15Z4

#endif
