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

#define CLOCK_IS_VALID_BUS_DIVIDER(DIVIDER) (((DIVIDER) == CLOCK_BUSDIVIDER_1) || \
                                             ((DIVIDER) == CLOCK_BUSDIVIDER_2) || \
                                             ((DIVIDER) == CLOCK_BUSDIVIDER_3) || \
                                             ((DIVIDER) == CLOCK_BUSDIVIDER_4) || \
                                             ((DIVIDER) == CLOCK_BUSDIVIDER_5) || \
                                             ((DIVIDER) == CLOCK_BUSDIVIDER_6) || \
                                             ((DIVIDER) == CLOCK_BUSDIVIDER_7) || \
                                             ((DIVIDER) == CLOCK_BUSDIVIDER_8))


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
    .regmapSim       = SIM,

    // Start-up: FEI mode, with LIRC active, DIV1 = 1
    .systemCoreClock = 20480000u,
    .state           = CLOCK_STATE_FEI,

    .externalClock   = 0u,
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
 * This function set the current mode of MCG module to FEI:
 * FLL Engaged Internal.
 *
 * @note Pay attention to ERRATA 7993 (e7993) at this link
 *       https://www.nxp.com/docs/en/errata/KINETIS_W_1N41U.pdf
 */
static System_Errors Clock_setFeiMode (void)
{
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
    tmpreg &= (~(MCG_C1_CLKS_MASK | MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK));
    tmpreg |= MCG_C1_CLKS(0)  | // PLL/FLL select
              MCG_C1_IREFS(1);  // External source select
    clk.regmap->C1 = tmpreg;

    // Wait until reference Source of FLL is the external reference clock.
    while ((clk.regmap->S & MCG_S_IREFST_MASK) == 0);

    // Restore DRST and DMX32 after initialization for e7993
    if (changeDrs == TRUE)
    {
        clk.regmap->C4 = tmpC4;
    }

    // Set MCG->C4
    // in FEI mode C4[DMX32] must be set to 0
    tmpC4 &= (~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK));
    tmpC4 |= (clk.changeParam.dmx_drst & MCG_C4_DRST_DRS_MASK);
    clk.regmap->C4 = tmpC4;

    //  Wait until output of the FLL is selected
    while ((clk.regmap->S & MCG_S_CLKST_MASK) != 0);

    return ERRORS_NO_ERROR;
}

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
              clk.changeParam.erefs0;                 // Set oscillator/external
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
    while ((clk.regmap->S & MCG_S_CLKST_MASK) != 0);

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
              clk.changeParam.erefs0;                 // Set oscillator/external
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

    return ERRORS_NO_ERROR;
}

/**
 * This function set the current mode of MCG module to PBE:
 * PLL Bypassed External.
 */
static System_Errors Clock_setPbeMode (void)
{
    ohiassert(clk.changeParam.final != CLOCK_STATE_NONE);

    uint8_t tmpreg = 0;

    // Disable LP
    clk.regmap->C2 &= ~(MCG_C2_LP_MASK);

    // Set MCG->C1
    tmpreg = clk.regmap->C1;
    tmpreg &= (~(MCG_C1_CLKS_MASK | MCG_C1_IREFS_MASK));
    tmpreg |= MCG_C1_CLKS(2)  | // External reference clock is selected
              MCG_C1_IREFS(0);  // External source select
    clk.regmap->C1 = tmpreg;

    // Wait until reference Source of FLL is the internal reference clock.
    while ((clk.regmap->S & (MCG_S_IREFST_MASK | MCG_S_CLKST_MASK))
            != (MCG_S_IREFST(0) | MCG_S_CLKST(2)));

    // Disable PLL first and wait FLL is selected.
    clk.regmap->C6 &= ~MCG_C6_PLLS_MASK;
    while ((clk.regmap->S & MCG_S_PLLST_MASK) != 0);

    // Configure the PLL
    tmpreg = clk.regmap->C5;
    tmpreg &= ~(MCG_C5_PRDIV0_MASK);
    tmpreg |= clk.changeParam.prdiv;
    clk.regmap->C5 = tmpreg;

    tmpreg = clk.regmap->C6;
    tmpreg &= ~(MCG_C6_VDIV0_MASK);
    tmpreg |= clk.changeParam.vdiv;
    clk.regmap->C6 = tmpreg;

    // Enable PLL mode
    clk.regmap->C6 |= MCG_C6_PLLS_MASK;
    // Wait for PLL mode changed
    while (!(clk.regmap->S & MCG_S_PLLST_MASK) == 0);

    return ERRORS_NO_ERROR;
}

/**
 * This function set the current mode of MCG module to PEE:
 * PLL Engaged External.
 *
 * @note This function works fine only when the previous status is PBE, where
 *       every PLL configurations is done.
 */
static System_Errors Clock_setPeeMode (void)
{
    // Wait until PLL is locked
    while ((clk.regmap->S & MCG_S_LOCK0_MASK) != MCG_S_LOCK0_MASK);

    // Select PLL/FLL as clock source
    clk.regmap->C1 &= ~(MCG_C1_CLKS_MASK);

    // Wait the refresh of the status register
    while ((clk.regmap->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(3));

    return ERRORS_NO_ERROR;
}

/**
 * This function set the current mode of MCG module to BPLI:
 * Bypassed Low-Power Internal.
 */
static System_Errors Clock_setBlpiMode (void)
{
    // Just enable low-power mode
    clk.regmap->C2 |= MCG_C2_LP_MASK;

    return ERRORS_NO_ERROR;
}

/**
 * This function set the current mode of MCG module to BPLE:
 * Bypassed Low-Power External.
 */
static System_Errors Clock_setBlpeMode (void)
{
    // Just enable low-power mode
    clk.regmap->C2 |= MCG_C2_LP_MASK;

    return ERRORS_NO_ERROR;
}

static System_Errors Clock_stateTransition (void)
{
    Clock_State state = Clock_getCurrentState();
    System_Errors err = ERRORS_NO_ERROR;

    do
    {
        state = Clock_mcgStateMatrix[state][clk.changeParam.final];

        switch (state)
        {
        case CLOCK_STATE_FEI:
            err = Clock_setFeiMode();
            break;
        case CLOCK_STATE_FBI:
            err = Clock_setFbiMode();
            break;
        case CLOCK_STATE_BLPI:
            err = Clock_setBlpiMode();
            break;
        case CLOCK_STATE_FEE:
            err = Clock_setFeeMode();
            break;
        case CLOCK_STATE_FBE:
            err = Clock_setFbeMode();
            break;
        case CLOCK_STATE_BLPE:
            err = Clock_setBlpeMode();
            break;
        case CLOCK_STATE_PBE:
            err = Clock_setPbeMode();
            break;
        case CLOCK_STATE_PEE:
            err = Clock_setPeeMode();
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

    // Initialize SysTick with default clock value (FEI mode @20480000Hz)
    System_systickInit(0);
    // Check current mode
    Clock_getCurrentState();

    if (config == NULL)
    {
        return ERRORS_CLOCK_NO_CONFIG;
    }

    ohiassert(CLOCK_IS_VALID_BUS_DIVIDER(config->busDivider));

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
                prdiv = (clk.changeParam.prdiv >> MCG_C5_PRDIV0_SHIFT ) + 1;
                vdiv = (clk.changeParam.vdiv >> MCG_C6_VDIV0_SHIFT) + CLOCK_REG_VDIV_MIN;
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

    // Save core clock
    clk.systemCoreClock = (clk.changeParam.foutMcg / (clk.changeParam.outdiv1 + 1));

    // Set bus divider
    if ((clk.systemCoreClock / config->busDivider) > CLOCK_FREQ_BUS_MAX)
    {
        return ERRORS_CLOCK_WRONG_DIVIDER;
    }
    tempReg = clk.regmapSim->CLKDIV1;
    tempReg &= (~(SIM_CLKDIV1_OUTDIV4_MASK));
    tempReg |= (SIM_CLKDIV1_OUTDIV4(config->busDivider));
    clk.regmapSim->CLKDIV1 = tempReg;

    // Setup SysTick
    System_systickInit(0);

    return ERRORS_NO_ERROR;
}

System_Errors Clock_deInit (void)
{
    return ERRORS_NO_ERROR;
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

uint32_t Clock_getOutputValue (Clock_Output output)
{
    switch (output)
    {
    case CLOCK_OUTPUT_SYSCLK:
        return clk.systemCoreClock;

    default:
        return 0;
    }
}

uint32_t Clock_getOscillatorValue (void)
{
    return clk.externalClock;
}


#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif
