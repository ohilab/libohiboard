/******************************************************************************
 * Copyright (C) 2014-2015 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Alessio Paolucci <m.giammarini@warcomeb.it>
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
 ******************************************************************************/

/**
 * @file libohiboard/source/clock_K60DZ10.c
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @Clock implementations for K60DZ10.
 */

#include "platforms.h"
#include "clock.h"

#if defined (LIBOHIBOARD_K60DZ10) || \
	defined (LIBOHIBOARD_OHIBOARD_R1)

#define CLOCK_INIT_DIFF                       200000000
#define CLOCK_MAX_FREQ_MCG                    100000000
#define CLOCK_MAX_FREQ_SYS                    100000000
#define CLOCK_MAX_FREQ_EXT                    50000000
#define CLOCK_MAX_FREQ_BUS					  50000000
#define CLOCK_MAX_FREQ_FLEXBUS				  50000000
#define CLOCK_MAX_FREQ_FLASH				  25000000
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
#define CLOCK_MIN_FREQ_RANGE2_FLL_OUT         60000000
#define CLOCK_MAX_FREQ_RANGE2_FLL_OUT         75000000
#define CLOCK_CENTER_FREQ_RANGE2_FLL_OUT      72000000
#define CLOCK_MIN_FREQ_RANGE3_FLL_OUT         80000000
#define CLOCK_MAX_FREQ_RANGE3_FLL_OUT         100000000
#define CLOCK_CENTER_FREQ_RANGE3_FLL_OUT      96000000
#define CLOCK_MIN_FREQ_PLL_OUT				  48000000
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
#define CLOCK_EXTERNAL_32KHZ_REFERENCE		  32768

typedef struct Clock_Device
{
	MCG_MemMapPtr regmap;

    uint32_t foutMcg;
    Clock_State mcgState;
    
    uint8_t coreDivider;

    uint8_t devInitialized;
    
    System_Errors mcgError;
}Clock_Device;

static Clock_Device Clock_device = {
	.regmap = MCG_BASE_PTR,

	.foutMcg = 0,
	.mcgState = CLOCK_FEI,

	.coreDivider = 1,

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
    tempReg &= ~(MCG_C2_RANGE_MASK);
    tempReg |= MCG_C2_RANGE(range0);
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
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS_MASK)  == (MCG_C2_EREFS_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT_MASK) != MCG_S_OSCINIT_MASK); //esce quanto oscillatore è inizializzato
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); //esce quando seleziona il riferimento esterno
    
    //Now in CLOCK_FEE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
    range0Tmp = ((MCG_C2_REG(regmap) & MCG_C2_RANGE_MASK) >> MCG_C2_RANGE_SHIFT);
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
    
    //Now in CLOCK_FEI
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
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
    
    //Now on CLOCK_FBI
    
    MCG_C1 |= MCG_C1_IRCLKEN_MASK;
    
    if(ircs == 1)
    {
        foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
    }
    else foutMcg = CLOCK_FREQ_INTERNAL_SLOW;
    
    return foutMcg;
}

/**
 * @brief
 * 
 * @param dmx32 [0 or 1]
 * @param drst_drs [0 or 1]
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
    
    //Now in CLOCK_FEI 
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
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
    tempReg &= ~(MCG_C2_RANGE_MASK | MCG_C2_LP_MASK);
    tempReg |= MCG_C2_RANGE(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK | MCG_C1_CLKS_MASK);
    tempReg |= (MCG_C1_FRDIV(frdiv) | MCG_C1_CLKS(2));
    MCG_C1_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS_MASK)  == (MCG_C2_EREFS_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT_MASK) != MCG_S_OSCINIT_MASK);
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); 
    while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)));
    
    //Now in CLOCK_FBE
    
	MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param dmx32 [0 or 1]
 * @param drst_drs [0 or 1]
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
    
    //Now in CLOCK_FEI 
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
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
    
    //Now in CLOCK_FBI
    
    MCG_C1 |= MCG_C1_IRCLKEN_MASK;
    
    
    if(ircs == 1)
    {
        foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
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
    tempReg &= ~(MCG_C2_RANGE_MASK);
    tempReg |= MCG_C2_RANGE(range0);
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
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS_MASK)  == (MCG_C2_EREFS_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT_MASK) != MCG_S_OSCINIT_MASK); //esce quanto oscillatore è inizializzato
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); //esce quando seleziona il riferimento esterno
    while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(0)));
    
    //Now in CLOCK_FEE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
    range0Tmp = ((MCG_C2_REG(regmap) & MCG_C2_RANGE_MASK) >> MCG_C2_RANGE_SHIFT);
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
    tempReg &= ~(MCG_C2_RANGE_MASK | MCG_C2_LP_MASK);
    tempReg |= MCG_C2_RANGE(range0);
    MCG_C2_REG(regmap) = tempReg;

    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_CLKS_MASK);
    tempReg |= (MCG_C1_FRDIV(frdiv) | MCG_C1_CLKS(2));
    MCG_C1_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS_MASK)  == (MCG_C2_EREFS_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT_MASK) != MCG_S_OSCINIT_MASK); //esce quanto oscillatore è inizializzato
    }
    while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)));
    
    //Now in CLOCK_FBE
    
	MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    
    return foutMcg = fext;
}

/**
 * @brief
 * 
 * @param fext external frequency reference
 * @param dmx32 [0 or 1]
 * @param drst_drs [0 or 1]
 * @param range0 [0,1 or 2]
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
    tempReg &= ~(MCG_C2_RANGE_MASK);
    tempReg |= MCG_C2_RANGE(range0);
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
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS_MASK)  == (MCG_C2_EREFS_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT_MASK) != MCG_S_OSCINIT_MASK); //esce quanto oscillatore è inizializzato
    }
    
    while((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(0)); 
    
    //Now in CLOCK_FEE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);
    
    range0Tmp = ((MCG_C2_REG(regmap) & MCG_C2_RANGE_MASK) >> MCG_C2_RANGE_SHIFT);
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
    tempReg &= ~(MCG_C2_RANGE_MASK | MCG_C2_LP_MASK);
    tempReg |= MCG_C2_RANGE(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap); 
    tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK | MCG_C1_CLKS_MASK);
    tempReg |= (MCG_C1_FRDIV(frdiv) | MCG_C1_CLKS(2));
    MCG_C1_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS_MASK)  == (MCG_C2_EREFS_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT_MASK) != MCG_S_OSCINIT_MASK); 
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); 
    while((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(2));
    
    //Now in CLOCK_FBE
    
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
        
    //Now in CLOCK_FBI
    
    MCG_C1 |= MCG_C1_IRCLKEN_MASK;
    
    
    if(ircs == 1)
    {
        foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
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
    tempReg &= ~(MCG_C5_PRDIV_MASK);
    tempReg |= MCG_C5_PRDIV(prdiv);
    MCG_C5_REG(regmap) = tempReg;
    
    tempReg = MCG_C6_REG(regmap);
    tempReg &= ~(MCG_C6_VDIV_MASK | MCG_C6_PLLS_MASK);
    tempReg |= (MCG_C6_VDIV(vdiv) | MCG_C6_PLLS_MASK); //Set VDIV0 multiplier and PLL use (PLLS = 1, if PLLS = 0 FLL)
    MCG_C6_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) != MCG_S_PLLST_MASK);
    while((MCG_S_REG(regmap) & MCG_S_LOCK_MASK) != MCG_S_LOCK_MASK);
    
    //Now in CLOCK_PBE
    
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
    tempReg &= ~(MCG_C2_RANGE_MASK | MCG_C2_LP_MASK);
    tempReg |= MCG_C2_RANGE(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK);
    tempReg |= (MCG_C1_FRDIV(frdiv));
    MCG_C1_REG(regmap) = tempReg;
    
    MCG_C6_REG(regmap) &= ~(MCG_C6_PLLS_MASK);
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS_MASK)  == (MCG_C2_EREFS_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT_MASK) != MCG_S_OSCINIT_MASK); 
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); 
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) == MCG_S_PLLST_MASK);
    
    //Now in CLOCK_FBE
    
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
    
    /* For changing foutMcg from the transition CLOCK_PBE to CLOCK_PEE I have to turn off the pll and so I go in CLOCK_BLPE */
    MCG_C2_REG(regmap) |=  MCG_C2_LP_MASK; //LP = 1 for Low Power Mode.
    MCG_C6_REG(regmap) &= ~(MCG_C6_PLLS_MASK);
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) == MCG_S_PLLST_MASK);
    
    //Now in CLOCK_BLPE
    
    MCG_C2_REG(regmap) &= ~(MCG_C2_LP_MASK);
    
    tempReg = MCG_C5_REG(regmap); 
    tempReg &= ~(MCG_C5_PRDIV_MASK);
    tempReg |= MCG_C5_PRDIV(prdiv); //Set PRDIV0 divider
    MCG_C5_REG(regmap) = tempReg;
    
    tempReg = MCG_C6_REG(regmap);
    tempReg &= ~(MCG_C6_VDIV_MASK | MCG_C6_PLLS_MASK);
    tempReg |= (MCG_C6_VDIV(vdiv) | MCG_C6_PLLS_MASK); //Set VDIV0 multiplier and PLL use (PLLS = 1, if PLLS = 0 FLL)
    MCG_C6_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) != MCG_S_PLLST_MASK);
    while((MCG_S_REG(regmap) & MCG_S_LOCK_MASK) != MCG_S_LOCK_MASK);
    
    //Now in CLOCK_PBE
    
    MCG_C1_REG(regmap) &= ~(MCG_C1_CLKS_MASK); //Set C1_CLKS = 0 for PLL_OUT/FLL_OUT as MCGCLKOUT
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(3));
    
    //Now in CLOCK_PEE
    
    MCG_C1 &= ~(MCG_C1_IRCLKEN_MASK);
    SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
    
    prdivTmp = ((MCG_C5_REG(regmap) & MCG_C5_PRDIV_MASK) >> MCG_C5_PRDIV_SHIFT);
    vdivTmp = ((MCG_C6_REG(regmap) & MCG_C6_VDIV_MASK) >> MCG_C6_VDIV_SHIFT);
    
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
    
    //Now in CLOCK_PBE
    
    /* For changing foutMcg from the transition CLOCK_PBE to CLOCK_PEE I have to turn off the pll and so I go in CLOCK_BLPE */
    MCG_C2_REG(regmap) |=  MCG_C2_LP_MASK; //LP = 1 for Low Power Mode.
    MCG_C6_REG(regmap) &= ~(MCG_C6_PLLS_MASK);
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) == MCG_S_PLLST_MASK);
    
    //Now in CLOCK_BLPE
    
    MCG_C2_REG(regmap) &= ~(MCG_C2_LP_MASK);
   
    tempReg = MCG_C5_REG(regmap);
    tempReg &= ~(MCG_C5_PRDIV_MASK);
    tempReg |= MCG_C5_PRDIV(prdiv); //Set PRDIV0 divider
    MCG_C5_REG(regmap) = tempReg;
    
    tempReg = MCG_C6_REG(regmap);
    tempReg &= ~(MCG_C6_VDIV_MASK | MCG_C6_PLLS_MASK);
    tempReg |= (MCG_C6_VDIV(vdiv) | MCG_C6_PLLS_MASK); //Set VDIV0 multiplier and PLL use (PLLS = 1, if PLLS = 0 FLL)
    MCG_C6_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) != MCG_S_PLLST_MASK);
    while((MCG_S_REG(regmap) & MCG_S_LOCK_MASK) != MCG_S_LOCK_MASK);

    //Now in CLOCK_PBE
    
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
    

    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
    tempReg |= ((ircs << MCG_C2_IRCS_SHIFT) | MCG_C2_LP_MASK);
    MCG_C2_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while(((MCG_S_REG(regmap) & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT) != ircs); 
    
    //Now in CLOCK_BLPI
    
    MCG_C1 |= MCG_C1_IRCLKEN_MASK;
    
    if(ircs == 1)
    {
        foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
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
    
    tempReg = MCG_C2_REG(regmap);
    tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
    tempReg |= (ircs << MCG_C2_IRCS_SHIFT);
    MCG_C2_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    while(((MCG_S_REG(regmap) & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT) != ircs);
    
    //Now in CLOCK_FBI
    
    MCG_C1 |= MCG_C1_IRCLKEN_MASK;
    
    
    if(ircs == 1)
    {
        foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
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
    
    //Now in CLOCK_BLPE
    
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
    tempReg &= ~(MCG_C2_RANGE_MASK | MCG_C2_LP_MASK);
    tempReg |= MCG_C2_RANGE(range0);
    MCG_C2_REG(regmap) = tempReg;
    
    tempReg = MCG_C1_REG(regmap);
    tempReg &= ~(MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK);
    tempReg |= (MCG_C1_FRDIV(frdiv));
    MCG_C1_REG(regmap) = tempReg;
    
    /* wait the refresh of the status register */
    if((MCG_C2_REG(regmap) & MCG_C2_EREFS_MASK)  == (MCG_C2_EREFS_MASK))
    {
        while((MCG_S_REG(regmap) & MCG_S_OSCINIT_MASK) != MCG_S_OSCINIT_MASK); //esce quanto oscillatore è inizializzato
    }
    
    while((MCG_S_REG(regmap) & MCG_S_IREFST_MASK) == MCG_S_IREFST_MASK); //esce quando seleziona il riferimento esterno
    
    //Now in CLOCK_FBE
    
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
    
    //Now in CLOCK_BLPE
    
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
    tempReg &= ~(MCG_C5_PRDIV_MASK);
    tempReg |= MCG_C5_PRDIV(prdiv); //Set PRDIV0 divider
    MCG_C5_REG(regmap) = tempReg; 
    
    tempReg = MCG_C6_REG(regmap);
    tempReg &= ~(MCG_C6_VDIV_MASK | MCG_C6_PLLS_MASK);
    tempReg |= (MCG_C6_VDIV(vdiv) | MCG_C6_PLLS_MASK); //Set VDIV0 multiplier and PLL use (PLLS = 1, if PLLS = 0 FLL)
    MCG_C6_REG(regmap) = tempReg;
    
    MCG_C2_REG(regmap) &=  ~(MCG_C2_LP_MASK); //LP = 0
    
    /* wait the refresh of the status register */
    while((MCG_S_REG(regmap) & MCG_S_PLLST_MASK) != MCG_S_PLLST_MASK);
    while((MCG_S_REG(regmap) & MCG_S_LOCK_MASK) != MCG_S_LOCK_MASK);
    
    //Now in CLOCK_PBE
    
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
            tempReg = MCG_C2_REG(regmap);
            tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
            tempReg |= (ircs << MCG_C2_IRCS_SHIFT);
            MCG_C2_REG(regmap) = tempReg;

            if(ircs == 1)
            {
                foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
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
            tempReg &= ~(MCG_C2_RANGE_MASK | MCG_C2_LP_MASK);
            tempReg |= MCG_C2_RANGE(range0);
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
    	    tempReg = MCG_C2_REG(regmap);
    	    tempReg &= ~(MCG_C2_IRCS_MASK | MCG_C2_LP_MASK); //Clear the IRCS and LP fields of C2_REG
    	    tempReg |= ((ircs << MCG_C2_IRCS_SHIFT) | MCG_C2_LP_MASK);
    	    MCG_C2_REG(regmap) = tempReg;

    	    if(ircs == 1)
    	    {
    	    	foutMcg = CLOCK_FREQ_INTERNAL_FAST/2;
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
 * @param source CLOCK_SYSTEM or CLOCK_BUS
 * @return Source frequency.
 */
uint32_t Clock_getFrequency (Clock_Source source)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;

	uint8_t cpuDiv;
	uint8_t busDiv;
	uint8_t flexbusDiv;
	uint8_t flashDiv;

	cpuDiv = ((SIM_CLKDIV1_REG(SIM_BASE_PTR) & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT);
	busDiv = ((SIM_CLKDIV1_REG(SIM_BASE_PTR) & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT);
	flexbusDiv = ((SIM_CLKDIV1_REG(SIM_BASE_PTR) & SIM_CLKDIV1_OUTDIV3_MASK) >> SIM_CLKDIV1_OUTDIV3_SHIFT);
	flashDiv = ((SIM_CLKDIV1_REG(SIM_BASE_PTR) & SIM_CLKDIV1_OUTDIV4_MASK) >> SIM_CLKDIV1_OUTDIV4_SHIFT);

	if (Clock_device.devInitialized == 1)
	{
		switch (source)
		{
		    case CLOCK_BUS:
		    	return Clock_device.foutMcg/(busDiv + 1);
		    case CLOCK_SYSTEM:
		    	return Clock_device.foutMcg/(cpuDiv + 1);
		    case CLOCK_FLEXBUS:
		    	return Clock_device.foutMcg/(flexbusDiv + 1);
		    case CLOCK_FLASH:
		    	return Clock_device.foutMcg/(flashDiv + 1);
		}
	}

	return 0; //return 0 if unknown source or MCG not initialized
}


/**
 * @brief
 *
 * @param busDivider value of bus_clock divider (This value must be equal to coreDivider or equal a multiple of coreDivider. In addiction to this bus clock have to be less or equal than 50MHz)
 * @param flexbusDivider value of flexbus_clock divider (This value must be more or equal than busDivider. In addiction to this flexbus_clock have to be less or equal than 50MHz)
 * @param flashDivider value of flash_clock divider (This value must be equal to busDivider or equal to a multiple of busDivider. In addiction to this bus clock have to be less or equal than 25MHz)
 * @return error if the peripheral clock is out of range.
 */
System_Errors Clock_setDividers(uint8_t busDivider, uint8_t flexbusDivider, uint8_t flashDivider)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;

	uint32_t mcgFreq = Clock_device.foutMcg;
	uint8_t coreDivider = Clock_device.coreDivider;
	uint32_t tempReg;

	if(Clock_device.devInitialized == 0)
	{
		return ERRORS_MCG_NOT_INIT;
	}
	else if((busDivider % coreDivider != 0) || (flexbusDivider < busDivider) || (flashDivider % busDivider !=0))
	{
		return ERRORS_MCG_ERRATA_DIVIDER;
	}
	else if((mcgFreq / busDivider > CLOCK_MAX_FREQ_BUS) || (mcgFreq / flexbusDivider > CLOCK_MAX_FREQ_FLEXBUS) || (mcgFreq / flashDivider > CLOCK_MAX_FREQ_FLASH))
	{
		return ERRORS_MCG_OUT_OF_RANGE;
	}
	else
	{
	    tempReg = SIM_CLKDIV1_REG(SIM_BASE_PTR);
	    tempReg &= ~(SIM_CLKDIV1_OUTDIV2_MASK | SIM_CLKDIV1_OUTDIV3_MASK | SIM_CLKDIV1_OUTDIV4_MASK);
	    tempReg |= (SIM_CLKDIV1_OUTDIV2(busDivider-1) | SIM_CLKDIV1_OUTDIV3(flexbusDivider-1) | SIM_CLKDIV1_OUTDIV4(flashDivider-1));
	    SIM_CLKDIV1_REG(SIM_BASE_PTR) = tempReg;
	    return ERRORS_NO_ERROR;
	}
}

uint8_t Clock_getCoreDivider()
{
	MCG_MemMapPtr regmap = Clock_device.regmap;

	uint8_t coreDiv;

	coreDiv = ((SIM_CLKDIV1_REG(SIM_BASE_PTR) & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT);

	return coreDiv;
}


/**
 * @brief
 *
 * @param source CLOCK_INTERNAL, CLOCK_EXTERNAL or CLOCK_CRYSTAL
 * @param fext external frequency reference
 * @param foutSys desired output frequency
 * @param busDivider value of bus_clock divider
 * @return Error code.
 */
System_Errors Clock_Init (Clock_Config *config)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;

	uint32_t fext = config->fext;
	Clock_Origin source = config->source;
	uint32_t foutSys = config->foutSys;

	uint8_t busDivider = config->busDivider;
	uint8_t flexbusDivider = config->flexbusDivider;
	uint8_t flashDivider = config->flashDivider;

    uint32_t fdiff = CLOCK_INIT_DIFF; //impongo all'inizio un valore di fdiff più alto del massimo possibile (in questo caso 200MHz)
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
    uint8_t prdivTmp = (MCG_C5_REG(regmap) & MCG_C5_PRDIV_MASK) >> MCG_C5_PRDIV_SHIFT;
    uint8_t vdivTmp = (MCG_C6_REG(regmap) & MCG_C6_VDIV_MASK) >> MCG_C6_VDIV_SHIFT;
    uint8_t dmx32Tmp = (MCG_C4_REG(regmap) & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT;
    uint8_t drstDrsTmp = (MCG_C4_REG(regmap) & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT;
    uint8_t ircsTmp = (MCG_C2_REG(regmap) & MCG_C2_IRCS_MASK) >> MCG_C2_IRCS_SHIFT;
    uint8_t fcrdivTmp = 1;
    uint8_t frdivTmp = (MCG_C1_REG(regmap) & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT;
    uint8_t range0Tmp = (MCG_C2_REG(regmap) & MCG_C2_RANGE_MASK) >> MCG_C2_RANGE_SHIFT;
    //uint8_t oscselTmp = (MCG_C7_REG(regmap) & MCG_C7_OSCSEL_MASK) >> MCG_C7_OSCSEL_SHIFT;
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
//  uint8_t oscsel;

    //variabili divisori del clock
    uint8_t outdiv1 = 1;
	uint8_t outdiv2 = 2;
	uint8_t outdiv3 = 2;
	uint8_t outdiv4 = 4;

    if (Clock_device.devInitialized == 1)
    {
    	Clock_device.foutMcg = foutMcg;
    	Clock_device.mcgState = Clock_getCurrentState();
    	Clock_device.coreDivider = 0;
    	Clock_device.mcgError = ERRORS_MCG_JUST_INIT;
    	Clock_device.devInitialized = 0;
    	return error = ERRORS_MCG_JUST_INIT;
    }

    if(foutSys < 100000)
    {
    	Clock_device.foutMcg = foutMcg;
    	Clock_device.mcgState = Clock_getCurrentState();
    	Clock_device.coreDivider = 0;
    	Clock_device.mcgError = ERRORS_MCG_UNDER_100khz;
    	Clock_device.devInitialized = 0;
    	return error = ERRORS_MCG_UNDER_100khz;
    }

    /* calculation of foutMcg from foutSys */

    //MCG_C2_REG(regmap) |= MCG_C2_HGO0_MASK;

    /* calculation of RANGE 0 and frdivider */
    if((source == CLOCK_EXTERNAL) || (source == CLOCK_CRYSTAL))
    {
    	if (fext > CLOCK_MAX_FREQ_EXT)
    	{
        	Clock_device.foutMcg = foutMcg;
        	Clock_device.mcgState = Clock_getCurrentState();
        	Clock_device.coreDivider = 0;
        	Clock_device.mcgError = ERRORS_MCG_EXTERNAL_REFERENCE_OUT_OF_RANGE;;
        	Clock_device.devInitialized = 0;
        	return error = ERRORS_MCG_EXTERNAL_REFERENCE_OUT_OF_RANGE;;
    	}

        if(source == CLOCK_CRYSTAL)
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
        	    Clock_device.coreDivider = 0;
        	    Clock_device.mcgError = ERRORS_MCG_EXTERNAL_REFERENCE_OUT_OF_RANGE;
        	    Clock_device.devInitialized = 0;
        	    return error = ERRORS_MCG_EXTERNAL_REFERENCE_OUT_OF_RANGE;
            }
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
            else if(fext <= CLOCK_MAX_FREQ_FLL_IN*1024) //39062.5*1024
            {
                fll_r = 1024;
                frdivTmp = 5;
            }
            else if(fext <= CLOCK_MAX_FREQ_FLL_IN*1280) //39062.5*1280
            {
                fll_r = 1280;
                frdivTmp = 6;
            }
            else if(fext <= CLOCK_MAX_FREQ_FLL_IN*1536) //39062.5*1536
            {
                fll_r = 1536;
                frdivTmp = 7;
            }
        }

        if(source == CLOCK_EXTERNAL)
        {
            MCG_C2_REG(regmap) &=  ~(MCG_C2_EREFS_MASK); // select the external reference
        }
        else if (source == CLOCK_CRYSTAL)
        {
            MCG_C2_REG(regmap) |= MCG_C2_EREFS_MASK; // select the oscillator as reference
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
//                if(fext > (CLOCK_MAX_FREQ_EXT)) //maximum value that can be externally bypassing the oscillator
//                {
//                    error = ERRORS_MCG_EXTERNAL_REFERENCE_OUT_OF_RANGE;
//                    continue;
//                }
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
                    else if(foutMcg == (fext/fll_r)*1280)
                    {
                        stateOutTmp = CLOCK_FEE;
                        dmx32Tmp = 0;
                        drstDrsTmp = 1;
                        f = foutMcg;
                        error = ERRORS_NO_ERROR;
                    }
                    else if(foutMcg == (fext/fll_r)*1920)
                    {
                        stateOutTmp = CLOCK_FEE;
                        dmx32Tmp = 0;
                        drstDrsTmp = 2;
                        f = foutMcg;
                        error = ERRORS_NO_ERROR;
                    }
                    else if(foutMcg == (fext/fll_r)*2560)
                    {
                        stateOutTmp = CLOCK_FEE;
                        dmx32Tmp = 0;
                        drstDrsTmp = 3;
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
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_SLOW*1920)-((CLOCK_FREQ_INTERNAL_SLOW*1920)/100))) && (foutMcg <= ((CLOCK_FREQ_INTERNAL_SLOW*1920)+((CLOCK_FREQ_INTERNAL_SLOW*1920)/100))))
                {
                	stateOutTmp = CLOCK_FEI;
                    dmx32Tmp = 0;
                    drstDrsTmp = 2;
                    f = CLOCK_FREQ_INTERNAL_SLOW*1920; //F = 1920
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_SLOW*2560)-((CLOCK_FREQ_INTERNAL_SLOW*2560)/100))) && (foutMcg <= ((CLOCK_FREQ_INTERNAL_SLOW*2560)+((CLOCK_FREQ_INTERNAL_SLOW*2560)/100))))
                {
                	stateOutTmp = CLOCK_FEI;
                    dmx32Tmp = 0;
                    drstDrsTmp = 3;
                    f = CLOCK_FREQ_INTERNAL_SLOW*2560; //F = 2560
                    error = ERRORS_NO_ERROR;
                }
                else if((foutMcg >= ((CLOCK_FREQ_INTERNAL_SLOW)-((CLOCK_FREQ_INTERNAL_SLOW)/100))) && ((foutMcg <= ((CLOCK_FREQ_INTERNAL_SLOW)+((CLOCK_FREQ_INTERNAL_SLOW)/100)))))
                {
                	stateOutTmp = CLOCK_BLPI;
                    ircsTmp = 0;
                    f = CLOCK_FREQ_INTERNAL_SLOW;
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
    	Clock_device.coreDivider = 0;
    	Clock_device.mcgError = error;
    	Clock_device.devInitialized = 0;
    	return error;
    }

    if (outdiv1 <= 8)
    {
    	outdiv2 = outdiv1*2;
    	outdiv3 = outdiv2;
    	if(outdiv2 <= 8)
    	{
    		outdiv4 = outdiv2*2;
    	}
    	else
    	{
    		outdiv4 = busDivider;
    	}
    }
    else
    {
    	outdiv2 = outdiv1;
    	outdiv3 = outdiv2;
    	outdiv4 = outdiv2;
    }
    /* select system_clock divider and bus_clock divider */
    tempReg = SIM_CLKDIV1_REG(SIM_BASE_PTR);
    tempReg &= ~(SIM_CLKDIV1_OUTDIV1_MASK | SIM_CLKDIV1_OUTDIV2_MASK | SIM_CLKDIV1_OUTDIV3_MASK | SIM_CLKDIV1_OUTDIV4_MASK);
    tempReg |= (SIM_CLKDIV1_OUTDIV1(outdiv1-1) | SIM_CLKDIV1_OUTDIV2(outdiv2-1) | SIM_CLKDIV1_OUTDIV3(outdiv3-1) | SIM_CLKDIV1_OUTDIV4(outdiv4-1));
    SIM_CLKDIV1_REG(SIM_BASE_PTR) = tempReg;



    /* state transition */
    foutMcg = Clock_StateTransition(fext, stateOut, prdiv, vdiv, dmx32, drstDrs, range0, frdiv, ircs, fcrdiv);
    if(foutMcg == 0)
    {
    	Clock_device.foutMcg = 0;
    	Clock_device.mcgState = Clock_getCurrentState();
    	Clock_device.mcgError = ERRORS_MCG_NO_STATE;
    	Clock_device.devInitialized = 0;
    	return error = ERRORS_MCG_NO_STATE;
    }
    else
    {
        Clock_device.foutMcg = foutMcg;
        Clock_device.mcgState = Clock_getCurrentState();
        Clock_device.devInitialized = 1;
		error = Clock_setDividers(busDivider, flexbusDivider, flashDivider);
		Clock_device.mcgError = error;
        return error;
    }

    return error;
}

#endif  // LIBOHIBOARD_K60DZ10




