/******************************************************************************
 * Copyright (C) 2014 A. C. Open Hardware Ideas Lab
 *
 * Authors:
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
 ******************************************************************************/

/**
 * @file libohiboard/include/clock_kl03.c
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @Clock implementations for MKL03Z4.
 */

#include "platforms.h"
#include "clock.h"

#if defined(MKL03Z4)

#define CLOCK_MAX_FREQ_MCG                    48000000
#define CLOCK_MAX_FREQ_SYS                    48000000
#define CLOCK_MAX_FREQ_EXT                    48000000
#define CLOCK_MAX_FREQ_BUS			          24000000
#define CLOCK_MAX_FREQ_FLASH				  24000000
#define CLOCK_FREQ_LIRC_SLOWREF               2000000
#define CLOCK_FREQ_LIRC_FASTREF               8000000
#define CLOCK_FREQ_HIRC_REF                   48000000
#define CLOCK_EXTERNAL_CRYSTAL				  32768

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
	.mcgState = CLOCK_LIRC8M,

	.devInitialized = 0,

	.mcgError = ERRORS_NO_ERROR,
};

/**
 * @brief: function to move to CLOCK_LIRC8M state
 *
 * @param fcrdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_lirc8m(uint8_t fcrdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
	
	uint8_t tempReg;
	uint8_t fcrdivTmp;
	uint32_t foutMcg = 0;

	tempReg = MCG_SC_REG(regmap);
	tempReg &= ~(MCG_SC_FCRDIV_MASK);
	tempReg |= MCG_SC_FCRDIV(fcrdiv);
	MCG_SC_REG(regmap) = tempReg;

	MCG_C2_REG(regmap) |= MCG_C2_IRCS_MASK;
	MCG_C1_REG(regmap) |= MCG_C1_IRCLKEN_MASK;

	tempReg = MCG_C1_REG(regmap);
	tempReg &= ~(MCG_C1_CLKS_MASK);
	tempReg |= MCG_C1_CLKS(1);
	MCG_C1_REG(regmap) = tempReg;

	while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(1)));

	fcrdivTmp = ((MCG_SC_REG(regmap) & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT);

    if(fcrdivTmp == 0)  foutMcg = CLOCK_FREQ_LIRC_FASTREF/1;
    else if(fcrdivTmp == 1) foutMcg = CLOCK_FREQ_LIRC_FASTREF/2;
    else if(fcrdivTmp == 2) foutMcg = CLOCK_FREQ_LIRC_FASTREF/4;
    else if(fcrdivTmp == 3) foutMcg = CLOCK_FREQ_LIRC_FASTREF/8;
    else if(fcrdivTmp == 4) foutMcg = CLOCK_FREQ_LIRC_FASTREF/16;
    else if(fcrdivTmp == 5) foutMcg = CLOCK_FREQ_LIRC_FASTREF/32;
    else if(fcrdivTmp == 6) foutMcg = CLOCK_FREQ_LIRC_FASTREF/64;
    else if(fcrdivTmp == 7) foutMcg = CLOCK_FREQ_LIRC_FASTREF/128;

    return foutMcg;
}

/**
 * @brief function to move to CLOCK_HIRC state
 *
 * @return output frequency of MCG module.
 */
static uint32_t Clock_hirc()
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
	uint8_t tempReg;

	MCG_MC_REG(regmap) |= MCG_MC_HIRCEN_MASK; //For enable the hirc clock also on Very Low Power Mode

	tempReg = MCG_C1_REG(regmap);
	tempReg &= ~(MCG_C1_CLKS_MASK);
	MCG_C1_REG(regmap) = tempReg;

    while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(0)));

    return CLOCK_FREQ_HIRC_REF;
}

/**
 * @brief function to move to CLOCK_EXT state
 *
 * @param fext External Frequency Reference
 * @param erefs0 0 = External Clock Reference, 1 = Oscillator Request
 * @return output frequency of MCG module.
 */
static uint32_t Clock_ext(uint32_t fext, uint8_t erefs0)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
	
	uint8_t tempReg;

	tempReg = MCG_C2_REG(regmap);
	tempReg &= ~(MCG_C2_EREFS0_MASK);
	tempReg |= erefs0 << MCG_C2_EREFS0_SHIFT;
	MCG_C2_REG(regmap) = tempReg;

	tempReg = MCG_C1_REG(regmap);
	tempReg &= ~(MCG_C1_CLKS_MASK);
	tempReg |= MCG_C1_CLKS(2);
	MCG_C1_REG(regmap) = tempReg;

	while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)));

	return fext;
}

/**
 * @brief function to move to CLOCK_LIRC2M state
 *
 * @param fcrdiv [0:7]
 * @return output frequency of MCG module.
 */
static uint32_t Clock_lirc2m(uint8_t fcrdiv)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
	
	uint8_t tempReg;
	uint8_t fcrdivTmp;
	uint32_t foutMcg = 0;

	tempReg = MCG_SC_REG(regmap);
	tempReg &= ~(MCG_SC_FCRDIV_MASK);
	tempReg |= MCG_SC_FCRDIV(fcrdiv);
	MCG_SC_REG(regmap) = tempReg;

	MCG_C2_REG(regmap) &= ~(MCG_C2_IRCS_MASK);
	MCG_C1_REG(regmap) |= MCG_C1_IRCLKEN_MASK;

	tempReg = MCG_C1_REG(regmap);
	tempReg &= ~(MCG_C1_CLKS_MASK);
	tempReg |= MCG_C1_CLKS(1);
	MCG_C1_REG(regmap) = tempReg;

	while(((MCG_S_REG(regmap) & MCG_S_CLKST_MASK) != MCG_S_CLKST(1)));

	fcrdivTmp = ((MCG_SC_REG(regmap) & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT);

    if(fcrdivTmp == 0)  foutMcg = CLOCK_FREQ_LIRC_SLOWREF/1;
    else if(fcrdivTmp == 1) foutMcg = CLOCK_FREQ_LIRC_SLOWREF/2;
    else if(fcrdivTmp == 2) foutMcg = CLOCK_FREQ_LIRC_SLOWREF/4;
    else if(fcrdivTmp == 3) foutMcg = CLOCK_FREQ_LIRC_SLOWREF/8;
    else if(fcrdivTmp == 4) foutMcg = CLOCK_FREQ_LIRC_SLOWREF/16;
    else if(fcrdivTmp == 5) foutMcg = CLOCK_FREQ_LIRC_SLOWREF/32;
    else if(fcrdivTmp == 6) foutMcg = CLOCK_FREQ_LIRC_SLOWREF/64;
    else if(fcrdivTmp == 7) foutMcg = CLOCK_FREQ_LIRC_SLOWREF/128;

    return foutMcg;
}

/**
 * @brief Function for transiction of states
 *
 * @param fext External Frequency Reference
 * @param stateOut Output State
 * @param fcrdiv [0:7]
 * @return MCG frequency.
 */
static uint32_t Clock_StateTransition(uint32_t fext, Clock_State stateOut, uint8_t fcrdiv, uint8_t erefs0)
{
	MCG_MemMapPtr regmap = Clock_device.regmap;
	
    uint32_t foutMcg = 0;
    Clock_State stateIn;
    uint8_t tempReg;

    stateIn = Clock_getCurrentState();

    if (stateIn == CLOCK_LIRC8M)
    {
        if (stateOut == CLOCK_LIRC8M)
        {
        	foutMcg = Clock_lirc8m(fcrdiv);
        }
        else if (stateOut == CLOCK_EXT)
        {
        	foutMcg = Clock_ext(fext, erefs0);
        }
        else if (stateOut == CLOCK_HIRC)
        {
        	foutMcg = Clock_hirc();
        }
        else if (stateOut == CLOCK_LIRC2M)
        {
        	Clock_hirc();
        	foutMcg = Clock_lirc2m(fcrdiv);
        }
    }
    else if (stateIn == CLOCK_HIRC)
    {
    	if (stateOut == CLOCK_HIRC)
    	{
    		foutMcg = Clock_hirc();
    	}
    	else if (stateOut == CLOCK_LIRC8M)
    	{
    		foutMcg = Clock_lirc8m(fcrdiv);
    	}
    	else if (stateOut == CLOCK_EXT)
    	{
    		foutMcg = Clock_ext(fext, erefs0);
    	}
    	else if (stateOut == CLOCK_LIRC2M)
    	{
    		foutMcg = Clock_lirc2m(fcrdiv);
    	}
    }
    else if (stateIn == CLOCK_EXT)
    {
    	if (stateOut == CLOCK_EXT)
    	{
    		foutMcg = Clock_ext(fext, erefs0);
    	}
    	else if (stateOut == CLOCK_HIRC)
    	{
    		foutMcg = Clock_hirc();
    	}
    	else if (stateOut == CLOCK_LIRC8M)
    	{
    		foutMcg = Clock_lirc8m(fcrdiv);
    	}
    	else if (stateOut == CLOCK_LIRC2M)
    	{
    		foutMcg = Clock_lirc2m(fcrdiv);
    	}
    }
    else if (stateIn == CLOCK_LIRC2M)
    {
    	if (stateOut == CLOCK_LIRC2M)
    	{
    	    foutMcg = Clock_lirc2m(fcrdiv);
    	}
    	else if (stateOut == CLOCK_EXT)
    	{
    		foutMcg = Clock_ext(fext, erefs0);
    	}
    	else if (stateOut == CLOCK_HIRC)
    	{
    		foutMcg = Clock_hirc();
    	}
    	else if (stateOut == CLOCK_LIRC8M)
    	{
    		Clock_hirc();
    		foutMcg = Clock_lirc8m(fcrdiv);
    	}
    }

    return foutMcg;
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
    	return CLOCK_HIRC;
    }
    else if (((MCG_C1_REG(regmap) & MCG_C1_CLKS_MASK) >> MCG_C1_CLKS_SHIFT) == 2)
    {
    	return CLOCK_EXT;
    }
    else if (((MCG_C1_REG(regmap) & MCG_C1_CLKS_MASK) >> MCG_C1_CLKS_SHIFT) == 1)
    {
    	if (((MCG_C2_REG(regmap) & MCG_C2_IRCS_MASK) >> MCG_C2_IRCS_SHIFT) == 0)
    	{
    		return CLOCK_LIRC2M;
    	}
    	else
    	{
    		return CLOCK_LIRC8M;
    	}
    }
}

/**
 * @brief
 *
 * @param busDivider value of bus_clock divider
 * @param flexbusDivider value of flexbus_clock divider
 * @param flashDivider value of flash_clock divider
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
 * @brief Get Frequency of some source
 *
 * @param source CLOCK_SYSTEM or CLOCK_BUS
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
	
    uint8_t outdiv1 = 1; //system clock divider for mcgout
    uint32_t foutMcg = 0;
    uint32_t f = 0; //temporay f
    Clock_State stateOut;
    System_Errors error = ERRORS_NO_ERROR;
    uint32_t tempReg;
    uint8_t erefs0;
    uint8_t fcrdiv;
    uint8_t i = 0;

	if(foutSys < 100000)
    {
    	Clock_device.foutMcg = foutMcg;
    	Clock_device.mcgState = Clock_getCurrentState();
    	Clock_device.mcgError = ERRORS_MCG_UNDER_100khz;
    	Clock_device.devInitialized = 0;
    	return error = ERRORS_MCG_UNDER_100khz;
    }

    for(i = 1; i < 17; i++)
    {
        f = 0;
        foutMcg = foutSys*i;
        if(foutMcg <= (CLOCK_MAX_FREQ_MCG))
        {
            if(source == CLOCK_EXTERNAL)
            {
            	erefs0 = 0;
            	if (foutMcg == fext)
            	{
            		stateOut = CLOCK_EXT;
            		f = foutMcg;
            		error = ERRORS_NO_ERROR;
            	}
            	else
            	{
            		error = ERRORS_MCG_NO_FREQUENCY;
            		continue;
            	}
            }
            else if(source == CLOCK_CRYSTAL)
            {
            	erefs0 = 1;
            	if (foutMcg == CLOCK_EXTERNAL_CRYSTAL)
            	{
            		stateOut = CLOCK_EXT;
            		f = CLOCK_EXTERNAL_CRYSTAL;
            		error = ERRORS_NO_ERROR;
            	}
            	else
            	{
            		error = ERRORS_MCG_NO_FREQUENCY;
            		continue;
            	}
            }
            else if(source == CLOCK_INTERNAL)
            {
            	if (foutMcg == CLOCK_FREQ_HIRC_REF)
            	{
            		stateOut = CLOCK_HIRC;
            		f = foutMcg;
            		error = ERRORS_NO_ERROR;
            	}
            	else if (foutMcg == CLOCK_FREQ_LIRC_FASTREF)
            	{
            		stateOut = CLOCK_LIRC8M;
            		f = foutMcg;
            		fcrdiv = 0;
            		error = ERRORS_NO_ERROR;
            	}
            	else if (foutMcg == CLOCK_FREQ_LIRC_FASTREF/2)
            	{
            		stateOut = CLOCK_LIRC8M;
            		f = foutMcg;
            		fcrdiv = 1;
            		error = ERRORS_NO_ERROR;
            	}
            	else if (foutMcg == CLOCK_FREQ_LIRC_SLOWREF)
            	{
            		stateOut = CLOCK_LIRC2M;
            		f = foutMcg;
            		fcrdiv = 0;
            		error = ERRORS_NO_ERROR;
            	}
            	else if (foutMcg == CLOCK_FREQ_LIRC_SLOWREF/2)
            	{
            		stateOut = CLOCK_LIRC2M;
            		f = foutMcg;
            		fcrdiv = 1;
            		error = ERRORS_NO_ERROR;
            	}
            	else if (foutMcg == CLOCK_FREQ_LIRC_SLOWREF/4)
            	{
            		stateOut = CLOCK_LIRC2M;
            		f = foutMcg;
            		fcrdiv = 2;
            		error = ERRORS_NO_ERROR;
            	}
            	else if (foutMcg == CLOCK_FREQ_LIRC_SLOWREF/8)
            	{
            		stateOut = CLOCK_LIRC2M;
            		f = foutMcg;
            		fcrdiv = 3;
            		error = ERRORS_NO_ERROR;
            	}
            	else if (foutMcg == CLOCK_FREQ_LIRC_SLOWREF/16)
            	{
            		stateOut = CLOCK_LIRC2M;
            		f = foutMcg;
            		fcrdiv = 4;
            		error = ERRORS_NO_ERROR;
            	}
            	else if (foutMcg == CLOCK_FREQ_LIRC_SLOWREF/32)
            	{
            		stateOut = CLOCK_LIRC2M;
            		f = foutMcg;
            		fcrdiv = 5;
            		error = ERRORS_NO_ERROR;
            	}
            	else if (foutMcg == CLOCK_FREQ_LIRC_SLOWREF/64)
            	{
            		stateOut = CLOCK_LIRC2M;
            		f = foutMcg;
            		fcrdiv = 6;
            		error = ERRORS_NO_ERROR;
            	}
            	else if (foutMcg == CLOCK_FREQ_LIRC_SLOWREF/128)
            	{
            		stateOut = CLOCK_LIRC2M;
            		f = foutMcg;
            		fcrdiv = 7;
            		error = ERRORS_NO_ERROR;
            	}
            	else
            	{
            		error = ERRORS_MCG_NO_FREQUENCY;
            		continue;
            	}
            }
            if (error == ERRORS_NO_ERROR)
            {
            	break;
            }
        }
        else
        {
        	error = ERRORS_MCG_OUT_OF_RANGE;
        }
    }

    if(error != ERRORS_NO_ERROR)
    {
        Clock_device.foutMcg = 0;
        Clock_device.mcgState = Clock_getCurrentState();
        Clock_device.mcgError = error;
        Clock_device.devInitialized = 0;
        return error;
     }

     /* select system_clock divider and set a secure bus_clock divider */
     tempReg = SIM_CLKDIV1_REG(SIM_BASE_PTR);
     tempReg &= ~((SIM_CLKDIV1_OUTDIV1_MASK) | (SIM_CLKDIV1_OUTDIV4_MASK));
     tempReg |= (SIM_CLKDIV1_OUTDIV1(i-1) | SIM_CLKDIV1_OUTDIV4(1));
     SIM_CLKDIV1_REG(SIM_BASE_PTR) = tempReg;

     /* state transition */
     foutMcg = Clock_StateTransition(fext, stateOut, fcrdiv, erefs0);
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
		 error = Clock_setDividers(busDivider, 0, 0);
		 Clock_device.mcgError = error;
    	 return error = ERRORS_NO_ERROR;
     }
}

#endif
