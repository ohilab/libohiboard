/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  
 * Project: libohiboard
 * Package: System
 * Version: 0.0
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>
 ******************************************************************************/

/**
 * @file libohiboard/source/system.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief 
 */

#include "system.h"

#if defined(MK60DZ10)
#elif defined(MKL15Z4)
/**
 * This function checks if the program has been downloaded to the right device.
 */
System_Errors System_controlDevice (void)
{
    /* TODO: implement... */

    return ERRORS_NO_ERROR;
}

/**
 * This function set FEE mode with MCG out to 47972352Hz.
 * Core clock was set to 47972352Hz and bus clock to 23986176Hz.
 */
System_Errors System_initClock (void)
{
    int i;

    /* Clock prescaler */
    SIM_CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(0x00) | SIM_CLKDIV1_OUTDIV4(0x01));
    
    /* Set low frequency range and select external clock. */
    MCG_C2 = (MCG_C2_RANGE0(0x00) | MCG_C2_EREFS0_MASK);
    /* Enable external reference clock and active 12pF internal capacitor. */
    OSC0_CR = (OSC_CR_ERCLKEN_MASK | OSC_CR_SC4P_MASK | OSC_CR_SC8P_MASK);
    /* Output of FLL is select, divide by 1 external clock and disable internal */
    MCG_C1 = (MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x00) | MCG_C1_IRCLKEN_MASK);
    /* DCO mid range and DMX32 disabled (x1280): output of FLL 47972352Hz */
    MCG_C4 = (uint8_t)((MCG_C4 & (uint8_t)~(uint8_t)(
            MCG_C4_DRST_DRS(0x02)
        )) | (uint8_t)(
            MCG_C4_DMX32_MASK |
            MCG_C4_DRST_DRS(0x01)
        ));
    /* Disable PLL */
    MCG_C5 = MCG_C5_PRDIV0(0x00);
    /* Clock monitor disabled */
    MCG_C6 = MCG_C6_VDIV0(0x00);
    
    /* Wait for reference clock status bit to clear (select external) */
    for (i = 0; i < 2000; ++i)
    {
        if ((MCG_S & MCG_S_IREFST_MASK) == 0x00U) break;
    }
    if (MCG_S & MCG_S_IREFST_MASK)
        return ERRORS_EXT_OSC_NOT_SELECT;
    
    while ((MCG_S & 0x0CU) != 0x00U);
    
    return ERRORS_NO_ERROR;
}
#endif
