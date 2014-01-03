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

/**
 * This function checks if the program has been downloaded to the right device.
 */
System_Errors System_controlDevice (void)
{
    /* TODO: implement... */

    return ERRORS_NO_ERROR;
}

/**
 * For MKL15Z4:
 *   - This function set FEE mode with MCG out to 47972352Hz.
 *   - Core clock was set to 47972352Hz and bus clock to 23986176Hz.
 * For FRDMKL25Z:
 *   - This function set FEE mode with MCG out to 40MHz.
 *   - Core clock was set to 40MHz and bus clock to 20MHz.
 */
System_Errors System_initClock (void)
{
#if defined(MKL15Z4)
    int i;

    /* Clock prescaler */
    SIM_CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(0x00) | SIM_CLKDIV1_OUTDIV4(0x01));
    
    /* Set low frequency range and select external clock. */
    MCG_C2 = (MCG_C2_RANGE0(0x00) | MCG_C2_EREFS0_MASK);
    /* Enable external reference clock and active 12pF internal capacitor. */
    OSC0_CR = (OSC_CR_ERCLKEN_MASK | OSC_CR_SC4P_MASK | OSC_CR_SC8P_MASK);
    /* Output of FLL is select, divide by 1 external clock and disable internal */
    MCG_C1 = (MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x00) | MCG_C1_IRCLKEN_MASK);
    /* DCO mid range and DMX32 enabled (x1464): output of FLL 47972352Hz */
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

#elif defined(MK60DZ10)

    int i;
    
    /* SIM_CLKDIV1: Clock prescaler */
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                  SIM_CLKDIV1_OUTDIV2(0x01) |
                  SIM_CLKDIV1_OUTDIV3(0x01) |
                  SIM_CLKDIV1_OUTDIV4(0x03);

    /* SIM_CLKDIV2: USB clock prescaler */
    SIM_CLKDIV2 &= (uint32_t)~(uint32_t)(
                    SIM_CLKDIV2_USBDIV(0x07) |
                    SIM_CLKDIV2_USBFRAC_MASK);

    /* Select FLL as a clock source for various peripherals */
    SIM_SOPT2 &= (uint32_t)~(uint32_t)(SIM_SOPT2_PLLFLLSEL_MASK);
    /* System oscillator drives 32 kHz clock for various peripherals */
    SIM_SOPT1 &= (uint32_t)~(uint32_t)(SIM_SOPT1_OSC32KSEL_MASK);

    /* Enable external oscillator clock pins */
    PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
    PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));

    /* Switch to FBE Mode */
    /* Set very high frequency range and select external clock. */
    MCG_C2 = (MCG_C2_RANGE(0x02) | MCG_C2_EREFS_MASK);
    /* Enable external reference clock */
    OSC_CR = OSC_CR_ERCLKEN_MASK;
    /* External clock is select, divide by 512 (31250Hz) and disable internal */
    MCG_C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x04) | MCG_C1_IRCLKEN_MASK);
    /* DCO low range and DMX32 disable (x640): output of FLL 20MHz */
    MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
    /* Disable PLL and selects the amount to divide down the external 
     * reference clock for the PLL. The resulting frequency must
     * be in the range of 2 MHz to 4 MHz: 16MHz/4 = 4MHz */
    MCG_C5 = MCG_C5_PRDIV(0x03);
    /* Disable PLL and select multiply factor by 25 (100MHz) */
    MCG_C6 = MCG_C6_VDIV(0x01);

    /* Check that the oscillator is running */
    while((MCG_S & MCG_S_OSCINIT_MASK) == 0x00U);
    
    /* Check that the source of the FLL reference clock is the external reference clock. */
    for (i = 0; i < 2000; ++i)
    {
        if ((MCG_S & MCG_S_IREFST_MASK) == 0x00U) break;
    }
    if (MCG_S & MCG_S_IREFST_MASK)
        return ERRORS_EXT_OSC_NOT_SELECT;

    /* Wait until external reference clock is selected as MCG output */
    while((MCG_S & 0x0CU) != 0x08U);

    /* Switch to PBE Mode */
//    /* Select PLL as a clock source for various peripherals */
//    SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
    /* Enable PLL */
    MCG_C6 = (MCG_C6_PLLS_MASK | MCG_C6_VDIV(0x01));                                   

    /* Wait until external reference clock is selected as MCG output */
    while((MCG_S & 0x0CU) != 0x08U);

    /* Wait until locked */
    while((MCG_S & MCG_S_LOCK_MASK) == 0x00U);

    /* Switch to PEE Mode */
    /* Enable PLL output for MCGOUTCLK */
    MCG_C1 = (MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x04) | MCG_C1_IRCLKEN_MASK);                                   

    /* Wait until output of the PLL is selected */
    while((MCG_S & 0x0CU) != 0x0CU);
    
#elif defined(FRDMKL05Z)
#elif defined(FRDMKL25Z)
    int i;
    
    /* Clock prescaler */
    SIM_CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(0x00) | SIM_CLKDIV1_OUTDIV4(0x01));
    
    /* Set high frequency range and select external clock. */
    MCG_C2 = (MCG_C2_RANGE0(0x02) | MCG_C2_EREFS0_MASK);
    /* Enable external reference clock. */
    OSC0_CR = OSC_CR_ERCLKEN_MASK;
    /* Output of FLL is select, divide by 256 external clock and disable internal */
    MCG_C1 = (MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);
    /* DCO mid range and DMX32 disabled (x1280): output of FLL 40MHz */
    MCG_C4 = (uint8_t)((MCG_C4 & (uint8_t)~(uint8_t)(
              MCG_C4_DMX32_MASK |
              MCG_C4_DRST_DRS(0x02)
             )) | (uint8_t)(
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
    
#elif defined(FRDMK20D50M)
#endif
    
    return ERRORS_NO_ERROR;
}
