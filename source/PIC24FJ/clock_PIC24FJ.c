/*
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/source/PIC24FJ/clock_PIC24FJ.c
 * @author Leonardo Morichelli
 * @brief Clock implementations for PIC24FJ
 */

#include "clock.h"

#if defined (LIBOHIBOARD_PIC24FJ)

#ifdef __cplusplus
extern "C" {
#endif

Clock_Device clockDevice =
{
    .regmap = OSCILLATOR,
    .regmapRefo = REFO,
    .regmapPmd = PMD,
    
    .systemCoreClock      = 32000000U,
    .secondaryOscillator  = 32768U,
    .primaryOscillator    = 0U,
    .rccError             = ERRORS_NO_ERROR,
};

static System_Errors Clock_deInit (void);
    
System_Errors Clock_init (Clock_Config* config)
{
    System_Errors error = ERRORS_NO_ERROR;
    if (config == NULL)
    {
        return ERRORS_CLOCK_NO_CONFIG;
    }
    
    //Reset the Oscillator to FRC source
    Clock_deInit();
    
    // Initialize SysTick with default clock value (8MHz)
    System_systickInit(0);
    
    uint8_t oscconh = 0, oscconl = 0;
    
    //Enable Secondary Oscillator
    if(config->source & CLOCK_EXTERNAL_SOSC)
    {
        oscconl |= _OSCCON_SOSCEN_MASK;
    }
    
    //Select the system oscillator source
    if(config->source & CLOCK_INTERNAL_OSCFDIV)
    {
        oscconh = OSCSCR_OSCFDIV;
    }
    else if(config->source & CLOCK_INTERNAL_DCO)
    {
        oscconh = OSCSCR_DCO;
    }
    else if(config->source & CLOCK_INTERNAL_LPRC)
    {
        oscconh = OSCSCR_LPRC;
    }
    else if(config->source & CLOCK_EXTERNAL_XTPLL)
    {
        oscconh = OSCSCR_XTPLL;
    }
    else if(config->source & CLOCK_EXTERNAL_XT)
    {
        oscconh = OSCSCR_XT;
    }
    else if(config->source & CLOCK_INTERNAL_FRCPLL)
    {
        oscconh = OSCSCR_FRCPLL;
    }
    else if(config->source & CLOCK_INTERNAL_FRC)
    {
        oscconh = OSCSCR_FRC;
    }
    else if(config->source & CLOCK_EXTERNAL_SOSC)
    {
        oscconh = OSCSCR_SOSC;
    }
    
    if((config->source & CLOCK_EXTERNAL_XTPLL) || (config->source & CLOCK_INTERNAL_FRCPLL))
    {
        // in case of PLL enable clock switching
        oscconl |= _OSCCON_OSWEN_MASK;
    }
    __builtin_write_OSCCONH(oscconh);
    __builtin_write_OSCCONL(oscconl);
    if(config->source & CLOCK_EXTERNAL_XTPLL || config->source & CLOCK_INTERNAL_FRCPLL)
    {
        // Wait for Clock switch to occur
        //while (UTILITY_READ_REGISTER_BIT(clockDevice.regmap->OSCCON, _OSCCON_OSWEN_MASK) != 0);
        //while (UTILITY_READ_REGISTER_BIT(clockDevice.regmap->OSCCON, _OSCCON_LOCK_MASK) == 0);
        while (OSCCONbits.OSWEN != 0);
        while (OSCCONbits.LOCK != 1);
    }
    
    Nop();
    Nop();
    Nop();
    Nop();
    
    // Initialize SysTick with correct clock value
    System_systickInit(0);
    
    return error;
}

static System_Errors Clock_deInit (void)
{
    System_Errors error = ERRORS_NO_ERROR;
    
    // CPDIV 1:1; PLLEN disabled; DOZE 1:8; RCDIV FRC; DOZEN disabled; ROI disabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmap->CLKDIV, 0x3000);
    // STOR disabled; STORPOL Interrupt when STOR is 1; STSIDL disabled; STLPOL Interrupt when STLOCK is 1; STLOCK disabled; STSRC SOSC; STEN disabled; TUN Center frequency; 
    UTILITY_WRITE_REGISTER(clockDevice.regmap->OSCTUN, 0x0000);
    // ROEN disabled; ROSWEN disabled; ROSEL FOSC; ROOUT disabled; ROSIDL disabled; ROSLP disabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmapRefo->REFOCONL, 0x0000);
    // RODIV 0; 
    UTILITY_WRITE_REGISTER(clockDevice.regmapRefo->REFOCONH, 0x0000);
    // DCOTUN 0; 
    UTILITY_WRITE_REGISTER(clockDevice.regmap->DCOTUN, 0x0000);
    // DCOFSEL 8; DCOEN disabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmap->DCOCON, 0x0700);
    // DIV 0; 
    UTILITY_WRITE_REGISTER(clockDevice.regmap->OSCDIV, 0x0000);
    // TRIM 0; 
    UTILITY_WRITE_REGISTER(clockDevice.regmap->OSCFDIV, 0x0000);
    // AD1MD enabled; T3MD enabled; T4MD enabled; T1MD enabled; U2MD enabled; T2MD enabled; U1MD enabled; SPI2MD enabled; SPI1MD enabled; T5MD enabled; I2C1MD enabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmapPmd->PMD1, 0x0000);
    // OC5MD enabled; OC6MD enabled; OC7MD enabled; OC8MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; OC4MD enabled; IC6MD enabled; IC7MD enabled; IC5MD enabled; IC8MD enabled; IC4MD enabled; IC3MD enabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmapPmd->PMD2, 0x0000);
    // I2C3MD enabled; PMPMD enabled; U3MD enabled; RTCCMD enabled; CMPMD enabled; CRCMD enabled; I2C2MD enabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmapPmd->PMD3, 0x0000);
    // U4MD enabled; USB1MD enabled; CTMUMD enabled; REFOMD enabled; LVDMD enabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmapPmd->PMD4, 0x0000);
    // IC9MD enabled; OC9MD enabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmapPmd->PMD5, 0x0000);
    // SPI3MD enabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmapPmd->PMD6, 0x0000);
    // DMA1MD enabled; DMA0MD enabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmapPmd->PMD7, 0x0000);
    // U5MD enabled; CLC3MD enabled; CLC4MD enabled; CLC1MD enabled; CLC2MD enabled; U6MD enabled; 
    UTILITY_WRITE_REGISTER(clockDevice.regmapPmd->PMD8, 0x0000);
    // CF no clock failure; NOSC FRC; SOSCEN enabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete; IOLOCK not-active; 
    __builtin_write_OSCCONH((uint8_t) (0x00));
    __builtin_write_OSCCONL((uint8_t) (0x02));
    
    Nop();
    Nop();
    Nop();
    Nop();
    
    return error;
}

uint32_t Clock_getOutputValue (Clock_Output output)
{
    uint32_t outputValue = 0;
    switch(output)
    {
        case CLOCK_OUTPUT_SYSCLK:
            outputValue = Clock_getOscillatorValue();
            break;
            
        case CLOCK_OUTPUT_PERIPHERAL:
            outputValue = Clock_getOscillatorValue() / 2;
            break;
    }
    return outputValue;
}

static uint32_t Clock_getDcoOscillatorValue(void)
{
    uint32_t frequency = 0ul;
    uint8_t dcofsel = DCOCONbits.DCOFSEL;
    int8_t tune = DCOTUNbits.DCOTUN;

    switch (dcofsel) {
        case 0b0000:
            frequency = 1000000ul;
            break;

        case 0b0001:
            frequency = 2000000ul;
            break;

        case 0b0010:
            frequency = 3000000ul;
            break;

        case 0b0011:
            frequency = 4000000ul;
            break;

        case 0b0100:
            frequency = 5000000ul;
            break;

        case 0b0101:
            frequency = 6000000ul;
            break;

        case 0b0110:
            frequency = 7000000ul;
            break;

        default:
        case 0b0111:
            frequency = 8000000ul;
            break;

        case 0b1110:
            frequency = 15000000ul; //16000000ul;
            break;

        case 0b1111:
            switch (tune) {
                default:
                case 10:
                    frequency = 30000000ul;
                    break;

                case 31:
                    frequency = 32000000ul;
                    break;
            }
            break;
    }

    return frequency;
}

uint32_t Clock_getOscillatorValue (void)
{
    uint32_t clockValue = 0;
    uint32_t cosc = _COSC;
    switch(cosc) {
        case OSCSCR_OSCFDIV:
            clockValue = 8000000;
            break;
            
        case OSCSCR_DCO:
            clockValue = Clock_getDcoOscillatorValue();
            break;
            
        case OSCSCR_LPRC:
            clockValue = 31000;
            break;
            
        case OSCSCR_SOSC:
            clockValue = clockDevice.secondaryOscillator;
            break;
            
        case OSCSCR_XTPLL:
            clockValue = 8000000;
            break;
            
        case OSCSCR_XT:
            clockValue = clockDevice.primaryOscillator;
            break;
            
        case OSCSCR_FRCPLL:
            clockValue = 32000000;
            break;
            
        case OSCSCR_FRC:
            clockValue = 8000000;
            break;
            
    }    
    return clockValue;
}

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_PIC24FJ
