/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/include/hardware/PIC24fJ/clock_PIC24fJ.h
 * @author Leonardo Morichelli
 * @brief Clock useful definitions for PIC24fJ
 */

#ifndef __CLOCK_PIC24FJ_H
#define __CLOCK_PIC24FJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_PIC24FJ)

#include "clock.h"
#include "utility.h"
#include "system.h"

#include <xc.h>
    
//typedef struct _CLOCK_TypeDef
//{
//    OSCCONBITS* osccon;
//    CLKDIVBITS* clkdiv;
//    OSCTUNBITS* osctune;
//    OSCDIVBITS* oscdiv;
//    OSCFDIVBITS* oscfdiv;
//    DCOCONBITS* dcocon;
//    DCOTUNBITS* dcotun;
//} CLOCK_TypeDef;

//typedef struct _CLOCKEN_TypeDef
//{
//    PMD1BITS* pmd1;
//    PMD2BITS* pmd2;
//    PMD3BITS* pmd3;
//    PMD4BITS* pmd4;
//    PMD5BITS* pmd5;    
//    PMD6BITS* pmd6;
//    PMD7BITS* pmd7;
//    PMD8BITS* pmd8;
//} CLOCKEN_TypeDef;

typedef enum _Oscillator_Source
{
    OSCSCR_OSCFDIV     = 0b111, //Oscillator with Frequency Divider (OSCFDIV)
    OSCSCR_DCO         = 0b110, //Digitally Controlled Oscillator (DCO)
    OSCSCR_LPRC        = 0b101, //Low-Power RC Oscillator (LPRC)
    OSCSCR_SOSC        = 0b100, //Secondary Oscillator (SOSC)
    OSCSCR_XTPLL       = 0b011, //Primary Oscillator with PLL module (XTPLL, ECPLL)
    OSCSCR_XT          = 0b010, //Primary Oscillator (XT, HS, EC)
    OSCSCR_FRCPLL      = 0b001, //Fast RC Oscillator with PLL module (FRCPLL)
    OSCSCR_FRC         = 0b000, //Fast RC Oscillator (FRC)
} Oscillator_Source;
    
typedef struct _Clock_Device
{
    OSCILLATOR_TypeDef *regmap;
    REFO_TypeDef *regmapRefo;
    PMD_Typedef *regmapPmd;

    uint32_t systemCoreClock;
    uint32_t primaryOscillator;
    uint32_t secondaryOscillator;
    
    System_Errors rccError;
} Clock_Device;

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // __CLOCK_PIC24FJ_H
