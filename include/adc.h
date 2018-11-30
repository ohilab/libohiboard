/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2018 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Francesco Piunti <francesco.piunti89@gmail.com>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
 *  Matteo Civale <m.civale@gmail.com>
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
 * @file libohiboard/include/adc.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Francesco Piunti <francesco.piunti89@gmail.com>
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @author Matteo Civale    <matteo.civale@gmail.com>
 * @brief ADC definitions and prototypes.
 */

#ifdef LIBOHIBOARD_ADC

#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "utility.h"

/**
 * Device handle for ADC peripheral.
 */
typedef struct _Adc_Device* Adc_DeviceHandle;

#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/adc_STM32L4.h"

#else

typedef enum {
#if defined (LIBOHIBOARD_FRDMKL02Z) || \
    defined (LIBOHIBOARD_KL02Z4)

    ADC_PINS_INTERNAL,

#elif defined (LIBOHIBOARD_FRDMKL03Z) || \
      defined (LIBOHIBOARD_KL03Z4)

    ADC_PINS_PTA0,
    ADC_PINS_PTA8,
    ADC_PINS_PTA9,
    ADC_PINS_PTA12,

    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB5,

    ADC_PINS_INTERNAL,

#elif defined(LIBOHIBOARD_KL15Z4)

    ADC_PINS_PTE16,
    ADC_PINS_PTE17,
    ADC_PINS_PTE18,
    ADC_PINS_PTE19,
    ADC_PINS_PTE20,
    ADC_PINS_PTE21,
    ADC_PINS_PTE22,
    ADC_PINS_PTE23,
    ADC_PINS_PTE29,
    ADC_PINS_PTE30,
    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,
    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,
    
    ADC_PINS_INTERNAL,

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    ADC_PINS_PTE20,
    ADC_PINS_PTE21,
    ADC_PINS_PTE22,
    ADC_PINS_PTE23,
    ADC_PINS_PTE29,
    ADC_PINS_PTE30,
    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,
    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,
    
    ADC_PINS_INTERNAL,

#elif defined(LIBOHIBOARD_K12D5)

    ADC_PINS_DP0,
    ADC_PINS_DP1,
    ADC_PINS_DP3,

    ADC_PINS_DM0,
    ADC_PINS_DM1,
    ADC_PINS_DM3,

    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,

    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,

    ADC_PINS_PTD1,
    ADC_PINS_PTD4,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,
    ADC_PINS_PTD7,

    ADC_PINS_PTE0,
    ADC_PINS_PTE1,
    ADC_PINS_PTE16,
    ADC_PINS_PTE17,
    ADC_PINS_PTE18,
    ADC_PINS_PTE19,

    ADC_PINS_SE23,

    ADC_PINS_INTERNAL,
    
#elif defined(LIBOHIBOARD_K10DZ10)

    ADC_PINS_INTERNAL,

#elif defined(LIBOHIBOARD_K10D10)
    
    ADC_PINS_ADC0_DP0,
    ADC_PINS_ADC0_DP1,
    ADC_PINS_PGA0_DP,
    ADC_PINS_ADC0_DP3,
    ADC_PINS_PTE16,
    ADC_PINS_PTE17,
    ADC_PINS_PTE18,
    ADC_PINS_PTE19,
    ADC_PINS_PTC2,
    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,
    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTA7,
    ADC_PINS_PTA8,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTE24,
    ADC_PINS_PTE25,
    ADC_PINS_ADC0_DM0,
    ADC_PINS_ADC0_DM1,
    ADC_PINS_ADC0_SE21,
    ADC_PINS_ADC0_SE22,
    ADC_PINS_ADC0_SE23,
    
    ADC_PINS_ADC1_DP0,
    ADC_PINS_ADC1_DP1,
    ADC_PINS_PGA_DP,
    ADC_PINS_ADC1_DP3,
    ADC_PINS_PTE0,
    ADC_PINS_PTE1,
    ADC_PINS_PTE2,
    ADC_PINS_PTE3,
    ADC_PINS_PTC7,
    ADC_PINS_PTC8,
    ADC_PINS_PTC9,
    ADC_PINS_PTC10,
    
    ADC_PINS_PTB4,
    ADC_PINS_PTB5,
    ADC_PINS_PTB6,
    ADC_PINS_PTB7,
    ADC_PINS_PTB10,
    ADC_PINS_PTB11,
    ADC_PINS_ADC1_SE16,
    ADC_PINS_PTA17,
    ADC_PINS_ADC1_SE18,
    ADC_PINS_ADC1_DM0,
    ADC_PINS_ADC1_DM1,
    ADC_PINS_ADC1_SE23,
    
    ADC_PINS_INTERNAL,
    
#elif defined(LIBOHIBOARD_K60DZ10)

    ADC_PINS_INTERNAL,
    
#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    ADC_PINS_PTE0,
    ADC_PINS_PTE1,
    ADC_PINS_PTE2,
    ADC_PINS_PTE3,
    ADC_PINS_ADC0_DP1,
    ADC_PINS_ADC0_DM1,
    ADC_PINS_ADC1_DP1,
    ADC_PINS_ADC1_DM1,
    ADC_PINS_ADC0_DP0,
    ADC_PINS_ADC1_DP3,
    ADC_PINS_ADC0_DM0,
    ADC_PINS_ADC1_DM3,
    ADC_PINS_ADC1_DP0,
    ADC_PINS_ADC0_DP3,
    ADC_PINS_ADC1_DM0,
    ADC_PINS_ADC0_DM3,
    ADC_PINS_ADC1_SE16,
    ADC_PINS_ADC0_SE22,
    ADC_PINS_ADC0_SE16,
    ADC_PINS_ADC0_SE21,
    ADC_PINS_ADC1_SE18,
    ADC_PINS_ADC0_SE23,
    ADC_PINS_ADC1_SE23,
    ADC_PINS_PTE24,
    ADC_PINS_PTE25,
    ADC_PINS_PTA7,
    ADC_PINS_PTA8,
    ADC_PINS_PTA17,
    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTB4,
    ADC_PINS_PTB5,
    ADC_PINS_PTB6,
    ADC_PINS_PTB7,
    ADC_PINS_PTB10,
    ADC_PINS_PTB11,
    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,
    ADC_PINS_PTC8,
    ADC_PINS_PTC9,
    ADC_PINS_PTC10,
    ADC_PINS_PTC11,
    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,

    ADC_PINS_INTERNAL,

#elif defined (LIBOHIBOARD_KV31F12)

    ADC_PINS_PTE0,
    ADC_PINS_PTE1,
    ADC_PINS_PTE2,
    ADC_PINS_PTE3,
    ADC_PINS_PTE16,
    ADC_PINS_PTE17,
    ADC_PINS_PTE18,
    ADC_PINS_PTE19,
    ADC_PINS_PTE24,
    ADC_PINS_PTE25,

    ADC_PINS_ADC0_DP1,
    ADC_PINS_ADC0_DM1,
    ADC_PINS_ADC1_DP1,
    ADC_PINS_ADC1_DM1,
    ADC_PINS_ADC0_DP0,
    ADC_PINS_ADC1_DP3,
    ADC_PINS_ADC0_DM0,
    ADC_PINS_ADC1_DM3,
    ADC_PINS_ADC1_DP0,
    ADC_PINS_ADC0_DP3,
    ADC_PINS_ADC1_DM0,
    ADC_PINS_ADC0_DM3,

    ADC_PINS_ADC1_SE18,
    ADC_PINS_ADC0_SE23,
    ADC_PINS_ADC1_SE23,

    ADC_PINS_PTA17,

    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTB4,
    ADC_PINS_PTB10,
    ADC_PINS_PTB11,

    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,
    ADC_PINS_PTC8,
    ADC_PINS_PTC9,
    ADC_PINS_PTC10,
    ADC_PINS_PTC11,

    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,

    ADC_PINS_INTERNAL,

#elif defined (LIBOHIBOARD_KV46F) || \
      defined (LIBOHIBOARD_TRWKV46F)

    ADC_PINS_PTE0,
    ADC_PINS_PTE1,
    ADC_PINS_PTE2,
    ADC_PINS_PTE3,
    ADC_PINS_PTE16,
    ADC_PINS_PTE17,
    ADC_PINS_PTE18,
    ADC_PINS_PTE19,
    ADC_PINS_PTE20,
    ADC_PINS_PTE21,
    ADC_PINS_PTE29,
    ADC_PINS_PTE30,
    ADC_PINS_PTE24,
    ADC_PINS_PTE25,

    ADC_PINS_PTA17,

    ADC_PINS_PTB0,
    ADC_PINS_PTB1,
    ADC_PINS_PTB2,
    ADC_PINS_PTB3,
    ADC_PINS_PTB10,
    ADC_PINS_PTB11,

    ADC_PINS_PTC0,
    ADC_PINS_PTC1,
    ADC_PINS_PTC2,
    ADC_PINS_PTC10,
    ADC_PINS_PTC11,

    ADC_PINS_PTD1,
    ADC_PINS_PTD5,
    ADC_PINS_PTD6,


    ADC_PINS_ACH6a,
    ADC_PINS_ACH7a,
    ADC_PINS_ACH2,
    ADC_PINS_ACH3,
    ADC_PINS_ACH6c,
    ADC_PINS_ACH7c,
    ADC_PINS_ACH6d,


#endif

    ADC_PINS_NONE,
} Adc_Pins;

typedef enum {
#if defined (LIBOHIBOARD_FRDMKL03Z) || \
    defined (LIBOHIBOARD_KL03Z4)

    ADC_CHL_A = 0x00,

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)  || \
      defined (LIBOHIBOARD_KL15Z4)

    ADC_CHL_A = 0x00,
    ADC_CHL_B = 0x01,

#elif defined (LIBOHIBOARD_K10DZ10)    || \
      defined (LIBOHIBOARD_K10D10)     || \
      defined (LIBOHIBOARD_K12D5)      || \
      defined (LIBOHIBOARD_K60DZ10)    || \
      defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)   || \
      defined (LIBOHIBOARD_KV31F12)

    ADC_CHL_A = 0x00,
    ADC_CHL_B = 0x01,

#elif defined (LIBOHIBOARD_FRDMKL02Z) || \
      defined (LIBOHIBOARD_KL02Z4)    || \
      defined (LIBOHIBOARD_FRDMKL03Z) || \
      defined (LIBOHIBOARD_KL03Z4)

    ADC_CHL_A = 0x00,

#elif defined (LIBOHIBOARD_KV46F) || \
      defined (LIBOHIBOARD_TWRKV46F)

    ADC_CHL_A        = 0x0,
    ADC_CHL_B        = 0x1,
    ADC_CHL_C        = 0x2,
    ADC_CHL_D        = 0x3,
    ADC_CHL_E        = 0x4,
    ADC_CHL_F        = 0x5,
    ADC_CHL_G        = 0x6,
    ADC_CHL_RESERVED = 0x7,

#endif
} Adc_ChannelMux;

#define ADC_MAX_CHANNEL_NUMBER 2

typedef enum {

#if defined (LIBOHIBOARD_FRDMKL02Z) || \
    defined (LIBOHIBOARD_KL02Z4)

    ADC_CH_TEMP          = 0x1A,
    ADC_CH_BANDGAP       = 0x1B,
    ADC_CH_VREFH         = 0x1D,
    ADC_CH_VREFL         = 0x1E,
    ADC_CH_DISABLE       = 0x1F,

#elif defined (LIBOHIBOARD_FRDMKL03Z) || \
      defined (LIBOHIBOARD_KL03Z4)

    ADC_CH_SE0     = 0x00,
    ADC_CH_SE1     = 0x01,
    ADC_CH_SE2     = 0x02,
    ADC_CH_SE3     = 0x03,
    ADC_CH_SE8     = 0x08,
    ADC_CH_SE9     = 0x09,
    ADC_CH_SE15    = 0x0F,

    ADC_CH_TEMP    = 0x1A,
    ADC_CH_BANDGAP = 0x1B,
    ADC_CH_VREFH   = 0x1D,
    ADC_CH_VREFL   = 0x1E,
    ADC_CH_DISABLE = 0x1F,

#elif defined(LIBOHIBOARD_KL15Z4)

    ADC_CH_SE0     = 0x00,
    ADC_CH_SE1     = 0x01,
    ADC_CH_SE2     = 0x02,
    ADC_CH_SE3     = 0x03,
    ADC_CH_SE4a    = 0x04,
    ADC_CH_SE4b    = 0x24,
    ADC_CH_SE5a    = 0x05,
    ADC_CH_SE5b    = 0x25,
    ADC_CH_SE6a    = 0x06,
    ADC_CH_SE6b    = 0x26,
    ADC_CH_SE7a    = 0x07,
    ADC_CH_SE7b    = 0x27,
    ADC_CH_SE8     = 0x08,
    ADC_CH_SE9     = 0x09,
    ADC_CH_SE11    = 0x0B,
    ADC_CH_SE12    = 0x0C,
    ADC_CH_SE13    = 0x0D,
    ADC_CH_SE14    = 0x0E,
    ADC_CH_SE15    = 0x0F,
    ADC_CH_SE23    = 0x17,

    ADC_CH_TEMP    = 0x1A,
    ADC_CH_BANDGAP = 0x1B,
    ADC_CH_VREFH   = 0x1D,
    ADC_CH_VREFL   = 0x1E,
    ADC_CH_DISABLE = 0x1F

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    ADC_CH_SE0     = 0x00,
    ADC_CH_SE3     = 0x03,
    ADC_CH_SE4a    = 0x04,
    ADC_CH_SE4b    = 0x24,
    ADC_CH_SE5b    = 0x25,
    ADC_CH_SE6b    = 0x26,
    ADC_CH_SE7a    = 0x07,
    ADC_CH_SE7b    = 0x27,
    ADC_CH_SE8     = 0x08,
    ADC_CH_SE9     = 0x09,
    ADC_CH_SE11    = 0x0B,
    ADC_CH_SE12    = 0x0C,
    ADC_CH_SE13    = 0x0D,
    ADC_CH_SE14    = 0x0E,
    ADC_CH_SE15    = 0x0F,
    ADC_CH_SE23    = 0x17,

    ADC_CH_TEMP    = 0x1A,
    ADC_CH_BANDGAP = 0x1B,
    ADC_CH_VREFH   = 0x1D,
    ADC_CH_VREFL   = 0x1E,
    ADC_CH_DISABLE = 0x1F

#elif defined (LIBOHIBOARD_K10DZ10)

    ADC_CH_ADC0_DP0      = 0x00,
    ADC_CH_ADC0_PGA0_DP  = 0x02,
    ADC_CH_ADC0_DP3      = 0x03,
    ADC_CH_ADC0_SE4      = 0x04,
    ADC_CH_ADC0_SE5      = 0x05,
    ADC_CH_ADC0_SE6      = 0x06,
    ADC_CH_ADC0_SE7      = 0x07,
    ADC_CH_ADC0_SE8      = 0x08,
    ADC_CH_ADC0_SE9      = 0x09,
    ADC_CH_ADC0_SE12     = 0x0C,
    ADC_CH_ADC0_SE13     = 0x0D,
    ADC_CH_ADC0_SE14     = 0x0E,
    ADC_CH_ADC0_SE15     = 0x0F,
    ADC_CH_ADC0_DM0      = 0x13,
    ADC_CH_ADC0_SE23     = 0x17,

    ADC_CH_ADC1_DP0      = 0x00,
    ADC_CH_ADC1_PGA1_DP  = 0x02,
    ADC_CH_ADC1_DP3      = 0x03,
    ADC_CH_ADC1_SE4      = 0x04,
    ADC_CH_ADC1_SE5      = 0x05,
    ADC_CH_ADC1_SE6      = 0x06,
    ADC_CH_ADC1_SE7      = 0x07,
    ADC_CH_ADC1_SE8      = 0x08,
    ADC_CH_ADC1_SE9      = 0x09,
    ADC_CH_ADC1_SE14     = 0x0E,
    ADC_CH_ADC1_SE15     = 0x0F,
    ADC_CH_ADC1_SE17     = 0x11,
    ADC_CH_ADC1_VREF_OUT = 0x12,
    ADC_CH_ADC1_DM0      = 0x13,

    ADC_CH_TEMP          = 0x1A,
    ADC_CH_BANDGAP       = 0x1B,
    ADC_CH_VREFH         = 0x1D,
    ADC_CH_VREFL         = 0x1E,
    ADC_CH_DISABLE       = 0x1F

#elif defined (LIBOHIBOARD_K10D10)

    ADC_CH_DP0          = 0x00,
    ADC_CH_DP1          = 0x01,
    ADC_CH_PGA_DP       = 0x02,
    ADC_CH_DP3          = 0x03,
    ADC_CH_SE4a         = 0x04,
    ADC_CH_SE5a         = 0x05,
    ADC_CH_SE6a         = 0x06,
    ADC_CH_SE7a         = 0x07,
    ADC_CH_SE4b         = 0x24,
    ADC_CH_SE5b         = 0x25,
    ADC_CH_SE6b         = 0x26,
    ADC_CH_SE7b         = 0x27,
    ADC_CH_SE8          = 0x08,
    ADC_CH_SE9          = 0x09,
    ADC_CH_SE10         = 0x0A,
    ADC_CH_SE11         = 0x0B,
    ADC_CH_SE12         = 0x0C,
    ADC_CH_SE13         = 0x0D,
    ADC_CH_SE14         = 0x0E,
    ADC_CH_SE15         = 0x0F,
    ADC_CH_SE16         = 0x10,
    ADC_CH_SE17         = 0x11,
    ADC_CH_SE18         = 0x12,
    ADC_CH_DM0          = 0x13,
    ADC_CH_DM1          = 0x14,
    ADC_CH_SE21         = 0x15,
    ADC_CH_SE22         = 0x16,
    ADC_CH_SE23         = 0x17,

    ADC_CH_TEMP          = 0x1A,
    ADC_CH_BANDGAP       = 0x1B,
    ADC_CH_VREFH         = 0x1D,
    ADC_CH_VREFL         = 0x1E,
    ADC_CH_DISABLE       = 0x1F,

#elif defined (LIBOHIBOARD_K12D5)

    ADC_CH_DP0      = 0x00,
    ADC_CH_DP1      = 0x01,
    ADC_CH_DP3      = 0x03,
    ADC_CH_SE4a     = 0x04,
    ADC_CH_SE4b     = 0x24,
    ADC_CH_SE5a     = 0x05,
    ADC_CH_SE5b     = 0x25,
    ADC_CH_SE6a     = 0x06,
    ADC_CH_SE6b     = 0x26,
    ADC_CH_SE7a     = 0x07,
    ADC_CH_SE7b     = 0x27,
    ADC_CH_SE8      = 0x08,
    ADC_CH_SE9      = 0x09,
    ADC_CH_SE10     = 0x0A,
    ADC_CH_SE11     = 0x0B,
    ADC_CH_SE12     = 0x0C,
    ADC_CH_SE13     = 0x0D,
    ADC_CH_SE14     = 0x0E,
    ADC_CH_SE15     = 0x0F,
    ADC_CH_SE21     = 0x15,
    ADC_CH_SE22     = 0x16,
    ADC_CH_SE23     = 0x17,

    ADC_CH_TEMP     = 0x1A,
    ADC_CH_BANDGAP  = 0x1B,
    ADC_CH_VREFH    = 0x1D,
    ADC_CH_VREFL    = 0x1E,
    ADC_CH_DISABLE  = 0x1F

#elif defined(LIBOHIBOARD_K60DZ10)

    ADC_CH_TEMP    = 0x1A,
    ADC_CH_BANDGAP = 0x1B,
    ADC_CH_VREFH   = 0x1D,
    ADC_CH_VREFL   = 0x1E,
    ADC_CH_DISABLE = 0x1F

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    ADC_CH_DP0          = 0x00,
    ADC_CH_DP1          = 0x01,
    ADC_CH_DP2          = 0X02,
    ADC_CH_DM2,
    ADC_CH_DP3          = 0x03,
    ADC_CH_DM3          = 0x03,
    ADC_CH_SE4a         = 0x04,
    ADC_CH_SE5a         = 0x05,
    ADC_CH_SE6a         = 0x06,
    ADC_CH_SE7a         = 0x07,
    ADC_CH_SE4b         = 0x24,
    ADC_CH_SE5b         = 0x25,
    ADC_CH_SE6b         = 0x26,
    ADC_CH_SE7b         = 0x27,
    ADC_CH_SE8          = 0x08,
    ADC_CH_SE9          = 0x09,
    ADC_CH_SE10         = 0x0A,
    ADC_CH_SE11         = 0x0B,
    ADC_CH_SE12         = 0x0C,
    ADC_CH_SE13         = 0x0D,
    ADC_CH_SE14         = 0x0E,
    ADC_CH_SE15         = 0x0F,
    ADC_CH_SE16         = 0x10,
    ADC_CH_SE17         = 0x11,
    ADC_CH_SE18         = 0x12,
    ADC_CH_DM0          = 0x13,
    ADC_CH_DM1          = 0x14,
    ADC_CH_SE21         = 0x15,
    ADC_CH_SE22         = 0x16,
    ADC_CH_SE23         = 0x17,

    ADC_CH_TEMP          = 0x1A,
    ADC_CH_BANDGAP       = 0x1B,
    ADC_CH_VREFH         = 0x1D,
    ADC_CH_VREFL         = 0x1E,
    ADC_CH_DISABLE       = 0x1F,

#elif defined (LIBOHIBOARD_KV31F12)

    ADC_CH_DP0          = 0x00,
    ADC_CH_DP1          = 0x01,
    ADC_CH_DP2          = 0X02,
    ADC_CH_DP3          = 0x03,
    ADC_CH_SE4a         = 0x04,
    ADC_CH_SE5a         = 0x05,
    ADC_CH_SE6a         = 0x06,
    ADC_CH_SE7a         = 0x07,
    ADC_CH_SE4b         = 0x24,
    ADC_CH_SE5b         = 0x25,
    ADC_CH_SE6b         = 0x26,
    ADC_CH_SE7b         = 0x27,
    ADC_CH_SE8          = 0x08,
    ADC_CH_SE9          = 0x09,
    ADC_CH_SE12         = 0x0C,
    ADC_CH_SE13         = 0x0D,
    ADC_CH_SE14         = 0x0E,
    ADC_CH_SE15         = 0x0F,
    ADC_CH_SE17         = 0x11,
    ADC_CH_SE18         = 0x12,
    ADC_CH_SE23         = 0x17,

    ADC_CH_TEMP          = 0x1A,
    ADC_CH_BANDGAP       = 0x1B,
    ADC_CH_VREFH         = 0x1D,
    ADC_CH_VREFL         = 0x1E,
    ADC_CH_DISABLE       = 0x1F,

#endif
} Adc_ChannelNumber;

#endif

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Adc_DeviceState
{
    ADC_DEVICESTATE_RESET,
    ADC_DEVICESTATE_READY,
    ADC_DEVICESTATE_BUSY,
    ADC_DEVICESTATE_ERROR,

} Adc_DeviceState;

/**
 * The list of possible ADC resolution.
 */
typedef enum _Adc_Resolution
{
#if defined (LIBOHIBOARD_MKL)     || \
	defined (LIBOHIBOARD_MK)
	ADC_RESOLUTION_16BIT,
	ADC_RESOLUTION_12BIT,
	ADC_RESOLUTION_10BIT,
	ADC_RESOLUTION_8BIT,
#endif
#if defined (LIBOHIBOARD_STM32L4)
    ADC_RESOLUTION_12BIT = 0x00000000u,
    ADC_RESOLUTION_10BIT = ADC_CFGR_RES_0,
    ADC_RESOLUTION_8BIT  = ADC_CFGR_RES_1,
    ADC_RESOLUTION_6BIT  = ADC_CFGR_RES,
#endif
} Adc_Resolution;

typedef enum _Adc_InputType
{
    ADC_INPUTTYPE_SINGLE_ENDED = 0,
    ADC_INPUTTYPE_DIFFERENTIAL = 1,
} Adc_InputType;

#if defined (LIBOHIBOARD_NXP_KINETIS)
typedef enum {
    ADC_AVERAGE_1_SAMPLES,
    ADC_AVERAGE_4_SAMPLES,
    ADC_AVERAGE_8_SAMPLES,
    ADC_AVERAGE_16_SAMPLES,
    ADC_AVERAGE_32_SAMPLES,
} Adc_Average;

typedef enum {
	ADC_BUS_CLOCK,
	ADC_BUS_CLOCK_DIV2,
	ADC_ALTERNATE_CLOCK,
	ADC_ASYNCHRONOUS_CLOCK,
} Adc_ClockSource;

typedef enum {
    ADC_SHORT_SAMPLE,
    ADC_LONG_SAMPLE_2,
    ADC_LONG_SAMPLE_6,
    ADC_LONG_SAMPLE_12,
    ADC_LONG_SAMPLE_20,
} Adc_SampleLength;

typedef enum {
    ADC_NORMAL_CONVERTION,
    ADC_HIGH_SPEED_CONVERTION,
}Adc_ConvertionSpeed;

typedef enum {
    ADC_SINGLE_CONVERTION,
    ADC_CONTINUOUS_CONVERTION,
}Adc_ContinuousConvertion;

typedef enum {
    ADC_VREF,
    ADC_VALT,
}Adc_VoltReference;

#endif // LIBOHIBOARD_NXP_KINETIS

/**
 * List of all possible ADC clock source
 *
 * @note STM32: The ADC clock configuration is common to all ADC instances.
 */
typedef enum _Adc_ClockSource
{
#if defined (LIBOHIBOARD_NXP_KINETIS)

    ADC_BUS_CLOCK,
    ADC_BUS_CLOCK_DIV2,
    ADC_ALTERNATE_CLOCK,
    ADC_ASYNCHRONOUS_CLOCK,

#elif defined (LIBOHIBOARD_ST_STM32)

    ADC_CLOCKSOURCE_NONE        = 0x00000000U,
    ADC_CLOCKSOURCE_PLLADC1CLK  = RCC_CCIPR_ADCSEL_0,

#if defined(LIBOHIBOARD_STM32L471) || \
    defined(LIBOHIBOARD_STM32L475) || \
    defined(LIBOHIBOARD_STM32L476) || \
    defined(LIBOHIBOARD_STM32L485) || \
    defined(LIBOHIBOARD_STM32L486) || \
    defined(LIBOHIBOARD_STM32L496) || \
    defined(LIBOHIBOARD_STM32L4A6)
    ADC_CLOCKSOURCE_PLLADC2CLK  = RCC_CCIPR_ADCSEL_1,
#endif
    ADC_CLOCKSOURCE_SYSCLK      = RCC_CCIPR_ADCSEL,
#endif
} Adc_ClockSource;

typedef enum _Adc_Prescaler
{
#if defined (LIBOHIBOARD_ST_STM32)

    ADC_PRESCALER_SYNC_DIV1    = (ADC_CCR_CKMODE_0),
    ADC_PRESCALER_SYNC_DIV2    = (ADC_CCR_CKMODE_1),
    ADC_PRESCALER_SYNC_DIV4    = (ADC_CCR_CKMODE_1 | ADC_CCR_CKMODE_0),
    ADC_PRESCALER_ASYNC_DIV1   = 0x00000000u,
    ADC_PRESCALER_ASYNC_DIV2   = (ADC_CCR_PRESC_0),
    ADC_PRESCALER_ASYNC_DIV4   = (ADC_CCR_PRESC_1),
    ADC_PRESCALER_ASYNC_DIV6   = (ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0),
    ADC_PRESCALER_ASYNC_DIV8   = (ADC_CCR_PRESC_2),
    ADC_PRESCALER_ASYNC_DIV10  = (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_0),
    ADC_PRESCALER_ASYNC_DIV12  = (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1),
    ADC_PRESCALER_ASYNC_DIV16  = (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0),
    ADC_PRESCALER_ASYNC_DIV32  = (ADC_CCR_PRESC_3),
    ADC_PRESCALER_ASYNC_DIV64  = (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_0),
    ADC_PRESCALER_ASYNC_DIV128 = (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1),
    ADC_PRESCALER_ASYNC_DIV256 = (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0),

#endif
} Adc_Prescaler;

#if defined (LIBOHIBOARD_ST_STM32)

typedef enum _Adc_DataAlign
{
    ADC_DATAALIGN_RIGHT = 0x00000000u,
    ADC_DATAALIGN_LEFT  = ADC_CFGR_ALIGN,

} Adc_DataAlign;

typedef enum _Adc_EndOfConversion
{
    ADC_ENDOFCONVERSION_SINGLE   = ADC_ISR_EOC,
    ADC_ENDOFCONVERSION_SEQUENCE = ADC_ISR_EOS,

} Adc_EndOfConversion;

typedef enum _Adc_Trigger
{
    ADC_TRIGGER_EXT0_TIM1_CH1     = 0x00000000u,
    ADC_TRIGGER_EXT1_TIM1_CH2     = (ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT2_TIM1_CH3     = (ADC_CFGR_EXTSEL_1),
    ADC_TRIGGER_EXT3_TIM2_CH2     = (ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT4_TIM3_TRGO    = (ADC_CFGR_EXTSEL_2),
    ADC_TRIGGER_EXT5_TIM4_CH4     = (ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT6_EXTI_11      = (ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_1),
    ADC_TRIGGER_EXT7_TIM8_TRGO    = (ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT8_TIM8_TRGO2   = (ADC_CFGR_EXTSEL_3),
    ADC_TRIGGER_EXT9_TIM1_TRGO    = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT10_TIM1_TRGO2  = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_1),
    ADC_TRIGGER_EXT11_TIM2_TRGO   = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT12_TIM4_TRGO   = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_2),
    ADC_TRIGGER_EXT13_TIM6_TRGO   = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_0),
    ADC_TRIGGER_EXT14_TIM15_TRGO  = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_1),
    ADC_TRIGGER_EXT15_TIM3_CH4    = (ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0),

} Adc_Trigger;

typedef enum _Adc_TriggerPolarity
{
    ADC_TRIGGERPOLARITY_DISABLE = 0x00000000u,
    ADC_TRIGGERPOLARITY_RISING  = ADC_CFGR_EXTEN_0,
    ADC_TRIGGERPOLARITY_FALLING = ADC_CFGR_EXTEN_1,
    ADC_TRIGGERPOLARITY_BOTH    = ADC_CFGR_EXTEN,

} Adc_TriggerPolarity;

#endif

typedef struct _Adc_Config
{
    Adc_Resolution resolution;                           /**< ADC resolutions */

    Adc_ClockSource clockSource;               /**< The clock source selected */
    Adc_Prescaler prescaler;                       /**< input clock prescaler */

    /**
     * Specify whether the conversion is performed in single mode or
     * continuous mode.
     */
    Utility_State continuousConversion;

#if defined (LIBOHIBOARD_NXP_KINETIS)
    uint8_t                  clkDiv;
    Adc_SampleLength         sampleLength;
    Adc_ConvertionSpeed      covertionSpeed;


    Adc_Average              average;
    Adc_ContinuousConvertion contConv;
    Adc_VoltReference        voltRef;

    bool                     doCalibration;

    bool                     enableHwTrigger;
#endif

#if defined (LIBOHIBOARD_ST_STM32)

    Adc_DataAlign dataAlign;  /**< Data align position for conversion results */

    Adc_EndOfConversion eoc;                      /**< End-Of-Conversion type */

    /**
     * Specify whether the conversions is performed in complete sequence or
     * discontinuous sequence.
     * When both continuous and discontinuous are enabled, the ADC behaves
     * as if continuous mode was disabled.
     */
    Utility_State discontinuousConversion;
    /**
     * the number of regular channels to be converted in discontinuous mode,
     * after receiving an external trigger.
     * The number must be between 0 and 7, that is 1 channel to 8 channels.
     */
    uint8_t discontinuousConversionNumber;

    /**
     * Select the behavior in case of overrun: data overwritten (ENABLE) or preserved (DISABLE)
     * that is the default behavior.
     */
    Utility_State overrun;

    /**
     * Configure the sequencer of ADC groups regular conversion.
     */
    Utility_State sequence;
    /**
     * Specify the number of channel that will be converted within the group sequencer.
     * To use the sequencer and convert several channels, parameter 'sequence' must be enabled.
     * The number must be between 0x00 and 0x0F, that is 1 channel to 16 channels.
     */
    uint8_t sequenceNumber;

    /**
     * Select the external event source used to trigger ADC conversion start.
     */
    Adc_Trigger externalTrigger;
    /**
     * External trigger enable and polarity selection for regular channels.
     */
    Adc_TriggerPolarity externalTriggerPolarity;

    /**
     * Callback for End-of-Conversion
     */
    void (* eocCallback)(struct _Adc_Device *dev);
    /**
     * Callback for End-of-Sequence
     */
    void (* eosCallback)(struct _Adc_Device *dev);
    /**
     * Callback for Overrun
     */
    void (* overrunCallback)(struct _Adc_Device *dev);

#endif

} Adc_Config;

/** @name Configuration functions
 *  Functions to initialize and de-initialize a ADC peripheral.
 */
///@{

/**
 * This function initialize the ADC device and setup operational mode according
 * to the specified parameters in the @ref Adc_Config
 *
 * @param[in] dev Adc device handle
 * @param[in] config A pointer to configuration object
 * @return A System_Errors elements that indicate the status of initialization.
 */
System_Errors Adc_init (Adc_DeviceHandle dev, Adc_Config *config);

/**
 * This function de-initialize the selected peripheral.
 *
 * @param[in] dev Adc device handle
 */
System_Errors Adc_deInit (Adc_DeviceHandle dev);

///@}

/** @name ADC read functions
 *  Functions to configure and read data from pins or internal channel.
 */
///@{

typedef struct _Adc_ChannelConfig
{
    /**
     * Specify if the channel must be converted in single ended or differential mode.
     */
    Adc_InputType type;

    /**
     * Specify if the configuration is for internal channel. In this case, the configuration
     * use the channel parameter, otherwise search the correct channel using the pin name.
     */
    bool isInternal;
    /**
     * Specify the internal channel to configure.
     * This field can be used only for internal channel.
     */
    Adc_Channels channel;

    Adc_SequencePosition position;

    Adc_SamplingTime samplingTime;

} Adc_ChannelConfig;

/**
 * Configure pin or internal channel to be used as Adc input.
 *
 * @param[in] dev Adc device handle
 * @param[in] config Configuration list for selected pin
 * @param[in] pin Selected microcontroller pin or ADC_PIN_INTERNAL for internal channel
 * @return ERRORS_NO_ERROR for configuration without problems, otherwise a specific error.
 */
System_Errors Adc_configPin (Adc_DeviceHandle dev, Adc_ChannelConfig* config, Adc_Pins pin);

/**
 * Enable Adc and start conversion of regular group.
 *
 * @param[in] dev Adc device handle
 * @return ERRORS_NO_ERROR for start conversion without problems, otherwise a specific error.
 */
System_Errors Adc_start (Adc_DeviceHandle dev);

/**
 * Stop Adc conversion of regular group and disable peripheral.
 *
 * @param[in] dev Adc device handle
 * @return ERRORS_NO_ERROR for stop conversion without problems, otherwise a specific error.
 */
System_Errors Adc_stop (Adc_DeviceHandle dev);

/**
 * Get Adc conversion result.
 *
 * @param[in] dev Adc device handle
 * @return The converted values
 */
uint32_t Adc_read (Adc_DeviceHandle dev);

/**
 * Wait until conversion ends.
 * Check EOC or EOS based on what the user has selected during configuration.
 *
 * @param[in] dev Adc device handle
 * @param[in] timeout Maximum wait time in milli-second
 * @return
 */
System_Errors Adc_poll (Adc_DeviceHandle dev, uint32_t timeout);

///@}

/** @name ADC utility functions
 *  Useful functions to manage read data form Adc.
 */
///@{

/**
 * Convert raw data read from channel 17 to temperature, in Celsius.
 *
 * @param[in] dev Adc device handle
 * @param[in] data Raw data from Adc
 * @param[in] vref Analog reference voltage connected to microcontroller in milli-volt
 */
int32_t Adc_getTemperature (Adc_DeviceHandle dev, uint32_t data, uint32_t vref);

///@}

//System_Errors Adc_readValue (Adc_DeviceHandle dev,
//                             Adc_ChannelNumber channel,
//                             uint16_t *value,
//                             Adc_InputType type);
//System_Errors Adc_readValueFromInterrupt (Adc_DeviceHandle dev, uint16_t *value);
//
//System_Errors Adc_setHwChannelTrigger (Adc_DeviceHandle dev,
//                                       Adc_ChannelConfig* config,
//                                       uint8_t numChannel);
//
//System_Errors Adc_enableDmaTrigger (Adc_DeviceHandle dev);
//
///**
// * See AN4662 for info in calibration process.
// */
//System_Errors Adc_calibration (Adc_DeviceHandle dev);

#if defined (LIBOHIBOARD_FRDMKL02Z) || \
	defined (LIBOHIBOARD_KL02Z4)    || \
	defined (LIBOHIBOARD_FRDMKL03Z) || \
	defined (LIBOHIBOARD_KL03Z4)    || \
    defined (LIBOHIBOARD_KL15Z4)

extern Adc_DeviceHandle ADC0;

#elif defined (LIBOHIBOARD_KL25Z4)    || \
	  defined (LIBOHIBOARD_FRDMKL25Z)

/* Bandgap value */
#define ADC_BGR_mV               1000

void ADC0_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;

#elif defined (LIBOHIBOARD_K12D5)

void ADC0_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;

#elif defined (LIBOHIBOARD_K10DZ10)    || \
	  defined (LIBOHIBOARD_K10D10)

extern Adc_DeviceHandle OB_ADC0;
extern Adc_DeviceHandle OB_ADC1;

#elif defined (LIBOHIBOARD_K60DZ10)

extern Adc_DeviceHandle ADC0;
extern Adc_DeviceHandle ADC1;

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

void ADC0_IRQHandler();
void ADC1_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;
extern Adc_DeviceHandle OB_ADC1;

#elif defined (LIBOHIBOARD_KV31F12)

void ADC0_IRQHandler();
void ADC1_IRQHandler();

extern Adc_DeviceHandle OB_ADC0;
extern Adc_DeviceHandle OB_ADC1;

#endif

#ifdef __cplusplus
}
#endif

#endif // __ADC_H

#endif // LIBOHIBOARD_ADC
