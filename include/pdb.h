/******************************************************************************
 * Copyright (C) 2015-2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
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
 ******************************************************************************/

/**
 * @file libohiboard/include/pdb.h
 * @author Matteo Civale <m.civale@gmail.com>
 * @brief PDB definitions and prototypes.
 */

#ifdef LIBOHIBOARD_PDB

#ifndef __PDB_H
#define __PDB_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

/* Define pre-trigger masks */
/* Enable mask for pre-trigger */
#define PDB_PRE_TRIG0_EN       0x01
#define PDB_PRE_TRIG1_EN       0x02

/* Enable back to back mask */
#define PDB_PRE_TRIG0_BTB_EN   0x01
#define PDB_PRE_TRIG1_BTB_EN   0x02

/* Define max boundary value */
#define PDB_MAX_PRETRIGGER_SOURCE  2
#define PDB_MAX_ADC_CHANNEL_CONFIG 2
#define PDB_MAX_DAC_CHANNEL_CONFIG 2

typedef enum
{
#if defined (LIBOHIBOARD_K64F12) || \
    defined (LIBOHIBOARD_FRDMK64F)

    PDB_DEVICETRIGGERED_ADC0 = 0,
    PDB_DEVICETRIGGERED_ADC1 = 1,
    PDB_DEVICETRIGGERED_DAC0 = 2,
    PDB_DEVICETRIGGERED_DAC1 = 3,

#endif
} Pdb_DeviceTriggered;

typedef enum
{
#if defined (LIBOHIBOARD_K64F12) || \
    defined (LIBOHIBOARD_FRDMK64F)

    PDB_TRIGGERTYPE_EXT    = 0,
    PDB_TRIGGERTYPE_CMP0   = 1,
    PDB_TRIGGERTYPE_CMP1   = 2,
    PDB_TRIGGERTYPE_CMP2   = 3,
    PDB_TRIGGERTYPE_PITCH0 = 4,
    PDB_TRIGGERTYPE_PITCH1 = 5,
    PDB_TRIGGERTYPE_PITCH2 = 6,
    PDB_TRIGGERTYPE_PITCH3 = 7,
    PDB_TRIGGERTYPE_FTM0   = 8,
    PDB_TRIGGERTYPE_FTM1   = 9,
    PDB_TRIGGERTYPE_FTM2   = 10,
    PDB_TRIGGERTYPE_FTM3   = 11,
    PDB_TRIGGERTYPE_RTC_A  = 12,
    PDB_TRIGGERTYPE_RTC_S  = 13,
    PDB_TRIGGERTYPE_LPTMR  = 14,
    PDB_TRIGGERTYPE_SOFT   = 15,

#endif
} Pdb_TriggerType;

typedef struct _Pdb_AdcChannelConfig
{
    Pdb_DeviceTriggered device;

    uint8_t preTriggerEnabled;
    uint16_t preTriggerDelay[PDB_MAX_PRETRIGGER_SOURCE];
    uint8_t backToBackEnabled;

} Pdb_AdcChannelConfig;

typedef struct _Pdb_DacChannelConfig
{
    Pdb_DeviceTriggered device;

    /* TODO!!! */

} Pdb_DacChannelConfig;

typedef struct _Pdb_Config
{
    uint8_t numChannelToConfigAdc;
    Pdb_AdcChannelConfig channelConfigAdc[PDB_MAX_ADC_CHANNEL_CONFIG];

    uint8_t numChannelToConfigDac;
    Pdb_DacChannelConfig channelConfigDac[PDB_MAX_DAC_CHANNEL_CONFIG];

    Pdb_TriggerType triggerType;

    uint8_t multiplicator;
    uint8_t prescaler;
    uint8_t loadingMode;
    bool enableErrorInterrupt;
    bool enableContinus;

} Pdb_Config;

typedef struct Pdb_Device* Pdb_DeviceHandle;

#if defined (LIBOHIBOARD_K64F12) || \
    defined (LIBOHIBOARD_FRDMK64F)

extern Pdb_DeviceHandle OB_PDB0;

#endif

System_Errors Pdb_init (Pdb_DeviceHandle dev, void *callback, Pdb_Config *config);

#endif /* __PDB_H */

#endif /* LIBOHIBOARD_PDB */
