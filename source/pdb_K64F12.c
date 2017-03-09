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
 * @file libohiboard/source/pdb_K64F12.c
 * @author Matteo Civale <m.civale@gmail.com>
 * @brief PDB implementations for K64F12.
 */

#ifdef LIBOHIBOARD_PDB

#include "platforms.h"
#include "interrupt.h"
#include "pdb.h"

#if defined (LIBOHIBOARD_K64F12) || \
    defined (LIBOHIBOARD_FRDMK64F)

typedef struct Pdb_Device
{
    PDB_MemMapPtr regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    void (*callback)(void);

    uint8_t devInizialized;

} Pdb_Device;

static Pdb_Device pdb0 =
{
    .regMap           = PDB0_BASE_PTR,

    .simScgcPtr       = &SIM_SCGC6,
    .simScgcBitEnable = SIM_SCGC6_PDB_MASK,

    .callback         = 0,

    .devInizialized   = 0
};
Pdb_DeviceHandle OB_PDB0 = &pdb0;

System_Errors Pdb_init (Pdb_DeviceHandle dev, void *callback, Pdb_Config *config)
{
    uint8_t channelIndex;
    uint8_t channelNum;
    uint8_t preChannelIndex;

    /* Enable clock for the PDB peripheral */
    *(dev->simScgcPtr) |= dev->simScgcBitEnable;

    /* Scan configuration channels for ADC */
    for (channelIndex=0; channelIndex < config->numChannelToConfigAdc; channelIndex++)
    {
        switch (config->channelConfigAdc[channelIndex].device)
        {
        case PDB_DEVICETRIGGERED_ADC0:
            channelNum = 0;
            break;
        case PDB_DEVICETRIGGERED_ADC1:
            channelNum = 1;
            break;
        default:
            return ERROR_PTB_DEVICE_WRONG;
        }

        PDB_C1_REG(dev->regMap,channelNum) |= PDB_C1_EN(config->channelConfigAdc[channelIndex].preTriggerEnabled)  |
                                              PDB_C1_TOS(config->channelConfigAdc[channelIndex].preTriggerEnabled) |
                                              PDB_C1_BB(config->channelConfigAdc[channelIndex].backToBackEnabled);

        for (preChannelIndex=0; preChannelIndex<PDB_MAX_PRETRIGGER_SOURCE; preChannelIndex++)
        {
            if (((config->channelConfigAdc[channelIndex].preTriggerEnabled) & (1<<preChannelIndex)) &&
               !((config->channelConfigAdc[channelIndex].backToBackEnabled) & (1<<preChannelIndex)))
            {
                PDB_DLY_REG(dev->regMap,channelNum,preChannelIndex) |=
                        (config->channelConfigAdc[channelIndex].preTriggerDelay[preChannelIndex]);
            }
        }
    }

    if (callback)
    {
        PDB_SC_REG(dev->regMap) |= PDB_SC_PDBIE_MASK;
        dev->callback = callback;
        Interrupt_enable(INTERRUPT_PDB);
    }

    // Configure the Status and control Register
    if (config->enableErrorInterrupt) PDB_SC_REG(dev->regMap) |= PDB_SC_PDBEIE_MASK;

    PDB_SC_REG(dev->regMap) |= PDB_SC_TRGSEL(config->triggerType)  | PDB_SC_LDMOD(config->loadingMode) |
                               PDB_SC_PRESCALER(config->prescaler) | PDB_SC_MULT(config->multiplicator) |
                               PDB_SC_LDOK_MASK | PDB_SC_PDBEN_MASK;

    return ERRORS_NO_ERROR;
}

/* TODO: INTERRUPT! */
/* TODO: DAC HANDLING */

#endif /* LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F */

#endif /* LIBOHIBOARD_PDB */
