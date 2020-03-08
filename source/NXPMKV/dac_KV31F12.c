/******************************************************************************
 * Copyright (C) 2017 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/source/dac_KV31F12.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief DAC implementations for KV31F12.
 */

#ifdef LIBOHIBOARD_DAC

#if defined (LIBOHIBOARD_KV31F12)

#include "platforms.h"
#include "utility.h"
#include "dac.h"
#include "clock.h"

typedef struct Dac_Device
{
    DAC_MemMapPtr regMap;                          /**< Device memory pointer */

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Dac_BufferMode bufferMode;

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Dac_Device;

static Dac_Device dac0 = {
        .regMap           = DAC0_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_DAC0_MASK,

        .devInitialized   = 0,
};
Dac_DeviceHandle OB_DAC0 = &dac0;

static Dac_Device dac1 = {
        .regMap           = DAC1_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_DAC1_MASK,

        .devInitialized   = 0,
};
Dac_DeviceHandle OB_DAC1 = &dac1;

System_Errors Dac_writeValue (Dac_DeviceHandle dev, uint16_t value)
{
    if (!dev->devInitialized) return ERRORS_DAC_DEVICE_NOT_INIT;

    if (dev->bufferMode == DAC_BUFFERMODE_OFF)
    {
        DAC_DATL_REG(dev->regMap,0) = (value & DAC_DATL_DATA0_MASK);
        DAC_DATH_REG(dev->regMap,0) = ((value >> 8) & DAC_DATH_DATA1_MASK);
    }
    else
    {
        /* FIXME: buffer mode! */
    }

    return ERRORS_NO_ERROR;
}

System_Errors Dac_init (Dac_DeviceHandle dev, void *callback, Dac_Config *config)
{
    if (dev->devInitialized) return ERRORS_DAC_DEVICE_JUST_INIT;

    /* Enable the clock to the selected DAC */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* Ensure that the DAC is disabled */
    DAC_C0_REG(dev->regMap) = 0x00;

    /* Clean DAC value */
    DAC_DATL_REG(dev->regMap,0) = 0x00;
    DAC_DATH_REG(dev->regMap,0) = 0x00;

    /* Select voltage reference */
    if (config->ref == DAC_VOLTAGEREF_VDDA)
        DAC_C0_REG(dev->regMap) |= DAC_C0_DACRFS_MASK;
    else
        DAC_C0_REG(dev->regMap) &= ~DAC_C0_DACRFS_MASK;

    /* Select high power or low power mode */
    if (config->powerMode == DAC_POWERMODE_LOW)
        DAC_C0_REG(dev->regMap) |= DAC_C0_LPEN_MASK;
    else
        DAC_C0_REG(dev->regMap) &= ~DAC_C0_LPEN_MASK;

    /* FIXME:  Just now we disable DMA */

    dev->bufferMode = config->buffer;

    switch(config->buffer)
    {
    case DAC_BUFFERMODE_OFF:
        DAC_C1_REG(dev->regMap) &= ~DAC_C1_DACBFEN_MASK;
        break;

    case DAC_BUFFERMODE_NORMAL:
        DAC_C1_REG(dev->regMap) |= DAC_C1_DACBFEN_MASK;
        DAC_C1_REG(dev->regMap) &= ~DAC_C1_DACBFMD_MASK;
        break;

    case DAC_BUFFERMODE_SWING:
        break;

    case DAC_BUFFERMODE_ONETIME:
        DAC_C1_REG(dev->regMap) |= DAC_C1_DACBFEN_MASK;
        DAC_C1_REG(dev->regMap) |= DAC_C1_DACBFMD_MASK;
        break;
    }

    /* Enable module */
    DAC_C0_REG(dev->regMap) |= DAC_C0_DACEN_MASK;

    dev->devInitialized = 1;
    return ERRORS_NO_ERROR;
}

#endif /* LIBOHIBOARD_KV31F12 */

#endif /* LIBOHIBOARD_DAC */
