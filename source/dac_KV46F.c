/* Copyright (C) 2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Matteo Civale <matteo.civale@gmail.com>
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
 * @file libohiboard/source/dac_KV46F.c
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief DAC implementations for KV46F or TWRKV46F
 */

#ifdef LIBOHIBOARD_DAC

#if defined (LIBOHIBOARD_KV46F) || \
    defined (LIBOHIBOARD_TRWKV46F)

#include "utility.h"
#include "dac.h"
#include "clock.h"
#include "interrupt.h"

typedef struct Dac_Device
{
    DAC_MemMapPtr regMap;                          /**< Device memory pointer */

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Dac_BufferMode bufferMode;

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */

    uint8_t dmaChannel;

    Interrupt_Vector isrNumber;
    void (*intisr)(void);

} Dac_Device;

static Dac_Device dac0 = {
        .regMap           = DAC0_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_DAC0_MASK,

        .dmaChannel       = 45,

        .devInitialized   = 0,
        .isrNumber        = INTERRUPT_DAC0,
};
Dac_DeviceHandle OB_DAC0 = &dac0;



void DAC0_IRQHandler(void)
{
    OB_DAC0->intisr();
}

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

    DAC_C1_REG(dev->regMap) &= ~DAC_C1_DMAEN_MASK;


    /* Settings for  buffer mode */
//    DAC_C1_REG(dev->regMap) &= ~DAC_C1_DACBFEN_MASK;
//    DAC_C0_REG(dev->regMap) &= ~(DAC_C0_DACBWIEN_MASK|DAC_C0_DACBTIEN_MASK|
//                                 DAC_C0_DACBBIEN_MASK);


    DAC_SR_REG(dev->regMap) = 0x0;

    if (config->buffer != DAC_BUFFERMODE_OFF)
    {
        DAC_C1_REG(dev->regMap) &= ~DAC_C1_DACBFMD_MASK;
        DAC_C1_REG(dev->regMap) |= DAC_C1_DACBFMD(config->buffer);

        DAC_C0_REG(dev->regMap) |= DAC_C0_DACBWIEN(config->interruptEvent.intWaterMark);
        DAC_C0_REG(dev->regMap) |= DAC_C0_DACBTIEN(config->interruptEvent.intTopEn);
        DAC_C0_REG(dev->regMap) |= DAC_C0_DACBBIEN(config->interruptEvent.intBottmEn);

        /* Enable buffer mode */
        DAC_C1_REG(dev->regMap) |= DAC_C1_DACBFEN_MASK;

        if(config->intisr)
        {
            dev->intisr = config->intisr;
            Interrupt_enable(dev->isrNumber);
        }
    }

    /* End buffer mode settings */

    /* Select trigger type */
    DAC_C0_REG(dev->regMap) &= ~DAC_C0_DACTRGSEL_MASK;
    DAC_C0_REG(dev->regMap) |= DAC_C0_DACTRGSEL(config->trigger);

    /* Enable module */
    DAC_C0_REG(dev->regMap) |= DAC_C0_DACEN_MASK;

    dev->devInitialized = 1;
    return ERRORS_NO_ERROR;
}

uint8_t Dac_enableDmaTrigger (Dac_DeviceHandle dev, Dma_RequestSource request)
{

    /* Enable module */
//    DAC_C0_REG(dev->regMap) &= ~DAC_C0_DACEN_MASK;
    DAC_C1_REG(dev->regMap) |= DAC_C1_DMAEN_MASK;
//    DAC_C0_REG(dev->regMap) |= DAC_C0_DACEN_MASK;
    return dev->dmaChannel;

}

System_Errors Dac_loadBuffer(Dac_DeviceHandle dev, uint16_t* buffer, uint8_t startPos, uint8_t len)
{
    if(!dev->devInitialized)
        return ERRORS_DAC_DEVICE_NOT_INIT;

    uint8_t i;
    uint8_t endPos=startPos+len;
    if((endPos>0x10)||(startPos > 0xF))
        return ERRORS_DAC_WRONG_PARAMETER;

    for(i=startPos;i<(endPos);i++)
    {
        DAC_DATL_REG(dev->regMap, i) = buffer[i]&0xFF;
        DAC_DATH_REG(dev->regMap, i) = (buffer[i]>>8)&0xF;
    }

    return ERRORS_NO_ERROR;
}

#endif /* LIBOHIBOARD_K12D5 */

#endif /* LIBOHIBOARD_DAC */
