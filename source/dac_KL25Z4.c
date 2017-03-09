/******************************************************************************
 * Copyright (C) 2015 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Alessio Paolucci <a.paolucci89@gmail.com>
 *   Matteo Civale <matteo.civale@gmail.com>
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
 * @file libohiboard/source/dac_K64F12.h
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @brief DAC implementations for K64F12 and FRDMK64F.
 */

#ifdef LIBOHIBOARD_DAC

#if defined (LIBOHIBOARD_KL25Z4)     || \
    defined (LIBOHIBOARD_FRDMKL25Z)

#include "platforms.h"
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

    void (*callback)(void);
} Dac_Device;

static Dac_Device dac0 = {
        .regMap           = DAC0_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_DAC0_MASK,  // maschera per il bit di abilitazione dell'adc0

        .devInitialized   = 0,
};

Dac_DeviceHandle OB_DAC0 = &dac0;

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
        // FIXME: buffer mode!
    }

    return ERRORS_NO_ERROR;
}


void Dac_setInterruptEvent (Dac_DeviceHandle dev, Dac_InterruptEvent event)
{
    switch(event)
    {
    case DAC_INTERRUPTEVENT_TOP:
        DAC_C0_REG(dev->regMap) |= DAC_C0_DACBTIEN_MASK;
        break;

    case DAC_INTERRUPTEVENT_BOTTOM:
        DAC_C0_REG(dev->regMap) |= DAC_C0_DACBBIEN_MASK;
        break;

    case DAC_INTERRUPTEVENT_BOOTH:
        DAC_C0_REG(dev->regMap) |= DAC_C0_DACBBIEN_MASK|DAC_C0_DACBTIEN_MASK;
        break;
    }
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

    dev->bufferMode = config->buffer;
    switch (config->buffer)
    {
    case DAC_BUFFERMODE_OFF:
        DAC_C1_REG(dev->regMap) &= ~DAC_C1_DACBFEN_MASK;
        break;

    case DAC_BUFFERMODE_NORMAL:
        DAC_C1_REG(dev->regMap) |= DAC_C1_DACBFEN_MASK;
        DAC_C1_REG(dev->regMap) &=~DAC_C1_DACBFMD_MASK;
        break;

    case DAC_BUFFERMODE_SWING:
        break;

    case DAC_BUFFERMODE_ONETIME:
        DAC_C1_REG(dev->regMap) |= DAC_C1_DACBFEN_MASK;
        DAC_C1_REG(dev->regMap) |= DAC_C1_DACBFMD_MASK;
        break;
    }
    Dac_setInterruptEvent(dev, config->interruptEvent);

    if(callback)
    {
        dev->callback = callback;
        Interrupt_enable(INTERRUPT_DAC0);
    }
    /* enable DMA */
    DAC_C1_REG(dev->regMap) |= ((config->dmaEnable<<DAC_C1_DMAEN_SHIFT)&DAC_C1_DMAEN_MASK);

    /* Enable trigger module */
    DAC_C0_REG(dev->regMap)|= ((config->trigger<<DAC_C0_DACTRGSEL_SHIFT)&DAC_C0_DACTRGSEL_MASK);

    /* Enable DAC*/
    DAC_C0_REG(dev->regMap) |= DAC_C0_DACEN_MASK;

    dev->devInitialized = 1;

    return ERRORS_NO_ERROR;
}

#ifdef LIBOHIBOARD_DMA
System_Errors Dac_enableDmaTrigger (Dac_DeviceHandle dev, Dac_InterruptEvent event)
{
	Dac_setInterruptEvent(dev,event);
	return ERRORS_NO_ERROR;
}
#endif

void DAC0_IRQHandler(void)
{
	OB_DAC0->callback();



void DAC0_IRQHandler(void)
{
	OB_DAC0->callback();

	if(DAC_SR_REG(OB_DAC0->regMap)&DAC_SR_DACBFRPBF_MASK)
		DAC_SR_REG(OB_DAC0->regMap)&=~DAC_SR_DACBFRPBF_MASK;
	if(DAC_SR_REG(OB_DAC0->regMap)&DAC_SR_DACBFRPTF_MASK)
		DAC_SR_REG(OB_DAC0->regMap)&=~DAC_SR_DACBFRPTF_MASK;
}

#endif /* LIBOHIBOARD_KL25Z4 || LIBOHIBOARD_FRDMKL25Z */

#endif /* LIBOHIBOARD_DAC */
