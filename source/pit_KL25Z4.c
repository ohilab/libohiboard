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
 * @file libohiboard/source/pit_KL25Z4.c
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief PIT implementations for KL25Z4 and FRDMKL25Z.
 */

#ifdef LIBOHIBOARD_PIT

#if defined (LIBOHIBOARD_KL25Z4) || \
    defined (LIBOHIBOARD_FRDMKL25Z)

#include "platforms.h"

#include "pit.h"
#include "interrupt.h"
#include "clock.h"

#define PIT_MAX_NUMBER   2

typedef struct Pit_Device
{
    PIT_MemMapPtr regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    /** The functions pointer for ISR. */
    void (*isr)(void);
    /** The functions pointer for user callback. */
    void (*callback[PIT_MAX_NUMBER])(void);
    /** ISR vector numbers. */
    Interrupt_Vector isrNumber;

    /** Indicate which timer was been initialized. */
    bool isInitialized[PIT_MAX_NUMBER];
} Pit_Device;


static Pit_Device pit0 = {
    .regMap           = PIT_BASE_PTR,

    .simScgcPtr       = &SIM_SCGC6,
    .simScgcBitEnable = SIM_SCGC6_PIT_MASK,

    .isr              = PIT_IRQHandler,

    .isrNumber        = INTERRUPT_PIT,

    .callback         = {
                        0,
						0
    },

    .isInitialized    = {
                        FALSE
    },
};
Pit_DeviceHandle OB_PIT0 = &pit0;

void PIT_IRQHandler (void)
{
	uint8_t i;
	for (i=0;i<PIT_MAX_NUMBER;i++)
	{
		if(PIT_TFLG_REG(OB_PIT0->regMap,i)&PIT_TFLG_TIF_MASK)
	    {
			OB_PIT0->callback[i]();
			PIT_TFLG_REG(OB_PIT0->regMap,i)=PIT_TFLG_TIF_MASK;
	    }
	}
}

System_Errors Pit_init (Pit_DeviceHandle dev)
{
    /* Enable the clock to the selected PIT */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* Turn on the PIT */
    PIT_MCR_REG(dev->regMap) = 0;

    return ERRORS_NO_ERROR;
}

System_Errors Pit_config (Pit_DeviceHandle dev, void *callback, Pit_Config* config)
{
    float currentBusPeriod = (float) 1000000000/Clock_getFrequency(CLOCK_BUS);//[ns]
    float requestPeriod    = (float) 1000000000/config->frequency; //[ns]
    uint32_t regValue;

    if (config->number > (PIT_MAX_NUMBER - 1))
        return ERRORS_PIT_NOT_EXIST;

    /* Clear status */
    dev->isInitialized[config->number] = FALSE;

    regValue = (uint32_t) requestPeriod/currentBusPeriod;
    if (regValue == 0)
        return ERRORS_PIT_WRONG_VALUE;

    PIT_LDVAL_REG(dev->regMap,config->number) = regValue - 1;

    if (callback)
    {
    	PIT_TFLG_REG(dev->regMap,config->number) |= PIT_TFLG_TIF_MASK; //reset interrupt
    	PIT_TCTRL_REG(dev->regMap,config->number) |= PIT_TCTRL_TIE_MASK;
        dev->callback[config->number] = callback;
        /* Enable interrupt */
        Interrupt_enable(dev->isrNumber);
    }

    dev->isInitialized[config->number] = TRUE;

    return ERRORS_NO_ERROR;
}

System_Errors Pit_start (Pit_DeviceHandle dev, uint8_t number)
{
    if (number > (PIT_MAX_NUMBER - 1))
        return ERRORS_PIT_NOT_EXIST;

    if (!dev->isInitialized[number])
        return ERRORS_PIT_NOT_INITIALIZED;

    PIT_TCTRL_REG(dev->regMap,number) |= PIT_TCTRL_TEN_MASK;
    return ERRORS_NO_ERROR;

}

System_Errors Pit_stop (Pit_DeviceHandle dev, uint8_t number)
{
    if (number > (PIT_MAX_NUMBER - 1))
        return ERRORS_PIT_NOT_EXIST;

    if (!dev->isInitialized[number])
        return ERRORS_PIT_NOT_INITIALIZED;

    PIT_TCTRL_REG(dev->regMap,number) &= ~PIT_TCTRL_TEN_MASK;
    return ERRORS_NO_ERROR;
}
#endif /* LIBOHIBOARD_KL25Z4 || LIBOHIBOARD_FRDMKL25Z */

#endif /* LIBOHIBOARD_PIT */
