/* Copyright (C) 2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/source/pit_K64F12.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief PIT implementations for K64F12 and FRDMK64F.
 */

#ifdef LIBOHIBOARD_PIT

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

#include "platforms.h"

#include "pit.h"
#include "interrupt.h"
#include "clock.h"

#define PIT_MAX_NUMBER   4

typedef struct Pit_Device
{
	PIT_Type* regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    /** The functions pointer for ISR. */
    void (*isr[PIT_MAX_NUMBER])(void);
    /** The functions pointer for user callback. */
    void (*callback[PIT_MAX_NUMBER])(void);
    /** ISR vector numbers. */
    Interrupt_Vector isrNumber[PIT_MAX_NUMBER];

    /** Indicate which timer was been initialized. */
    bool isInitialized[PIT_MAX_NUMBER];
} Pit_Device;

void PIT0_IRQHandler (void);
void PIT1_IRQHandler (void);
void PIT2_IRQHandler (void);
void PIT3_IRQHandler (void);

static Pit_Device pit0 = {
    .regMap           = PIT_BASE_PTR,

    .simScgcPtr       = &SIM_SCGC6,
    .simScgcBitEnable = SIM_SCGC6_PIT_MASK,

    .isr              = {
                        PIT0_IRQHandler,
                        PIT1_IRQHandler,
                        PIT2_IRQHandler,
                        PIT3_IRQHandler
    },
    .isrNumber        = {
                        INTERRUPT_PIT0,
                        INTERRUPT_PIT1,
                        INTERRUPT_PIT2,
                        INTERRUPT_PIT3
    },
    .callback         = {
                        0,
                        0,
                        0,
                        0
    },

    .isInitialized    = {
                        FALSE,
                        FALSE,
                        FALSE,
                        FALSE
    },
};
Pit_DeviceHandle OB_PIT0 = &pit0;

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_PIT0
void PIT0_IRQHandler (void)
{
    PIT_TFLG_REG(OB_PIT0->regMap,0) |= PIT_TFLG_TIF_MASK;
    OB_PIT0->callback[0]();
}
#endif

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_PIT1
void PIT1_IRQHandler (void)
{
    PIT_TFLG_REG(OB_PIT0->regMap,1) |= PIT_TFLG_TIF_MASK;
    OB_PIT0->callback[1]();
}
#endif

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_PIT2
void PIT2_IRQHandler (void)
{
    PIT_TFLG_REG(OB_PIT0->regMap,2) |= PIT_TFLG_TIF_MASK;
    OB_PIT0->callback[2]();
}
#endif

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_PIT3
void PIT3_IRQHandler (void)
{
    PIT_TFLG_REG(OB_PIT0->regMap,3) |= PIT_TFLG_TIF_MASK;
    OB_PIT0->callback[3]();
}
#endif

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
    float requestPeriod = (float) 1000000000/config->frequency; //[ns]
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
        PIT_TCTRL_REG(dev->regMap,config->number) |= PIT_TCTRL_TIE_MASK;
        /* Enable interrupt */
        Interrupt_enable(dev->isrNumber[config->number]);
        dev->callback[config->number] = callback;
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

System_Errors Pit_enableInterrupt (Pit_DeviceHandle dev, uint8_t number)
{
    if (number > (PIT_MAX_NUMBER - 1))
        return ERRORS_PIT_NOT_EXIST;

    if (!dev->isInitialized[number])
        return ERRORS_PIT_NOT_INITIALIZED;

    PIT_TCTRL_REG(dev->regMap,number) |= PIT_TCTRL_TIE_MASK;
    return ERRORS_NO_ERROR;
}

System_Errors Pit_disableInterrupt (Pit_DeviceHandle dev, uint8_t number)
{
    if (number > (PIT_MAX_NUMBER - 1))
        return ERRORS_PIT_NOT_EXIST;

    if (!dev->isInitialized[number])
        return ERRORS_PIT_NOT_INITIALIZED;

    PIT_TCTRL_REG(dev->regMap,number) &= ~PIT_TCTRL_TIE_MASK;
    return ERRORS_NO_ERROR;
}


#endif /* LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F */

#endif /* LIBOHIBOARD_PIT */
