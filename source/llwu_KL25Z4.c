/******************************************************************************
 * Copyright (C) 2017-2018 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Matteo Pirro
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
 * @file libohiboard/source/llwu.c
 * @author Matteo Pirro
 * @brief LLWU implementations for KL25Z4.
 */
 
#ifdef LIBOHIBOARD_LLWU

#if defined (LIBOHIBOARD_KL15Z4) || \
    defined (LIBOHIBOARD_KL25Z4)

#include "llwu.h"

/* https://community.nxp.com/docs/DOC-332687 */

typedef enum
{
    LLWU_REGISTER_EXTPIN_0_3,
    LLWU_REGISTER_EXTPIN_4_7,
    LLWU_REGISTER_EXTPIN_8_11,
    LLWU_REGISTER_EXTPIN_12_15,
    LLWU_REGISTER_WAKEUP_MODULE,
} Llwu_Register;

typedef struct _Llwu_InterruptRegister
{
    Llwu_Register reg;
    uint8_t pinNumber;
} Llwu_InterruptRegister;

typedef struct Llwu_Device
{
    LLWU_MemMapPtr regMap;

    Llwu_InterruptRegister pinReg[LLWU_MAX_EXTPIN+LLWU_MAX_WAKEUP_MODULE];

    uint32_t intPinCounter;
    void (*isrExtPinVector[LLWU_MAX_EXTPIN]) (void);

    uint32_t intWakeupCounter;
    void (*isrWakeupModuleVector[LLWU_MAX_WAKEUP_MODULE]) (void);

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Llwu_Device;

static Llwu_Device llwu0 =
{
		.regMap = LLWU_BASE_PTR,

		.pinReg = {
            {LLWU_REGISTER_EXTPIN_0_3,0},
            {LLWU_REGISTER_EXTPIN_0_3,1},
            {LLWU_REGISTER_EXTPIN_0_3,2},
            {LLWU_REGISTER_EXTPIN_0_3,3},
            {LLWU_REGISTER_EXTPIN_4_7,0},
            {LLWU_REGISTER_EXTPIN_4_7,1},
            {LLWU_REGISTER_EXTPIN_4_7,2},
            {LLWU_REGISTER_EXTPIN_4_7,3},

            {LLWU_REGISTER_EXTPIN_8_11,0},
            {LLWU_REGISTER_EXTPIN_8_11,1},
            {LLWU_REGISTER_EXTPIN_8_11,2},
            {LLWU_REGISTER_EXTPIN_8_11,3},
            {LLWU_REGISTER_EXTPIN_12_15,0},
            {LLWU_REGISTER_EXTPIN_12_15,1},
            {LLWU_REGISTER_EXTPIN_12_15,2},
            {LLWU_REGISTER_EXTPIN_12_15,3},

            {LLWU_REGISTER_WAKEUP_MODULE,0},
            {LLWU_REGISTER_WAKEUP_MODULE,1},
            {LLWU_REGISTER_WAKEUP_MODULE,2},
            {LLWU_REGISTER_WAKEUP_MODULE,3},
            {LLWU_REGISTER_WAKEUP_MODULE,0},
            {LLWU_REGISTER_WAKEUP_MODULE,1},
            {LLWU_REGISTER_WAKEUP_MODULE,2},
            {LLWU_REGISTER_WAKEUP_MODULE,3},
		},

		.intPinCounter = 0,
};
Llwu_DeviceHandle OB_LLWU0 = &llwu0;

void Llwu_init (Llwu_DeviceHandle dev)
{
	// Clear flags for external sources
	dev->regMap->F1 |= 0xFF;
	dev->regMap->F2 |= 0xFF;

    // Disable all external sources
	dev->regMap->PE1 &= 0;
	dev->regMap->PE2 &= 0;
	dev->regMap->PE3 &= 0;
	dev->regMap->PE4 &= 0;

    // Disable all internal sources
	dev->regMap->ME &= 0;
}

System_Errors Llwu_configExtPinInterrupt (Llwu_DeviceHandle dev,
                                          Llwu_Pins pin,
                                          Llwu_EventType event,
                                          void* callback)
{
    if (pin >= LLWU_PINS_NONE) return ERRORS_LLWU_WRONG_EXTPIN;

    switch(dev->pinReg[pin].reg)
    {
    case LLWU_REGISTER_EXTPIN_0_3:
        dev->regMap->PE1 &= ~(0b11 << pin*2);
        dev->regMap->PE1 |= (event << pin*2);
        dev->regMap->F1  |= (1 << pin*2);
        break;
    case LLWU_REGISTER_EXTPIN_4_7:
        dev->regMap->PE2 &= ~(0b11 << (pin-4)*2);
        dev->regMap->PE2 |= (event << (pin-4)*2);
        dev->regMap->F1  |= (1 << pin*2);
        break;
    case LLWU_REGISTER_EXTPIN_8_11:
        dev->regMap->PE3 &= ~(0b11 << (pin-8)*2);
        dev->regMap->PE3 |= (event << (pin-8)*2);
        dev->regMap->F2  |= (1 << (pin-8)*2);
        break;
    case LLWU_REGISTER_EXTPIN_12_15:
        dev->regMap->PE4 &= ~(0b11 << (pin-12)*2);
        dev->regMap->PE4 |= (event << (pin-12)*2);
        dev->regMap->F2  |= (1 << (pin-8)*2);
        break;
    default:
        assert(0);
        return ERRORS_LLWU_WRONG_EXTPIN;
    }

    dev->isrExtPinVector[pin] = callback;
    dev->intPinCounter |= (1 << pin);

    Interrupt_enable(INTERRUPT_LLWU);

    return ERRORS_NO_ERROR;
}

System_Errors Llwu_configWakeupModuleInterrupt (Llwu_DeviceHandle dev,
                                                Llwu_WakeupModules wum,
                                                Llwu_WakeupModuleEnable enable,
                                                void* callback)
{
    if(wum > 0 && wum < LLWU_MAX_WAKEUP_MODULE)
    {
        dev->regMap->ME &= ~(1 << wum);
        dev->regMap->ME |= (enable << wum);
        // F3 is a read-only register
        //dev->regMap->F3 |= (1 << wum);
    }
    else
    {
        return ERRORS_LLWU_WRONG_WAKEUPMODULE;
    }

	dev->isrWakeupModuleVector[wum] = callback;
	dev->intWakeupCounter |= 1 << wum;

    Interrupt_enable(INTERRUPT_LLWU);

    return ERRORS_NO_ERROR;
}

#ifndef LIBOHIBOARD_CUSTOMINTERRUPT_LLWU
void LLWU_IRQHandler (void)
{
	uint8_t i=0;

	while (i < LLWU_MAX_EXTPIN)
    {
        uint8_t mask = 0;
        if (i < LLWU_MAX_EXTPIN/2)
        {
            mask = (1 << i);
            if ((OB_LLWU0->regMap->F1 & mask) == mask)
            {
                OB_LLWU0->isrExtPinVector[i]();
                //reset interrupt
                OB_LLWU0->regMap->F1 |= (1 << i);
            }
        }
        else
        {
            mask = (1 << (i-8));
            if ((OB_LLWU0->regMap->F2 & mask) == mask)
            {
                OB_LLWU0->isrExtPinVector[i]();
                //reset interrupt
                OB_LLWU0->regMap->F2 |= (1 << (i-8));
            }
        }
        i++;
    }
	i=0;

// FIXME: TEST IT!
//	while (i < LLWU_MAX_WAKEUP_MODULE)
//	{
//		if(INT_WAKEUP_MODULE & (1 << i))
//		{
//			Llwu_isrWakeupModuleRequestVector[i]();
//		}
//		i++;
//	}

}
#endif // LIBOHIBOARD_CUSTOMINTERRUPT_LLWU

#endif // LIBOHIBOARD_KL15Z4 || LIBOHIBOARD_KL25Z4

#endif // LIBOHIBOARD_LLWU
