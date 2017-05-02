/******************************************************************************
 * Copyright (C) 2017 A. C. Open Hardware Ideas Lab
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

#include "llwu.h"


static uint32_t INT_EXTPIN = 0x0;
static uint32_t INT_WAKEUP_MODULE = 0x0;

static void (*Llwu_isrExtPinRequestVector[LLWU_MAX_EXTPIN]) (void);
static void (*Llwu_isrWakeupModuleRequestVector[LLWU_MAX_WAKEUP_MODULE]) (void);

static Llwu_Device llwu0 = {
		.regMap = LLWU_BASE_PTR,
};

Llwu_DeviceHandle OB_LLWU0 = &llwu0;


typedef enum
{
    LLWU_EXTPIN_0_3,
	LLWU_EXTPIN_4_7,
	LLWU_EXTPIN_8_11,
	LLWU_EXTPIN_12_15,
	LLWU_WAKEUP_MODULE,
} Llwu_register;

typedef struct _Llwu_IntDevice
{
	Llwu_register reg;
    uint8_t pinNumber;
} Llwu_IntDevice;

static Llwu_IntDevice Llwu_IntDevices[] =
{
        {LLWU_EXTPIN_0_3,0},
		{LLWU_EXTPIN_0_3,1},
		{LLWU_EXTPIN_0_3,2},
		{LLWU_EXTPIN_0_3,3},
		{LLWU_EXTPIN_4_7,0},
		{LLWU_EXTPIN_4_7,1},
		{LLWU_EXTPIN_4_7,2},
		{LLWU_EXTPIN_4_7,3},

		{LLWU_EXTPIN_8_11,0},
		{LLWU_EXTPIN_8_11,1},
		{LLWU_EXTPIN_8_11,2},
		{LLWU_EXTPIN_8_11,3},
		{LLWU_EXTPIN_12_15,0},
		{LLWU_EXTPIN_12_15,1},
		{LLWU_EXTPIN_12_15,2},
		{LLWU_EXTPIN_12_15,3},

		{LLWU_WAKEUP_MODULE,0},
		{LLWU_WAKEUP_MODULE,1},
		{LLWU_WAKEUP_MODULE,2},
		{LLWU_WAKEUP_MODULE,3},
		{LLWU_WAKEUP_MODULE,0},
		{LLWU_WAKEUP_MODULE,1},
		{LLWU_WAKEUP_MODULE,2},
		{LLWU_WAKEUP_MODULE,3},
};

System_Errors Llwu_Init (Llwu_DeviceHandle dev)
{
	/* Clear flags for external sources */
	dev->regMap->F1 |= 0xFF;
	dev->regMap->F2 |= 0xFF;

    /* Disable all external sources*/
	dev->regMap->PE1 &= 0;
	dev->regMap->PE2 &= 0;
	dev->regMap->PE3 &= 0;
	dev->regMap->PE4 &= 0;

    /* Disable all internal sources */
	dev->regMap->ME &= 0;

}

System_Errors Llwu_configExtPin_Interrupt (Llwu_DeviceHandle dev, Llwu_ExtPins pin, Llwu_ExtPin_EventType event, void* callback)
{

	switch(Llwu_IntDevices[pin].reg)
    {
    case LLWU_EXTPIN_0_3:
    	dev->regMap->PE1 &= ~(0b11 << pin*2);
    	dev->regMap->PE1 |= (event << pin*2);
    	dev->regMap->F1 |= (1 << pin*2);
        break;
    case LLWU_EXTPIN_4_7:
    	dev->regMap->PE2 &= ~(0b11 << (pin-4)*2);
    	dev->regMap->PE2 |= (event << (pin-4)*2);
    	dev->regMap->F1 |= (1 << pin*2);
        break;
    case LLWU_EXTPIN_8_11:
    	dev->regMap->PE3 &= ~(0b11 << (pin-8)*2);
    	dev->regMap->PE3 |= (event << (pin-8)*2);
    	dev->regMap->F2 |= (1 << (pin-8)*2);
		break;
    case LLWU_EXTPIN_12_15:
    	dev->regMap->PE4 &= ~(0b11 << (pin-12)*2);
    	dev->regMap->PE4 |= (event << (pin-12)*2);
    	dev->regMap->F2 |= (1 << (pin-8)*2);
		break;
    default:
        assert(0);
        return ERRORS_LLWU_WRONG_EXTPIN;
    }

    Llwu_isrExtPinRequestVector[pin] = callback;
	INT_EXTPIN |= 1 << pin;

	Interrupt_enable(INTERRUPT_LLWU);

    return ERRORS_NO_ERROR;
}

System_Errors Llwu_configWakeupModule_Interrupt (Llwu_DeviceHandle dev, Llwu_WakeupModules wum, Llwu_WakeupModule_Enable enable, void* callback)
{
	if(wum > 0 && wum < LLWU_MAX_WAKEUP_MODULE)
	{
		dev->regMap->ME &= ~(1 << wum);
		dev->regMap->ME |= (enable << wum);
		dev->regMap->F3 |= (1 << wum);
	}
	else
	{
		return ERRORS_LLWU_WRONG_WAKEUPMODULE;
	}

	Llwu_isrWakeupModuleRequestVector[wum] = callback;
	INT_WAKEUP_MODULE |= 1 << wum;

	Interrupt_enable(INTERRUPT_LLWU);

    return ERRORS_NO_ERROR;
}


void LLWU_IRQHandler(void)
{

	uint8_t i=0;

	int a,mask = 0;
	mask = 1 << 6;
	a = 0x40 & mask;
	if(a == mask)
	{
		a++;
	}

	while (i < LLWU_MAX_EXTPIN)
	{
		uint8_t mask = 0;
		if(i < LLWU_MAX_EXTPIN/2)
		{
			mask = (1 << i);
			if((OB_LLWU0->regMap->F1 >> i) & mask == mask)
			{
				Llwu_isrExtPinRequestVector[i]();
				//reset interrupt
				OB_LLWU0->regMap->F1 |= (1 << i);
			}
		}
		else
		{
			mask = (1 << (i-8));
			if((OB_LLWU0->regMap->F2 >> (i-8)) & mask == mask)
			{
				Llwu_isrExtPinRequestVector[i]();
				//reset interrupt
				OB_LLWU0->regMap->F2 |= (1 << (i-8));
			}
		}
		i++;
	}

	i=0;


/* FixMe: Da testare */
//	while (i < LLWU_MAX_WAKEUP_MODULE)
//	{
//		if(INT_WAKEUP_MODULE & (1 << i))
//		{
//			Llwu_isrWakeupModuleRequestVector[i]();
//		}
//		i++;
//	}

}

#endif // LIBOHIBOARD_LLWU
