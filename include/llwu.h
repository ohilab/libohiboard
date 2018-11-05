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
 * @file libohiboard/include/adc.h
 * @author Matteo Pirro
 * @brief LLWU definitions and prototypes.
 */

#ifdef LIBOHIBOARD_LLWU
 
#ifndef __LLWU_H
#define __LLWU_H

#include "platforms.h"
#include "interrupt.h"
#include "errors.h"
#include "types.h"

typedef struct Llwu_Device* Llwu_DeviceHandle;

extern Llwu_DeviceHandle OB_LLWU0;

#define LLWU_MAX_REG_EXTPIN	4
#define LLWU_MAX_EXTPIN		16

typedef enum
{
#if defined (LIBOHIBOARD_KL25Z4)     || \
    defined (LIBOHIBOARD_FRDMKL25Z)  || \
    defined (LIBOHIBOARD_KL15Z4)

    LLWU_PINS_P0,
    LLWU_PINS_P1,
    LLWU_PINS_P2,
    LLWU_PINS_P3,
    LLWU_PINS_P4,
    LLWU_PINS_P5,
    LLWU_PINS_P6,
    LLWU_PINS_P7,
    LLWU_PINS_P8,
    LLWU_PINS_P9,
    LLWU_PINS_P10,
    LLWU_PINS_P11,
    LLWU_PINS_P12,
    LLWU_PINS_P13,
    LLWU_PINS_P14,
    LLWU_PINS_P15,
#endif
	LLWU_PINS_NONE,
} Llwu_Pins;

typedef enum
{
    LLWU_EVENTTYPE_DISABLED      = 0x0,
	LLWU_EVENTTYPE_ON_RISING     = 0x1,
	LLWU_EVENTTYPE_ON_FALLING    = 0x2,
	LLWU_EVENTTYPE_ON_ANY_CHANGE = 0x3,
} Llwu_EventType;

#define LLWU_MAX_WAKEUP_MODULE	8

typedef enum
{

#if defined (LIBOHIBOARD_KL25Z4)     || \
    defined (LIBOHIBOARD_FRDMKL25Z)  || \
    defined (LIBOHIBOARD_KL15Z4)

    LLWU_WAKEUPMODULE_0,
    LLWU_WAKEUPMODULE_1,
    LLWU_WAKEUPMODULE_2,
    LLWU_WAKEUPMODULE_3,
    LLWU_WAKEUPMODULE_4,
    LLWU_WAKEUPMODULE_5,
    LLWU_WAKEUPMODULE_6,
    LLWU_WAKEUPMODULE_7,

#endif

    LLWU_WAKEUPMODULE_NONE,

} Llwu_WakeupModules;


typedef enum
{
    LLWU_WAKEUPMODULEENABLE_NO  = 0x0,
	LLWU_WAKEUPMODULEENABLE_YES = 0x1,
} Llwu_WakeupModuleEnable;

/**
 *
 */
void Llwu_init (Llwu_DeviceHandle dev);

/**
 *
 */
System_Errors Llwu_configExtPinInterrupt (Llwu_DeviceHandle dev,
                                          Llwu_Pins pin,
                                          Llwu_EventType event,
                                          void* callback);

/**
 *
 */
System_Errors Llwu_configWakeupModuleInterrupt (Llwu_DeviceHandle dev,
                                                Llwu_WakeupModules wum,
                                                Llwu_WakeupModuleEnable enable,
                                                void* callback);

#endif // __LLWU_H

#endif // LIBOHIBOARD_LLWU
