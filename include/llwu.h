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
 * @file libohiboard/include/adc.h
 * @author Matteo Pirro
 * @brief LLWU definitions and prototypes.
 */

#ifdef LIBOHIBOARD_LLWU
 
#ifndef __LLWU_H
#define __LLWU_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef struct _Llwu_Device {

	LLWU_MemMapPtr regMap;

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Llwu_Device;



typedef Llwu_Device* Llwu_DeviceHandle;

extern Llwu_DeviceHandle OB_LLWU0;


#define LLWU_MAX_REG_EXTPIN	4
#define LLWU_MAX_EXTPIN		16


typedef enum
{

#if defined (LIBOHIBOARD_KL25Z4)     || \
	  defined (LIBOHIBOARD_FRDMKL25Z) || \
	  defined (LIBOHIBOARD_KL15Z4)

    LLWU_P0,
	LLWU_P1,
	LLWU_P2,
	LLWU_P3,
	LLWU_P4,
	LLWU_P5,
	LLWU_P6,
	LLWU_P7,
	LLWU_P8,
	LLWU_P9,
	LLWU_P10,
	LLWU_P11,
	LLWU_P12,
	LLWU_P13,
	LLWU_P14,
	LLWU_P15,

#endif

} Llwu_ExtPins;


typedef enum
{
    LLWU_EXTPIN_EVENT_DISABLED   		= 0x0,
	LLWU_EXTPIN_EVENT_ON_RISING  		= 0x1,
	LLWU_EXTPIN_EVENT_ON_FALLING 		= 0x2,
	LLWU_EXTPIN_EVENT_ON_ANY_CHANGE 	= 0x3,
} Llwu_ExtPin_EventType;


#define LLWU_MAX_WAKEUP_MODULE	8

typedef enum
{

#if defined (LIBOHIBOARD_KL25Z4)     || \
	  defined (LIBOHIBOARD_FRDMKL25Z)||	\
	  defined (LIBOHIBOARD_KL15Z4)

    LLWU_WUME0,
	LLWU_WUME1,
	LLWU_WUME2,
	LLWU_WUME3,
	LLWU_WUME4,
	LLWU_WUME5,
	LLWU_WUME6,
	LLWU_WUME7,

#endif

} Llwu_WakeupModules;


typedef enum
{
    LLWU_WAKEUP_MODULE_DISABLE   		= 0x0,
	LLWU_WAKEUP_MODULE_ENABLE	  		= 0x1,
} Llwu_WakeupModule_Enable;

System_Errors Llwu_configExtPin_Interrupt (Llwu_DeviceHandle dev, Llwu_ExtPins pin, Llwu_ExtPin_EventType event, void* callback);
System_Errors Llwu_configWakeupModule_Interrupt (Llwu_DeviceHandle dev, Llwu_WakeupModules wum, Llwu_WakeupModule_Enable enable, void* callback);

#endif /* __LLWU_H */

#endif // LIBOHIBOARD_LLWU
