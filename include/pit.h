/* Copyright (C) 2016-2017 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/pit.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief PIT definitions and prototypes.
 */

#ifdef LIBOHIBOARD_PIT

#ifndef __PIT_H
#define __PIT_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef struct Pit_Device* Pit_DeviceHandle;

typedef struct _Pit_Config
{
    uint8_t number;         /**< The number of timer that must be configured. */
    uint32_t frequency;  /**< The frequency for interrupt based on bus clock. */

} Pit_Config;

#if defined (LIBOHIBOARD_KL15Z4)   || \
    defined (LIBOHIBOARD_KL25Z4)   || \
    defined (LIBOHIBOARD_FRDMKL25Z)

void PIT_IRQHandler (void);

extern Pit_DeviceHandle OB_PIT0;

#elif defined (LIBOHIBOARD_K12D5)


void PIT0_IRQHandler (void);
void PIT1_IRQHandler (void);
void PIT2_IRQHandler (void);
void PIT3_IRQHandler (void);

extern Pit_DeviceHandle OB_PIT0;

#elif defined (LIBOHIBOARD_K60DZ10) || \
      defined (LIBOHIBOARD_OHIBOARD_R1)

#elif defined (LIBOHIBOARD_KV46F)      || \
	  defined (LIBOHIBOARD_TWRKV46F)

void PIT0_IRQHandler (void);
void PIT1_IRQHandler (void);
void PIT2_IRQHandler (void);
void PIT3_IRQHandler (void);

extern Pit_DeviceHandle OB_PIT0;

#elif defined (LIBOHIBOARD_K64F12)   || \
      defined (LIBOHIBOARD_FRDMK64F)

void PIT0_IRQHandler (void);
void PIT1_IRQHandler (void);
void PIT2_IRQHandler (void);
void PIT3_IRQHandler (void);

extern Pit_DeviceHandle OB_PIT0;

#endif

/**
 * TODO: description...
 *
 * @param[in] dev    The Pit device
 */
System_Errors Pit_init (Pit_DeviceHandle dev);

/**
 * TODO: description...
 *
 * @param[in] dev       The Pit device
 * @param[in] callback  The callback function to manage interrupt
 * @param[in] config    The configuration parameters for the timer
 */
System_Errors Pit_config (Pit_DeviceHandle dev, void *callback, Pit_Config* config);

/**
 * TODO: description...
 *
 * @param[in] dev       The Pit device
 * @param[in] number    The number of timer that must be started
 */
System_Errors Pit_start (Pit_DeviceHandle dev, uint8_t number);

/**
 * TODO: description...
 *
 * @param[in] dev       The Pit device
 * @param[in] number    The number of timer that must be stopped
 */
System_Errors Pit_stop (Pit_DeviceHandle dev, uint8_t number);

#endif /* __PIT_H */

#endif /* LIBOHIBOARD_ETHERNET */
