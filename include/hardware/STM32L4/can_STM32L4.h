/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2021 A. C. Open Hardware Ideas Lab
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
 */

/**
 * @file libohiboard/include/hardware/STM32L4/can_STM32L4.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief CAN pins and device definitions for STM32L4 series
 */

#ifndef __CAN_STM32L4_H
#define __CAN_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_CAN) & defined(LIBOHIBOARD_STM32L4)


#if defined (LIBOHIBOARD_STM32L4x6)

void CAN1_TX_IRQHandler (void);
void CAN1_RX0_IRQHandler (void);
void CAN1_RX1_IRQHandler (void);
void CAN1_SCE_IRQHandler (void);

extern Can_DeviceHandle OB_CAN1;

#endif

#endif // LIBOHIBOARD_CAN & LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // __CAN_STM32L4_H
