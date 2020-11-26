/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2020 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/NXPMKL/uart_MKL.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART pins and device definitions for NXP MKL series
 */

#ifndef __UART_MKL_H
#define __UART_MKL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_UART) & defined(LIBOHIBOARD_MKL)

/**
 * @addtogroup UART
 * @{
 */

/**
 * @defgroup UART_Hardware UART specific hardware types
 * @{
 */

/**
 * The maximum Baud Rate is derived from the maximum clock on L0 that is 32MHz
 * divided by the smallest oversampling used on the USART (8).
 */
#define UART_MAX_BAUDRATE                (40000001u)

/**
 * List of all Rx pins.
 */
typedef enum _Uart_RxPins
{
#if defined (LIBOHIBOARD_MKL15)

    UART_PINS_PTA1,
#if defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTA15,
#endif
    UART_PINS_PTA18,

#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTB16,
#endif

    UART_PINS_PTC3,

#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTD2,
#endif
    UART_PINS_PTD4,
    UART_PINS_PTD6,

#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTE1,
#endif
    UART_PINS_PTE17,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTE21,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTE23,
#endif

#elif defined (LIBOHIBOARD_MKL25)

	// TODO

#endif

    UART_PINS_RXNONE,

} Uart_RxPins;

/**
 * List of all Tx pins.
 */
typedef enum _Uart_TxPins
{
#if defined (LIBOHIBOARD_MKL15)

    UART_PINS_PTA2,
#if defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTA14,
#endif
    UART_PINS_PTA19,

#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTB17,
#endif

    UART_PINS_PTC4,

#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTD3,
#endif
    UART_PINS_PTD5,
    UART_PINS_PTD7,

#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTE0,
#endif
    UART_PINS_PTE16,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTE20,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    UART_PINS_PTE22,
#endif

#elif defined (LIBOHIBOARD_MKL25)

	// TODO

#endif

    UART_PINS_TXNONE,

} Uart_TxPins;

/**
 * List of all Cts pins.
 */
typedef enum _Uart_CtsPins
{
	// TODO!

    UART_PINS_CTSNONE,

} Uart_CtsPins;

/**
 * List of all Rts pins.
 */
typedef enum _Uart_RtsPins
{
	// TODO

    UART_PINS_RTSNONE,

} Uart_RtsPins;

#if defined (LIBOHIBOARD_MKL15) || \
    defined (LIBOHIBOARD_MKL25)

void UART0_IRQHandler ();
void UART1_IRQHandler ();
void UART2_IRQHandler ();

extern Uart_DeviceHandle OB_UART0;
extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;

#endif

#endif // LIBOHIBOARD_UART & LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // __UART_MKL_H
