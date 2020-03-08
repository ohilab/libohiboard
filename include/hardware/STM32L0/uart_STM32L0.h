/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019-2020 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/STM32L0/uart_STM32L0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART pins and device definitions for STM32L0 series
 */

#ifndef __UART_STM32L0_H
#define __UART_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_UART) & defined(LIBOHIBOARD_STM32L0)

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
#if defined (LIBOHIBOARD_STM32L0x2)

#if defined (LIBOHIBOARD_STM32L072)

    UART_PINS_PA1_RX,
    UART_PINS_PA3,
    UART_PINS_PA10,
    UART_PINS_PA13,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PA15_RX,
#endif

    UART_PINS_PB4_RX,
    UART_PINS_PB7_RX,

#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PB10_RX,
    UART_PINS_PB11_RX,
#endif

#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PC0,
#endif

#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PC5,
    UART_PINS_PC11,
#endif

#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PD2,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PD6,
    UART_PINS_PD9,

    UART_PINS_PE9,
    UART_PINS_PE11,
#endif

#endif

#elif defined (LIBOHIBOARD_STM32L0x3)

#if defined (LIBOHIBOARD_STM32L073)

    UART_PINS_PA1,
    UART_PINS_PA3,
    UART_PINS_PA10,
    UART_PINS_PA13,
    UART_PINS_PA15,

    UART_PINS_PB4,
    UART_PINS_PB7,
    UART_PINS_PB10_RX,
    UART_PINS_PB11_RX,

#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    UART_PINS_PC0,
    UART_PINS_PC5,
    UART_PINS_PC11,

    UART_PINS_PD2,
#endif

#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    UART_PINS_PD6,
    UART_PINS_PD9,

    UART_PINS_PE9,
    UART_PINS_PE11,
#endif

#endif

#endif

    UART_PINS_RXNONE,

} Uart_RxPins;

/**
 * List of all Tx pins.
 */
typedef enum _Uart_TxPins
{
#if defined (LIBOHIBOARD_STM32L0x2)

#if defined (LIBOHIBOARD_STM32L072)

    UART_PINS_PA0_TX,
    UART_PINS_PA2,
    UART_PINS_PA9,
    UART_PINS_PA14,

#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PB3_TX,
#endif
    UART_PINS_PB6,

#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PB10_TX,
    UART_PINS_PB11_TX,
#endif

#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PC1,
#endif

#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PC4,
    UART_PINS_PC10,
    UART_PINS_PC12,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PD5,
    UART_PINS_PD8,

    UART_PINS_PE8,
    UART_PINS_PE10,
#endif

#endif

#elif defined (LIBOHIBOARD_STM32L0x3)

#if defined (LIBOHIBOARD_STM32L073)

    UART_PINS_PA0,
    UART_PINS_PA2,
    UART_PINS_PA9,
    UART_PINS_PA14,

    UART_PINS_PB3,
    UART_PINS_PB6,
    UART_PINS_PB10_TX,
    UART_PINS_PB11_TX,

#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    UART_PINS_PC1,
    UART_PINS_PC4,
    UART_PINS_PC10,
    UART_PINS_PC12,
#endif

#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    UART_PINS_PD5,
    UART_PINS_PD8,

    UART_PINS_PE8,
    UART_PINS_PE10,
#endif

#endif

#endif

    UART_PINS_TXNONE,

} Uart_TxPins;

/**
 * List of all Cts pins.
 */
typedef enum _Uart_CtsPins
{
#if defined (LIBOHIBOARD_STM32L0x2)

#if defined (LIBOHIBOARD_STM32L072)

    UART_PINS_PA0_CTS,
    UART_PINS_PA6_CTS,
    UART_PINS_PA11,

    UART_PINS_PB4_CTS,
    UART_PINS_PB7_CTS,
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PB13,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PD3,
    UART_PINS_PD11,
#endif

#endif

#endif

    UART_PINS_CTSNONE,

} Uart_CtsPins;

/**
 * List of all Rts pins.
 */
typedef enum _Uart_RtsPins
{
#if defined (LIBOHIBOARD_STM32L0x2)

#if defined (LIBOHIBOARD_STM32L072)

    UART_PINS_PA1_RTS,
    UART_PINS_PA12,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PA15_RTS,
#endif

    UART_PINS_PB1,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PB3_RTS,
#endif
    UART_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PB12,
    UART_PINS_PB14,
#endif

#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PD2,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    UART_PINS_PD4,
    UART_PINS_PD12,
    UART_PINS_PE7,
#endif

#endif

#endif

    UART_PINS_RTSNONE,

} Uart_RtsPins;

#if defined (LIBOHIBOARD_STM32L0x3)

void LPUART1_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);

extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;
extern Uart_DeviceHandle OB_UART3;
extern Uart_DeviceHandle OB_UART4;
extern Uart_DeviceHandle OB_UART5;
extern Uart_DeviceHandle OB_LPUART1;

#elif defined (LIBOHIBOARD_STM32L0x2)

void RNG_LPUART1_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART4_5_IRQHandler(void);

extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;
extern Uart_DeviceHandle OB_UART4;
extern Uart_DeviceHandle OB_UART5;
extern Uart_DeviceHandle OB_LPUART1;

#endif

#endif // LIBOHIBOARD_UART & LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // __SPI_STM32L0_H
