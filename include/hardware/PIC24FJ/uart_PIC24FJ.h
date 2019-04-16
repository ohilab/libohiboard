/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 *   Marco Giammarini
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
 * @file libohiboard/include/hardware/PIC24fJ/uart_PIC24fJ.h
 * @author Leonardo Morichelli
 * @brief UART pins and device definitions for PIC24fJ series
 */

#ifndef __UART_PIC24FJ_H
#define __UART_PIC24FJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_UART) && defined(LIBOHIBOARD_PIC24FJ)

/**
 * @addtogroup UART
 * @{
 */

/**
 * @defgroup UART_Hardware UART specific hardware types
 * @{
 */

/**
 * List of all TX pins.
 */
typedef enum _Uart_txPins
{
    UART_PINS_TXNONE  = -1,
    
    UART_PINS_TX_RP0  = 0,  /**< RB0 */
    UART_PINS_TX_RP1  = 1,  /**< RB1 */
    UART_PINS_TX_RP2  = 2,  /**< RD8 */    
    UART_PINS_TX_RP3  = 3,  /**< RD10 */
    UART_PINS_TX_RP4  = 4,  /**< RD9 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    UART_PINS_TX_RP5  = 5,  /**< RD15 */        
#endif
    UART_PINS_TX_RP6  = 6,  /**< RB6 */
    UART_PINS_TX_RP7  = 7,  /**< RB7 */
    UART_PINS_TX_RP8  = 8,  /**< RB8 */
    UART_PINS_TX_RP9  = 9,  /**< RB9 */
    UART_PINS_TX_RP10 = 10, /**< RF4 */
    UART_PINS_TX_RP11 = 11, /**< RD0 */
    UART_PINS_TX_RP12 = 12, /**< RD11 */
    UART_PINS_TX_RP13 = 13, /**< RB2 */
    UART_PINS_TX_RP14 = 14, /**< RB14 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    UART_PINS_TX_RP15 = 15, /**< RF8 */        
#endif
    UART_PINS_TX_RP16 = 16, /**< RF3 */
    UART_PINS_TX_RP17 = 17, /**< RF5 */
    UART_PINS_TX_RP18 = 18, /**< RB5 */
    UART_PINS_TX_RP19 = 19, /**< RG8 */
    UART_PINS_TX_RP20 = 20, /**< RD5 */
    UART_PINS_TX_RP21 = 21, /**< RG6 */
    UART_PINS_TX_RP22 = 22, /**< RD3 */
    UART_PINS_TX_RP23 = 23, /**< RD2 */
    UART_PINS_TX_RP24 = 24, /**< RD1 */
    UART_PINS_TX_RP25 = 25, /**< RD4 */
    UART_PINS_TX_RP26 = 26, /**< RG7 */ 
    UART_PINS_TX_RP27 = 27, /**< RG9 */
    UART_PINS_TX_RP28 = 28, /**< RB4 */
    UART_PINS_TX_RP29 = 29, /**< RB15 */
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    UART_PINS_TX_RP30 = 30, /**< RF2 */
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    UART_PINS_TX_RP31 = 31, /**< RF13 */        
#endif
        
    UART_PINS_TX_FIXED,
            
} Uart_TxPins;

/**
 * List of all Uart pins.
 */
typedef enum _Uart_RxPins
{
    UART_PINS_RXNONE  = -1,
    
    UART_PINS_RX_RP0  = 0,  /**< RB0 */
    UART_PINS_RX_RP1  = 1,  /**< RB1 */
    UART_PINS_RX_RP2  = 2,  /**< RD8 */    
    UART_PINS_RX_RP3  = 3,  /**< RD10 */
    UART_PINS_RX_RP4  = 4,  /**< RD9 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    UART_PINS_RX_RP5  = 5,  /**< RD15 */        
#endif
    UART_PINS_RX_RP6  = 6,  /**< RB6 */
    UART_PINS_RX_RP7  = 7,  /**< RB7 */
    UART_PINS_RX_RP8  = 8,  /**< RB8 */
    UART_PINS_RX_RP9  = 9,  /**< RB9 */
    UART_PINS_RX_RP10 = 10, /**< RF4 */
    UART_PINS_RX_RP11 = 11, /**< RD0 */
    UART_PINS_RX_RP12 = 12, /**< RD11 */
    UART_PINS_RX_RP13 = 13, /**< RB2 */
    UART_PINS_RX_RP14 = 14, /**< RB14 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    UART_PINS_RX_RP15 = 15, /**< RF8 */        
#endif
    UART_PINS_RX_RP16 = 16, /**< RF3 */
    UART_PINS_RX_RP17 = 17, /**< RF5 */
    UART_PINS_RX_RP18 = 18, /**< RB5 */
    UART_PINS_RX_RP19 = 19, /**< RG8 */
    UART_PINS_RX_RP20 = 20, /**< RD5 */
    UART_PINS_RX_RP21 = 21, /**< RG6 */
    UART_PINS_RX_RP22 = 22, /**< RD3 */
    UART_PINS_RX_RP23 = 23, /**< RD2 */
    UART_PINS_RX_RP24 = 24, /**< RD1 */
    UART_PINS_RX_RP25 = 25, /**< RD4 */
    UART_PINS_RX_RP26 = 26, /**< RG7 */ 
    UART_PINS_RX_RP27 = 27, /**< RG9 */
    UART_PINS_RX_RP28 = 28, /**< RB4 */
    UART_PINS_RX_RP29 = 29, /**< RB15 */
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    UART_PINS_RX_RP30 = 30, /**< RF2 */
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    UART_PINS_RX_RP31 = 31, /**< RF13 */        
#endif
        
    UART_PINS_RX_FIXED,

} Uart_RxPins;

#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)

extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;
extern Uart_DeviceHandle OB_UART3;
extern Uart_DeviceHandle OB_UART4;
extern Uart_DeviceHandle OB_UART5;
extern Uart_DeviceHandle OB_UART6;

#endif

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_UART & LIBOHIBOARD_PIC24FJ

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __UART_PIC24FJ_H
