/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/PIC24FJ/interrupt_PIC24FJ.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief INTERRUPT vector definitions for PIC24FJ series
 */

#ifndef __INTERRUPT_PIC24FJ_H
#define __INTERRUPT_PIC24FJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_PIC24FJ)
    
#define INTERRUPT_IS_VALID_PRIORITY(PRIORITY)  ((PRIORITY >= 0x00u) && (PRIORITY <= 7))

typedef enum  _Interrupt_Vector
{
    INTERRUPT_INT0             = 0,
    INTERRUPT_INPUT_CAPTURE_1  = 1,
    INTERRUPT_OUTPUT_COMPARE_1 = 2,
    INTERRUPT_TIMER1           = 3,
    INTERRUPT_DMA0             = 4,
    INTERRUPT_INPUT_CAPTURE_2  = 5,
    INTERRUPT_OUTPUT_COMPARE_2 = 6,
    INTERRUPT_TIMER2           = 7,
    INTERRUPT_TIMER3           = 8,
    INTERRUPT_SPI1             = 9,
    INTERRUPT_SPI1_TX          = 10,
    INTERRUPT_UART1_RX         = 11,
    INTERRUPT_UART1_TX         = 12,
    INTERRUPT_ADC1             = 13,
    INTERRUPT_DMA1             = 14,
    INTERRUPT_NVM              = 15,
    INTERRUPT_I2C1_SLAVE       = 16,
    INTERRUPT_I2C1_MASTER      = 17,
    INTERRUPT_COMP             = 18,
    INTERRUPT_IOC              = 19,
    INTERRUPT_INT1             = 20,

    INTERRUPT_CCP5             = 22,
    INTERRUPT_CCP6             = 23,
    INTERRUPT_DMA2             = 24,
    INTERRUPT_OUTPUT_COMPARE_3 = 25,
    INTERRUPT_OUTPUT_COMPARE_4 = 26,
    INTERRUPT_TIMER4           = 27,
    INTERRUPT_TIMER5           = 28,
    INTERRUPT_INT2             = 29,
    INTERRUPT_UART2_RX         = 30,
    INTERRUPT_UART2_TX         = 31,
    INTERRUPT_SPI2             = 32,
    INTERRUPT_SPI2_TX          = 33,

    INTERRUPT_DMA3             = 36,
    INTERRUPT_INPUT_CAPTURE_3  = 37,
    INTERRUPT_INPUT_CAPTURE_4  = 38,
    INTERRUPT_INPUT_CAPTURE_5  = 39,
    INTERRUPT_INPUT_CAPTURE_6  = 40,
    INTERRUPT_OUTPUT_COMPARE_5 = 41,
    INTERRUPT_OUTPUT_COMPARE_6 = 42,
    INTERRUPT_CCT3             = 43,
    INTERRUPT_CCT4             = 44,
    INTERRUPT_PMP              = 45,
    INTERRUPT_DMA4             = 46,
    INTERRUPT_CCT5             = 47,
    INTERRUPT_CCT6             = 48,
    INTERRUPT_I2C2_SLAVE       = 49,
    INTERRUPT_I2C2_MASTER      = 50,
    INTERRUPT_CCT7             = 51,

    INTERRUPT_INT3             = 53,
    INTERRUPT_INT4             = 54,

    INTERRUPT_SPI1_RX          = 58,
    INTERRUPT_SPI2_RX          = 59,
    INTERRUPT_SPI3_RX          = 60,
    INTERRUPT_DMA5             = 61,
    INTERRUPT_RTCC             = 62,
    INTERRUPT_CCP1             = 63,
    INTERRUPT_CCP2             = 64,
    INTERRUPT_UART1_ERROR      = 65,
    INTERRUPT_UART2_ERROR      = 66,
    INTERRUPT_CRC              = 67,
    INTERRUPT_DMA6             = 68,
    INTERRUPT_DMA7             = 69,
    INTERRUPT_I2C3_SLAVE       = 70,
    INTERRUPT_I2C3_MASTER      = 71,
    INTERRUPT_HLVD             = 72,
    INTERRUPT_CCP7             = 73,

    INTERRUPT_CTMU             = 77,

    INTERRUPT_UART3_ERROR      = 81,
    INTERRUPT_UART3_RX         = 82,
    INTERRUPT_UART3_TX         = 83,
    INTERRUPT_I2C1_COLLISION   = 84,
    INTERRUPT_I2C2_COLLISION   = 85,
    INTERRUPT_USB1             = 86,
    INTERRUPT_UART4_ERROR      = 87,
    INTERRUPT_UART4_RX         = 88,
    INTERRUPT_UART4_TX         = 89,
    INTERRUPT_SPI3             = 90,
    INTERRUPT_SPI3_TX          = 91,

    INTERRUPT_CCP3             = 94,
    INTERRUPT_CCP4             = 95,
    INTERRUPT_CLC1             = 96,
    INTERRUPT_CLC2             = 97,
    INTERRUPT_CLC3             = 98,
    INTERRUPT_CLC4             = 99,

    INTERRUPT_CCT1             = 101,
    INTERRUPT_CCT2             = 102,
            
    INTERRUPT_FST_FRC          = 106,

    INTERRUPT_I2C3_COLLISION   = 109,
    INTERRUPT_RTCC_TS          = 110,
    INTERRUPT_UART5_RX         = 111,
    INTERRUPT_UART5_TX         = 112,
    INTERRUPT_UART5_ERROR      = 113,
    INTERRUPT_UART6_RX         = 114,
    INTERRUPT_UART6_TX         = 115,
    INTERRUPT_UART6_ERROR      = 116,
    INTERRUPT_JTAG             = 117,

} Interrupt_Vector;

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // __INTERRUPT_PIC24FJ_H
