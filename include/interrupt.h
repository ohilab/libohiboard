/* Copyright (C) 2012-2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Marco Contigiani <m.contigiani86@gmail.com>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
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
 * @file libohiboard/include/interrupt.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Marco Contigiani <m.contigiani86@gmail.com>
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @brief Interrupt definitions.
 */

#include "platforms.h"
#include "errors.h"
#include "types.h"

#ifndef __INTERRUPT_H
#define __INTERRUPT_H

typedef enum {
    INTERRUPT_ENABLE_OFF,
    INTERRUPT_ENABLE_ON,
} Interrupt_Status;

typedef enum {

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

    INTERRUPT_FTFA       = 5,
    INTERRUPT_PMC        = 6,
    INTERRUPT_LLWU       = 7,
    INTERRUPT_IIC0       = 8,
    INTERRUPT_SPI0       = 10,
    INTERRUPT_LPUART0    = 12,
    INTERRUPT_ADC0       = 15,
    INTERRUPT_CMP0       = 16,
    INTERRUPT_TPM0       = 17,
    INTERRUPT_TPM1       = 18,
    INTERRUPT_RTC_ALARM  = 20,
    INTERRUPT_RTC_SECOND = 21,
    INTERRUPT_LPTPM0     = 28,
    INTERRUPT_PORTA      = 30,
    INTERRUPT_PORTD      = 31,

#elif defined(LIBOHIBOARD_KL15Z4)

    INTERRUPT_DMA0       = 0,
    INTERRUPT_DMA1       = 1,
    INTERRUPT_DMA2       = 2,
    INTERRUPT_DMA3       = 3,
    INTERRUPT_FTFA       = 5,
    INTERRUPT_PMC        = 6,
    INTERRUPT_LLWU       = 7,
    INTERRUPT_IIC0       = 8,
    INTERRUPT_IIC1       = 9,
    INTERRUPT_SPI0       = 10,
    INTERRUPT_SPI1       = 11,
    INTERRUPT_UART0      = 12,
    INTERRUPT_UART1      = 13,
    INTERRUPT_UART2      = 14,
    INTERRUPT_ADC0       = 15,
    INTERRUPT_CMP0       = 16,
    INTERRUPT_TPM0       = 17,
    INTERRUPT_TPM1       = 18,
    INTERRUPT_TPM2       = 19,
    INTERRUPT_RTC_ALARM  = 20,
    INTERRUPT_RTC_SECOND = 21,
    INTERRUPT_PIT        = 22,
    INTERRUPT_DAC0       = 25,
    INTERRUPT_TSI0       = 26,
    INTERRUPT_MCG        = 27,
    INTERRUPT_LPTMR0     = 28,
    INTERRUPT_PORTA      = 30,
    INTERRUPT_PORTD      = 31,

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    INTERRUPT_DMA0       = 0,
    INTERRUPT_DMA1       = 1,
    INTERRUPT_DMA2       = 2,
    INTERRUPT_DMA3       = 3,
    INTERRUPT_FTFA       = 5,
    INTERRUPT_PMC        = 6,
    INTERRUPT_LLWU       = 7,
    INTERRUPT_IIC0       = 8,
    INTERRUPT_IIC1       = 9,
    INTERRUPT_SPI0       = 10,
    INTERRUPT_SPI1       = 11,
    INTERRUPT_UART0      = 12,
    INTERRUPT_UART1      = 13,
    INTERRUPT_UART2      = 14,
    INTERRUPT_ADC0       = 15,
    INTERRUPT_CMP0       = 16,
    INTERRUPT_TPM0       = 17,
    INTERRUPT_TPM1       = 18,
    INTERRUPT_TPM2       = 19,
    INTERRUPT_RTC_ALARM  = 20,
    INTERRUPT_RTC_SECOND = 21,
    INTERRUPT_PIT        = 22,
    INTERRUPT_USBOTG     = 24,
    INTERRUPT_DAC0       = 25,
    INTERRUPT_TSI0       = 26,
    INTERRUPT_MCG        = 27,
    INTERRUPT_LPTMR0     = 28,
    INTERRUPT_PORTA      = 30,
    INTERRUPT_PORTD      = 31,

#elif defined (LIBOHIBOARD_K10D10) || \
      defined(LIBOHIBOARD_K10D10)

    INTERRUPT_DMA0       = 0,
    INTERRUPT_DMA1       = 1,
    INTERRUPT_DMA2       = 2,
    INTERRUPT_DMA3       = 3,
    INTERRUPT_DMA4       = 4,
    INTERRUPT_DMA5       = 5,
    INTERRUPT_DMA6       = 6,
    INTERRUPT_DMA7       = 7,
    INTERRUPT_DMA8       = 8,
    INTERRUPT_DMA9       = 9,
    INTERRUPT_DMA10      = 10,
    INTERRUPT_DMA11      = 11,
    INTERRUPT_DMA12      = 12,
    INTERRUPT_DMA13      = 13,
    INTERRUPT_DMA14      = 14,
    INTERRUPT_DMA15      = 15,
    INTERRUPT_FTM0       = 62,
    INTERRUPT_FTM1       = 63,
    INTERRUPT_FTM2       = 64,
    INTERRUPT_RTC_ALARM  = 66,
    INTERRUPT_RTC_SECOND = 67,

#elif defined (LIBOHIBOARD_K60DZ10) || \
	  defined (LIBOHIBOARD_OHIBOARD_R1)

    INTERRUPT_DMA0       = 0,
    INTERRUPT_DMA1       = 1,
    INTERRUPT_DMA2       = 2,
    INTERRUPT_DMA3       = 3,
    INTERRUPT_DMA4       = 4,
    INTERRUPT_DMA5       = 5,
    INTERRUPT_DMA6       = 6,
    INTERRUPT_DMA7       = 7,
    INTERRUPT_DMA8       = 8,
    INTERRUPT_DMA9       = 9,
    INTERRUPT_DMA10      = 10,
    INTERRUPT_DMA11      = 11,
    INTERRUPT_DMA12      = 12,
    INTERRUPT_DMA13      = 13,
    INTERRUPT_DMA14      = 14,
    INTERRUPT_DMA15      = 15,
    INTERRUPT_FTM0       = 62,
    INTERRUPT_FTM1       = 63,
    INTERRUPT_FTM2       = 64,
    INTERRUPT_RTC_ALARM  = 66,
    INTERRUPT_RTC_SECOND = 67,

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

    INTERRUPT_DMA0         = 0,
    INTERRUPT_DMA1         = 1,
    INTERRUPT_DMA2         = 2,
    INTERRUPT_DMA3         = 3,
    INTERRUPT_DMA4         = 4,
    INTERRUPT_DMA5         = 5,
    INTERRUPT_DMA6         = 6,
    INTERRUPT_DMA7         = 7,
    INTERRUPT_DMA8         = 8,
    INTERRUPT_DMA9         = 9,
    INTERRUPT_DMA10        = 10,
    INTERRUPT_DMA11        = 11,
    INTERRUPT_DMA12        = 12,
    INTERRUPT_DMA13        = 13,
    INTERRUPT_DMA14        = 14,
    INTERRUPT_DMA15        = 15,
    INTERRUPT_FTM0         = 42,
    INTERRUPT_FTM1         = 43,
    INTERRUPT_FTM2         = 44,
    INTERRUPT_RTC_ALARM    = 46,
    INTERRUPT_RTC_SECOND   = 47,
    INTERRUPT_PIT0         = 48,
    INTERRUPT_PIT1         = 49,
    INTERRUPT_PIT2         = 50,
    INTERRUPT_PIT3         = 51,
    INTERRUPT_PDB          = 52,
    INTERRUPT_PORTA		   = 59,
    INTERRUPT_PORTB		   = 60,
    INTERRUPT_PORTC		   = 61,
    INTERRUPT_PORTD		   = 62,
    INTERRUPT_PORTE		   = 63,
    INTERRUPT_FTM3         = 71,
    INTERRUPT_ETHERNET_1588= 82,
    INTERRUPT_ETHERNET_TX  = 83,
    INTERRUPT_ETHERNET_RX  = 84,
    INTERRUPT_ETHERNET_ERR = 85,

#endif
} Interrupt_Vector;

System_Errors Interrupt_enable (Interrupt_Vector vectorNumber);
System_Errors Interrupt_disable (Interrupt_Vector vectorNumber);

#endif /* __INTERRUPT_H */


