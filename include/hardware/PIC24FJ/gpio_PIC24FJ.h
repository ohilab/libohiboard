/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
 *   Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/include/hardware/PIC24FJ/gpio_PIC24FJ.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief GPIO pins and device definitions for PIC24FJ series
 */

#ifndef __GPIO_PIC24FJ_H
#define __GPIO_PIC24FJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_PIC24FJ)

/**
 * @addtogroup GPIO
 * @{
 */

/**
 * @defgroup GPIO_Hardware GPIO specific hardware types
 * @{
 */

/**
 * List of possible GPIO pins.
 */
typedef enum _Gpio_Pins
{
    GPIO_PINS_NONE = 0,

    //PORT A
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PA0,
    GPIO_PINS_PA1,
    GPIO_PINS_PA2,
    GPIO_PINS_PA3,
    GPIO_PINS_PA4,
    GPIO_PINS_PA5,
    GPIO_PINS_PA6,
    GPIO_PINS_PA7,
    GPIO_PINS_PA9,
    GPIO_PINS_PA10,
    GPIO_PINS_PA14,
    GPIO_PINS_PA15,
#endif

    //PORT B
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PB0,
    GPIO_PINS_PB1,
    GPIO_PINS_PB2,
    GPIO_PINS_PB3,
    GPIO_PINS_PB4,
    GPIO_PINS_PB5,
    GPIO_PINS_PB6,
    GPIO_PINS_PB7,
    GPIO_PINS_PB8,
    GPIO_PINS_PB9,
    GPIO_PINS_PB10,
    GPIO_PINS_PB11,
    GPIO_PINS_PB12,
    GPIO_PINS_PB13,
    GPIO_PINS_PB14,
    GPIO_PINS_PB15,
#endif

    //PORT C
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PC1,
    GPIO_PINS_PC2,
    GPIO_PINS_PC3,
    GPIO_PINS_PC4,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PC12,
    GPIO_PINS_PC13,
    GPIO_PINS_PC14,
    GPIO_PINS_PC15,
#endif

    //PORT D
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PD0,
    GPIO_PINS_PD1,
    GPIO_PINS_PD2,
    GPIO_PINS_PD3,
    GPIO_PINS_PD4,
    GPIO_PINS_PD5,
    GPIO_PINS_PD6,
    GPIO_PINS_PD7,
    GPIO_PINS_PD8,
    GPIO_PINS_PD9,
    GPIO_PINS_PD10,
    GPIO_PINS_PD11,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PD12,
    GPIO_PINS_PD13,
    GPIO_PINS_PD14,
    GPIO_PINS_PD15,
#endif

    //PORT E
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PE0,
    GPIO_PINS_PE1,
    GPIO_PINS_PE2,
    GPIO_PINS_PE3,
    GPIO_PINS_PE4,
    GPIO_PINS_PE5,
    GPIO_PINS_PE6,
    GPIO_PINS_PE7,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PE8,
    GPIO_PINS_PE9,
#endif

    //PORT F
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PF0,
    GPIO_PINS_PF1,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PF2,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PF3,
    GPIO_PINS_PF4,
    GPIO_PINS_PF5,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610)
    GPIO_PINS_PF6,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PF7,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PF8,
    GPIO_PINS_PF12,
    GPIO_PINS_PF13,
#endif

    //PORT G
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PG0,
    GPIO_PINS_PG1,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PG2,
    GPIO_PINS_PG3,
    GPIO_PINS_PG6,
    GPIO_PINS_PG7,
    GPIO_PINS_PG8,
    GPIO_PINS_PG9,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PINS_PG12,
    GPIO_PINS_PG13,
    GPIO_PINS_PG14,
    GPIO_PINS_PG15,
#endif

    GPIO_PINS_NUMBER,
} Gpio_Pins;

/**
 * List of possible GPIO ports.
 */
typedef enum _Gpio_Ports
{
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PORTS_A,
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    GPIO_PORTS_B,
    GPIO_PORTS_C,
    GPIO_PORTS_D,
    GPIO_PORTS_E,
    GPIO_PORTS_F,
    GPIO_PORTS_G,
#endif

    GPIO_PORTS_NUMBER,
} Gpio_Ports;

/**
 * List of possible PPS output functionality.
 */
typedef enum _Gpio_PpsOutputFunction
{
    GPIO_PPSOUTPUTFUNCTION_NULL    = 0u,   /**< None (Pin Disabled) */
    GPIO_PPSOUTPUTFUNCTION_C1OUT   = 1u,   /**< Comparator 1 Output */
    GPIO_PPSOUTPUTFUNCTION_C2OUT   = 2u,   /**< Comparator 2 Output */
    GPIO_PPSOUTPUTFUNCTION_U1TX    = 3u,   /**< UART1 Transmit */
    GPIO_PPSOUTPUTFUNCTION_U1RTS   = 4u,   /**< UART1 Request-to-Send */
    GPIO_PPSOUTPUTFUNCTION_U2TX    = 5u,   /**< UART2 Transmit */
    GPIO_PPSOUTPUTFUNCTION_U2RTS   = 6u,   /**< UART2 Request-to-Send */
    GPIO_PPSOUTPUTFUNCTION_SDO1    = 7u,   /**< SPI1 Data Output */
    GPIO_PPSOUTPUTFUNCTION_SCK1OUT = 8u,   /**< SPI1 Clock Output */
    GPIO_PPSOUTPUTFUNCTION_SS1OUT  = 9u,   /**< SPI1 Slave Select Output */
    GPIO_PPSOUTPUTFUNCTION_SDO2    = 10u,  /**< SPI2 Data Output */
    GPIO_PPSOUTPUTFUNCTION_SCK2OUT = 11u,  /**< SPI2 Clock Output */
    GPIO_PPSOUTPUTFUNCTION_SS2OUT  = 12u,  /**< SPI2 Slave Select Output */
    GPIO_PPSOUTPUTFUNCTION_OC1     = 13u,  /**< Output Compare 1 */
    GPIO_PPSOUTPUTFUNCTION_OC2     = 14u,  /**< Output Compare 2 */
    GPIO_PPSOUTPUTFUNCTION_OC3     = 15u,  /**< Output Compare 3 */
    GPIO_PPSOUTPUTFUNCTION_OCM4    = 16u,  /**< CCP4 Output Compare */
    GPIO_PPSOUTPUTFUNCTION_OCM5    = 17u,  /**< CCP5 Output Compare */
    GPIO_PPSOUTPUTFUNCTION_OCM6    = 18u,  /**< CCP6 Output Compare */
    GPIO_PPSOUTPUTFUNCTION_U3TX    = 19u,  /**< UART3 Transmit */
    GPIO_PPSOUTPUTFUNCTION_U3RTS   = 20u,  /**< UART3 Request-to-Send */
    GPIO_PPSOUTPUTFUNCTION_U4TX    = 21u,  /**< UART4 Transmit */
    GPIO_PPSOUTPUTFUNCTION_U4RTS   = 22u,  /**< UART4 Request-to-Send */
    GPIO_PPSOUTPUTFUNCTION_SDO3    = 23u,  /**< SPI3 Data Output */
    GPIO_PPSOUTPUTFUNCTION_SCK3OUT = 24u,  /**< SPI3 Clock Output */
    GPIO_PPSOUTPUTFUNCTION_SS3OUT  = 25u,  /**< SPI3 Slave Select Output */
    GPIO_PPSOUTPUTFUNCTION_C3OUT   = 26u,  /**< Comparator 3 Output */
    GPIO_PPSOUTPUTFUNCTION_OCM7    = 27u,  /**< CCP7 Output Compare */
    GPIO_PPSOUTPUTFUNCTION_REFO    = 28u,  /**< Reference Clock Output */
    GPIO_PPSOUTPUTFUNCTION_CLC1OUT = 29u,  /**< CLC1 Output */
    GPIO_PPSOUTPUTFUNCTION_CLC2OUT = 30u,  /**< CLC2 Output */
    GPIO_PPSOUTPUTFUNCTION_RTCC    = 31u,  /**< RTCC Output */

} Gpio_PpsOutputFunction;

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_PIC24FJ

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __GPIO_PIC24FJ_H
