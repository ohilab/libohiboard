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
 * @file libohiboard/include/hardware/PIC24FJ/spi_PIC24FJ.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief SPI pins and device definitions for PIC24FJ series
 */

#ifndef __SPI_PIC24FJ_H
#define __SPI_PIC24FJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_SPI) && defined(LIBOHIBOARD_PIC24FJ)

/**
 * @addtogroup SPI
 * @{
 */

/**
 * @defgroup SPI_Hardware SPI specific hardware types
 * @{
 */

/**
 * List of all SS pins.
 */
typedef enum _Spi_PcsPins
{

    SPI_PINS_PCSNONE,

} Spi_PcsPins;

/**
 * List of all MOSI pins.
 */
typedef enum _Spi_SoutPins
{
    SPI_PINS_SOUTNONE  = -1,
    
    SPI_PINS_SOUT_RP0  = 0,  /**< RB0 */
    SPI_PINS_SOUT_RP1  = 1,  /**< RB1 */
    SPI_PINS_SOUT_RP2  = 2,  /**< RD8 */    
    SPI_PINS_SOUT_RP3  = 3,  /**< RD10 */
    SPI_PINS_SOUT_RP4  = 4,  /**< RD9 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SOUT_RP5  = 5,  /**< RD15 */        
#endif
    SPI_PINS_SOUT_RP6  = 6,  /**< RB6 */
    SPI_PINS_SOUT_RP7  = 7,  /**< RB7 */
    SPI_PINS_SOUT_RP8  = 8,  /**< RB8 */
    SPI_PINS_SOUT_RP9  = 9,  /**< RB9 */
    SPI_PINS_SOUT_RP10 = 10, /**< RF4 */
    SPI_PINS_SOUT_RP11 = 11, /**< RD0 */
    SPI_PINS_SOUT_RP12 = 12, /**< RD11 */
    SPI_PINS_SOUT_RP13 = 13, /**< RB2 */
    SPI_PINS_SOUT_RP14 = 14, /**< RB14 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SOUT_RP15 = 15, /**< RF8 */        
#endif
    SPI_PINS_SOUT_RP16 = 16, /**< RF3 */
    SPI_PINS_SOUT_RP17 = 17, /**< RF5 */
    SPI_PINS_SOUT_RP18 = 18, /**< RB5 */
    SPI_PINS_SOUT_RP19 = 19, /**< RG8 */
    SPI_PINS_SOUT_RP20 = 20, /**< RD5 */
    SPI_PINS_SOUT_RP21 = 21, /**< RG6 */
    SPI_PINS_SOUT_RP22 = 22, /**< RD3 */
    SPI_PINS_SOUT_RP23 = 23, /**< RD2 */
    SPI_PINS_SOUT_RP24 = 24, /**< RD1 */
    SPI_PINS_SOUT_RP25 = 25, /**< RD4 */
    SPI_PINS_SOUT_RP26 = 26, /**< RG7 */ 
    SPI_PINS_SOUT_RP27 = 27, /**< RG9 */
    SPI_PINS_SOUT_RP28 = 28, /**< RB4 */
    SPI_PINS_SOUT_RP29 = 29, /**< RB15 */
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SOUT_RP30 = 30, /**< RF2 */
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SOUT_RP31 = 31, /**< RF13 */        
#endif

} Spi_SoutPins;

/**
 * List of all MISO pins.
 */
typedef enum _Spi_SinPins
{
    SPI_PINS_SINNONE  = -1,

    SPI_PINS_SIN_RP0  = 0,  /**< RB0 */
    SPI_PINS_SIN_RP1  = 1,  /**< RB1 */
    SPI_PINS_SIN_RP2  = 2,  /**< RD8 */    
    SPI_PINS_SIN_RP3  = 3,  /**< RD10 */
    SPI_PINS_SIN_RP4  = 4,  /**< RD9 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SIN_RP5  = 5,  /**< RD15 */        
#endif
    SPI_PINS_SIN_RP6  = 6,  /**< RB6 */
    SPI_PINS_SIN_RP7  = 7,  /**< RB7 */
    SPI_PINS_SIN_RP8  = 8,  /**< RB8 */
    SPI_PINS_SIN_RP9  = 9,  /**< RB9 */
    SPI_PINS_SIN_RP10 = 10, /**< RF4 */
    SPI_PINS_SIN_RP11 = 11, /**< RD0 */
    SPI_PINS_SIN_RP12 = 12, /**< RD11 */
    SPI_PINS_SIN_RP13 = 13, /**< RB2 */
    SPI_PINS_SIN_RP14 = 14, /**< RB14 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SIN_RP15 = 15, /**< RF8 */        
#endif
    SPI_PINS_SIN_RP16 = 16, /**< RF3 */
    SPI_PINS_SIN_RP17 = 17, /**< RF5 */
    SPI_PINS_SIN_RP18 = 18, /**< RB5 */
    SPI_PINS_SIN_RP19 = 19, /**< RG8 */
    SPI_PINS_SIN_RP20 = 20, /**< RD5 */
    SPI_PINS_SIN_RP21 = 21, /**< RG6 */
    SPI_PINS_SIN_RP22 = 22, /**< RD3 */
    SPI_PINS_SIN_RP23 = 23, /**< RD2 */
    SPI_PINS_SIN_RP24 = 24, /**< RD1 */
    SPI_PINS_SIN_RP25 = 25, /**< RD4 */
    SPI_PINS_SIN_RP26 = 26, /**< RG7 */ 
    SPI_PINS_SIN_RP27 = 27, /**< RG9 */
    SPI_PINS_SIN_RP28 = 28, /**< RB4 */
    SPI_PINS_SIN_RP29 = 29, /**< RB15 */
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SIN_RP30 = 30, /**< RF2 */
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SIN_RP31 = 31, /**< RF13 */        
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SIN_RP32 = 32, /**< RF12 */
    SPI_PINS_SIN_RP33 = 33, /**< RE8 */
    SPI_PINS_SIN_RP34 = 34, /**< RE9 */
    SPI_PINS_SIN_RP35 = 35, /**< RA15 */
    SPI_PINS_SIN_RP36 = 36, /**< RA16 */
#endif
    SPI_PINS_SIN_RP37 = 37, /**< RC14 */            
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SIN_RP38 = 38, /**< RC1 */
    SPI_PINS_SIN_RP39 = 39, /**< RC2 */
    SPI_PINS_SIN_RP40 = 40, /**< RC3 */
    SPI_PINS_SIN_RP41 = 41, /**< RC4 */
    SPI_PINS_SIN_RP42 = 42, /**< RD12 */
    SPI_PINS_SIN_RP43 = 43, /**< RD14 */
#endif

} Spi_SinPins;

/**
 * List of all SCK pins.
 */
typedef enum _Spi_SckPins
{
    SPI_PINS_SCKNONE  = -1,

    SPI_PINS_SCK_RP0  = 0,  /**< RB0 */
    SPI_PINS_SCK_RP1  = 1,  /**< RB1 */
    SPI_PINS_SCK_RP2  = 2,  /**< RD8 */    
    SPI_PINS_SCK_RP3  = 3,  /**< RD10 */
    SPI_PINS_SCK_RP4  = 4,  /**< RD9 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SCK_RP5  = 5,  /**< RD15 */        
#endif
    SPI_PINS_SCK_RP6  = 6,  /**< RB6 */
    SPI_PINS_SCK_RP7  = 7,  /**< RB7 */
    SPI_PINS_SCK_RP8  = 8,  /**< RB8 */
    SPI_PINS_SCK_RP9  = 9,  /**< RB9 */
    SPI_PINS_SCK_RP10 = 10, /**< RF4 */
    SPI_PINS_SCK_RP11 = 11, /**< RD0 */
    SPI_PINS_SCK_RP12 = 12, /**< RD11 */
    SPI_PINS_SCK_RP13 = 13, /**< RB2 */
    SPI_PINS_SCK_RP14 = 14, /**< RB14 */
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SCK_RP15 = 15, /**< RF8 */        
#endif
    SPI_PINS_SCK_RP16 = 16, /**< RF3 */
    SPI_PINS_SCK_RP17 = 17, /**< RF5 */
    SPI_PINS_SCK_RP18 = 18, /**< RB5 */
    SPI_PINS_SCK_RP19 = 19, /**< RG8 */
    SPI_PINS_SCK_RP20 = 20, /**< RD5 */
    SPI_PINS_SCK_RP21 = 21, /**< RG6 */
    SPI_PINS_SCK_RP22 = 22, /**< RD3 */
    SPI_PINS_SCK_RP23 = 23, /**< RD2 */
    SPI_PINS_SCK_RP24 = 24, /**< RD1 */
    SPI_PINS_SCK_RP25 = 25, /**< RD4 */
    SPI_PINS_SCK_RP26 = 26, /**< RG7 */ 
    SPI_PINS_SCK_RP27 = 27, /**< RG9 */
    SPI_PINS_SCK_RP28 = 28, /**< RB4 */
    SPI_PINS_SCK_RP29 = 29, /**< RB15 */
#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SCK_RP30 = 30, /**< RF2 */
#endif
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
    SPI_PINS_SCK_RP31 = 31, /**< RF13 */        
#endif

} Spi_SckPins;

#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)

extern Spi_DeviceHandle OB_SPI1;
extern Spi_DeviceHandle OB_SPI2;
extern Spi_DeviceHandle OB_SPI3;

#endif

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_SPI & LIBOHIBOARD_PIC24FJ

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __SPI_PIC24FJ_H
