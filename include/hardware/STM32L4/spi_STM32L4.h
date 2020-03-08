/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/STM32L4/spi_STM32L4.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief SPI pins and device definitions for STM32L4 series
 */

#ifndef __SPI_STM32L4_H
#define __SPI_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_SPI) & defined(LIBOHIBOARD_STM32L4)

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
#if defined (LIBOHIBOARD_STM32L476)

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

    SPI_PINS_PA4,
    SPI_PINS_PA15,

    SPI_PINS_PB9,
    SPI_PINS_PB12,

#if defined (LIBOHIBOARD_STM32L476Jx)
    SPI_PINS_PG12,
#endif

#endif

#endif // LIBOHIBOARD_STM32L476

    SPI_PINS_PCSNONE,

} Spi_PcsPins;

/**
 * List of all MOSI pins.
 */
typedef enum _Spi_SoutPins
{
#if defined (LIBOHIBOARD_STM32L476)

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

    SPI_PINS_PA7,

    SPI_PINS_PB5,
    SPI_PINS_PB15,

    SPI_PINS_PC3,
    SPI_PINS_PC12,

#if defined (LIBOHIBOARD_STM32L476Jx)
    SPI_PINS_PG11,
#endif

#endif

#endif // LIBOHIBOARD_STM32L476

    SPI_PINS_SOUTNONE,

} Spi_SoutPins;

/**
 * List of all MISO pins.
 */
typedef enum _Spi_SinPins
{
#if defined (LIBOHIBOARD_STM32L476)

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

    SPI_PINS_PA6,

    SPI_PINS_PB4,
    SPI_PINS_PB14,

    SPI_PINS_PC2,
    SPI_PINS_PC11,

#if defined (LIBOHIBOARD_STM32L476Jx)
    SPI_PINS_PG10,
#endif

#endif

#endif // LIBOHIBOARD_STM32L476

    SPI_PINS_SINNONE,

} Spi_SinPins;

/**
 * List of all SCK pins.
 */
typedef enum _Spi_SckPins
{
#if defined (LIBOHIBOARD_STM32L476)

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

    SPI_PINS_PA5,

    SPI_PINS_PB3,
    SPI_PINS_PB10,
    SPI_PINS_PB13,

    SPI_PINS_PC10,

#if defined (LIBOHIBOARD_STM32L476Jx)
    SPI_PINS_PG9,
#endif

#endif

#endif // LIBOHIBOARD_STM32L476

    SPI_PINS_SCKNONE,

} Spi_SckPins;

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

extern Spi_DeviceHandle OB_SPI1;
extern Spi_DeviceHandle OB_SPI2;
extern Spi_DeviceHandle OB_SPI3;

#endif // LIBOHIBOARD_STM32L476Jx || LIBOHIBOARD_STM32L476Rx

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_SPI & LIBOHIBOARD_STM32L4

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __SPI_STM32L4_H
