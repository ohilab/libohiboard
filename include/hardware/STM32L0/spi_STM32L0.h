/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019-2021 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/hardware/STM32L0/spi_STM32L0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief SPI pins and device definitions for STM32L0 series
 */

#ifndef __SPI_STM32L0_H
#define __SPI_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_SPI) & defined(LIBOHIBOARD_STM32L0)

/**
 * @addtogroup SPI
 * @{
 */

/**
 * @defgroup SPI_Hardware SPI specific hardware types
 * @{
 */

#define SPI_CR1_BR_Pos                      (3u)

/**
 * List of all SS pins.
 */
typedef enum _Spi_PcsPins
{
#if defined (LIBOHIBOARD_STM32L0x1)

    SPI_PINS_PA4,
#if defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    SPI_PINS_PA15,
#endif

#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    SPI_PINS_PB9,
    SPI_PINS_PB12,
#endif

#elif defined (LIBOHIBOARD_STM32L0x2)

    SPI_PINS_PA4,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    SPI_PINS_PA15,  // Internally connected into LIBOHIBOARD_CMWX1ZZABZ_091
#endif

#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    SPI_PINS_PB9,
    SPI_PINS_PB12,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    SPI_PINS_PD0,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    SPI_PINS_PE12,
#endif

#elif defined (LIBOHIBOARD_STM32L0x3)
#if defined (LIBOHIBOARD_STM32L073)

    SPI_PINS_PA4,
    SPI_PINS_PA15,

    SPI_PINS_PB9,
    SPI_PINS_PB12,

#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)

    SPI_PINS_PD0,

    SPI_PINS_PE12,
#endif

#endif
#endif // LIBOHIBOARD_STM32L0x3

    SPI_PINS_PCSNONE,

} Spi_PcsPins;

/**
 * List of all MOSI pins.
 */
typedef enum _Spi_SoutPins
{
#if defined (LIBOHIBOARD_STM32L0x1)

    SPI_PINS_PA7,
    SPI_PINS_PA12,

    SPI_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    SPI_PINS_PB15,
#endif

#elif defined (LIBOHIBOARD_STM32L0x2)

    SPI_PINS_PA7, // Internally connected into LIBOHIBOARD_CMWX1ZZABZ_091
    SPI_PINS_PA12,

    SPI_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    SPI_PINS_PB15,
#endif

#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    SPI_PINS_PC3,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    SPI_PINS_PD4,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    SPI_PINS_PE15,
#endif

#elif defined (LIBOHIBOARD_STM32L0x3)
#if defined (LIBOHIBOARD_STM32L073)

    SPI_PINS_PA7,
    SPI_PINS_PA12,

    SPI_PINS_PB5,
    SPI_PINS_PB15,

#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    SPI_PINS_PC3,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)

    SPI_PINS_PD4,

    SPI_PINS_PE15,
#endif

#endif
#endif
    SPI_PINS_SOUTNONE,

} Spi_SoutPins;

/**
 * List of all MISO pins.
 */
typedef enum _Spi_SinPins
{
#if defined (LIBOHIBOARD_STM32L0x1)

    SPI_PINS_PA6,
    SPI_PINS_PA11,

    SPI_PINS_PB4,
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    SPI_PINS_PB14,
#endif

#elif defined (LIBOHIBOARD_STM32L0x2)

    SPI_PINS_PA6,  // Internally connected into LIBOHIBOARD_CMWX1ZZABZ_091
    SPI_PINS_PA11,

    SPI_PINS_PB4,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    SPI_PINS_PB14,
#endif

#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    SPI_PINS_PC2,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    SPI_PINS_PD3,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    SPI_PINS_PE14,
#endif

#elif defined (LIBOHIBOARD_STM32L0x3)
#if defined (LIBOHIBOARD_STM32L073)

    SPI_PINS_PA6,
    SPI_PINS_PA11,

    SPI_PINS_PB4,
    SPI_PINS_PB14,

#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    SPI_PINS_PC2,
#endif

#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    SPI_PINS_PD3,

    SPI_PINS_PE14,
#endif

#endif
#endif // LIBOHIBOARD_STM32L0x3

    SPI_PINS_SINNONE,

} Spi_SinPins;

/**
 * List of all SCK pins.
 */
typedef enum _Spi_SckPins
{
#if defined (LIBOHIBOARD_STM32L0x1)

    SPI_PINS_PA5,

#if defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    SPI_PINS_PB3,
#endif
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    SPI_PINS_PB10,
    SPI_PINS_PB13,
#endif

#elif defined (LIBOHIBOARD_STM32L0x2)

    SPI_PINS_PA5,

#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
	defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    SPI_PINS_PB3, // Internally connected into LIBOHIBOARD_CMWX1ZZABZ_091
#endif
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
#if !defined (LIBOHIBOARD_CMWX1ZZABZ_091)
    SPI_PINS_PB10,
#endif
    SPI_PINS_PB13,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    SPI_PINS_PD1,
#endif

#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
    SPI_PINS_PE13,
#endif

#elif defined (LIBOHIBOARD_STM32L0x3)
#if defined (LIBOHIBOARD_STM32L073)

    SPI_PINS_PA5,

    SPI_PINS_PB3,
    SPI_PINS_PB10,
    SPI_PINS_PB13,

#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    SPI_PINS_PD1,

    SPI_PINS_PE13,
#endif

#endif
#endif // LIBOHIBOARD_STM32L0x3

    SPI_PINS_SCKNONE,

} Spi_SckPins;



extern Spi_DeviceHandle OB_SPI1;
#if defined (LIBOHIBOARD_STM32L073)    || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU) || \
    defined (LIBOHIBOARD_CMWX1ZZABZ_091)
extern Spi_DeviceHandle OB_SPI2;
#endif

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_SPI & LIBOHIBOARD_STM32L0

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __SPI_STM32L0_H
