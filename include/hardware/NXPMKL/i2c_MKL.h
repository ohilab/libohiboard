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
 * @file libohiboard/include/hardware/NXPMKL/i2c_MKL.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C pins and device definitions for NXP MKL series
 */

#ifndef __I2C_MKL_H
#define __I2C_MKL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_IIC) & defined(LIBOHIBOARD_MKL)

/**
 * @addtogroup I2C
 * @{
 */

/**
 * @defgroup I2C_Hardware I2C specific hardware types
 * @{
 */


/**
 * List of all SCL pins.
 */
typedef enum _Iic_SclPins
{
    IIC_PINS_PTA3,

    IIC_PINS_PTB0,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    IIC_PINS_PTB2,
#endif

    IIC_PINS_PTC1,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    IIC_PINS_PTC8,
    IIC_PINS_PTC10,
#endif

#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    IIC_PINS_PTE1,
#endif
    IIC_PINS_PTE19,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    IIC_PINS_PTE24,
#endif

    IIC_PINS_SCLNONE,

} Iic_SclPins;

/**
 * List of all SDA pins.
 */
typedef enum _Iic_SdaPins
{
    IIC_PINS_PTA4,

    IIC_PINS_PTB1,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    IIC_PINS_PTB3,
#endif

    IIC_PINS_PTC2,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    IIC_PINS_PTC9,
    IIC_PINS_PTC11,
#endif

#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    IIC_PINS_PTE0,
#endif
    IIC_PINS_PTE18,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
    IIC_PINS_PTE25,
#endif

    IIC_PINS_SDANONE,

} Iic_SdaPins;


#if defined (LIBOHIBOARD_MKL15)

extern Iic_DeviceHandle OB_IIC0;
extern Iic_DeviceHandle OB_IIC1;

#endif

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_IIC & LIBOHIBOARD_MKL

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __I2C_MKL_H
