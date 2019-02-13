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
 * @file libohiboard/include/hardware/STM32L4/i2c_STM32L4.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C pins and device definitions for STM32L4 series
 */

#ifndef __I2C_STM32L4_H
#define __I2C_STM32L4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_IIC) & defined(LIBOHIBOARD_STM32L4)

typedef enum _Iic_SclPins
{
#if defined (LIBOHIBOARD_STM32L476)

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)


    IIC_PINS_PB6,
    IIC_PINS_PB8,
    IIC_PINS_PB10,
    IIC_PINS_PB13,

    IIC_PINS_PC0,

#if defined (LIBOHIBOARD_STM32L476Jx)
    IIC_PINS_PG14,
#endif

#endif

#endif // LIBOHIBOARD_STM32L476

    IIC_PINS_SCLNONE,

} Iic_SclPins;

typedef enum _Iic_SdaPins
{
#if defined (LIBOHIBOARD_STM32L476)

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

    IIC_PINS_PB7,
    IIC_PINS_PB9,
    IIC_PINS_PB11,
    IIC_PINS_PB14,

    IIC_PINS_PC1,

#if defined (LIBOHIBOARD_STM32L476Jx)
    IIC_PINS_PG13,
#endif

#endif

#endif // LIBOHIBOARD_STM32L476

    IIC_PINS_SDANONE,

} Iic_SdaPins;

// WLCSP72 ballout
// LQFP64
#if defined (LIBOHIBOARD_STM32L476Jx) || \
    defined (LIBOHIBOARD_STM32L476Rx)

extern Iic_DeviceHandle OB_IIC1;
extern Iic_DeviceHandle OB_IIC2;
extern Iic_DeviceHandle OB_IIC3;

#endif // LIBOHIBOARD_STM32L476Jx || LIBOHIBOARD_STM32L476Rx

#endif // LIBOHIBOARD_IIC & LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // __I2C_STM32L4_H
