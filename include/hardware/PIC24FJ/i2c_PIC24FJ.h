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
 * @file libohiboard/include/hardware/PIC24FJ/i2c_PIC24FJ.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C pins and device definitions for PIC24FJ series
 */

#ifndef __I2C_PIC24FJ_H
#define __I2C_PIC24FJ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined(LIBOHIBOARD_IIC) && defined(LIBOHIBOARD_PIC24FJ)

typedef enum _Iic_SclPins
{
    IIC_PINS_SCLNONE,

} Iic_SclPins;

typedef enum _Iic_SdaPins
{
    IIC_PINS_SDANONE,

} Iic_SdaPins;

#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)

extern Iic_DeviceHandle OB_IIC1;
extern Iic_DeviceHandle OB_IIC2;
extern Iic_DeviceHandle OB_IIC3;

#endif

#endif // LIBOHIBOARD_IIC & LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // __I2C_PIC24FJ_H
