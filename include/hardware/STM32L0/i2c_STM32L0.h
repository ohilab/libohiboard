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
 * @file libohiboard/include/hardware/STM32L0/i2c_STM32L0.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C pins and device definitions for STM32L0 series
 */

#ifndef __I2C_STM32L0_H
#define __I2C_STM32L0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#if defined(LIBOHIBOARD_IIC) & defined(LIBOHIBOARD_STM32L0)

/**
 * @addtogroup I2C
 * @{
 */

/**
 * @defgroup I2C_Hardware I2C specific hardware types
 * @{
 */

#if defined (LIBOHIBOARD_STM32L073)

//#define RCC_CCIPR_I2C1SEL_Pos        (12u)
//#define RCC_CCIPR_I2C3SEL_Pos        (16u)
//
//#define I2C_CR1_NOSTRETCH_Msk        (I2C_CR1_NOSTRETCH)
//
//#define I2C_CR2_NBYTES_Pos           (16u)
//#define I2C_CR2_NBYTES_Msk           (I2C_CR2_NBYTES)
//#define I2C_CR2_SADD_Pos             (0u)
//#define I2C_CR2_SADD_Msk             (I2C_CR2_SADD)
//#define I2C_CR2_RELOAD_Msk           (I2C_CR2_RELOAD)
//#define I2C_CR2_AUTOEND_Msk          (I2C_CR2_AUTOEND)
//#define I2C_CR2_START_Msk            (I2C_CR2_START)
//#define I2C_CR2_STOP_Msk             (I2C_CR2_STOP)
//#define I2C_CR2_RD_WRN_Msk           (I2C_CR2_RD_WRN)
//#define I2C_CR2_ADD10_Msk            (I2C_CR2_ADD10)
//#define I2C_CR2_NACK_Msk             (I2C_CR2_NACK)
//
//#define I2C_TIMINGR_PRESC_Pos        (28u)
//#define I2C_TIMINGR_SCLH_Pos         (8u)
//#define I2C_TIMINGR_SCLL_Pos         (0u)
//
//#define I2C_OAR1_OA1EN_Msk           (I2C_OAR1_OA1EN)
//#define I2C_OAR1_OA1MODE_Msk         (I2C_OAR1_OA1MODE)
//
//#define I2C_OAR2_OA2EN_Msk           (I2C_OAR2_OA2EN)
//#define I2C_OAR2_OA2MSK_Pos          (8u)
//
//#define I2C_ICR_NACKCF_Msk           (I2C_ICR_NACKCF)
//#define I2C_ICR_STOPCF_Msk           (I2C_ICR_STOPCF)

//#define I2C_ISR_TXE_Msk              (I2C_ISR_TXE)

#endif

/**
 * List of all SCL pins.
 */
typedef enum _Iic_SclPins
{
#if defined (LIBOHIBOARD_STM32L0x1)
    IIC_PINS_PA8,
    IIC_PINS_PA9,

    IIC_PINS_PB6,
    IIC_PINS_PB8,
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    IIC_PINS_PB10,
    IIC_PINS_PB13,
#endif

#elif defined (LIBOHIBOARD_STM32L0x3)
#if defined (LIBOHIBOARD_STM32L073)

    IIC_PINS_PA8,
    IIC_PINS_PA9,

    IIC_PINS_PB6,
    IIC_PINS_PB8,
    IIC_PINS_PB10,
    IIC_PINS_PB13,

#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    IIC_PINS_PC0,
#endif

#endif // LIBOHIBOARD_STM32L073
#endif // LIBOHIBOARD_STM32L0x3

    IIC_PINS_SCLNONE,

} Iic_SclPins;

/**
 * List of all SDA pins.
 */
typedef enum _Iic_SdaPins
{
#if defined (LIBOHIBOARD_STM32L0x1)
    IIC_PINS_PA10,

    IIC_PINS_PB4,
    IIC_PINS_PB7,
#if defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
    IIC_PINS_PB9,
    IIC_PINS_PB11,
    IIC_PINS_PB14,
#endif

#elif defined (LIBOHIBOARD_STM32L0x3)

#if defined (LIBOHIBOARD_STM32L073)

    IIC_PINS_PA10,

    IIC_PINS_PB4,
    IIC_PINS_PB7,
    IIC_PINS_PB9,
    IIC_PINS_PB11,
    IIC_PINS_PB14,

#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
    IIC_PINS_PC1,
    IIC_PINS_PC9,
#endif

#endif // LIBOHIBOARD_STM32L073
#endif // LIBOHIBOARD_STM32L0x3

    IIC_PINS_SDANONE,

} Iic_SdaPins;


extern Iic_DeviceHandle OB_IIC1;
#if !defined (LIBOHIBOARD_STM32L081KxT) && \
    !defined (LIBOHIBOARD_STM32L081KxU)
extern Iic_DeviceHandle OB_IIC2;
#endif
extern Iic_DeviceHandle OB_IIC3;

/**
 * @}
 */

/**
 * @}
 */

#endif // LIBOHIBOARD_IIC & LIBOHIBOARD_STM32L0

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __I2C_STM32L0_H
