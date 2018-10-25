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
 * @file libohiboard/source/i2c_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C implementations for STM32L4 series.
 */

#ifdef LIBOHIBOARD_IIC

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_STM32L4)

#include "i2c.h"

#include "utility.h"
#include "gpio.h"
#include "clock.h"


#define IIC_MAX_PINS           8

#define IIC_CLOCK_ENABLE(REG,MASK) do { \
                                     UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                     asm("nop"); \
                                     (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                   } while (0)

/**
 * @brief Enable the I2C peripheral
 */
#define IIC_DEVICE_ENABLE(REGMAP)        (REGMAP->CR1 |= I2C_CR1_PE)
/**
 * @brief Disable the I2C peripheral
 */
#define IIC_DEVICE_DISABLE(REGMAP)       (REGMAP->CR1 &= ~I2C_CR1_PE)

#define IIC_VALID_CLOCK_SOURCE(CLOCKSOURCE) (((CLOCKSOURCE) == IIC_CLOCKSOURCE_PCLK)  || \
                                             ((CLOCKSOURCE) == IIC_CLOCKSOURCE_HSI)   || \
                                             ((CLOCKSOURCE) == IIC_CLOCKSOURCE_SYSCLK))

#define IIC_VALID_ADDRESSMODE(ADDRESSMODE) (((ADDRESSMODE) == IIC_SEVEN_BIT)  || \
                                            ((ADDRESSMODE) == IIC_TEN_BIT))

#define IIC_VALID_MODE(MODE) (((MODE) == IIC_MASTER_MODE) || \
                              ((MODE) == IIC_SLAVE_MODE))

#if defined (LIBOHIBOARD_STM32L476Jx) // WLCSP72 ballout

#define IIC_IS_DEVICE(DEVICE) (((DEVICE) == OB_IIC1)  || \
                               ((DEVICE) == OB_IIC2)  || \
                               ((DEVICE) == OB_IIC3))

#endif // LIBOHIBOARD_STM32L476Jx - WLCSP72 ballout


typedef struct _Iic_Device
{
    I2C_TypeDef* regmap;                           /**< Device memory pointer */

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    volatile uint32_t* rccTypeRegisterPtr;  /**< Register for clock enabling. */
    uint32_t rccTypeRegisterMask;      /**< Register mask for user selection. */
    uint32_t rccTypeRegisterPos;       /**< Mask position for user selection. */

    Iic_SclPins sclPins[IIC_MAX_PINS];
    Iic_SdaPins sdaPins[IIC_MAX_PINS];

    Gpio_Pins sclPinsGpio[IIC_MAX_PINS];
    Gpio_Pins sdaPinsGpio[IIC_MAX_PINS];
    Gpio_Alternate sclPinsMux[IIC_MAX_PINS];
    Gpio_Alternate sdaPinsMux[IIC_MAX_PINS];

    Iic_ClockSource clockSource;
    Iic_AddressMode addressMode;
    Iic_DeviceType deviceType;

//    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Iic_Device;

#if defined (LIBOHIBOARD_STM32L476Jx) // WLCSP72 ballout

static Iic_Device iic1 =
{
        .regmap              = I2C1,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_I2C1EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_I2C1SEL,
        .rccTypeRegisterPos  = RCC_CCIPR_I2C1SEL_Pos,
};
Iic_DeviceHandle OB_IIC1 = &iic1;

static Iic_Device iic2 =
{
        .regmap              = I2C2,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_I2C2EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_I2C2SEL,
        .rccTypeRegisterPos  = RCC_CCIPR_I2C2SEL_Pos,
};
Iic_DeviceHandle OB_IIC2 = &iic2;

static Iic_Device iic3 =
{
        .regmap              = I2C3,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_I2C3EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_I2C3SEL,
        .rccTypeRegisterPos  = RCC_CCIPR_I2C3SEL_Pos,
};
Iic_DeviceHandle OB_IIC3 = &iic3;

#endif // LIBOHIBOARD_STM32L476Jx

static System_Errors Iic_config (Iic_DeviceHandle dev, Iic_Config* config)
{
    System_Errors err = ERRORS_NO_ERROR;

    err |= ohiassert(IIC_VALID_MODE(config->devType));
    err |= ohiassert(IIC_VALID_ADDRESSMODE(config->addressMode));
}

System_Errors Iic_init (Iic_DeviceHandle dev, Iic_Config* config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the I2C device
    if (dev == NULL)
    {
        return ERRORS_IIC_NO_DEVICE;
    }
    // Check the SPI instance
    err = ohiassert(IIC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_DEVICE;
    }
    // Check clock source selections
    err = ohiassert(IIC_IS_VALID_CLOCK_SOURCE(dev,config->clockSource));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_PARAM;
    }
    dev->clockSource = config->clockSource;

    // Select clock source
    UTILITY_MODIFY_REGISTER(*dev->rccTypeRegisterPtr,dev->rccTypeRegisterMask,(config->clockSource << dev->rccTypeRegisterPos));
    // Enable peripheral clock
    IIC_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

    // Disable the device
    IIC_DEVICE_DISABLE(dev->regmap);

    // FIXME: define pins!!

    // Configure the peripheral
    err = Iic_config(dev,config);
    if (err != ERRORS_NO_ERROR)
    {
        // FIXME: Call deInit?
        return err;
    }

    return ERRORS_NO_ERROR;
}

System_Errors Iic_deInit (Iic_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the I2C device
    if (dev == NULL)
    {
        return ERRORS_IIC_NO_DEVICE;
    }
    // Check the SPI instance
    err = ohiassert(IIC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_DEVICE;
    }

    // Disable the device
    IIC_DEVICE_DISABLE(dev->regmap);

    return err;
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_IIC
