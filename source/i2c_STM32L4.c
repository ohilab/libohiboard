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


#define IIC_MAX_PINS                      8
#define IIC_MAX_BAUDRATE                  1000000u
#define IIC_MAX_SCL_TICK                  256

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

#define IIC_VALID_DUALADDRESS(DUALADDRESS) (((DUALADDRESS) == IIC_DUALADDRESS_ENABLE)  || \
                                            ((DUALADDRESS) == IIC_DUALADDRESS_DISABLE))

#define IIC_VALID_DUALMASK(DUALMASK) (((DUALMASK) == IIC_DUALADDRESSMASK_NO_MASK)  || \
		                              ((DUALMASK) == IIC_DUALADDRESSMASK_MASK_01)  || \
                                      ((DUALMASK) == IIC_DUALADDRESSMASK_MASK_02)  || \
                                      ((DUALMASK) == IIC_DUALADDRESSMASK_MASK_03)  || \
                                      ((DUALMASK) == IIC_DUALADDRESSMASK_MASK_04)  || \
                                      ((DUALMASK) == IIC_DUALADDRESSMASK_MASK_05)  || \
                                      ((DUALMASK) == IIC_DUALADDRESSMASK_MASK_06)  || \
                                      ((DUALMASK) == IIC_DUALADDRESSMASK_MASK_07))

#define IIC_VALID_NOSTRETCH(STRETCH) (((STRETCH) == IIC_NOSTRETCH_ENABLE)  || \
                                      ((STRETCH) == IIC_NOSTRETCH_DISABLE))

#define IIC_VALID_OWN_ADDRESS1(ADDRESS1) ((ADDRESS1) <= 0x000003FFu)

#define IIC_VALID_OWN_ADDRESS2(ADDRESS2) ((ADDRESS2) <= (uint16_t)0x00FFu)

/**
 * @brief Check the baudrate value
 */
#define IIC_VALID_BAUDRATE(BAUDRATE) ((BAUDRATE) <= IIC_MAX_BAUDRATE)

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

    // Slave mode
    uint32_t address1;
    uint32_t address2;
    Iic_DualAddress dualAddressMode;
    Iic_DualAddressMask dualAddressMask;
    Iic_NoStrech noStretch;

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

static System_Errors Iic_setBaudrate (Iic_DeviceHandle dev, uint32_t baudrate)
{
    uint32_t frequency = 0;
    uint32_t scaled = 0;

    // Get current parent clock
    switch (dev->clockSource)
    {
    case IIC_CLOCKSOURCE_HSI:
        frequency = (uint32_t)CLOCK_FREQ_HSI;
        break;
    case IIC_CLOCKSOURCE_SYSCLK:
        frequency = Clock_getOutputClock(CLOCK_OUTPUT_SYSCLK);
        break;
    case IIC_CLOCKSOURCE_PCLK:
        frequency = Clock_getOutputClock(CLOCK_OUTPUT_PCLK2);
        break;
    default:
        ohiassert(0);
        return ERRORS_IIC_NO_CLOCKSOURCE;
    }

    // Current clock is different from 0
    if (frequency != 0u)
    {
        for (uint8_t i = 0; i < 16; ++i)
        {
            // Divide by possible prescaler
            scaled = frequency / (i + 1);

            // Check if frequency scaled is grater then baudrate
            if (scaled < baudrate)
            {
                // Exit from for...
                return ERRORS_IIC_WRONG_PARAM;
            }
            else
            {
                // Compute tick for each SCL level
                // 4 ticks is for TSYNC1 and TSYNC2
                uint16_t tick = (uint16_t)(((scaled / baudrate) - 4 ) / 2);
                // Save ticks if it is usable
                if (tick < IIC_MAX_SCL_TICK)
                {
                    dev->regmap->TIMINGR = (i << I2C_TIMINGR_PRESC_Pos)               |
                                           (((tick - 1) & 0x00FFu) << I2C_TIMINGR_SCLH_Pos) |
                                           (((tick - 1) & 0x00FFu) << I2C_TIMINGR_SCLL_Pos);
                    break;
                }
                else
                {
                    // Try with the next prescaler!
                    continue;
                }
            }
        }
    }
    else
    {
        return ERRORS_IIC_CLOCKSOURCE_FREQUENCY_TOO_LOW;
    }

    return ERRORS_NO_ERROR;
}

static System_Errors Iic_config (Iic_DeviceHandle dev, Iic_Config* config)
{
    System_Errors err = ERRORS_NO_ERROR;

    err |= ohiassert(IIC_VALID_MODE(config->devType));
    err |= ohiassert(IIC_VALID_ADDRESSMODE(config->addressMode));
    err |= ohiassert(IIC_VALID_OWN_ADDRESS1(config->address1));
    err |= ohiassert(IIC_VALID_OWN_ADDRESS2(config->address2));
    err |= ohiassert(IIC_VALID_DUALADDRESS(config->dualAddressMode));
    err |= ohiassert(IIC_VALID_DUALMASK(config->dualAddressMask));
    err |= ohiassert(IIC_VALID_NOSTRETCH(config->noStretch));
    err |= ohiassert(IIC_VALID_BAUDRATE(config->baudRate));
    if (err != ERRORS_NO_ERROR)
        return ERRORS_IIC_WRONG_PARAM;

    // Disable the device
    IIC_DEVICE_DISABLE(dev->regmap);

    // Configure Baudrate
    err = Iic_setBaudrate(dev,config->baudRate);
    if (err != ERRORS_NO_ERROR)
        return ERRORS_IIC_WRONG_PARAM;


    // Save address mode (7-bit or 10-bit)
    dev->addressMode = config->addressMode;
    dev->regmap->CR2 = dev->regmap->CR2 & (~(I2C_CR2_ADD10_Msk));
    if (config->addressMode == IIC_TEN_BIT)
    {
        dev->regmap->CR2 |= I2C_CR2_ADD10_Msk;
    }

    // Enable by default AUTOEND and NACK (NOT usable in master mode)
    dev->regmap->CR2 |= (I2C_CR2_AUTOEND_Msk | I2C_CR2_NACK_Msk);

    // Disable own address1 before set new one
    dev->regmap->OAR1 = dev->regmap->OAR1 & (~(I2C_OAR1_OA1EN_Msk));
    dev->address1 = config->address1;
    // Configure own address 1
    if (config->addressMode == IIC_SEVEN_BIT)
    {
        dev->regmap->OAR1 = I2C_OAR1_OA1EN_Msk | ((config->address1 << 1) & 0x000000FEu);
    }
    else
    {
        dev->regmap->OAR1 = I2C_OAR1_OA1EN_Msk | I2C_OAR1_OA1MODE_Msk | config->address1;
    }

    // Disable own address 2 and check the user config
    dev->regmap->OAR2 = dev->regmap->OAR2 & (~(I2C_OAR2_OA2EN_Msk));
    dev->address2 = config->address2;
    dev->dualAddressMode = config->dualAddressMode;
    dev->dualAddressMask = config->dualAddressMask;
    if (config->dualAddressMode == IIC_DUALADDRESS_ENABLE)
    {
        dev->regmap->OAR2 = I2C_OAR2_OA2EN_Msk | (((uint8_t)config->dualAddressMask) << I2C_OAR2_OA2MSK_Pos) | config->address2;
    }

    // Configure no-stretch mode
    dev->regmap->CR1 = dev->regmap->CR1 & (~(I2C_CR1_NOSTRETCH_Msk));
    dev->noStretch = config->noStretch;
    if (config->noStretch == IIC_NOSTRETCH_ENABLE)
    {
        dev->regmap->CR1 |= I2C_CR1_NOSTRETCH_Msk;
    }

    // Configuration ended... enable device
    IIC_DEVICE_ENABLE(dev->regmap);
    return ERRORS_NO_ERROR;
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
    err = ohiassert(IIC_VALID_CLOCK_SOURCE(config->clockSource));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_PARAM;
    }
    dev->clockSource = config->clockSource;

    // Select clock source
    UTILITY_MODIFY_REGISTER(*dev->rccTypeRegisterPtr,dev->rccTypeRegisterMask,(config->clockSource << dev->rccTypeRegisterPos));
    // Enable peripheral clock
    IIC_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);


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
