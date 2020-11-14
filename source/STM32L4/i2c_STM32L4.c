/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018-2020 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/STM32L4/i2c_STM32L4.c
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
#define IIC_MAX_SCL_TICK                  256u

#define IIC_MAX_NBYTE_SIZE                255u

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

#define IIC_VALID_REGISTERADDRESSSIZE(SIZE) (((SIZE) == IIC_REGISTERADDRESSSIZE_8BIT)  || \
                                             ((SIZE) == IIC_REGISTERADDRESSSIZE_16BIT))

#define IIC_VALID_MODE(MODE) (((MODE) == IIC_MASTER_MODE) || \
                              ((MODE) == IIC_SLAVE_MODE))

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

//    Iic_ClockSource clockSource;
//    Iic_AddressMode addressMode;
//    Iic_DeviceType deviceType;
//
//    // Slave mode
//    uint32_t address1;
//    uint32_t address2;
//    Iic_DualAddress dualAddressMode;
//    Iic_DualAddressMask dualAddressMask;
//    Iic_NoStrech noStretch;

    // Write/Read useful buffer and counter
    uint8_t* rdata;                      /**< Pointer to I2C reception buffer */
    const uint8_t* tdata;             /**< Pointer to I2C transmission buffer */
    uint16_t bufferSize;                        /**< I2C buffer transfer size */
    volatile uint16_t bufferCount;           /**< I2C buffer transfer counter */

    Iic_Config config;

    Iic_DeviceState state;                     /**< Current peripheral state. */

} Iic_Device;

#if defined (LIBOHIBOARD_STM32L4x6) ||\
    defined (LIBOHIBOARD_STM32WB55)

#define IIC_IS_DEVICE(DEVICE) (((DEVICE) == OB_IIC1)  || \
                               ((DEVICE) == OB_IIC2)  || \
                               ((DEVICE) == OB_IIC3))

static Iic_Device iic1 =
{
        .regmap               = I2C1,

        .rccRegisterPtr       = &RCC->APB1ENR1,
        .rccRegisterEnable    = RCC_APB1ENR1_I2C1EN,

        .rccTypeRegisterPtr   = &RCC->CCIPR,
        .rccTypeRegisterMask  = RCC_CCIPR_I2C1SEL,
        .rccTypeRegisterPos   = RCC_CCIPR_I2C1SEL_Pos,

        .sclPins              =
        {
                               IIC_PINS_PB6,
                               IIC_PINS_PB8,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               IIC_PINS_PG14,
#endif
        },
        .sclPinsGpio          =
        {
                               GPIO_PINS_PB6,
                               GPIO_PINS_PB8,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG14,
#endif
        },
        .sclPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_4,
#endif
        },

        .sdaPins              =
        {
                               IIC_PINS_PB7,
                               IIC_PINS_PB9,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               IIC_PINS_PG13,
#endif
        },
        .sdaPinsGpio          =
        {
                               GPIO_PINS_PB7,
                               GPIO_PINS_PB9,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG13,
#endif
        },
        .sdaPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_4,
#endif
        },

        .state                = IIC_DEVICESTATE_RESET,
};
Iic_DeviceHandle OB_IIC1 = &iic1;

static Iic_Device iic2 =
{
        .regmap               = I2C2,

        .rccRegisterPtr       = &RCC->APB1ENR1,
        .rccRegisterEnable    = RCC_APB1ENR1_I2C2EN,

        .rccTypeRegisterPtr   = &RCC->CCIPR,
        .rccTypeRegisterMask  = RCC_CCIPR_I2C2SEL,
        .rccTypeRegisterPos   = RCC_CCIPR_I2C2SEL_Pos,

        .sclPins              =
        {
                               IIC_PINS_PB10,
                               IIC_PINS_PB13,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               IIC_PINS_PF1,
#endif
        },
        .sclPinsGpio          =
        {
                               GPIO_PINS_PB10,
                               GPIO_PINS_PB13,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PF1,
#endif
        },
        .sclPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_4,
#endif
        },

        .sdaPins              =
        {
                               IIC_PINS_PB11,
                               IIC_PINS_PB14,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               IIC_PINS_PF0,
#endif
        },
        .sdaPinsGpio          =
        {
                               GPIO_PINS_PB11,
                               GPIO_PINS_PB14,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PF0,
#endif
        },
        .sdaPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_4,
#endif
        },

        .state                = IIC_DEVICESTATE_RESET,
};
Iic_DeviceHandle OB_IIC2 = &iic2;

static Iic_Device iic3 =
{
        .regmap               = I2C3,

        .rccRegisterPtr       = &RCC->APB1ENR1,
        .rccRegisterEnable    = RCC_APB1ENR1_I2C3EN,

        .rccTypeRegisterPtr   = &RCC->CCIPR,
        .rccTypeRegisterMask  = RCC_CCIPR_I2C3SEL,
        .rccTypeRegisterPos   = RCC_CCIPR_I2C3SEL_Pos,

        .sclPins              =
        {
                               IIC_PINS_PC0,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               IIC_PINS_PG7,
#endif
        },
        .sclPinsGpio          =
        {
                               GPIO_PINS_PC0,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG7,
#endif
        },
        .sclPinsMux           =
        {
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_4,
#endif
        },

        .sdaPins              =
        {
                               IIC_PINS_PC1,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               IIC_PINS_PG8,
#endif
        },
        .sdaPinsGpio          =
        {
                               GPIO_PINS_PC1,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG8,
#endif
        },
        .sdaPinsMux           =
        {
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_4,
#endif
        },

        .state                = IIC_DEVICESTATE_RESET,
};
Iic_DeviceHandle OB_IIC3 = &iic3;

#endif

#define IIC_PIN_CONFIGURATION             (GPIO_PINS_PULL                    | \
                                           GPIO_PINS_ENABLE_PULLUP           | \
                                           GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN)

static System_Errors Iic_setSdaPin(Iic_DeviceHandle dev, Iic_SdaPins sdaPin, bool pullupEnable)
{
    uint8_t devPinIndex;
    uint16_t configuration = (pullupEnable == TRUE) ? IIC_PIN_CONFIGURATION : GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN;

    for (devPinIndex = 0; devPinIndex < IIC_MAX_PINS; ++devPinIndex)
    {
        if (dev->sdaPins[devPinIndex] == sdaPin)
        {
            Gpio_configAlternate(dev->sdaPinsGpio[devPinIndex],
                                 dev->sdaPinsMux[devPinIndex],
                                 configuration);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_IIC_NO_PIN_FOUND;
}

static System_Errors Iic_setSclPin(Iic_DeviceHandle dev, Iic_SclPins sclPin, bool pullupEnable)
{
    uint8_t devPinIndex;
    uint16_t configuration = (pullupEnable == TRUE) ? IIC_PIN_CONFIGURATION : GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN;

    for (devPinIndex = 0; devPinIndex < IIC_MAX_PINS; ++devPinIndex)
    {
        if (dev->sclPins[devPinIndex] == sclPin)
        {
            Gpio_configAlternate(dev->sclPinsGpio[devPinIndex],
                                 dev->sclPinsMux[devPinIndex],
                                 configuration);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_IIC_NO_PIN_FOUND;
}

static System_Errors Iic_setBaudrate (Iic_DeviceHandle dev, uint32_t baudrate)
{
    uint32_t frequency = 0;
    uint32_t scaled = 0;

    if (baudrate > IIC_MAX_BAUDRATE)
    {
        return ERRORS_IIC_WRONG_BAUDRATE;
    }

    // Get current parent clock
    switch (dev->config.clockSource)
    {
    case IIC_CLOCKSOURCE_HSI:
        frequency = (uint32_t)CLOCK_FREQ_HSI;
        break;
    case IIC_CLOCKSOURCE_SYSCLK:
        frequency = Clock_getOutputValue(CLOCK_OUTPUT_SYSCLK);
        break;
    case IIC_CLOCKSOURCE_PCLK:
        frequency = Clock_getOutputValue(CLOCK_OUTPUT_PCLK2);
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
                uint16_t tick = (uint16_t)((scaled / baudrate) / 2);
                // Save ticks if it is usable
                if ((tick < IIC_MAX_SCL_TICK) && (tick > 4))
                {
                    dev->regmap->TIMINGR = (i << I2C_TIMINGR_PRESC_Pos)                     |
                                           (((tick - 1) & 0x00FFu) << I2C_TIMINGR_SCLH_Pos) |
                                           (((tick - 5) & 0x00FFu) << I2C_TIMINGR_SCLL_Pos);
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
    err |= ohiassert(IIC_VALID_BAUDRATE(config->baudrate));
    if (err != ERRORS_NO_ERROR)
        return ERRORS_IIC_WRONG_PARAM;

    // Save current configuration
    dev->config = *config;

    // Disable the device
    IIC_DEVICE_DISABLE(dev->regmap);

    // Configure Baudrate
    err = Iic_setBaudrate(dev,dev->config.baudrate);
    if (err != ERRORS_NO_ERROR)
        return ERRORS_IIC_WRONG_PARAM;

    // Save address mode (7-bit or 10-bit)
    dev->regmap->CR2 = dev->regmap->CR2 & (~(I2C_CR2_ADD10_Msk));
    if (dev->config.addressMode == IIC_TEN_BIT)
    {
        dev->regmap->CR2 |= I2C_CR2_ADD10_Msk;
    }

    // Enable by default AUTOEND and NACK (NOT usable in master mode)
    dev->regmap->CR2 |= (I2C_CR2_AUTOEND_Msk | I2C_CR2_NACK_Msk);

    // Disable own address1 before set new one
    dev->regmap->OAR1 = dev->regmap->OAR1 & (~(I2C_OAR1_OA1EN_Msk));
    // Configure own address 1
    if (dev->config.addressMode == IIC_SEVEN_BIT)
    {
        dev->regmap->OAR1 = I2C_OAR1_OA1EN_Msk | ((dev->config.address1 << 1) & 0x000000FEu);
    }
    else
    {
        dev->regmap->OAR1 = I2C_OAR1_OA1EN_Msk | I2C_OAR1_OA1MODE_Msk | dev->config.address1;
    }

    // Disable own address 2 and check the user config
    dev->regmap->OAR2 = dev->regmap->OAR2 & (~(I2C_OAR2_OA2EN_Msk));
    if (dev->config.dualAddressMode == IIC_DUALADDRESS_ENABLE)
    {
        dev->regmap->OAR2 = I2C_OAR2_OA2EN_Msk | (((uint8_t)dev->config.dualAddressMask) << I2C_OAR2_OA2MSK_Pos) | dev->config.address2;
    }

    // Configure no-stretch mode
    dev->regmap->CR1 = dev->regmap->CR1 & (~(I2C_CR1_NOSTRETCH_Msk));
    if (dev->config.noStretch == IIC_NOSTRETCH_ENABLE)
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
    // Check the I2C instance
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

    // Enable peripheral clock if needed
    if (dev->state == IIC_DEVICESTATE_RESET)
    {
        // Select clock source
        UTILITY_MODIFY_REGISTER(*dev->rccTypeRegisterPtr,dev->rccTypeRegisterMask,(config->clockSource << dev->rccTypeRegisterPos));
        // Enable peripheral clock
        IIC_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

        // Enable pins
        if (config->sclPin != IIC_PINS_SCLNONE)
            Iic_setSclPin(dev, config->sclPin, config->pullupEnable);

        if (config->sdaPin != IIC_PINS_SDANONE)
            Iic_setSdaPin(dev, config->sdaPin, config->pullupEnable);
    }
    dev->state = IIC_DEVICESTATE_BUSY;

    // Configure the peripheral
    err = Iic_config(dev,config);
    if (err != ERRORS_NO_ERROR)
    {
        Iic_deInit(dev);
        return err;
    }

    // Clear NACK flag
    dev->regmap->ICR |= I2C_ICR_NACKCF_Msk;

    // Clear STOPF flag
    dev->regmap->ICR |= I2C_ICR_STOPCF_Msk;

    dev->state = IIC_DEVICESTATE_READY;

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
    // Check the I2C instance
    err = ohiassert(IIC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_DEVICE;
    }
    dev->state = IIC_DEVICESTATE_BUSY;

    // Disable the device
    IIC_DEVICE_DISABLE(dev->regmap);

    dev->state = IIC_DEVICESTATE_RESET;
    return err;
}

#define IIC_TRANSFER_CONFIG_MASK          (I2C_CR2_SADD_Msk    | \
                                           I2C_CR2_NBYTES_Msk  | \
                                           I2C_CR2_RELOAD_Msk  | \
                                           I2C_CR2_AUTOEND_Msk | \
                                           I2C_CR2_START_Msk   | \
                                           I2C_CR2_STOP_Msk    | \
                                           I2C_CR2_RD_WRN_Msk)

static inline void __attribute__((always_inline)) Iic_flushTransmission (Iic_DeviceHandle dev)
{
    // Clear flag TXE
    if ((dev->regmap->ISR & I2C_ISR_TXE_Msk) == 0u)
    {
        dev->regmap->ISR |= I2C_ISR_TXE_Msk;
    }
}

static inline System_Errors __attribute__((always_inline)) Iic_waitUntilTXSI (Iic_DeviceHandle dev, uint32_t timeout)
{
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,I2C_ISR_TXIS) == 0)
    {
        // Check if received NACK
        if ((dev->regmap->ISR & I2C_ISR_NACKF) != 0)
        {
            return ERRORS_IIC_TX_ACK_NOT_RECEIVED;
        }

        if (System_currentTick() > timeout)
        {
            return ERRORS_IIC_TX_TIMEOUT;
        }
    }
    return ERRORS_NO_ERROR;
}

static inline System_Errors __attribute__((always_inline)) Iic_waitUntilRXNE (Iic_DeviceHandle dev, uint32_t timeout)
{
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,I2C_ISR_RXNE) == 0)
    {
        // Check if received NACK
        if ((dev->regmap->ISR & I2C_ISR_NACKF) != 0)
        {
            return ERRORS_IIC_RX_WRONG_EVENT;
        }

        // Check if received STOP
        if ((dev->regmap->ISR & I2C_ISR_STOPF) != 0)
        {
            // In case the RX buffer is full and there is available space
            // came back and read from the register
            if (((dev->regmap->ISR & I2C_ISR_RXNE) != 0) && (dev->bufferSize > 0))
            {
                return ERRORS_NO_ERROR;
            }
            else
            {
                return ERRORS_IIC_RX_WRONG_EVENT;
            }
        }

        if (System_currentTick() > timeout)
        {
            return ERRORS_IIC_RX_TIMEOUT;
        }
    }
    return ERRORS_NO_ERROR;
}

static inline System_Errors __attribute__((always_inline)) Iic_waitUntilClear (Iic_DeviceHandle dev,
                                                                               uint32_t flag,
                                                                               uint32_t timeout)
{
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,flag) == 0)
    {
        if (System_currentTick() > timeout)
        {
            return ERRORS_IIC_TIMEOUT;
        }
    }
    return ERRORS_NO_ERROR;
}

static inline System_Errors __attribute__((always_inline)) Iic_waitUntilSet (Iic_DeviceHandle dev,
                                                                             uint32_t flag,
                                                                             uint32_t timeout)
{
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,flag) == flag)
    {
        if (System_currentTick() > timeout)
        {
            return ERRORS_IIC_TIMEOUT;
        }
    }
    return ERRORS_NO_ERROR;
}

System_Errors Iic_writeMaster (Iic_DeviceHandle dev,
                               uint16_t address,
                               const uint8_t* data,
                               uint16_t length,
                               uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Check the I2C device
    if (dev == NULL)
    {
        return ERRORS_IIC_NO_DEVICE;
    }
    // Check the I2C instance
    if (ohiassert(IIC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_DEVICE;
    }

    // Check if the device is busy
    // Wait 25ms before enter into timeout!
    err = Iic_waitUntilSet(dev,I2C_ISR_BUSY,(System_currentTick() + 25u));
    if (err != ERRORS_NO_ERROR) goto i2cerror;

    dev->bufferCount = length;
    dev->tdata = data;

    // Set NBYTES to write and reload if length is grater then NBYTE max size
    if (dev->bufferCount > IIC_MAX_NBYTE_SIZE)
    {
        dev->bufferSize = IIC_MAX_NBYTE_SIZE;
        UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                (IIC_TRANSFER_CONFIG_MASK),                                                \
                                (((uint32_t)address          << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                (I2C_CR2_RELOAD_Msk)                                                     | \
                                (I2C_CR2_START_Msk));
    }
    else
    {
        dev->bufferSize = dev->bufferCount;
        UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                (IIC_TRANSFER_CONFIG_MASK),                                                \
                                (((uint32_t)address          << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                (I2C_CR2_AUTOEND_Msk)                                                    | \
                                (I2C_CR2_START_Msk));
    }


    // Start sending bytes
    while (dev->bufferCount > 0u)
    {
        err = Iic_waitUntilTXSI(dev,(System_currentTick() + timeout));
        if (err != ERRORS_NO_ERROR) goto i2cerror;

        // Write data to TXDR
        dev->regmap->TXDR = (*data++);
        dev->bufferSize--;
        dev->bufferCount--;

        // Check if new packet is request
        if ((dev->bufferCount != 0u) && (dev->bufferSize == 0u))
        {
            // Wait until transmission is complete
            err = Iic_waitUntilClear(dev,I2C_ISR_TCR,(System_currentTick() + timeout));
            if (err != ERRORS_NO_ERROR) goto i2cerror;

            // Configure new packet without start and stop bits
            // Set NBYTES to write and reload if length is grater then NBYTE max size
            if (dev->bufferCount > IIC_MAX_NBYTE_SIZE)
            {
                dev->bufferSize = IIC_MAX_NBYTE_SIZE;
                UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                        (IIC_TRANSFER_CONFIG_MASK),                                                \
                                        (((uint32_t)address          << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                        (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                        (I2C_CR2_RELOAD_Msk));
            }
            else
            {
                dev->bufferSize = dev->bufferSize;
                UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                        (IIC_TRANSFER_CONFIG_MASK),                                                \
                                        (((uint32_t)address          << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                        (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                        (I2C_CR2_AUTOEND_Msk));
            }
        }
    }

    // Wait until STOPF flag is set (with AUTOEND the STOP bit is generated automatically)
    err = Iic_waitUntilClear(dev,I2C_ISR_STOPF,(System_currentTick() + timeout));

    // Check the NACK status
    if ((dev->regmap->ISR & I2C_ISR_NACKF) != 0)
    {
        err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
    }

i2cerror:

    // Clear NACK flag
    dev->regmap->ICR |= I2C_ICR_NACKCF_Msk;

    // Clear STOPF flag
    dev->regmap->ICR |= I2C_ICR_STOPCF_Msk;

    // Flush transmission register
    Iic_flushTransmission(dev);

    // Clear configuration register
    UTILITY_MODIFY_REGISTER(dev->regmap->CR2,(IIC_TRANSFER_CONFIG_MASK),0u);

    return err;
}

System_Errors Iic_readMaster (Iic_DeviceHandle dev,
                              uint16_t address,
                              uint8_t* data,
                              uint8_t length,
                              uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;
    uint32_t tickStart = System_currentTick();
    // Check the I2C device
    if (dev == NULL)
    {
        return ERRORS_IIC_NO_DEVICE;
    }
    // Check the I2C instance
    err = ohiassert(IIC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_DEVICE;
    }

    // Check if the device is busy
    // Wait 25ms before enter into timeout!
    err = Iic_waitUntilSet(dev,I2C_ISR_BUSY,(System_currentTick() + 25u));
    if (err != ERRORS_NO_ERROR) goto i2cerror;

    dev->bufferCount = length;
    dev->rdata = data;

    // Set NBYTES to write and reload if length is grater then NBYTE max size
    if (dev->bufferCount > IIC_MAX_NBYTE_SIZE)
    {
        dev->bufferSize = IIC_MAX_NBYTE_SIZE;
        UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                (IIC_TRANSFER_CONFIG_MASK),                                                \
                                (((uint32_t)address          << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                (I2C_CR2_RELOAD_Msk)                                                     | \
                                (I2C_CR2_START_Msk)                                                      | \
                                (I2C_CR2_RD_WRN_Msk));
    }
    else
    {
        dev->bufferSize = dev->bufferCount;
        UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                (IIC_TRANSFER_CONFIG_MASK),                                                \
                                (((uint32_t)address          << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                (I2C_CR2_AUTOEND_Msk)                                                    | \
                                (I2C_CR2_START_Msk)                                                      | \
                                (I2C_CR2_RD_WRN_Msk));
    }

    // Start reading bytes
    while (dev->bufferCount > 0u)
    {
        // Wait until Receive Data Register Not Empty
        err = Iic_waitUntilRXNE(dev,tickStart+timeout);
        if (err != ERRORS_NO_ERROR) goto i2cerror;

        // Read data from RXDR
        (*dev->rdata++) = dev->regmap->RXDR;
        dev->bufferSize--;
        dev->bufferCount--;

        // Check if new packet is request
        if ((dev->bufferSize != 0u) && (dev->bufferCount == 0u))
        {
            // Wait until reception is complete
            err = Iic_waitUntilClear(dev,I2C_ISR_TCR,(tickStart+timeout));
            if (err != ERRORS_NO_ERROR) goto i2cerror;

            // Configure new packet without start and stop bits
            // Set NBYTES to read and reload if length is grater then NBYTE max size
            if (dev->bufferCount > IIC_MAX_NBYTE_SIZE)
            {
                dev->bufferSize = IIC_MAX_NBYTE_SIZE;
                UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                        (IIC_TRANSFER_CONFIG_MASK),                                                \
                                        (((uint32_t)address          << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                        (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                        (I2C_CR2_RELOAD_Msk));
            }
            else
            {
                dev->bufferSize = dev->bufferCount;
                UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                        (IIC_TRANSFER_CONFIG_MASK),                                                \
                                        (((uint32_t)address          << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                        (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                        (I2C_CR2_AUTOEND_Msk));
            }

        }
    }

    // Wait until STOPF flag is set (with AUTOEND the STOP bit is generated automatically)
    err = Iic_waitUntilClear(dev,I2C_ISR_STOPF,(tickStart+timeout));

    // Check the NACK status
    if ((dev->regmap->ISR & I2C_ISR_NACKF) != 0)
    {
        err = ERRORS_IIC_RX_WRONG_EVENT;
    }

i2cerror:

    // Clear NACK flag
    dev->regmap->ICR |= I2C_ICR_NACKCF_Msk;

    // Clear STOPF flag
    dev->regmap->ICR |= I2C_ICR_STOPCF_Msk;

    // Clear configuration register
    UTILITY_MODIFY_REGISTER(dev->regmap->CR2,(IIC_TRANSFER_CONFIG_MASK),0u);

    return err;
}

System_Errors Iic_writeRegister (Iic_DeviceHandle dev,
                                 uint16_t devAddress,
                                 uint16_t regAddress,
                                 Iic_RegisterAddressSize addressSize,
                                 const uint8_t* data,
                                 uint16_t length,
                                 uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the I2C device
    if (dev == NULL)
    {
        return ERRORS_IIC_NO_DEVICE;
    }
    // Check the I2C instance
    err = ohiassert(IIC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_DEVICE;
    }
    err = ohiassert(IIC_VALID_REGISTERADDRESSSIZE(addressSize));
    // Check the data buffer
    err = ohiassert(data != NULL);
    // Check the number of byte requested
    err = ohiassert(length != 0);
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_PARAM;
    }

    // Shift 7-bit address
    if (dev->config.addressMode == IIC_SEVEN_BIT)
    {
        devAddress = ((devAddress << 1u) & 0x00FFu);
    }

    // Check if the device is busy
    // Wait 25ms before enter into timeout!
    err = Iic_waitUntilSet(dev,I2C_ISR_BUSY,(System_currentTick() + 25u));
    if (err != ERRORS_NO_ERROR) goto i2cerror;

    // Init timeout
    uint32_t tickStart = System_currentTick();

    // Save transfer parameter
    dev->tdata = data;
    dev->bufferCount = length;

    // Setup write transmission, first step is send register address
    // Configure transfer
    UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                     \
                            (IIC_TRANSFER_CONFIG_MASK),                                           \
                            (((uint32_t)devAddress  << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                            (((uint32_t)addressSize << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                            (I2C_CR2_RELOAD_Msk)                                                | \
                            (I2C_CR2_START_Msk));
    // Check TXE status
    err = Iic_waitUntilClear(dev,I2C_ISR_TXE,(tickStart + timeout));
    if (err != ERRORS_NO_ERROR) goto i2cerror;

    // Write register/memory address
    if (addressSize == IIC_REGISTERADDRESSSIZE_8BIT)
    {
        dev->regmap->TXDR = (uint8_t)(regAddress & 0x00FFu);
    }
    else
    {
        // Write MSB part of address
        dev->regmap->TXDR = (uint8_t)((regAddress & 0xFF00u) >> 8u);
        err = Iic_waitUntilTXSI(dev,(tickStart+ timeout));
        if (err != ERRORS_NO_ERROR) goto i2cerror;
        // Write LSB part of address
        dev->regmap->TXDR = (uint8_t)(regAddress & 0x00FFu);
    }
    // Wait until the transmission ends
    Iic_waitUntilClear(dev,I2C_ISR_TCR,(tickStart + timeout));
    if (err != ERRORS_NO_ERROR) goto i2cerror;

    // Now write data to the selected register/memory
    // Set NBYTES to write and reload if length is grater then NBYTE max size
    if (dev->bufferCount > IIC_MAX_NBYTE_SIZE)
    {
        dev->bufferSize = IIC_MAX_NBYTE_SIZE;
        UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                (IIC_TRANSFER_CONFIG_MASK),                                                \
                                (((uint32_t)devAddress       << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                (I2C_CR2_RELOAD_Msk));
    }
    else
    {
        dev->bufferSize =  dev->bufferCount;
        UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                (IIC_TRANSFER_CONFIG_MASK),                                                \
                                (((uint32_t)devAddress       << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                (I2C_CR2_AUTOEND_Msk));
    }

    // Start writing bytes
    do
    {
        // Wait until transmission buffer is empty
        err = Iic_waitUntilTXSI(dev,(tickStart+ timeout));
        if (err != ERRORS_NO_ERROR) goto i2cerror;

        // Write data from RXDR
        dev->regmap->TXDR = (*dev->tdata++);
        dev->bufferSize--;
        dev->bufferCount--;

        // Check if new packet is request
        if ((dev->bufferCount != 0u) && (dev->bufferSize == 0u))
        {
            // Wait until reception is complete
            err = Iic_waitUntilClear(dev,I2C_ISR_TCR,(tickStart+timeout));
            if (err != ERRORS_NO_ERROR) goto i2cerror;

            // Configure new packet without start and stop bits
            // Set NBYTES to write and reload if length is grater then NBYTE max size
            if (dev->bufferCount > IIC_MAX_NBYTE_SIZE)
            {
                dev->bufferSize = IIC_MAX_NBYTE_SIZE;
                UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                        (IIC_TRANSFER_CONFIG_MASK),                                                \
                                        (((uint32_t)devAddress       << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                        (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                        (I2C_CR2_RELOAD_Msk));
            }
            else
            {
                dev->bufferSize = dev->bufferCount;
                UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                        (IIC_TRANSFER_CONFIG_MASK),                                                \
                                        (((uint32_t)devAddress       << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                        (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                        (I2C_CR2_AUTOEND_Msk));
            }
        }
    }
    while (dev->bufferCount > 0u);

    // Wait until STOPF flag is set (with AUTOEND the STOP bit is generated automatically)
    err = Iic_waitUntilClear(dev,I2C_ISR_STOPF,(tickStart+timeout));

    // Check the NACK status
    if ((dev->regmap->ISR & I2C_ISR_NACKF) != 0)
    {
        err = ERRORS_IIC_RX_WRONG_EVENT;
    }

i2cerror:

    // Clear NACK flag
    dev->regmap->ICR |= I2C_ICR_NACKCF_Msk;

    // Clear STOPF flag
    dev->regmap->ICR |= I2C_ICR_STOPCF_Msk;

    // Clear configuration register
    UTILITY_MODIFY_REGISTER(dev->regmap->CR2,(IIC_TRANSFER_CONFIG_MASK),0u);

    return err;
}

System_Errors Iic_readRegister (Iic_DeviceHandle dev,
                                uint16_t devAddress,
                                uint16_t regAddress,
                                Iic_RegisterAddressSize addressSize,
                                uint8_t* data,
                                uint16_t length,
                                uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the I2C device
    if (dev == NULL)
    {
        return ERRORS_IIC_NO_DEVICE;
    }
    // Check the I2C instance
    err = ohiassert(IIC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_DEVICE;
    }
    err = ohiassert(IIC_VALID_REGISTERADDRESSSIZE(addressSize));
    // Check the data buffer
    err = ohiassert(data != NULL);
    // Check the number of byte requested
    err = ohiassert(length != 0);
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_PARAM;
    }

    // Shift 7-bit address
    if (dev->config.addressMode == IIC_SEVEN_BIT)
    {
        devAddress = ((devAddress << 1u) & 0x00FFu);
    }

    // Check if the device is busy
    // Wait 25ms before enter into timeout!
    err = Iic_waitUntilSet(dev,I2C_ISR_BUSY,(System_currentTick() + 25u));
    if (err != ERRORS_NO_ERROR) goto i2cerror;

    // Init timeout
    uint32_t tickStart = System_currentTick();

    // Save transfer parameter
    dev->rdata = data;
    dev->bufferCount = length;

    // Send register address
    // Configure transfer
    UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                     \
                            (IIC_TRANSFER_CONFIG_MASK),                                           \
                            (((uint32_t)devAddress  << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                            (((uint32_t)addressSize << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                            ((I2C_CR2_START_Msk)));
    // Check TXE status
    err = Iic_waitUntilClear(dev,I2C_ISR_TXE,(tickStart + timeout));
    if (err != ERRORS_NO_ERROR) goto i2cerror;

    // Write register/memory address
    if (addressSize == IIC_REGISTERADDRESSSIZE_8BIT)
    {
        dev->regmap->TXDR = (uint8_t)(regAddress & 0x00FFu);
    }
    else
    {
        // Write MSB part of address
        dev->regmap->TXDR = (uint8_t)((regAddress & 0xFF00u) >> 8u);
        err = Iic_waitUntilTXSI(dev,(tickStart+ timeout));
        if (err != ERRORS_NO_ERROR) goto i2cerror;
        // Write LSB part of address
        dev->regmap->TXDR = (uint8_t)(regAddress & 0x00FFu);
    }
    // Wait until the transmission ends
    Iic_waitUntilClear(dev,I2C_ISR_TC,(tickStart + timeout));
    if (err != ERRORS_NO_ERROR) goto i2cerror;

    // Now read the data from the selected register
    // Set NBYTES to read and reload if length is grater then NBYTE max size
    if (dev->bufferCount > IIC_MAX_NBYTE_SIZE)
    {
        dev->bufferSize = IIC_MAX_NBYTE_SIZE;
        UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                (IIC_TRANSFER_CONFIG_MASK),                                                \
                                (((uint32_t)devAddress       << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                (I2C_CR2_RELOAD_Msk)                                                     | \
                                (I2C_CR2_START_Msk)                                                      | \
                                (I2C_CR2_RD_WRN_Msk));
    }
    else
    {
        dev->bufferSize =  dev->bufferCount;
        UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                (IIC_TRANSFER_CONFIG_MASK),                                                \
                                (((uint32_t)devAddress       << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                (I2C_CR2_AUTOEND_Msk)                                                    | \
                                (I2C_CR2_START_Msk)                                                      | \
                                (I2C_CR2_RD_WRN_Msk));
    }

    // Start reading bytes
    do
    {
        // Wait until Receive Data Register Not Empty
        err = Iic_waitUntilClear(dev,I2C_ISR_RXNE,tickStart+timeout);
        if (err != ERRORS_NO_ERROR) goto i2cerror;

        // Read data from RXDR
        (*dev->rdata++) = dev->regmap->RXDR;
        dev->bufferSize--;
        dev->bufferCount--;

        // Check if new packet is request
        if ((dev->bufferCount != 0u) && (dev->bufferSize == 0u))
        {
            // Wait until reception is complete
            err = Iic_waitUntilClear(dev,I2C_ISR_TCR,(tickStart+timeout));
            if (err != ERRORS_NO_ERROR) goto i2cerror;

            // Configure new packet without start and stop bits
            // Set NBYTES to read and reload if length is grater then NBYTE max size
            if (dev->bufferCount > IIC_MAX_NBYTE_SIZE)
            {
                dev->bufferSize = IIC_MAX_NBYTE_SIZE;
                UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                        (IIC_TRANSFER_CONFIG_MASK),                                                \
                                        (((uint32_t)devAddress       << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                        (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                        (I2C_CR2_RELOAD_Msk));
            }
            else
            {
                dev->bufferSize = dev->bufferCount;
                UTILITY_MODIFY_REGISTER(dev->regmap->CR2,                                                          \
                                        (IIC_TRANSFER_CONFIG_MASK),                                                \
                                        (((uint32_t)devAddress       << I2C_CR2_SADD_Pos)   & I2C_CR2_SADD_Msk)  | \
                                        (((uint32_t)dev->bufferSize  << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk)| \
                                        (I2C_CR2_AUTOEND_Msk));
            }

        }
    }
    while (dev->bufferCount > 0u);


    // Wait until STOPF flag is set (with AUTOEND the STOP bit is generated automatically)
    err = Iic_waitUntilClear(dev,I2C_ISR_STOPF,(tickStart+timeout));

    // Check the NACK status
    if ((dev->regmap->ISR & I2C_ISR_NACKF) != 0)
    {
        err = ERRORS_IIC_RX_WRONG_EVENT;
    }

i2cerror:

    // Clear NACK flag
    dev->regmap->ICR |= I2C_ICR_NACKCF_Msk;

    // Clear STOPF flag
    dev->regmap->ICR |= I2C_ICR_STOPCF_Msk;

    // Clear configuration register
    UTILITY_MODIFY_REGISTER(dev->regmap->CR2,(IIC_TRANSFER_CONFIG_MASK),0u);

    return err;
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_IIC
