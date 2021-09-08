/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
 *   Leonardo Morichelli <leonardo.morichelli@live.com>
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
 * @file libohiboard/source/STM32L0/spi_STM32L0.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Leonardo Morichelli <leonardo.morichelli@live.com>
 * @brief SPI implementations for STM32L4 Series.
 */

#if defined (LIBOHIBOARD_SPI)

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

#include "platforms.h"
#include "utility.h"
#include "gpio.h"
#include "clock.h"
#include "interrupt.h"

#if defined (LIBOHIBOARD_STM32L0)

/**
 * @brief Enable the SPI peripheral
 */
#define SPI_DEVICE_ENABLE(REGMAP)        (REGMAP->CR1 |= SPI_CR1_SPE)
/**
 * @brief Disable the SPI peripheral
 */
#define SPI_DEVICE_DISABLE(REGMAP)       (REGMAP->CR1 &= ~SPI_CR1_SPE)

#define SPI_CLOCK_ENABLE(REG,MASK)       do { \
                                           UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                           asm("nop"); \
                                           (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                         } while (0)

/**
 * Checks if SPI Serial clock steady state parameter is in allowed range.
 */
#define SPI_VALID_CPOL(CPOL) (((CPOL) == SPI_SCK_INACTIVE_STATE_LOW) || \
                              ((CPOL) == SPI_SCK_INACTIVE_STATE_HIGH))

/**
 * Checks if SPI Clock Phase parameter is in allowed range.
 */
#define SPI_VALID_CPHA(CPHA) (((CPHA) == SPI_SCK_LEADING_EDGE_DATA_CAPTURED) || \
                              ((CPHA) == SPI_SCK_LEADING_EDGE_DATA_CHANGED))

/**
 * Checks if the SPI Mode value is in allowed range.
 * This parameter can be a value of @ref Spi_DeviceType
 */
#define SPI_VALID_MODE(MODE) (((MODE) == SPI_MASTER_MODE) || \
                              ((MODE) == SPI_SLAVE_MODE))

/**
 * Check if SPI direction is in allowed range.
 * This parameter can be a value of @ref Spi_Direction
 */
#define SPI_VALID_DIRECTION(DIRECTION) (((DIRECTION) == SPI_DIRECTION_FULL_DUPLEX) || \
                                        ((DIRECTION) == SPI_DIRECTION_HALF_DUPLEX) || \
                                        ((DIRECTION) == SPI_DIRECTION_RX_ONLY))

#define SPI_VALID_TX_DIRECTION(DIRECTION) (((DIRECTION) == SPI_DIRECTION_FULL_DUPLEX) || \
                                           ((DIRECTION) == SPI_DIRECTION_HALF_DUPLEX))

/**
 * Checks if the SPI first bit type value is in allowed range.
 * This parameter can be a value of @ref Spi_FirstBit
 */
#define SPI_VALID_FIRST_BIT(FIRSTBIT) (((FIRSTBIT) == SPI_FIRSTBIT_MSB) || \
                                       ((FIRSTBIT) == SPI_FIRSTBIT_LSB))

/**
 * Checks if the SPI frame format value is in allowed range.
 * This parameter can be a value of @ref Spi_FrameFormat
 */
#define SPI_VALID_FRAME_FORMAT(FORMAT) (((FORMAT) == SPI_FRAMEFORMAT_MOTOROLA) || \
                                        ((FORMAT) == SPI_FRAMEFORMAT_TI))

/**
 * Checks if SPI Data Size parameter is in allowed range.
 * This parameter can be a value of @ref Spi_DataSize
 */
#define SPI_VALID_DATASIZE(DATASIZE) (((DATASIZE) == SPI_DATASIZE_16BIT) || \
                                      ((DATASIZE) == SPI_DATASIZE_8BIT))

/**
 * Checks if the SPI Slave Select management value is in allowed range.
 * This parameter can be a value of @ref Spi_SSManagement
 */
#define SPI_VALID_SSMANAGEMENT(NSS) (((NSS) == SPI_SSMANAGEMENT_SOFTWARE)         || \
                                     ((NSS) == SPI_SSMANAGEMENT_HARDWARE_INPUT)   || \
                                     ((NSS) == SPI_SSMANAGEMENT_HARDWARE_OUTPUT))

#define SPI_VALID_16BIT_ALIGNED(DATA) (((uint32_t)(DATA) % 2u) == 0u)

#define SPI_MAX_PINS           8

/**
 *
 */
typedef struct _Spi_Device
{
    SPI_TypeDef* regmap;                           /**< Device memory pointer */

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    Spi_SinPins sinPins[SPI_MAX_PINS];
    Spi_SoutPins soutPins[SPI_MAX_PINS];
    Spi_SckPins sckPins[SPI_MAX_PINS];
    Spi_PcsPins nssPins[SPI_MAX_PINS];

    Gpio_Pins sinPinsGpio[SPI_MAX_PINS];
    Gpio_Pins soutPinsGpio[SPI_MAX_PINS];
    Gpio_Pins sckPinsGpio[SPI_MAX_PINS];
    Gpio_Pins nssPinsGpio[SPI_MAX_PINS];
    Gpio_Alternate sinPinsMux[SPI_MAX_PINS];
    Gpio_Alternate soutPinsMux[SPI_MAX_PINS];
    Gpio_Alternate sckPinsMux[SPI_MAX_PINS];
    Gpio_Alternate nssPinsMux[SPI_MAX_PINS];

    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Spi_DeviceState state;                     /**< Current peripheral state. */

    Spi_Config config;

} Spi_Device;

#if defined (LIBOHIBOARD_STM32L073)    || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
#define SPI_IS_DEVICE(DEVICE) (((DEVICE) == OB_SPI1)  || \
                               ((DEVICE) == OB_SPI2))
#elif defined (LIBOHIBOARD_CMWX1ZZABZ_091)
#define SPI_IS_DEVICE(DEVICE) (((DEVICE) == OB_SPI2))
#else
#define SPI_IS_DEVICE(DEVICE) (((DEVICE) == OB_SPI1))
#endif

#if !defined (LIBOHIBOARD_CMWX1ZZABZ_091)
static Spi_Device spi1 = {
        .regmap              = SPI1,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_SPI1EN,

        .sinPins              =
        {
                               SPI_PINS_PA6,
                               SPI_PINS_PA11,
                               SPI_PINS_PB4,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               SPI_PINS_PE14,
#endif
        },
        .sinPinsGpio          =
        {
                               GPIO_PINS_PA6,
                               GPIO_PINS_PA11,
                               GPIO_PINS_PB4,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PE14,
#endif
        },
        .sinPinsMux           =
        {
                               GPIO_ALTERNATE_0,
                               GPIO_ALTERNATE_0,
                               GPIO_ALTERNATE_0,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_2,
#endif
        },

        .soutPins             =
        {
                               SPI_PINS_PA7,
                               SPI_PINS_PA12,
                               SPI_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
	defined (LIBOHIBOARD_STM32L073VxI)
                               SPI_PINS_PE15,
#endif

        },
        .soutPinsGpio         =
        {
                               GPIO_PINS_PA7,
                               GPIO_PINS_PA12,
                               GPIO_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PE15,
#endif
        },
        .soutPinsMux          =
        {
                               GPIO_ALTERNATE_0,
                               GPIO_ALTERNATE_0,
                               GPIO_ALTERNATE_0,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_2,
#endif
        },

        .sckPins              =
        {
                               SPI_PINS_PA5,
#if defined (LIBOHIBOARD_STM32L073)    || \
    defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
                               SPI_PINS_PB3,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               SPI_PINS_PE13,
#endif
        },
        .sckPinsGpio          =
        {
                               GPIO_PINS_PA5,
#if defined (LIBOHIBOARD_STM32L073)    || \
    defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
                               GPIO_PINS_PB3,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PE13,
#endif
        },
        .sckPinsMux           =
        {
                               GPIO_ALTERNATE_0,
#if defined (LIBOHIBOARD_STM32L073)    || \
    defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
                               GPIO_ALTERNATE_0,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_2,
#endif
        },

        .nssPins              =
        {
                               SPI_PINS_PA4,
#if defined (LIBOHIBOARD_STM32L073)    || \
    defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
                               SPI_PINS_PA15,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
	defined (LIBOHIBOARD_STM32L073VxI)
                               SPI_PINS_PE12,
#endif
        },
        .nssPinsGpio          =
        {
                               GPIO_PINS_PA4,
#if defined (LIBOHIBOARD_STM32L073)    || \
    defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
                               GPIO_PINS_PA15,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PE12,
#endif
        },
        .nssPinsMux           =
        {
                               GPIO_ALTERNATE_0,
#if defined (LIBOHIBOARD_STM32L073)    || \
    defined (LIBOHIBOARD_STM32L081KxT) || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU)
                               GPIO_ALTERNATE_0,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_2,
#endif
        },
};
Spi_DeviceHandle OB_SPI1 = &spi1;
#endif //LIBOHIBOARD_CMWX1ZZABZ_091

#if defined (LIBOHIBOARD_STM32L073)    || \
    defined (LIBOHIBOARD_STM32L081CxT) || \
    defined (LIBOHIBOARD_STM32L081CxU) || \
	defined (LIBOHIBOARD_CMWX1ZZABZ_091)
static Spi_Device spi2 = {
        .regmap              = SPI2,

        .rccRegisterPtr      = &RCC->APB1ENR,
        .rccRegisterEnable   = RCC_APB1ENR_SPI2EN,

        .sinPins              =
        {
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
#endif
        },
        .sinPinsGpio          =
        {
                               GPIO_PINS_PB14,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PC2,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PD3,
#endif
        },
        .sinPinsMux           =
        {
                               GPIO_ALTERNATE_0,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_2,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_2,
#endif
        },

        .soutPins             =
        {
                               SPI_PINS_PB15,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               SPI_PINS_PC3,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               SPI_PINS_PD4,
#endif
        },
        .soutPinsGpio         =
        {
                               GPIO_PINS_PB15,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PC3,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PD4,
#endif
        },
        .soutPinsMux          =
        {
                               GPIO_ALTERNATE_0,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_2,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_1,
#endif
        },

        .sckPins              =
        {
#if !defined (LIBOHIBOARD_CMWX1ZZABZ_091)
                               SPI_PINS_PB10,
#endif
                               SPI_PINS_PB13,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               SPI_PINS_PD1,
#endif
        },
        .sckPinsGpio          =
        {
#if !defined (LIBOHIBOARD_CMWX1ZZABZ_091)
                               GPIO_PINS_PB10,
#endif
                               GPIO_PINS_PB13,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PD1,
#endif
        },
        .sckPinsMux           =
        {
#if !defined (LIBOHIBOARD_CMWX1ZZABZ_091)
                               GPIO_ALTERNATE_5,
#endif
                               GPIO_ALTERNATE_0,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_2,
#endif
        },

        .nssPins              =
        {
#if !defined (LIBOHIBOARD_CMWX1ZZABZ_091)
                               SPI_PINS_PB9,
#endif
                               SPI_PINS_PB12,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               SPI_PINS_PD0,
#endif
        },
        .nssPinsGpio          =
        {
#if !defined (LIBOHIBOARD_CMWX1ZZABZ_091)
                               GPIO_PINS_PB9,
#endif
                               GPIO_PINS_PB12,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PD0,
#endif
        },
        .nssPinsMux           =
        {
#if !defined (LIBOHIBOARD_CMWX1ZZABZ_091)
                               GPIO_ALTERNATE_5,
#endif
                               GPIO_ALTERNATE_0,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_1,
#endif
        },
};
Spi_DeviceHandle OB_SPI2 = &spi2;
#endif

static System_Errors Spi_setSoutPin(Spi_DeviceHandle dev, Spi_SoutPins soutPin)
{
    uint8_t devPinIndex;

//    if (dev->devInitialized == 0)
//        return ERRORS_SPI_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->soutPins[devPinIndex] == soutPin)
        {
            Gpio_configAlternate(dev->soutPinsGpio[devPinIndex],
                                 dev->soutPinsMux[devPinIndex],
                                 GPIO_PINS_SPEED_HIGH); // Workaround: Errata STM32L073x8/B/Z, page 17
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

static System_Errors Spi_setSinPin(Spi_DeviceHandle dev, Spi_SinPins sinPin)
{
    uint8_t devPinIndex;

//    if (dev->devInitialized == 0)
//        return ERRORS_SPI_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->sinPins[devPinIndex] == sinPin)
        {
            Gpio_configAlternate(dev->sinPinsGpio[devPinIndex],
                                 dev->sinPinsMux[devPinIndex],
                                 GPIO_PINS_SPEED_HIGH); // Workaround: Errata STM32L073x8/B/Z, page 17
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

static System_Errors Spi_setSckPin(Spi_DeviceHandle dev, Spi_SckPins sckPin)
{
    uint8_t devPinIndex;

//    if (dev->devInitialized == 0)
//        return ERRORS_SPI_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->sckPins[devPinIndex] == sckPin)
        {
            Gpio_configAlternate(dev->sckPinsGpio[devPinIndex],
                                 dev->sckPinsMux[devPinIndex],
                                 GPIO_PINS_SPEED_HIGH); // Workaround: Errata STM32L073x8/B/Z, page 17
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

static System_Errors Spi_setNssPin(Spi_DeviceHandle dev, Spi_PcsPins nssPin)
{
    uint8_t devPinIndex;

//    if (dev->devInitialized == 0)
//        return ERRORS_SPI_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->nssPins[devPinIndex] == nssPin)
        {
            Gpio_configAlternate(dev->nssPinsGpio[devPinIndex],
                                 dev->nssPinsMux[devPinIndex],
                                 GPIO_PINS_SPEED_HIGH); // Workaround: Errata STM32L073x8/B/Z, page 17
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

/**
 * Useful constant to compute baudrate prescaler (see SPIx_CR1 register, BR bits)
 */
static const uint16_t Spi_brDiv[]  =
{
        /*00*/     2, /*01*/     4, /*02*/     8, /*03*/    16,
        /*04*/    32, /*05*/    64, /*06*/   128, /*07*/   256
};

System_Errors Spi_setBaudrate (Spi_DeviceHandle dev, uint32_t speed)
{
    uint32_t frequency = 0u;
    uint32_t computeSpeed = 0u;
    uint32_t diff = 0xFFFFFFFFu;
    uint8_t br = 0u;

    if (dev == OB_SPI1)
        frequency = Clock_getOutputValue(CLOCK_OUTPUT_PCLK2);
    else
        frequency = Clock_getOutputValue(CLOCK_OUTPUT_PCLK1);

    if (frequency != 0u)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            computeSpeed = (frequency / Spi_brDiv[i]);
            if (speed < computeSpeed)
            {
                if ((computeSpeed - speed) < diff)
                {
                    diff = computeSpeed - speed;
                    br = i;
                }
            }
            else
            {
                if ((speed - computeSpeed) < diff)
                {
                    diff = speed - computeSpeed;
                    br = i;
                }
            }
        }

        if (diff == 0xFFFFFFFFu)
            return ERRORS_SPI_BAUDRATE_NOT_FOUND;

        dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_BR));
        dev->regmap->CR1 |= (br << SPI_CR1_BR_Pos);
    }
    else
    {
        return ERRORS_SPI_CLOCKSOURCE_FREQUENCY_TOO_LOW;
    }

    return ERRORS_NO_ERROR;
}

/**
 * This function configure the device with user configuration.
 */
static System_Errors Spi_config (Spi_DeviceHandle dev, Spi_Config* config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check all parameters with asserts
    // The OR is to detect an error: it is not important where is, the important is that there is!
    err = ohiassert(SPI_VALID_MODE(config->devType));
    err |= ohiassert(SPI_VALID_DIRECTION(config->direction));
    err |= ohiassert(SPI_VALID_DATASIZE(config->datasize));
    err |= ohiassert(SPI_VALID_SSMANAGEMENT(config->ssManagement));
    err |= ohiassert(SPI_VALID_FIRST_BIT(config->firstBit));
    err |= ohiassert(SPI_VALID_FRAME_FORMAT(config->frameFormat));

    // In TI format, no CPOL e CPHA check
    if (config->frameFormat == SPI_FRAMEFORMAT_MOTOROLA)
    {
        err |= ohiassert(SPI_VALID_CPHA(config->sckPhase));
        err |= ohiassert(SPI_VALID_CPOL(config->sckPolarity));
    }

    if (err != ERRORS_NO_ERROR)
        return ERRORS_SPI_WRONG_PARAM;

    // Save current configuration
    dev->config = *config;

    // Disable the peripheral
    SPI_DEVICE_DISABLE(dev->regmap);

    // Configure peripheral
    // Configure Mode
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_MSTR | SPI_CR1_SSI));
    if (config->devType == SPI_MASTER_MODE)
    {
        dev->regmap->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI;
    }

    // Configure direction
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_BIDIMODE | SPI_CR1_RXONLY | SPI_CR1_BIDIOE));
    switch (config->direction)
    {
    case SPI_DIRECTION_FULL_DUPLEX:
        // Nothing to do!
        break;

    case SPI_DIRECTION_HALF_DUPLEX:
        dev->regmap->CR1 |= SPI_CR1_BIDIMODE;
        break;

    case SPI_DIRECTION_RX_ONLY:
        dev->regmap->CR1 |= SPI_CR1_RXONLY;
        break;
    }

    // Configure First Bit type
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_LSBFIRST));
    if (config->firstBit == SPI_FIRSTBIT_LSB)
    {
        dev->regmap->CR1 |= SPI_CR1_LSBFIRST;
    }

    // Configure CPOL and CPHA
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_CPOL | SPI_CR1_CPHA));
    if (config->sckPolarity == SPI_SCK_INACTIVE_STATE_HIGH)
    {
        dev->regmap->CR1 |= SPI_CR1_CPOL;
    }
    if (config->sckPhase == SPI_SCK_LEADING_EDGE_DATA_CHANGED)
    {
        dev->regmap->CR1 |= SPI_CR1_CPHA;
    }

    // Configure datasize
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_DFF));
    dev->regmap->CR1 |= ((config->datasize == SPI_DATASIZE_16BIT) ? SPI_CR1_DFF : 0x00000000u);

    // Configure Frame Format Type (Motorola or TI)
    dev->regmap->CR2 = dev->regmap->CR2 & (~(SPI_CR2_FRF));
    if (config->frameFormat == SPI_FRAMEFORMAT_TI)
    {
        dev->regmap->CR2 |= SPI_CR2_FRF;
    }

    // Configure SS management
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_SSM));
    dev->regmap->CR2 = dev->regmap->CR2 & (~(SPI_CR2_SSOE));
    switch (config->ssManagement)
    {
    case SPI_SSMANAGEMENT_SOFTWARE:
        dev->regmap->CR1 |= SPI_CR1_SSM;
        break;
    case SPI_SSMANAGEMENT_HARDWARE_INPUT:
        // Nothing to do!
        break;
    case SPI_SSMANAGEMENT_HARDWARE_OUTPUT:
        dev->regmap->CR2 |= SPI_CR2_SSOE;
        break;
    }

    // Set baudrate
    err = Spi_setBaudrate(dev,config->baudrate);
    if (err != ERRORS_NO_ERROR)
    {
        return err;
    }

    // Activate the SPI mode: this bit must be cleared!
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->I2SCFGR, SPI_I2SCFGR_I2SMOD);

    // The device will be enable only during transmission...
    // When the device was enable, the SS pin is set low automatically.
    // SPI_DEVICE_ENABLE(dev->regmap);

    return ERRORS_NO_ERROR;
}

System_Errors Spi_init (Spi_DeviceHandle dev, Spi_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the SPI device
    if (dev == NULL)
    {
        return ERRORS_SPI_NO_DEVICE;
    }
    // Check the SPI instance
    err = ohiassert(SPI_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_SPI_WRONG_DEVICE;
    }

    // Enable peripheral clock if needed
    if (dev->state == SPI_DEVICESTATE_RESET)
    {
        // Enable peripheral clock
        SPI_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

        // Enable pins
        if (config->sinPin != SPI_PINS_SINNONE)
            Spi_setSinPin(dev, config->sinPin);

        if (config->soutPin != SPI_PINS_SOUTNONE)
            Spi_setSoutPin(dev, config->soutPin);

        if (config->sckPin != SPI_PINS_SCKNONE)
            Spi_setSckPin(dev, config->sckPin);

        if ((config->pcs0Pin != SPI_PINS_PCSNONE) && (config->ssManagement != SPI_SSMANAGEMENT_SOFTWARE))
            Spi_setNssPin(dev, config->pcs0Pin);
    }
    dev->state = SPI_DEVICESTATE_BUSY;

    // Configure the peripheral
    err = Spi_config(dev,config);
    if (err != ERRORS_NO_ERROR)
    {
        // FIXME: Call deInit?
        dev->state = SPI_DEVICESTATE_ERROR;
        return err;
    }

    dev->state = SPI_DEVICESTATE_READY;

    return ERRORS_NO_ERROR;
}

System_Errors Spi_readByte (Spi_DeviceHandle dev, uint8_t * data)
{
    // deprecated
    return ohiassert(0);
}

System_Errors Spi_writeByte (Spi_DeviceHandle dev, uint8_t data)
{
    // deprecated
    return ohiassert(0);
}

System_Errors Spi_read (Spi_DeviceHandle dev, uint8_t* data, uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;

    if (dev->config.datasize == SPI_DATASIZE_16BIT)
    {
        err |= ohiassert(SPI_VALID_16BIT_ALIGNED(data));
    }
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_SPI_WRONG_PARAM;
    }

    if (dev->state != SPI_DEVICESTATE_READY)
    {
        err = ERRORS_SPI_DEVICE_BUSY;
        dev->state = SPI_DEVICESTATE_ERROR;
        // Release the device.
        goto spierror;
    }
    dev->state = SPI_DEVICESTATE_BUSY;

    // If one line transmission, setup the device
    if (dev->config.direction == SPI_DIRECTION_HALF_DUPLEX)
    {
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->CR1,SPI_CR1_BIDIOE);
    }

    // Check if the device is enabled
    if (UTILITY_READ_REGISTER_BIT(dev->regmap->CR1,SPI_CR1_SPE) == 0)
    {
        SPI_DEVICE_ENABLE(dev->regmap);
    }

    // Save timeout
    uint32_t timeoutEnd = System_currentTick() + timeout;
    // Check the connection type
    if ((dev->config.devType == SPI_MASTER_MODE) && (dev->config.direction == SPI_DIRECTION_FULL_DUPLEX))
    {
        // Send dummy data
        // Wait until the buffer is empty
        while (UTILITY_READ_REGISTER_BIT(dev->regmap->SR,SPI_SR_TXE) == 0)
        {
            if (System_currentTick() > timeoutEnd)
            {
                err = ERRORS_SPI_TIMEOUT_TX;
                // Release the device.
                goto spierror;
            }
        }

        // In case of datasize is grater the 8B cast the relative value
        if (dev->config.datasize > SPI_DATASIZE_8BIT)
        {
            *((volatile uint16_t *)&dev->regmap->DR) = SPI_EMPTY_WORD;
        }
        else
        {
            *((volatile uint8_t *)&dev->regmap->DR) = SPI_EMPTY_BYTE;
        }

        // Now read the data
        // Wait until the buffer is not empty
        while (UTILITY_READ_REGISTER_BIT(dev->regmap->SR,SPI_SR_RXNE) == 0)
        {
            if (System_currentTick() > timeoutEnd)
            {
                err = ERRORS_SPI_TIMEOUT_RX;
                // Release the device.
                goto spierror;
            }
        }

        // In case of datasize is grater the 8B cast the relative value
        if (dev->config.datasize > SPI_DATASIZE_8BIT)
        {
            *((uint16_t *)data) = *((volatile uint16_t *)&dev->regmap->DR);
        }
        else
        {
            *((uint8_t *)data) = *((volatile uint8_t *)&dev->regmap->DR);
        }
    }
    else
    {
        // Wait until the buffer is not empty
        while (UTILITY_READ_REGISTER_BIT(dev->regmap->SR,SPI_SR_RXNE) == 0)
        {
            if (System_currentTick() > timeoutEnd)
            {
                err = ERRORS_SPI_TIMEOUT_RX;
                // Release the device.
                goto spierror;
            }
        }

        // In case of datasize is grater the 8B cast the relative value
        if (dev->config.datasize > SPI_DATASIZE_8BIT)
        {
            *((uint16_t *)data) = *((volatile uint16_t *)&dev->regmap->DR);
        }
        else
        {
            *((uint8_t *)data) = *((volatile uint8_t *)&dev->regmap->DR);
        }
    }

spierror:
    // FIXME: Disable SPI?
    if (err == ERRORS_NO_ERROR)
        dev->state = SPI_DEVICESTATE_READY;
    else
        dev->state = SPI_DEVICESTATE_ERROR;
    return err;
}

System_Errors Spi_write (Spi_DeviceHandle dev, const uint8_t* data, uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Check the chosen direction for the peripheral
    err = ohiassert(SPI_VALID_TX_DIRECTION(dev->config.direction));
    if (dev->config.datasize == SPI_DATASIZE_16BIT)
    {
        err |= ohiassert(SPI_VALID_16BIT_ALIGNED(data));
    }
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_SPI_WRONG_PARAM;
    }

    if (dev->state != SPI_DEVICESTATE_READY)
    {
        err = ERRORS_SPI_DEVICE_BUSY;
        dev->state = SPI_DEVICESTATE_ERROR;
        // Release the device.
        goto spierror;
    }
    dev->state = SPI_DEVICESTATE_BUSY;

    // Save timeout
    uint32_t timeoutEnd = System_currentTick() + timeout;

    // If one line transmission, setup the device
    if (dev->config.direction == SPI_DIRECTION_HALF_DUPLEX)
    {
        UTILITY_SET_REGISTER_BIT(dev->regmap->CR1,SPI_CR1_BIDIOE);
    }

    // Check if the device is enabled
    if (UTILITY_READ_REGISTER_BIT(dev->regmap->CR1,SPI_CR1_SPE) == 0)
    {
        SPI_DEVICE_ENABLE(dev->regmap);
    }

    // In case of datasize is grater the 8B cast the relative value
    if (dev->config.datasize > SPI_DATASIZE_8BIT)
    {
        if (dev->config.devType == SPI_SLAVE_MODE)
        {
            *((volatile uint16_t *)&dev->regmap->DR) = *((uint16_t *)data);
        }

        // Wait until the buffer is empty
        while (UTILITY_READ_REGISTER_BIT(dev->regmap->SR,SPI_SR_TXE) == 0)
        {
            if (System_currentTick() > timeoutEnd)
            {
                err = ERRORS_SPI_TIMEOUT_TX;
                // Release the device.
                goto spierror;
            }
        }

        *((volatile uint16_t *)&dev->regmap->DR) = *((uint16_t *)data);
    }
    else
    {
        if (dev->config.devType == SPI_SLAVE_MODE)
        {
            *((volatile uint8_t *)&dev->regmap->DR) = *((uint8_t *)data);
        }

        // Wait until the buffer is empty
        while (UTILITY_READ_REGISTER_BIT(dev->regmap->SR,SPI_SR_TXE) == 0)
        {
            if (System_currentTick() > timeoutEnd)
            {
                err = ERRORS_SPI_TIMEOUT_TX;
                // Release the device.
                goto spierror;
            }
        }

        *((volatile uint8_t *)&dev->regmap->DR) = *((uint8_t *)data);
    }

    // In case of full-duplex transmission, wait until RXNE flag is set
    if (dev->config.direction == SPI_DIRECTION_FULL_DUPLEX)
    {
        uint16_t dummy = 0;

        while (UTILITY_READ_REGISTER_BIT(dev->regmap->SR,SPI_SR_RXNE) == 0)
        {
            if (System_currentTick() > timeoutEnd)
            {
                err = ERRORS_SPI_TIMEOUT_TX;
                // Release the device.
                goto spierror;
            }
        }

        // Read dummy element from RX FIFO
        if (dev->config.datasize > SPI_DATASIZE_8BIT)
        {
            *((uint16_t *)&dummy) = *((volatile uint16_t *)&dev->regmap->DR);
        }
        else
        {
            *((uint8_t *)&dummy) = *((volatile uint8_t *)&dev->regmap->DR);
        }
    }

    if (UTILITY_READ_REGISTER_BIT(dev->regmap->SR,SPI_SR_OVR))
    {
        // Clear overrun flag in two lines direction mode
        // The received data is not read
        if (dev->config.direction == SPI_DIRECTION_FULL_DUPLEX)
        {
            (void) dev->regmap->DR;
            (void) dev->regmap->SR;
        }
    }

spierror:
    // FIXME: Disable SPI?
    if (err == ERRORS_NO_ERROR)
        dev->state = SPI_DEVICESTATE_READY;
    else
        dev->state = SPI_DEVICESTATE_ERROR;
    return err;
}

System_Errors Spi_deInit (Spi_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the SPI device
    if (dev == NULL)
    {
        return ERRORS_SPI_NO_DEVICE;
    }
    // Check the SPI instance
    err = ohiassert(SPI_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_SPI_WRONG_DEVICE;
    }
    dev->state = SPI_DEVICESTATE_BUSY;

    // Disable the device
    SPI_DEVICE_DISABLE(dev->regmap);

    // TODO: Clear all register and flag interrupts!

    dev->state = SPI_DEVICESTATE_RESET;
    return err;
}

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_SPI
