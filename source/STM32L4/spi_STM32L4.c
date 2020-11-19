/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018-2020 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/STM32L4/spi_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Leonardo Morichelli <leonardo.morichelli@live.com>
 * @brief SPI implementations for STM32L4 Series.
 */

#ifdef LIBOHIBOARD_SPI

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

#include "platforms.h"
#include "utility.h"
#include "gpio.h"
#include "clock.h"

#if defined (LIBOHIBOARD_STM32L4)

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
 * Checks if SPI Data Size parameter is in allowed range.
 * This parameter can be a value of @ref Spi_DataSize
 */
#define SPI_VALID_DATASIZE(DATASIZE) (((DATASIZE) == SPI_DATASIZE_16BIT) || \
                                      ((DATASIZE) == SPI_DATASIZE_15BIT) || \
                                      ((DATASIZE) == SPI_DATASIZE_14BIT) || \
                                      ((DATASIZE) == SPI_DATASIZE_13BIT) || \
                                      ((DATASIZE) == SPI_DATASIZE_12BIT) || \
                                      ((DATASIZE) == SPI_DATASIZE_11BIT) || \
                                      ((DATASIZE) == SPI_DATASIZE_10BIT) || \
                                      ((DATASIZE) == SPI_DATASIZE_9BIT)  || \
                                      ((DATASIZE) == SPI_DATASIZE_8BIT)  || \
                                      ((DATASIZE) == SPI_DATASIZE_7BIT)  || \
                                      ((DATASIZE) == SPI_DATASIZE_6BIT)  || \
                                      ((DATASIZE) == SPI_DATASIZE_5BIT)  || \
                                      ((DATASIZE) == SPI_DATASIZE_4BIT))

/**
 * Check if SPI direction is in allowed range.
 * This parameter can be a value of @ref Spi_Direction
 */
#define SPI_VALID_DIRECTION(DIRECTION) (((DIRECTION) == SPI_DIRECTION_FULL_DUPLEX) || \
                                        ((DIRECTION) == SPI_DIRECTION_HALF_DUPLEX) || \
                                        ((DIRECTION) == SPI_DIRECTION_RX_ONLY)     || \
                                        ((DIRECTION) == SPI_DIRECTION_TX_ONLY))

#define SPI_VALID_TX_DIRECTION(DIRECTION) (((DIRECTION) == SPI_DIRECTION_FULL_DUPLEX) || \
                                           ((DIRECTION) == SPI_DIRECTION_HALF_DUPLEX) || \
                                           ((DIRECTION) == SPI_DIRECTION_TX_ONLY))

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
 * Checks if the SPI Slave Select management value is in allowed range.
 * This parameter can be a value of @ref Spi_SSManagement
 */
#define SPI_VALID_SSMANAGEMENT(NSS) (((NSS) == SPI_SSMANAGEMENT_SOFTWARE)              || \
                                     ((NSS) == SPI_SSMANAGEMENT_HARDWARE_INPUT)        || \
                                     ((NSS) == SPI_SSMANAGEMENT_HARDWARE_OUTPUT)       || \
                                     ((NSS) == SPI_SSMANAGEMENT_HARDWARE_OUTPUT_PULSE))

#define SPI_MAX_PINS           8

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

//    Spi_DeviceType devType;
//    uint32_t baudrate;
//    Spi_Direction direction;
//
//    Spi_DataSize datasize;
//    Spi_FirstBit firstBit;
//    Spi_FrameFormat frameFormat;
//    Spi_SSManagement ssManagement;

    Spi_Config config;

    Spi_DeviceState state;                     /**< Current peripheral state. */

} Spi_Device;

#if defined (LIBOHIBOARD_STM32L4x6) ||\
    defined (LIBOHIBOARD_STM32WB55)

#define SPI_IS_DEVICE(DEVICE) (((DEVICE) == OB_SPI1)  || \
                               ((DEVICE) == OB_SPI2)  || \
                               ((DEVICE) == OB_SPI3))

static Spi_Device spi1 =
{
        .regmap              = SPI1,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_SPI1EN,

        .sinPins              =
        {
                               SPI_PINS_PA6,
                               SPI_PINS_PB4,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PE14,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PG3,
#endif
        },
        .sinPinsGpio          =
        {
                               GPIO_PINS_PA6,
                               GPIO_PINS_PB4,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PE14,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG3,
#endif
        },
        .sinPinsMux           =
        {
                               GPIO_ALTERNATE_5,
                               GPIO_ALTERNATE_5,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
        },

        .soutPins             =
        {
                               SPI_PINS_PA7,
                               SPI_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PE15,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PG4,
#endif
        },
        .soutPinsGpio         =
        {
                               GPIO_PINS_PA7,
                               GPIO_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PE15,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG4,
#endif
        },
        .soutPinsMux          =
        {
                               GPIO_ALTERNATE_5,
                               GPIO_ALTERNATE_5,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
        },

        .sckPins              =
        {
                               SPI_PINS_PA5,
                               SPI_PINS_PB3,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PE13,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PG2,
#endif
        },
        .sckPinsGpio          =
        {
                               GPIO_PINS_PA5,
                               GPIO_PINS_PB3,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PE13,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG2,
#endif
        },
        .sckPinsMux           =
        {
                               GPIO_ALTERNATE_5,
                               GPIO_ALTERNATE_5,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
        },

        .nssPins              =
        {
                               SPI_PINS_PA4,
                               SPI_PINS_PA15,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PE12,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PG5,
#endif
        },
        .nssPinsGpio          =
        {
                               GPIO_PINS_PA4,
                               GPIO_PINS_PA15,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PE12,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG5,
#endif
        },
        .nssPinsMux           =
        {
                               GPIO_ALTERNATE_5,
                               GPIO_ALTERNATE_5,
#if defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
#if defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
        },

        .state                = SPI_DEVICESTATE_RESET,
};
Spi_DeviceHandle OB_SPI1 = &spi1;

static Spi_Device spi2 =
{
        .regmap              = SPI2,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_SPI2EN,

        .sinPins              =
        {
                               SPI_PINS_PB14,
                               SPI_PINS_PC2,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PD3,
#endif
        },
        .sinPinsGpio          =
        {
                               GPIO_PINS_PB14,
                               GPIO_PINS_PC2,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PD3,
#endif
        },
        .sinPinsMux           =
        {
                               GPIO_ALTERNATE_5,
                               GPIO_ALTERNATE_5,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
        },

        .soutPins             =
        {
                               SPI_PINS_PB15,
                               SPI_PINS_PC3,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PD4,
#endif
        },
        .soutPinsGpio         =
        {
                               GPIO_PINS_PB15,
                               GPIO_PINS_PC3,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PD4,
#endif
        },
        .soutPinsMux          =
        {
                               GPIO_ALTERNATE_5,
                               GPIO_ALTERNATE_5,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
        },

        .sckPins              =
        {
                               SPI_PINS_PB10,
                               SPI_PINS_PB13,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PD1,
#endif
        },
        .sckPinsGpio          =
        {
                               GPIO_PINS_PB10,
                               GPIO_PINS_PB13,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PD1,
#endif
        },
        .sckPinsMux           =
        {
                               GPIO_ALTERNATE_5,
                               GPIO_ALTERNATE_5,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
        },

        .nssPins              =
        {
                               SPI_PINS_PB9,
                               SPI_PINS_PB12,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PD0,
#endif
        },
        .nssPinsGpio          =
        {
                               GPIO_PINS_PB9,
                               GPIO_PINS_PB12,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PD0,
#endif
        },
        .nssPinsMux           =
        {
                               GPIO_ALTERNATE_5,
                               GPIO_ALTERNATE_5,
#if defined (LIBOHIBOARD_STM32WB55Rx)  || \
    defined (LIBOHIBOARD_STM32L476VxT) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_5,
#endif
        },

        .state                = SPI_DEVICESTATE_RESET,
};
Spi_DeviceHandle OB_SPI2 = &spi2;

static Spi_Device spi3 =
{
        .regmap              = SPI3,

        .rccRegisterPtr      = &RCC->APB1ENR1,
        .rccRegisterEnable   = RCC_APB1ENR1_SPI3EN,

        .sinPins              =
        {
                               SPI_PINS_PB4,
                               SPI_PINS_PC11,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PG10,
#endif
        },
        .sinPinsGpio          =
        {
                               GPIO_PINS_PB4,
                               GPIO_PINS_PC11,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG10,
#endif
        },
        .sinPinsMux           =
        {
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_6,
#endif
        },

        .soutPins             =
        {
                               SPI_PINS_PB5,
                               SPI_PINS_PC11,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PG11,
#endif
        },
        .soutPinsGpio         =
        {
                               GPIO_PINS_PB5,
                               GPIO_PINS_PC11,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG11,
#endif
        },
        .soutPinsMux          =
        {
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_6,
#endif
        },

        .sckPins              =
        {
                               SPI_PINS_PB3,
                               SPI_PINS_PC10,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PG9,
#endif
        },
        .sckPinsGpio          =
        {
                               GPIO_PINS_PB3,
                               GPIO_PINS_PC10,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG9,
#endif
        },
        .sckPinsMux           =
        {
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_6,
#endif
        },

        .nssPins              =
        {
                               SPI_PINS_PA4,
                               SPI_PINS_PA15,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               SPI_PINS_PG12,
#endif
        },
        .nssPinsGpio          =
        {
                               GPIO_PINS_PA4,
                               GPIO_PINS_PA15,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_PINS_PG12,
#endif
        },
        .nssPinsMux           =
        {
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L476JxY) || \
    defined (LIBOHIBOARD_STM32L476MxY) || \
    defined (LIBOHIBOARD_STM32L476QxI) || \
    defined (LIBOHIBOARD_STM32L476ZxT) || \
    defined (LIBOHIBOARD_STM32L476ZxJ)
                               GPIO_ALTERNATE_6,
#endif
        },

        .state                = SPI_DEVICESTATE_RESET,
};
Spi_DeviceHandle OB_SPI3 = &spi3;

#else

#warning "LIBOHIBOARD: NO SPI DEVICE DEFINED!"

#endif

static System_Errors Spi_setSoutPin(Spi_DeviceHandle dev, Spi_SoutPins soutPin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->soutPins[devPinIndex] == soutPin)
        {
            Gpio_configAlternate(dev->soutPinsGpio[devPinIndex],
                                 dev->soutPinsMux[devPinIndex],
                                 0);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

static System_Errors Spi_setSinPin(Spi_DeviceHandle dev, Spi_SinPins sinPin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->sinPins[devPinIndex] == sinPin)
        {
            Gpio_configAlternate(dev->sinPinsGpio[devPinIndex],
                                 dev->sinPinsMux[devPinIndex],
                                 0);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

static System_Errors Spi_setSckPin(Spi_DeviceHandle dev, Spi_SckPins sckPin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->sckPins[devPinIndex] == sckPin)
        {
            Gpio_configAlternate(dev->sckPinsGpio[devPinIndex],
                                 dev->sckPinsMux[devPinIndex],
                                 0);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

static System_Errors Spi_setNssPin(Spi_DeviceHandle dev, Spi_PcsPins nssPin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->nssPins[devPinIndex] == nssPin)
        {
            Gpio_configAlternate(dev->nssPinsGpio[devPinIndex],
                                 dev->nssPinsMux[devPinIndex],
                                 0);
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

#if defined (LIBOHIBOARD_STM32L476)

    if (dev == OB_SPI1)
        frequency = Clock_getOutputValue(CLOCK_OUTPUT_PCLK2);
    else
        frequency = Clock_getOutputValue(CLOCK_OUTPUT_PCLK1);

#endif

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

        dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_BR_Msk));
        dev->regmap->CR1 |= (br << SPI_CR1_BR_Pos);
    }
    else
    {
        return ERRORS_SPI_CLOCKSOURCE_FREQUENCY_TOO_LOW;
    }

    return ERRORS_NO_ERROR;
}

/**
 *
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
    if (err != ERRORS_NO_ERROR)
        return ERRORS_SPI_WRONG_PARAM;

    // Save current configuration
    dev->config = *config;


    // Disable the peripheral
    SPI_DEVICE_DISABLE(dev->regmap);

    // Configure peripheral
    // Configure Mode
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_MSTR_Msk | SPI_CR1_SSI_Msk));
    if (dev->config.devType == SPI_MASTER_MODE)
    {
        dev->regmap->CR1 |= SPI_CR1_MSTR_Msk | SPI_CR1_SSI_Msk;
    }

    // Configure direction
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_BIDIMODE_Msk | SPI_CR1_RXONLY_Msk | SPI_CR1_BIDIOE_Msk));
    switch (dev->config.direction)
    {
    case SPI_DIRECTION_FULL_DUPLEX:
        // Nothing to do!
        break;

    case SPI_DIRECTION_HALF_DUPLEX:
        dev->regmap->CR1 |= SPI_CR1_BIDIMODE_Msk;
        break;

    case SPI_DIRECTION_RX_ONLY:
        dev->regmap->CR1 |= SPI_CR1_RXONLY_Msk;
        break;

    case SPI_DIRECTION_TX_ONLY:
        // Nothing to do!
        break;
    }

    // Configure First Bit type
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_LSBFIRST_Msk));
    if (dev->config.firstBit == SPI_FIRSTBIT_LSB)
    {
        dev->regmap->CR1 |= SPI_CR1_LSBFIRST_Msk;
    }

    // Configure CPOL and CPHA
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_CPOL_Msk | SPI_CR1_CPHA_Msk));
    if (dev->config.sckPolarity == SPI_SCK_INACTIVE_STATE_HIGH)
    {
        dev->regmap->CR1 |= SPI_CR1_CPOL_Msk;
    }
    if (dev->config.sckPhase == SPI_SCK_LEADING_EDGE_DATA_CHANGED)
    {
        dev->regmap->CR1 |= SPI_CR1_CPHA_Msk;
    }

    // Configure datasize
    dev->regmap->CR2 = dev->regmap->CR2 & (~(SPI_CR2_DS_Msk));
    dev->regmap->CR2 |= ((dev->config.datasize - 1) << SPI_CR2_DS_Pos);

    // Configure Frame Format Type (Motorola or TI)
    dev->regmap->CR2 = dev->regmap->CR2 & (~(SPI_CR2_FRF_Msk));
    if (dev->config.frameFormat == SPI_FRAMEFORMAT_TI)
    {
        dev->regmap->CR2 |= SPI_CR2_FRF_Msk;
    }

    // Configure SS management
    dev->regmap->CR1 = dev->regmap->CR1 & (~(SPI_CR1_SSM_Msk));
    dev->regmap->CR2 = dev->regmap->CR2 & (~(SPI_CR2_SSOE_Msk | SPI_CR2_NSSP_Msk));
    if (dev->config.datasize <= SPI_DATASIZE_8BIT)
    {
    	dev->regmap->CR2 |= SPI_CR2_FRXTH_Msk;
    }

    switch (dev->config.ssManagement)
    {
    case SPI_SSMANAGEMENT_SOFTWARE:
        dev->regmap->CR1 |= SPI_CR1_SSM_Msk;
        break;
    case SPI_SSMANAGEMENT_HARDWARE_INPUT:
        // Nothing to do!
        break;
    case SPI_SSMANAGEMENT_HARDWARE_OUTPUT:
        dev->regmap->CR2 |= SPI_CR2_SSOE_Msk;
        break;
    case SPI_SSMANAGEMENT_HARDWARE_OUTPUT_PULSE:
        dev->regmap->CR2 |= SPI_CR2_SSOE_Msk | SPI_CR2_NSSP_Msk;
        break;
    }

    // Set baudrate
    err = Spi_setBaudrate(dev,dev->config.baudrate);
    if (err != ERRORS_NO_ERROR)
    {
        return err;
    }

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
        Spi_deInit(dev);
        dev->state = SPI_DEVICESTATE_ERROR;
        return err;
    }

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

    if (dev->state != SPI_DEVICESTATE_READY)
    {
        err = ERRORS_SPI_DEVICE_BUSY;
        dev->state = SPI_DEVICESTATE_ERROR;
        // Release the device.
        goto spierror;
    }
    dev->state = SPI_DEVICESTATE_BUSY;

    // Set FIFO RX threshold
    if (dev->config.datasize > SPI_DATASIZE_8BIT)
    {
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->CR2,SPI_CR2_FRXTH_Msk);
    }
    else
    {
        UTILITY_SET_REGISTER_BIT(dev->regmap->CR2,SPI_CR2_FRXTH_Msk);
    }

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
    {
        dev->state = SPI_DEVICESTATE_READY;
    }
    else
    {
        dev->state = SPI_DEVICESTATE_ERROR;
    }
    return err;
}

System_Errors Spi_write (Spi_DeviceHandle dev, const uint8_t* data, uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Check the chosen direction for the peripheral
    err = ohiassert(SPI_VALID_TX_DIRECTION(dev->config.direction));
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

        while (UTILITY_READ_REGISTER_BIT(dev->regmap->SR,SPI_SR_RXNE_Msk) == 0)
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

    if (UTILITY_READ_REGISTER_BIT(dev->regmap->SR,SPI_SR_OVR_Msk))
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
    {
        dev->state = SPI_DEVICESTATE_READY;
    }
    else
    {
        dev->state = SPI_DEVICESTATE_ERROR;
    }
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

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_SPI
