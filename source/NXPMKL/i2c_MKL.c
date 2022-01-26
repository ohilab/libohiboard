/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2020 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/NXPMKL/i2c_MKL.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C implementations for NXP MKL series.
 */

#ifdef LIBOHIBOARD_IIC

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_MKL)

#include "utility.h"
#include "i2c.h"
#include "clock.h"
#include "gpio.h"

#define IIC_MAX_PINS           5

#define IIC_PIN_ENABLED        1
#define IIC_PIN_DISABLED       0

//static uint8_t Iic_firstRead = 0;

typedef struct _Iic_Device
{
    I2C_Type* regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Iic_SclPins sclPins[IIC_MAX_PINS];
    Iic_SdaPins sdaPins[IIC_MAX_PINS];

    Gpio_Pins sclPinsGpio[IIC_MAX_PINS];
    Gpio_Pins sdaPinsGpio[IIC_MAX_PINS];

    volatile uint32_t* sclPinsPtr[IIC_MAX_PINS];
    volatile uint32_t* sdaPinsPtr[IIC_MAX_PINS];
    uint8_t sclPinsMux[IIC_MAX_PINS];
    uint8_t sdaPinsMux[IIC_MAX_PINS];

    // Write/Read useful buffer and counter
    uint8_t* rdata;                      /**< Pointer to I2C reception buffer */
    const uint8_t* tdata;             /**< Pointer to I2C transmission buffer */
    uint16_t bufferSize;                        /**< I2C buffer transfer size */
    volatile uint16_t bufferCount;           /**< I2C buffer transfer counter */

    Iic_Config config;

    Iic_DeviceState state;                     /**< Current peripheral state. */

} Iic_Device;

#define IIC_IS_DEVICE(DEVICE) (((DEVICE) == OB_IIC0)  || \
                               ((DEVICE) == OB_IIC1))

#define IIC_VALID_MODE(MODE) (((MODE) == IIC_MASTER_MODE) || \
                              ((MODE) == IIC_SLAVE_MODE))

#define IIC_VALID_ADDRESSMODE(ADDRESSMODE) (((ADDRESSMODE) == IIC_SEVEN_BIT)  || \
                                            ((ADDRESSMODE) == IIC_TEN_BIT))

#define IIC_VALID_REGISTERADDRESSSIZE(SIZE) (((SIZE) == IIC_REGISTERADDRESSSIZE_8BIT)  || \
                                             ((SIZE) == IIC_REGISTERADDRESSSIZE_16BIT))


//#define IIC_CLOCK_ENABLE(REG,MASK) do { \
//                                     UTILITY_SET_REGISTER_BIT(REG,MASK); \
//                                     asm("nop"); \
//                                     (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
//                                   } while (0)

/**
 * @brief Enable the I2C peripheral
 */
#define IIC_DEVICE_ENABLE(REGMAP)        (REGMAP->C1 |= I2C_C1_IICEN_MASK)
/**
 * @brief Disable the I2C peripheral
 */
#define IIC_DEVICE_DISABLE(REGMAP)       (REGMAP->C1 &= ~I2C_C1_IICEN_MASK)

static Iic_Device iic0 =
{
        .regMap           = I2C0,

        .simScgcPtr       = &SIM->SCGC4,
        .simScgcBitEnable = SIM_SCGC4_I2C0_MASK,

        .sclPins          =
        {
                             IIC_PINS_PTB0,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             IIC_PINS_PTB2,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             IIC_PINS_PTC8,
#endif
                             IIC_PINS_PTE19,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             IIC_PINS_PTE24,
#endif
        },
        .sclPinsGpio      =
        {
                             GPIO_PINS_PTB0,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTB2,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTC8,
#endif
                             GPIO_PINS_PTE19,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTE24,
#endif
        },
        .sclPinsPtr       =
        {
                             &PORTB->PCR[0],
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTB->PCR[2],
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTC->PCR[8],
#endif
                             &PORTE->PCR[19],
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTE->PCR[24],
#endif
        },
        .sclPinsMux       =
        {
                             2,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             2,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             2,
#endif
                             4,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             5,
#endif
        },

        .sdaPins          =
        {
                             IIC_PINS_PTB1,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             IIC_PINS_PTB3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             IIC_PINS_PTC9,
#endif
                             IIC_PINS_PTE18,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             IIC_PINS_PTE25,
#endif
        },
        .sdaPinsGpio      =
        {
                             GPIO_PINS_PTB1,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTB3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTC9,
#endif
                             GPIO_PINS_PTE18,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTE25,
#endif
        },
        .sdaPinsPtr       =
        {
                             &PORTB->PCR[1],
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTB->PCR[3],
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTC->PCR[9],
#endif
                             &PORTE->PCR[18],
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTE->PCR[25],
#endif
        },
        .sdaPinsMux       =
        {
                             2,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             2,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             2,
#endif
                             4,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             5,
#endif
        },

        .state             = IIC_DEVICESTATE_RESET,
};
Iic_DeviceHandle OB_IIC0 = &iic0;

static Iic_Device iic1 =
{
        .regMap           = I2C1,

        .simScgcPtr       = &SIM->SCGC4,
        .simScgcBitEnable = SIM_SCGC4_I2C1_MASK,

        .sclPins          =
        {
                             IIC_PINS_PTA3,
                             IIC_PINS_PTC1,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             IIC_PINS_PTC10,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             IIC_PINS_PTE1,
#endif
        },
        .sclPinsGpio      =
        {
                             GPIO_PINS_PTA3,
                             GPIO_PINS_PTC1,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTC10,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTE1,
#endif
        },
        .sclPinsPtr       =
        {
                             &PORTA->PCR[3],
                             &PORTC->PCR[1],
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTC->PCR[10],
#endif
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTE->PCR[1],
#endif
        },
        .sclPinsMux       =
        {
                             2,
                             2,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             2,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             6,
#endif
        },

        .sdaPins          =
        {
                             IIC_PINS_PTA4,
                             IIC_PINS_PTC2,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             IIC_PINS_PTC11,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             IIC_PINS_PTE0,
#endif
        },
        .sdaPinsGpio      =
        {
                             GPIO_PINS_PTA4,
                             GPIO_PINS_PTC2,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTC11,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTE0,
#endif
        },
        .sdaPinsPtr       =
        {
                             &PORTA->PCR[4],
                             &PORTC->PCR[2],
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTC->PCR[11],
#endif
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTE->PCR[0],
#endif
        },
        .sdaPinsMux       =
        {
                             2,
                             2,
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             2,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             6,
#endif
        },


        .state             = IIC_DEVICESTATE_RESET,
};
Iic_DeviceHandle OB_IIC1 = &iic1;

/* See Table 50-41 I2C Divider and Hold Values */
static uint16_t Iic_sclDivTab[] =
{
    /* 00 */   20, /* 01 */   22, /* 02 */   24, /* 03 */   26,
    /* 04 */   28, /* 05 */   30, /* 06 */   34, /* 07 */   40,
    /* 08 */   28, /* 09 */   32, /* 0A */   36, /* 0B */   40,
    /* 0C */   44, /* 0D */   48, /* 0E */   56, /* 0F */   68,

    /* 10 */   48, /* 11 */   56, /* 12 */   64, /* 13 */   72,
    /* 14 */   80, /* 15 */   88, /* 16 */  104, /* 17 */  128,
    /* 18 */   80, /* 19 */   96, /* 1A */  112, /* 1B */  128,
    /* 1C */  144, /* 1D */  160, /* 1E */  192, /* 1F */  240,

    /* 20 */  160, /* 21 */  192, /* 22 */  224, /* 23 */  256,
    /* 24 */  288, /* 25 */  320, /* 26 */  384, /* 27 */  480,
    /* 28 */  320, /* 29 */  384, /* 2A */  448, /* 2B */  512,
    /* 2C */  576, /* 2D */  640, /* 2E */  768, /* 2F */  960,

    /* 30 */  640, /* 31 */  768, /* 32 */  896, /* 33 */ 1024,
    /* 34 */ 1152, /* 35 */ 1280, /* 36 */ 1536, /* 37 */ 1920,
    /* 38 */ 1280, /* 39 */ 1536, /* 3A */ 1792, /* 3B */ 2048,
    /* 3C */ 2304, /* 3D */ 2560, /* 3E */ 3072, /* 3F */ 3840,
};

/**
 *
 * FIXME: implement pull-up enable
 */
static System_Errors Iic_setSclPin(Iic_DeviceHandle dev, Iic_SclPins sclPin, bool pullupEnable)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < IIC_MAX_PINS; ++devPinIndex)
    {
        if (dev->sclPins[devPinIndex] == sclPin)
        {
            Gpio_configAlternate(dev->sclPinsGpio[devPinIndex],
                                 dev->sclPinsMux[devPinIndex],
                                 0);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_IIC_NO_PIN_FOUND;
}

/**
 *
 * FIXME: implement pull-up enable
 */
static System_Errors Iic_setSdaPin(Iic_DeviceHandle dev, Iic_SdaPins sdaPin, bool pullupEnable)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < IIC_MAX_PINS; ++devPinIndex)
    {
        if (dev->sdaPins[devPinIndex] == sdaPin)
        {
            Gpio_configAlternate(dev->sdaPinsGpio[devPinIndex],
                                 dev->sdaPinsMux[devPinIndex],
                                 0);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_IIC_NO_PIN_FOUND;
}

/*
 * brief setting icr parameter of I2C_F register for generate the desired baudrate
 *
 * Thank for this function at https://github.com/laswick/kinetis/blob/master/phase2_embedded_c/i2c.c
 *
 * WARNING: On KL25 there is a silicon error on clock distribution. For more informations see:
 * https://mcuoneclipse.com/2012/12/05/kl25z-and-i2c-missing-repeated-start-condition/
 * https://www.nxp.com/docs/en/errata/KINETIS_L_2N97F.pdf
 */
static System_Errors Iic_setBaudrate (Iic_DeviceHandle dev, uint32_t speed)
{
    uint32_t busClk;
    uint32_t i2cClk;
    uint32_t error;
    uint32_t bestError = 0xFFFF;
    uint8_t bestIcr = 0xFF;
    uint8_t icr;
    uint8_t mul;
    uint16_t slt;

    busClk = Clock_getOutputValue(CLOCK_OUTPUT_BUS);

    for (icr = 0; icr < sizeof(Iic_sclDivTab) / sizeof(Iic_sclDivTab[0]); icr++)
    {
        i2cClk = busClk / Iic_sclDivTab[icr];
//        if (i2cClk > speed)
//        {
//            i2cClk /= 2;
//            if (i2cClk > speed)
//            {
//                i2cClk /= 2;
//                if (i2cClk > speed)
//                    continue;
//            }
//        }
        error = speed - i2cClk;
        if (error < bestError)
        {
            bestError = error;
            bestIcr = icr;
        }
    }

    if (bestIcr == 0xFF)
    {
        return ERRORS_IIC_WRONG_BAUDRATE;
    }

    icr = bestIcr;
    i2cClk = busClk / Iic_sclDivTab[bestIcr];
//    if (i2cClk > speed)
//    {
//        i2cClk /= 2;
//        if (i2cClk > speed)
//        {
//            mul = 2;
//        }
//        else
//        {
//            mul = 1;
//        }
//    }
//    else
//    {
//        mul = 0;
//    }
//
//    dev->regMap->F = I2C_F_MULT(mul) | I2C_F_ICR(icr);
    dev->regMap->F = I2C_F_MULT(0) | I2C_F_ICR(icr);

//    slt = Iic_sclDivTab[icr] / 2 + 1;
    return ERRORS_NO_ERROR;
}

static System_Errors Iic_config (Iic_DeviceHandle dev, Iic_Config* config)
{
    System_Errors err = ERRORS_NO_ERROR;

    err |= ohiassert(IIC_VALID_MODE(config->devType));
    err |= ohiassert(IIC_VALID_ADDRESSMODE(config->addressMode));
    //err |= ohiassert(IIC_VALID_BAUDRATE(config->baudrate));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_PARAM;
    }

    // Save current configuration
    dev->config = *config;

    IIC_DEVICE_DISABLE(dev->regMap);

    // Configure Baudrate
    err = Iic_setBaudrate(dev,dev->config.baudrate);
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_PARAM;
    }

    // TODO: Configure 7bit/10bit


    IIC_DEVICE_ENABLE(dev->regMap);

    return ERRORS_NO_ERROR;
}

System_Errors Iic_init (Iic_DeviceHandle dev, Iic_Config *config)
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

    // Enable peripheral clock if needed
    if (dev->state == IIC_DEVICESTATE_RESET)
    {
        // Turn on clock
        *dev->simScgcPtr |= dev->simScgcBitEnable;

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
        //Iic_deInit(dev);
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
    // Check the I2C instance
    err = ohiassert(IIC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_IIC_WRONG_DEVICE;
    }
    dev->state = IIC_DEVICESTATE_BUSY;

    // Disable the device
    IIC_DEVICE_DISABLE(dev->regMap);

    dev->state = IIC_DEVICESTATE_RESET;
    return err;
}

static System_Errors Iic_waitUntilBusy (Iic_DeviceHandle dev, uint32_t timeout)
{
    while (((dev->regMap->S) & I2C_S_BUSY_MASK) == I2C_S_BUSY_MASK)
    {
        if (System_currentTick() > timeout)
        {
            return ERRORS_IIC_TIMEOUT;
        }
    }
    return ERRORS_NO_ERROR;
}

void Iic_start (Iic_DeviceHandle dev)
{
    dev->regMap->C1 |= I2C_C1_TX_MASK;
    dev->regMap->C1 |= I2C_C1_MST_MASK;
}

void Iic_repeatedStart (Iic_DeviceHandle dev)
{
    dev->regMap->C1 |= I2C_C1_RSTA_MASK;
}

void Iic_stop (Iic_DeviceHandle dev)
{
    dev->regMap->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);

    for (uint8_t i = 0; i < 100; ++i ) __asm ("nop");
}

static System_Errors Iic_waitUntilTransfer (Iic_DeviceHandle dev, uint32_t timeout)
{
    while (((dev->regMap->S) & I2C_S_TCF_MASK) != I2C_S_TCF_MASK)
    {
        if (System_currentTick() > timeout)
        {
            return ERRORS_IIC_TIMEOUT;
        }
    }
    return ERRORS_NO_ERROR;
}

static System_Errors Iic_waitUntilInterrupt (Iic_DeviceHandle dev, uint32_t timeout)
{
    while (((dev->regMap->S) & I2C_S_IICIF_MASK) != I2C_S_IICIF_MASK)
    {
        if (System_currentTick() > timeout)
        {
            return ERRORS_IIC_TIMEOUT;
        }
    }
    // Clear flag
    dev->regMap->S |= I2C_S_IICIF_MASK;
    return ERRORS_NO_ERROR;
}

static inline void __attribute__((always_inline)) Iic_writeByte (Iic_DeviceHandle dev, uint8_t data)
{
    // Set TX mode
    dev->regMap->C1 |= I2C_C1_TX_MASK;

    // Write the data in the register
    dev->regMap->D = data;
}

static inline void __attribute__((always_inline)) Iic_readByte (Iic_DeviceHandle dev, uint8_t *data)
{
    // Read the data in the register
    *data = dev->regMap->D;
}

static System_Errors Iic_getAck (Iic_DeviceHandle dev)
{
    if ((dev->regMap->S & I2C_S_RXAK_MASK) == I2C_S_RXAK_MASK)
    {
        return ERRORS_IIC_TX_ACK_NOT_RECEIVED;
    }
    else
    {
        return ERRORS_IIC_TX_ACK_RECEIVED;
    }
}

static void Iic_sendNack (Iic_DeviceHandle dev)
{
    dev->regMap->C1 |= I2C_C1_TXAK_MASK;
}

static void Iic_sendAck (Iic_DeviceHandle dev)
{
    dev->regMap->C1 &= ~I2C_C1_TXAK_MASK;
}

static void Iic_setReceiveMode (Iic_DeviceHandle dev)
{
    dev->regMap->C1 &= ~I2C_C1_TX_MASK;
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
        devAddress = ((devAddress << 1u) & 0x00FEu);
    }

    // Check if the device is busy
    // Wait 25ms before enter into timeout!
    //err = Iic_waitUntilBusy(dev,(System_currentTick() + 25u));
    //if (err != ERRORS_NO_ERROR) goto i2cerror;

    // Init timeout
    uint32_t tickStart = System_currentTick();

    // Save transfer parameter
    dev->tdata = data;
    dev->bufferCount = length;

    Iic_start(dev);

    // Write data
    Iic_writeByte(dev,devAddress);
    // Wait until transmission buffer is empty
    // ??err = Iic_waitUntilTransfer(dev,(tickStart+timeout));
    err = Iic_waitUntilInterrupt(dev,(tickStart+timeout));
    if (err != ERRORS_NO_ERROR) goto i2cerror;
    // check ACK
    err = Iic_getAck(dev);
    if (err != ERRORS_IIC_TX_ACK_RECEIVED) goto i2cerror;


    // Write data
    Iic_writeByte(dev,regAddress);
    // Wait until transmission buffer is empty
    // ??err = Iic_waitUntilTransfer(dev,(tickStart+timeout));
    err = Iic_waitUntilInterrupt(dev,(tickStart+timeout));
    if (err != ERRORS_NO_ERROR) goto i2cerror;
    // check ACK
    err = Iic_getAck(dev);
    if (err != ERRORS_IIC_TX_ACK_RECEIVED) goto i2cerror;

    // Start writing bytes
    do
    {
        // Write data
        Iic_writeByte(dev,*dev->tdata++);
        dev->bufferCount--;

        // Wait until transmission buffer is empty
        // ??err = Iic_waitUntilTransfer(dev,(tickStart+timeout));
        err = Iic_waitUntilInterrupt(dev,(tickStart+timeout));
        if (err != ERRORS_NO_ERROR) goto i2cerror;

        // check ACK
        err = Iic_getAck(dev);
        if (err != ERRORS_IIC_TX_ACK_RECEIVED) goto i2cerror;
    }
    while (dev->bufferCount > 0u);

i2cerror:
    // send stop bit
    Iic_stop(dev);

    return ERRORS_NO_ERROR;
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
    uint8_t writeAddress = 0;
    uint8_t readAddress  = 0;
    if (dev->config.addressMode == IIC_SEVEN_BIT)
    {
        writeAddress = (uint8_t)((devAddress << 1u) & 0x00FEu);
        readAddress  = writeAddress | 0x01;
    }

    // Check if the device is busy
    // Wait 25ms before enter into timeout!
    //err = Iic_waitUntilBusy(dev,(System_currentTick() + 25u));
    //if (err != ERRORS_NO_ERROR) goto i2cerror;

    // Init timeout
    uint32_t tickStart = System_currentTick();

    // Save transfer parameter
    dev->rdata = data;
    dev->bufferCount = length;

    Iic_start(dev);

    // Write data
    Iic_writeByte(dev,writeAddress);
    // Wait until transmission buffer is empty
    // ??err = Iic_waitUntilTransfer(dev,(tickStart+timeout));
    err = Iic_waitUntilInterrupt(dev,(tickStart+timeout));
    if (err != ERRORS_NO_ERROR) goto i2cerror;
    // check ACK
    err = Iic_getAck(dev);
    if (err != ERRORS_IIC_TX_ACK_RECEIVED) goto i2cerror;

    // Write data
    Iic_writeByte(dev,regAddress);
    // Wait until transmission buffer is empty
    // ??err = Iic_waitUntilTransfer(dev,(tickStart+timeout));
    err = Iic_waitUntilInterrupt(dev,(tickStart+timeout));
    if (err != ERRORS_NO_ERROR) goto i2cerror;
    // check ACK
    err = Iic_getAck(dev);
    if (err != ERRORS_IIC_TX_ACK_RECEIVED) goto i2cerror;

    Iic_repeatedStart(dev);

    // Write data
    Iic_writeByte(dev,readAddress);
    // Wait until transmission buffer is empty
    // ??err = Iic_waitUntilTransfer(dev,(tickStart+timeout));
    err = Iic_waitUntilInterrupt(dev,(tickStart+timeout));
    if (err != ERRORS_NO_ERROR) goto i2cerror;
    // check ACK
    err = Iic_getAck(dev);
    if (err != ERRORS_IIC_TX_ACK_RECEIVED) goto i2cerror;

    Iic_setReceiveMode(dev);

    // If rxSize equals 1, configure to send NAK
    // FIX by NXP driver
    if (length == 1)
    {
        Iic_sendNack(dev);
    }

    // Dummy read to start transfer
    Iic_readByte(dev,dev->rdata);

    // Start reading bytes
    do
    {
        // Wait until data transfer complete.
        err = Iic_waitUntilInterrupt(dev,(tickStart+timeout));
        if (err != ERRORS_NO_ERROR) goto i2cerror;

        // send NACK
        dev->bufferCount--;
        if (dev->bufferCount == 1)
        {
            Iic_sendNack(dev);
        }

        // Write data
        Iic_readByte(dev,dev->rdata);
        dev->rdata++;

    }
    while (dev->bufferCount > 0u);

    Iic_stop(dev);

i2cerror:
    dev->regMap->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);
    return ERRORS_NO_ERROR;
}

#if 0

System_Errors Iic_writeBytes (Iic_DeviceHandle dev, uint8_t address,
        const uint8_t *data, uint8_t length, Iic_StopMode stopRequest)
{
    uint8_t i;
    System_Errors error;

    Iic_start(dev);

    /* Write address */
    I2C_D_REG(dev->regMap) = address;
    error = Iic_waitTxTransfer(dev);
    if (error == ERRORS_IIC_TX_TIMEOUT)
    {
        Iic_stop(dev);
        return error;
    }

    for (i = 0; i < length; ++i)
    {
        I2C_D_REG(dev->regMap) = data[i];
        error = Iic_waitTxTransfer(dev);
        if (error == ERRORS_IIC_TX_TIMEOUT)
        {
            Iic_stop(dev);
            return error;
        }
    }

    if (stopRequest == IIC_STOP)
        Iic_stop(dev);

    return ERRORS_IIC_TX_OK;
}

/* FIXME: Ci sono due implementazioni diverse */

System_Errors Iic_readByte (Iic_DeviceHandle dev, uint8_t *data,
        Iic_LastByteMode lastByte)
{
    /* Set RX mode */
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TX_MASK;

    /* First dummy read if necessary... */
    if (Iic_firstRead)
    {
        Iic_sendAck(dev);
        (void) I2C_D_REG(dev->regMap);
        Iic_waitRxTransfer(dev);
        Iic_firstRead = 0;
    }

    if (lastByte == IIC_LAST_BYTE)
    {
        /* Set to TX mode */
        I2C_C1_REG(dev->regMap) |= I2C_C1_TX_MASK;
        *data = (I2C_D_REG(dev->regMap) & 0xFF);
        Iic_sendNack(dev);
        return ERRORS_IIC_RX_OK;
    }
    else
    {
        Iic_sendAck(dev);
        *data = (I2C_D_REG(dev->regMap) & 0xFF);
    }

    return Iic_waitRxTransfer(dev);
}

System_Errors Iic_readBytes (Iic_DeviceHandle dev, uint8_t address,
        uint8_t *data, uint8_t length, Iic_StopMode stopRequest)
{
    uint8_t i;
    System_Errors errors;

    Iic_start(dev);

    /* Write address */
    I2C_D_REG(dev->regMap) = address;
    errors = Iic_waitTxTransfer(dev);
    if (errors == ERRORS_IIC_TX_TIMEOUT)
    {
        Iic_stop(dev);
        return ERRORS_IIC_TX_ERROR;
    }

    /* Set RX mode */
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TX_MASK;

    for (i = 0; i < length; ++i)
    {
        /* Ack type */
        if (i == (length-1))
            Iic_sendNack(dev);
        else
            Iic_sendAck(dev);

        /* Delete first dummy read */
        if (i == 0)
            (void) I2C_D_REG(dev->regMap);
        else
            data[i-1] = (I2C_D_REG(dev->regMap) & 0xFF);

        if (Iic_waitRxTransfer(dev) == ERRORS_IIC_RX_TIMEOUT)
        {
            Iic_stop(dev);
            return ERRORS_IIC_RX_TIMEOUT;
        }
    }

    if (stopRequest == IIC_STOP)
        Iic_stop(dev);

    /* Last read */
    data[i-1] = (I2C_D_REG(dev->regMap) & 0xFF);

    return ERRORS_IIC_RX_OK;
}

System_Errors Iic_waitTransfer (Iic_DeviceHandle dev)
{
    uint16_t i, timeoutResult = 0;

    /* Wait for interrupt flag */
    for (i = 0; i < 1000; ++i)
    {
        if (I2C_S_REG(dev->regMap) & I2C_S_IICIF_MASK)
        {
            timeoutResult = 1;
            break;
        }
    }
    if (timeoutResult == 0)
        return ERRORS_IIC_TIMEOUT;

    /* Reset value */
    I2C_S_REG(dev->regMap) |= I2C_S_IICIF_MASK;
    return ERRORS_IIC_OK;
//	while((I2C_S_REG(dev->regMap) & I2C_S_IICIF_MASK)==0) {}
}

System_Errors Iic_setSclTimeout (Iic_DeviceHandle dev, uint32_t usDelay)
{
    uint32_t ticks = usDelay/Iic_usSclTimeout;

    if (ticks > 65535)
        return ERRORS_IIC_SCLTIMEOUT_TOO_LARGE;

    I2C_SMB_REG(dev->regMap) &= ~I2C_SMB_TCKSEL_MASK;
    I2C_SLTL_REG(dev->regMap) = ticks & 0x000000FF;
    I2C_SLTH_REG(dev->regMap) = ((ticks >> 8) & 0x000000FF);

    /* Just for debug! */
    dev->sclTimeout = (0x0000FFFF & ticks);

    return ERRORS_NO_ERROR;
}

void Iic_resetSclTimeout (Iic_DeviceHandle dev)
{
    I2C_SMB_REG(dev->regMap) |= I2C_SMB_SLTF_MASK;
    I2C_SLTL_REG(dev->regMap) = 0;
    I2C_SLTH_REG(dev->regMap) = 0;

    /* Just for debug! */
    dev->sclTimeout = 0;
}

System_Errors Iic_isToggleSclTimeout (Iic_DeviceHandle dev)
{
    if (I2C_SMB_REG(dev->regMap) & I2C_SMB_SLTF_MASK)
        return ERRORS_IIC_SCLTIMEOUT;
    else
        return ERRORS_IIC_NO_SCLTIMEOUT;
}
#endif

#endif /* LIBOHIBOARD_KL15Z4 */

#endif /* LIBOHIBOARD_IIC */
