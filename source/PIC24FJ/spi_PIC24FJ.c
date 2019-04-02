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
 * @file libohiboard/source/PIC24FJ/spi_PIC24FJ.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief SPI implementations for PIC24FJ Series.
 */

#include "platforms.h"

#if defined (LIBOHIBOARD_SPI)

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

#include "utility.h"
#include "gpio.h"
#include "clock.h"
#include "interrupt.h"

#if defined (LIBOHIBOARD_PIC24FJ)

/**
 * @brief Enable the SPI peripheral
 */
#define SPI_DEVICE_ENABLE(REGMAP)        (REGMAP->SPICON1L |= _SPI1CON1L_SPIEN_MASK)
/**
 * @brief Disable the SPI peripheral
 */
#define SPI_DEVICE_DISABLE(REGMAP)       (REGMAP->SPICON1L &= ~_SPI1CON1L_SPIEN_MASK)

#define SPI_DEVICE_STOP(REGMAP)         do {                                       \
                                             REGMAP->SPICON1L = 0;                 \
                                             REGMAP->SPICON1H = 0;                 \
                                             REGMAP->SPICON2L = 0;                 \
                                        } while (0)
    
#define SPI_ENABLE_INTERRUPTS(DEVICE)   do {                                       \
                                            Interrupt_enable(DEVICE->isrGeneral);  \
                                            Interrupt_enable(DEVICE->isrTx);       \
                                            Interrupt_enable(DEVICE->isrRx);       \
                                        } while (0)

#define SPI_DISABLE_INTERRUPTS(DEVICE)  do {                                       \
                                            Interrupt_disable(DEVICE->isrGeneral); \
                                            Interrupt_disable(DEVICE->isrTx);      \
                                            Interrupt_disable(DEVICE->isrRx);      \
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
#define SPI_VALID_DATASIZE(DATASIZE) (((DATASIZE) == SPI_DATASIZE_32BIT) || \
                                      ((DATASIZE) == SPI_DATASIZE_16BIT) || \
                                      ((DATASIZE) == SPI_DATASIZE_8BIT))

/**
 * Checks if the SPI Slave Select management value is in allowed range.
 * This parameter can be a value of @ref Spi_SSManagement
 */
#define SPI_VALID_SSMANAGEMENT(NSS) (((NSS) == SPI_SSMANAGEMENT_SOFTWARE) || \
                                     ((NSS) == SPI_SSMANAGEMENT_HARDWARE))

#define SPI_MAX_PINS           8

/**
 *
 */
typedef struct _Spi_Device
{
    SPI_TypeDef* regmap;                           /**< Device memory pointer */

    volatile uint16_t* pmdRegisterPtr;     /**< Register for device enabling. */
    uint16_t pmdRegisterEnable;        /**< Register mask for current device. */

    volatile uint16_t* ppsSinRegisterPtr;
    uint16_t ppsSinRegisterMask;
    uint16_t ppsSinRegisterPosition;

    Gpio_PpsOutputFunction ppsSoutRegisterValue;

    Gpio_PpsOutputFunction ppsSckRegisterValue;

    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Spi_DeviceState state;                     /**< Current peripheral state. */

    Spi_Config config;

    Interrupt_Vector isrGeneral;
    Interrupt_Vector isrTx;
    Interrupt_Vector isrRx;

} Spi_Device;

#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)

#define SPI_IS_DEVICE(DEVICE) (((DEVICE) == OB_SPI1) || \
                               ((DEVICE) == OB_SPI2) || \
                               ((DEVICE) == OB_SPI3))

#endif 

static Spi_Device spi1 = 
{
        .regmap                  = SPI1,

        .pmdRegisterPtr          = &PMD->PMD1,
        .pmdRegisterEnable       = _PMD1_SPI1MD_MASK,

        .ppsSinRegisterPtr       = &PPS->RPINR[20],
        .ppsSinRegisterMask      = _RPINR20_SDI1R_MASK,
        .ppsSinRegisterPosition  = _RPINR20_SDI1R_POSITION,

        .ppsSoutRegisterValue    = GPIO_PPSOUTPUTFUNCTION_SDO1,

        .ppsSckRegisterValue     = GPIO_PPSOUTPUTFUNCTION_SCK1OUT,

        .isrGeneral              = INTERRUPT_SPI1,
        .isrTx                   = INTERRUPT_SPI1_TX,
        .isrRx                   = INTERRUPT_SPI1_RX,
};
Spi_DeviceHandle OB_SPI1 = &spi1;

static Spi_Device spi2 = 
{
        .regmap                 = SPI2,

        .pmdRegisterPtr         = &PMD->PMD1,
        .pmdRegisterEnable      = _PMD1_SPI2MD_MASK,

        .ppsSinRegisterPtr      = &PPS->RPINR[22],
        .ppsSinRegisterMask     = _RPINR22_SDI2R_MASK,
        .ppsSinRegisterPosition = _RPINR22_SDI2R_POSITION,

        .ppsSoutRegisterValue   = GPIO_PPSOUTPUTFUNCTION_SDO2,

        .ppsSckRegisterValue    = GPIO_PPSOUTPUTFUNCTION_SCK2OUT,

        .isrGeneral             = INTERRUPT_SPI2,
        .isrTx                  = INTERRUPT_SPI2_TX,
        .isrRx                  = INTERRUPT_SPI2_RX,
};
Spi_DeviceHandle OB_SPI2 = &spi2;

static Spi_Device spi3 = 
{
        .regmap                 = SPI3,

        .pmdRegisterPtr         = &PMD->PMD6,
        .pmdRegisterEnable      = _PMD6_SPI3MD_MASK,

        .ppsSinRegisterPtr      = &PPS->RPINR[28],
        .ppsSinRegisterMask     = _RPINR28_SDI3R_MASK,
        .ppsSinRegisterPosition = _RPINR28_SDI3R_POSITION,

        .ppsSoutRegisterValue   = GPIO_PPSOUTPUTFUNCTION_SDO3,

        .ppsSckRegisterValue    = GPIO_PPSOUTPUTFUNCTION_SCK3OUT,

        .isrGeneral             = INTERRUPT_SPI3,
        .isrTx                  = INTERRUPT_SPI3_TX,
        .isrRx                  = INTERRUPT_SPI3_RX,
};
Spi_DeviceHandle OB_SPI3 = &spi3;

static System_Errors Spi_setSoutPin(Spi_DeviceHandle dev, Spi_SoutPins soutPin)
{
    uint8_t regIndex = soutPin / 2;
    uint8_t regPosIndex = ((soutPin % 2 ) * 8);

    UTILITY_MODIFY_REGISTER(PPS->RPOR[regIndex], (0x3F << regPosIndex),(dev->ppsSoutRegisterValue << regPosIndex));

    return ERRORS_NO_ERROR;
}

static System_Errors Spi_setSinPin(Spi_DeviceHandle dev, Spi_SinPins sinPin)
{
    UTILITY_MODIFY_REGISTER(*dev->ppsSinRegisterPtr, dev->ppsSinRegisterMask, (sinPin << dev->ppsSinRegisterPosition));

    return ERRORS_NO_ERROR;
}

static System_Errors Spi_setSckPin(Spi_DeviceHandle dev, Spi_SckPins sckPin)
{
    uint8_t regIndex = sckPin / 2;
    uint8_t regPosIndex = ((sckPin % 2 ) * 8);

    UTILITY_MODIFY_REGISTER(PPS->RPOR[regIndex], (0x3F << regPosIndex), (dev->ppsSckRegisterValue << regPosIndex));

    return ERRORS_NO_ERROR;
}
#if 0
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
#endif

System_Errors Spi_setBaudrate (Spi_DeviceHandle dev, uint32_t speed)
{
    uint32_t peripheralClock = Clock_getOutputValue(CLOCK_OUTPUT_PERIPHERAL);

    if (peripheralClock != 0u)
    {
        uint16_t brg = (peripheralClock / (2 * speed)) - 1;
        uint16_t baudRate = (peripheralClock / (2 *(brg + 1)));
        uint32_t difference = ((uint32_t)baudRate > (uint32_t)speed)?((uint32_t)baudRate - (uint32_t)speed):((uint32_t)speed - (uint32_t)baudRate);

        if (difference < 1000000)
        {
            // The peripheral must be disabled:
            // Changing the BRG value when SPIEN = 1 causes undefined behavior
            SPI_DEVICE_DISABLE(dev->regmap);
            UTILITY_MODIFY_REGISTER(dev->regmap->SPIBRGL, _SPI1BRGL_BRG_MASK, brg);
            SPI_DEVICE_ENABLE(dev->regmap);
        }
        else
        {
            return ERRORS_SPI_BAUDRATE_NOT_FOUND;
        }
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
    err |= ohiassert(SPI_VALID_DATASIZE(config->datasize));
    err |= ohiassert(SPI_VALID_SSMANAGEMENT(config->ssManagement));
    err |= ohiassert(SPI_VALID_CPHA(config->sckPhase));
    err |= ohiassert(SPI_VALID_CPOL(config->sckPolarity));

    if (err != ERRORS_NO_ERROR)
        return ERRORS_SPI_WRONG_PARAM;

    // Save current configuration
    dev->config = *config;

    // Disable the pheripheral and reset settings
    SPI_DEVICE_STOP(dev->regmap);

    // Disable interrupts
    SPI_DISABLE_INTERRUPTS(dev);

    // Configure peripheral
    // Configure Mode
    dev->regmap->SPICON1L = dev->regmap->SPICON1L & (~(_SPI1CON1L_MSTEN_MASK));
    if (config->devType == SPI_MASTER_MODE)
    {        
        // Setup baudrate
        Spi_setBaudrate(dev,config->baudrate);
        // Clear Receive Overflow Status bit
        dev->regmap->SPISTATL &= ~_SPI1STATL_SPIROV_MASK;
        // Configure as Master
        dev->regmap->SPICON1L |= _SPI1CON1L_MSTEN_MASK;
    }
    else
    {
        // Clear Receive Overflow Status bit
        dev->regmap->SPISTATL &= ~_SPI1STATL_SPIROV_MASK;
    }

    // Configure CPOL and CPHA
    dev->regmap->SPICON1L = dev->regmap->SPICON1L & (~(_SPI1CON1L_CKE_MASK | _SPI1CON1L_CKP_MASK));
    if (config->sckPolarity == SPI_SCK_INACTIVE_STATE_HIGH)
    {
        dev->regmap->SPICON1L |= _SPI1CON1L_CKP_MASK;
    }
    if (config->sckPhase == SPI_SCK_LEADING_EDGE_DATA_CHANGED)
    {
        dev->regmap->SPICON1L |= _SPI1CON1L_CKE_MASK;
    }

    // Configure datasize
    dev->regmap->SPICON1L = dev->regmap->SPICON1L & (~(_SPI1CON1L_MODE32_MASK | _SPI1CON1L_MODE16_MASK));
    switch (config->datasize)
    {
    case SPI_DATASIZE_16BIT:
        dev->regmap->SPICON1L |= _SPI1CON1L_MODE16_MASK;
        break;
    case SPI_DATASIZE_32BIT:
        dev->regmap->SPICON1L |= _SPI1CON1L_MODE32_MASK;
        break;
    case SPI_DATASIZE_8BIT:
    default:
        break;
    }

    // Configure SS management
    dev->regmap->SPICON1H = dev->regmap->SPICON1H & (~(_SPI1CON1H_MSSEN_MASK));
    if (config->ssManagement == SPI_SSMANAGEMENT_HARDWARE)
    {
        dev->regmap->SPICON1H |= _SPI1CON1H_MSSEN_MASK;
    }

    // TODO: setup interrupt

    // Enable the pheripheral
    SPI_DEVICE_ENABLE(dev->regmap);

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
        // Enable peripheral
        UTILITY_CLEAR_REGISTER_BIT(*dev->pmdRegisterPtr, dev->pmdRegisterEnable);

        // unlock PPS
        __builtin_write_OSCCONL(OSCCON & 0xbf);

        // Enable pins
        if (config->sinPin != SPI_PINS_SINNONE)
            Spi_setSinPin(dev, config->sinPin);

        if (config->soutPin != SPI_PINS_SOUTNONE)
            Spi_setSoutPin(dev, config->soutPin);

        if (config->sckPin != SPI_PINS_SCKNONE)
            Spi_setSckPin(dev, config->sckPin);

#if 0
        if ((config->pcs0Pin != SPI_PINS_PCSNONE) && (config->ssManagement != SPI_SSMANAGEMENT_SOFTWARE))
            Spi_setNssPin(dev, config->pcs0Pin);
#endif

        // lock   PPS
        __builtin_write_OSCCONL(OSCCON | 0x40);  
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

System_Errors Spi_deInit (Spi_DeviceHandle dev)
{
    // Disable peripheral
    UTILITY_SET_REGISTER_BIT(*dev->pmdRegisterPtr, dev->pmdRegisterEnable);

    dev->state = SPI_DEVICESTATE_RESET;
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

static System_Errors spi_exchange16bit(Spi_DeviceHandle dev, const uint16_t *txData, uint16_t *rxData, uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;
    
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->SPISTATL,_SPI1STATL_SPIRBF_MASK) > 0)
    {
//        if (System_currentTick() > timeoutEnd)
//        {
//            err = ERRORS_SPI_TIMEOUT_TX;
//            // Release the device.
//            goto spierror;
//        }
    }

    dev->regmap->SPIBUFL = *((uint16_t*)txData);

    while (UTILITY_READ_REGISTER_BIT(dev->regmap->SPISTATL,_SPI1STATL_SPIRBE_MASK) > 0)
    {
//        if (System_currentTick() > timeoutEnd)
//        {
//            err = ERRORS_SPI_TIMEOUT_TX;
//            // Release the device.
//            goto spierror;
//        }
    }

    *((uint16_t*)rxData) = dev->regmap->SPIBUFL;

//spierror:
    return err;
}

static System_Errors spi_exchange8bit(Spi_DeviceHandle dev, const uint8_t *txData, uint8_t *rxData, uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;
    
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->SPISTATL,_SPI1STATL_SPIRBF_MASK) > 0)
    {
//        if (System_currentTick() > timeoutEnd)
//        {
//            err = ERRORS_SPI_TIMEOUT_TX;
//            // Release the device.
//            goto spierror;
//        }
    }

    dev->regmap->SPIBUFL = *((uint8_t*)txData);

    while (UTILITY_READ_REGISTER_BIT(dev->regmap->SPISTATL,_SPI1STATL_SPIRBE_MASK) > 0)
    {
//        if (System_currentTick() > timeoutEnd)
//        {
//            err = ERRORS_SPI_TIMEOUT_TX;
//            // Release the device.
//            goto spierror;
//        }
    }

    *((uint8_t*)rxData) = dev->regmap->SPIBUFL;

//spierror:
    return err;
}

System_Errors Spi_read (Spi_DeviceHandle dev, uint8_t* data, uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;
    dev->state = SPI_DEVICESTATE_BUSY;

    switch(dev->config.datasize)
    {
        default:
        case SPI_DATASIZE_8BIT:
        {
            const uint8_t txData = 0;
            err = spi_exchange8bit(dev, &txData, data, timeout);
        }
        break;

        case SPI_DATASIZE_16BIT:
        {
            const uint16_t txData = 0;
            err = spi_exchange16bit(dev, &txData, (uint16_t*)data, timeout);
        }
        break;

        case SPI_DATASIZE_32BIT:
        {

        }
        break;
    }

    dev->state = SPI_DEVICESTATE_READY;
    return err;
}

System_Errors Spi_write (Spi_DeviceHandle dev, const uint8_t* data, uint32_t timeout)
{
    System_Errors err = ERRORS_NO_ERROR;
    dev->state = SPI_DEVICESTATE_BUSY;

    switch(dev->config.datasize)
    {
        default:
        case SPI_DATASIZE_8BIT:
        {
            uint8_t rxData = 0;
            err = spi_exchange8bit(dev, data, &rxData, timeout);
        }
        break;

        case SPI_DATASIZE_16BIT:
        {
            uint16_t rxData = 0;
            err = spi_exchange16bit(dev, (uint16_t*)data, &rxData, timeout);
        }
        break;

        case SPI_DATASIZE_32BIT:
        {

        }
        break;
    }

    dev->state = SPI_DEVICESTATE_READY;
    return err;
}

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_SPI
