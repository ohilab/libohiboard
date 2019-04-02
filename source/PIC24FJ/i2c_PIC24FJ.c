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
 * @file libohiboard/source/PIC24FJ/i2c_PIC24FJ.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C implementations for PIC24FJ series.
 */

#ifdef LIBOHIBOARD_IIC

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_PIC24FJ)

#include "i2c.h"

#include "utility.h"
#include "gpio.h"
#include "clock.h"
#include "interrupt.h"

#define IIC_MAX_BAUDRATE                  1000000u
#define IIC_MAX_SCL_TICK                  256u

#define IIC_MAX_NBYTE_SIZE                255u

/**
 * @brief Enable the I2C peripheral
 */
#define IIC_DEVICE_ENABLE(REGMAP)        (REGMAP->I2CCON1 |= _I2C1CON1_I2CEN_MASK)
/**
 * @brief Disable the I2C peripheral
 */
#define IIC_DEVICE_DISABLE(REGMAP)       (REGMAP->I2CCON1 &= ~_I2C1CON1_I2CEN_MASK)
    
#define IIC_ENABLE_INTERRUPTS(DEVICE)   do {                                     \
                                            Interrupt_enable(DEVICE->isrMaster); \
                                            Interrupt_enable(DEVICE->isrSlave);  \
                                        } while (0)

#define IIC_DISABLE_INTERRUPTS(DEVICE)  do {                                      \
                                            Interrupt_disable(DEVICE->isrMaster); \
                                            Interrupt_disable(DEVICE->isrSlave);  \
                                        } while (0)

#define IIC_VALID_ADDRESSMODE(ADDRESSMODE) (((ADDRESSMODE) == IIC_SEVEN_BIT)  || \
                                            ((ADDRESSMODE) == IIC_TEN_BIT))

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

    volatile uint16_t* pmdRegisterPtr;     /**< Register for device enabling. */
    uint16_t pmdRegisterEnable;        /**< Register mask for current device. */

    Iic_Config config;
    
    Iic_DeviceState state;                     /**< Current peripheral state. */
    
    Interrupt_Vector isrMaster;
    Interrupt_Vector isrSlave;

    // Write/Read useful buffer and counter
    uint8_t* rdata;                      /**< Pointer to I2C reception buffer */
    const uint8_t* tdata;             /**< Pointer to I2C transmission buffer */
    uint16_t bufferSize;                        /**< I2C buffer transfer size */
    volatile uint16_t bufferCount;           /**< I2C buffer transfer counter */
    
} Iic_Device;

#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)

#define IIC_IS_DEVICE(DEVICE) (((DEVICE) == OB_IIC1)  || \
                               ((DEVICE) == OB_IIC2)  || \
                               ((DEVICE) == OB_IIC3))

static Iic_Device iic1 =
{
        .regmap              = I2C1,

        .pmdRegisterPtr      = &PMD->PMD2,
        .pmdRegisterEnable   = _PMD2_IC1MD_MASK,
        
        .isrMaster           = INTERRUPT_I2C1_MASTER,
        .isrSlave            = INTERRUPT_I2C1_SLAVE,
        
        .state               = IIC_DEVICESTATE_RESET,
};
Iic_DeviceHandle OB_IIC1 = &iic1;

static Iic_Device iic2 =
{
        .regmap              = I2C2,

        .pmdRegisterPtr      = &PMD->PMD2,
        .pmdRegisterEnable   = _PMD2_IC2MD_MASK,
        
        .isrMaster           = INTERRUPT_I2C2_MASTER,
        .isrSlave            = INTERRUPT_I2C2_SLAVE,
        
        .state               = IIC_DEVICESTATE_RESET,
};
Iic_DeviceHandle OB_IIC2 = &iic2;

static Iic_Device iic3 =
{
        .regmap              = I2C3,

        .pmdRegisterPtr      = &PMD->PMD2,
        .pmdRegisterEnable   = _PMD2_IC3MD_MASK,
        
        .isrMaster           = INTERRUPT_I2C3_MASTER,
        .isrSlave            = INTERRUPT_I2C3_SLAVE,
        
        .state               = IIC_DEVICESTATE_RESET,
};
Iic_DeviceHandle OB_IIC3 = &iic3;

#endif

static System_Errors Iic_setBaudrate (Iic_DeviceHandle dev, uint32_t baudrate)
{
    uint32_t frequency = 0;
    uint32_t scaled = 0;
    uint32_t diff = 0xFFFFFFFFu;
    uint16_t br = 2u;

    frequency = Clock_getOutputValue(CLOCK_OUTPUT_PERIPHERAL);

    // Current clock is different from 0
    if (frequency != 0u)
    {
        for (uint16_t i = 2; i < 65536; ++i)
        {
            // Divide by possible prescaler
            scaled = frequency / ((i + 2) * 2);

            // Check if frequency scaled is grater then baudrate
            if (scaled < baudrate)
            {
                if ((baudrate - scaled) < diff)
                {
                    diff = baudrate - scaled;
                    br = i;
                }
            }
            else if (scaled > baudrate)
            {
                if ((scaled - baudrate) < diff)
                {
                    diff = scaled - baudrate;
                    br = i;
                }
            }
            else
            {
                br = i;
                // The prescaled value is equal to request baudrate!
                break;
            }
        }
        
        if (diff == 0xFFFFFFFFul)
            return ERRORS_IIC_WRONG_BAUDRATE;

        // Save new prescaler into register
        dev->regmap->I2CBRG = br;
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
    err |= ohiassert(IIC_VALID_BAUDRATE(config->baudrate));
    if (err != ERRORS_NO_ERROR)
        return ERRORS_IIC_WRONG_PARAM;

    dev->config = *config;
    
    // Disable the device
    IIC_DEVICE_DISABLE(dev->regmap);

    // Configure Baudrate
    err = Iic_setBaudrate(dev,config->baudrate);
    if (err != ERRORS_NO_ERROR)
        return ERRORS_IIC_WRONG_PARAM;

    // Save address mode (7-bit or 10-bit)
    dev->regmap->I2CCON1 = dev->regmap->I2CCON1 & (~(_I2C1CON1_A10M_MASK));
    if (config->addressMode == IIC_TEN_BIT)
    {
        dev->regmap->I2CCON1 |= _I2C1CON1_A10M_MASK;
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
    
    // Enable peripheral clock if needed
    if (dev->state == IIC_DEVICESTATE_RESET)
    {
        // Enable peripheral
        UTILITY_CLEAR_REGISTER_BIT(*dev->pmdRegisterPtr, dev->pmdRegisterEnable);
        
        // Enable pins
        // Not used because the pins are fixed for I2C
#if 0
        if (config->sclPin != IIC_PINS_SCLNONE)
            Iic_setSclPin(dev, config->sclPin, config->pullupEnable);

        if (config->sdaPin != IIC_PINS_SDANONE)
            Iic_setSdaPin(dev, config->sdaPin, config->pullupEnable);
#endif
    }
    dev->state = IIC_DEVICESTATE_BUSY;
    
    // Configure the peripheral
    err = Iic_config(dev,config);
    if (err != ERRORS_NO_ERROR)
    {
        // FIXME: Call deInit?
        return err;
    }

#if 0
    // Clear NACK flag
    dev->regmap->ICR |= I2C_ICR_NACKCF_Msk;

    // Clear STOPF flag
    dev->regmap->ICR |= I2C_ICR_STOPCF_Msk;
#endif
    
    dev->state = IIC_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Iic_deInit (Iic_DeviceHandle dev)
{
    // Check the I2C device
    if (dev == NULL)
    {
        return ERRORS_IIC_NO_DEVICE;
    }
    // Check the I2C instance
    if (ohiassert(IIC_IS_DEVICE(dev) != ERRORS_NO_ERROR))
    {
        return ERRORS_IIC_WRONG_DEVICE;
    }

    // Disable the device
    IIC_DEVICE_DISABLE(dev->regmap);

    // Clear configuration registers
    dev->regmap->I2CCON1 = 0;
    dev->regmap->I2CCON2 = 0;
    
    // Disable interrupt
    IIC_DISABLE_INTERRUPTS(dev);
    
    // Disable peripheral
    UTILITY_SET_REGISTER_BIT(*dev->pmdRegisterPtr, dev->pmdRegisterEnable);
    
    
    dev->state = IIC_DEVICESTATE_RESET;
    return ERRORS_NO_ERROR;
}

#define IIC_IDLE_MASK (_I2C1CON1_SEN_MASK  | \
                       _I2C1CON1_RSEN_MASK | \
                       _I2C1CON1_PEN_MASK  | \
                       _I2C1CON1_RCEN_MASK | \
                       _I2C1CON1_ACKEN_MASK)

static inline void __attribute__((always_inline)) Iic_waitUntilIdle (Iic_DeviceHandle dev, uint32_t timeout)
{
    while (((dev->regmap->I2CCON1 & IIC_IDLE_MASK) > 0) || 
           ((dev->regmap->I2CSTAT & _I2C1STAT_TRSTAT_MASK) > 0));
}

#if 0
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
#endif
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

    // Check if the device is busy
    // FIXME: the timeout is not usable
    Iic_waitUntilIdle(dev,0);

    // Save transfer parameter
    dev->tdata = data;
    dev->bufferCount = length;

    // Send start conditions
    Iic_start(dev);
    Iic_waitUntilIdle(dev,0);
    
    // Write device address
    // Wait till data is transmitted.
    dev->regmap->I2CTRN = ((devAddress << 1) & 0x00FF);
    if (dev->regmap->I2CSTAT & _I2C1STAT_IWCOL_MASK);
    Iic_waitUntilIdle(dev,0);
    // Check ACK
    if (dev->regmap->I2CSTAT & _I2C1STAT_ACKSTAT_MASK)
    {
        err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
        goto i2cerror;
    }

    // Write register/memory address
    if (addressSize == IIC_REGISTERADDRESSSIZE_8BIT)
    {
        dev->regmap->I2CTRN = (regAddress & 0x00FF);
        // Check collision and idle status
        if (dev->regmap->I2CSTAT & _I2C1STAT_IWCOL_MASK);
        Iic_waitUntilIdle(dev,0);
        // Check ACK
        if (dev->regmap->I2CSTAT & _I2C1STAT_ACKSTAT_MASK)
        {
            err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
            goto i2cerror;
        }
    }
    else
    {
        // MSB part of register address
        dev->regmap->I2CTRN = (uint8_t)((regAddress & 0xFF00u) >> 8u);
        // Check collision and idle status
        if (dev->regmap->I2CSTAT & _I2C1STAT_IWCOL_MASK);
        Iic_waitUntilIdle(dev,0);
        // Check ACK
        if (dev->regmap->I2CSTAT & _I2C1STAT_ACKSTAT_MASK)
        {
            err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
            goto i2cerror;
        }
        
        // LSB part of register address        
        dev->regmap->I2CTRN = (uint8_t)(regAddress & 0x00FFu);
        // Check collision and idle status
        if (dev->regmap->I2CSTAT & _I2C1STAT_IWCOL_MASK);
        Iic_waitUntilIdle(dev,0);
        // Check ACK
        if (dev->regmap->I2CSTAT & _I2C1STAT_ACKSTAT_MASK)
        {
            err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
            goto i2cerror;
        }
    }

    // Start writing bytes
    do
    {
        // Write data from RXDR
        dev->regmap->I2CTRN = (uint8_t)(*dev->tdata++);
        dev->bufferCount--;

        if (dev->regmap->I2CSTAT & _I2C1STAT_IWCOL_MASK);
        Iic_waitUntilIdle(dev,0);
        if (dev->regmap->I2CSTAT & _I2C1STAT_ACKSTAT_MASK)
        {
            err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
            goto i2cerror;
        }
    }
    while (dev->bufferCount > 0u);

i2cerror:

    // Send stop conditions
    Iic_stop(dev);
    // Check idle status...
    Iic_waitUntilIdle(dev,0);

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

    // Check if the device is busy
    // FIXME: the timeout is not usable
    Iic_waitUntilIdle(dev,0);

    // Save transfer parameter
    dev->rdata = data;
    dev->bufferCount = length;

    // Send start conditions
    Iic_start(dev);
    Iic_waitUntilIdle(dev,0);

    // Write device address
    // Wait till data is transmitted.
    dev->regmap->I2CTRN = ((devAddress << 1) & 0x00FF);
    // Check collision and idle status
    if (dev->regmap->I2CSTAT & _I2C1STAT_IWCOL_MASK);
    Iic_waitUntilIdle(dev,0);
    // Check ACK
    if (dev->regmap->I2CSTAT & _I2C1STAT_ACKSTAT_MASK)
    {
        err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
        goto i2cerror;
    }

    // Write register/memory address
    if (addressSize == IIC_REGISTERADDRESSSIZE_8BIT)
    {
        dev->regmap->I2CTRN = (regAddress & 0x00FF);
        // Check collision and idle status
        if (dev->regmap->I2CSTAT & _I2C1STAT_IWCOL_MASK);
        Iic_waitUntilIdle(dev,0);
        // Check ACK
        if (dev->regmap->I2CSTAT & _I2C1STAT_ACKSTAT_MASK)
        {
            err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
            goto i2cerror;
        }
    }
    else
    {
        // MSB part of register address
        dev->regmap->I2CTRN = (uint8_t)((regAddress & 0xFF00u) >> 8u);
        // Check collision and idle status
        if (dev->regmap->I2CSTAT & _I2C1STAT_IWCOL_MASK);
        Iic_waitUntilIdle(dev,0);
        // Check ACK
        if (dev->regmap->I2CSTAT & _I2C1STAT_ACKSTAT_MASK)
        {
            err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
            goto i2cerror;
        }

        // LSB part of register address        
        dev->regmap->I2CTRN = (uint8_t)(regAddress & 0x00FFu);
        // Check collision and idle status
        if (dev->regmap->I2CSTAT & _I2C1STAT_IWCOL_MASK);
        Iic_waitUntilIdle(dev,0);
        // Check ACK
        if (dev->regmap->I2CSTAT & _I2C1STAT_ACKSTAT_MASK)
        {
            err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
            goto i2cerror;
        }        
    }

    // Send repeated start...
    Iic_repeatedStart(dev);
    Iic_waitUntilIdle(dev,0);

    // Write device address for reading
    // Wait till data is transmitted.
    dev->regmap->I2CTRN = (((devAddress << 1) | 0x0001) & 0x00FF);
    if (dev->regmap->I2CSTAT & _I2C1STAT_IWCOL_MASK);
    Iic_waitUntilIdle(dev,0);
    if (dev->regmap->I2CSTAT & _I2C1STAT_ACKSTAT_MASK)
    {
        err = ERRORS_IIC_TX_ACK_NOT_RECEIVED;
        goto i2cerror;
    }

    // Start reading bytes
    do
    {
        // Put the peripheral in receive mode
        // This bit automatically cleared by hardware at end of 8-bit receive data byte
        dev->regmap->I2CCON1 |= _I2C1CON1_RCEN_MASK;
        while (dev->regmap->I2CCON1 & _I2C1CON1_RCEN_MASK);
        // Clear Receive Overflow Flag Bit (I2COV)
        dev->regmap->I2CSTAT &= ~_I2C1STAT_I2COV_MASK;
        // Check idle status...
        Iic_waitUntilIdle(dev,0);

        // Read data from RXDR
        (*dev->rdata++) = dev->regmap->I2CRCV;

        // Send ACK
        if (dev->bufferCount > 1)
        {
            dev->regmap->I2CCON1 &= ~_I2C1CON1_ACKDT_MASK;
            dev->regmap->I2CCON1 |=  _I2C1CON1_ACKEN_MASK;            
        }
        else
        {
            // After last byte, send NACK
            dev->regmap->I2CCON1 |= _I2C1CON1_ACKDT_MASK;
            dev->regmap->I2CCON1 |= _I2C1CON1_ACKEN_MASK;            
        }

        // Check idle status...
        Iic_waitUntilIdle(dev,0);

        dev->bufferCount--;
    }
    while (dev->bufferCount > 0u);

i2cerror:

    // Send stop conditions
    Iic_stop(dev);
    // Check idle status...
    Iic_waitUntilIdle(dev,0);

    return err;
}

void Iic_start (Iic_DeviceHandle dev) 
{
    dev->regmap->I2CCON1 |= _I2C1CON1_SEN_MASK;
}

void Iic_repeatedStart (Iic_DeviceHandle dev) 
{
    dev->regmap->I2CCON1 |= _I2C1CON1_RSEN_MASK;
}

void Iic_stop (Iic_DeviceHandle dev)
{
    dev->regmap->I2CCON1 |= _I2C1CON1_PEN_MASK;
}

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_IIC
