/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Marco Giammarini <m.giammarini@warcomeb.it>
 *	
 * Project: libohiboard
 * Package: I2C
 * Version: 0.0
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>
 ******************************************************************************/

/**
 * @file libohiboard/source/i2c.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C functions implementation
 */


#include "platforms.h"
#include "system.h"
#include "i2c.h"

#define IIC_TIMEOUT_CLOCK_KHZ  SYSTEM_CLOCK_KHZ/64
#define IIC_TIMEOUT_CLOCK_us   (1/IIC_TIMEOUT_CLOCK_KHZ) * 1000

#define IIC_DEF_BAUDRATE       100000 /* 100kbps */

#define IIC_PIN_ENABLED        1
#define IIC_PIN_DISABLED       0

static uint8_t Iic_firstRead = 0;

typedef struct Iic_Device {
    I2C_MemMapPtr 		  regMap;

    uint32_t              baudRate;
    Iic_DeviceType        devType;
    Iic_AddressMode       addressMode;

    uint16_t              sclTimeout;
    
    uint8_t               pinEnabled;
} Iic_Device;

#if defined(MKL15Z4) || defined(FRDMKL25Z)
static Iic_Device iic0 = {
        .regMap           = I2C0_BASE_PTR,

        .baudRate         = IIC_DEF_BAUDRATE,
        .devType          = IIC_MASTER_MODE,
        .addressMode      = IIC_SEVEN_BIT,
        .sclTimeout       = 0,
        .pinEnabled       = IIC_PIN_DISABLED,
};
Iic_DeviceHandle IIC0 = &iic0; 

static Iic_Device iic1 = {
        .regMap           = I2C1_BASE_PTR,

        .baudRate         = IIC_DEF_BAUDRATE,
        .devType          = IIC_MASTER_MODE,
        .addressMode      = IIC_SEVEN_BIT,
        .sclTimeout       = 0,
        .pinEnabled       = IIC_PIN_DISABLED
};
Iic_DeviceHandle IIC1 = &iic1; 
#elif defined(MK60DZ10)
static Iic_Device iic0 = {
        .regMap           = I2C0_BASE_PTR,
        .baudRate         = IIC_DEF_BAUDRATE,
        .devType          = IIC_MASTER_MODE,
        .addressMode      = IIC_SEVEN_BIT,
        .sclTimeout       = 0,
        .pinEnabled       = IIC_PIN_DISABLED,
};
Iic_DeviceHandle IIC0 = &iic0;
#elif defined(FRDMKL05Z)
static Iic_Device iic0 = {
        .regMap           = I2C0_BASE_PTR,
        .baudRate         = IIC_DEF_BAUDRATE,
        .devType          = IIC_MASTER_MODE,
        .addressMode      = IIC_SEVEN_BIT,
        .sclTimeout       = 0,
        .pinEnabled       = IIC_PIN_DISABLED,
};
Iic_DeviceHandle IIC0 = &iic0;
#endif

/**
 * 
 * @param dev
 */
System_Errors Iic_init(Iic_DeviceHandle dev)
{
    I2C_MemMapPtr regmap = dev->regMap;
    Iic_DeviceType devType = dev->devType;
//    uint32_t baudrate = dev->baudRate;

    if (dev->pinEnabled == IIC_PIN_DISABLED)
    	return ERRORS_HW_NOT_ENABLED;
    
    /* Turn on clock */
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    if (regmap == I2C0_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;
    else if (regmap == I2C1_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK;
    else
        return ERRORS_PARAM_VALUE;
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#endif

    /* Select device type */
    if (devType == IIC_MASTER_MODE)
    {
        /* TODO: automatically selects the correct value. */
        /* WARNING: Current configurations is static for 100kbps!! */
#if defined(MKL15Z4)
        I2C_F_REG(regmap) = (0x00 | 0x1F);
#elif defined(FRDMKL25Z)
//        I2C_F_REG(regmap) = (0x40 | 0x16);
		I2C_F_REG(regmap) = (0x00 | 0x21);
#endif

        /* enable IIC */
        I2C_C1_REG(regmap) = I2C_C1_IICEN_MASK;
    }
    else
    {
        /* TODO: implement slave setup */
    }

    /* Setup for manage dummy read. */
    Iic_firstRead = 1;

    return ERRORS_NO_ERROR;
}

/**
 * @brief Set Baud Rate
 * 
 * Sets baud rate value into device structure. 
 * The settings are applied only after the device is initialized.
 * 
 * 
 * @param dev I2C device to be initialized
 * @param baudrate Baud Rate value
 * @return Error code
 */
System_Errors Iic_setBaudRate(Iic_DeviceHandle dev, uint32 baudrate)
{
    if (baudrate >= 10000 && baudrate <= 100000)
    {
        dev->baudRate = baudrate;
        return ERRORS_NO_ERROR;
    }
    else 
    {
        return ERRORS_PARAM_VALUE;
    }
}

/**
 * @brief
 * 
 * @param dev
 * @param devType
 * @return Error code.
 */
System_Errors Iic_setDeviceType (Iic_DeviceHandle dev, Iic_DeviceType devType)
{
    dev->devType = devType;

    return ERRORS_NO_ERROR;
}

/**
 * @brief Indicate that device pin was selected.
 * @param dev Iic device.
 */
void Iic_pinEnabled (Iic_DeviceHandle dev)
{
	dev->pinEnabled = IIC_PIN_ENABLED;
}

void Iic_start (Iic_DeviceHandle dev)
{
    /* If we are in communication set repeat star flag */
    if (I2C_S_REG(dev->regMap) & I2C_S_BUSY_MASK)
    {
        I2C_C1_REG(dev->regMap) |= I2C_C1_RSTA_MASK;
    }
    else
    {
        I2C_C1_REG(dev->regMap) |= I2C_C1_TX_MASK;
        I2C_C1_REG(dev->regMap) |= I2C_C1_MST_MASK;
    }
    /* Setup for manage dummy read. */
    Iic_firstRead = 1;
}

void Iic_stop (Iic_DeviceHandle dev)
{
	uint8_t i;
	
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_MST_MASK;
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TX_MASK;
    
    for (i = 0; i < 100; ++i ) __asm ("nop");

    /* Setup for manage dummy read. */
    Iic_firstRead = 1;
}

static System_Errors Iic_waitTxTransfer (Iic_DeviceHandle dev)
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
        return ERRORS_IIC_TX_TIMEOUT;
    
    /* Reset value */
    I2C_S_REG(dev->regMap) |= I2C_S_IICIF_MASK;
    timeoutResult = 0;
    
    /* Wait for transfer complete */
    for (i = 0; i < 1000; ++i)
    {
        if (I2C_S_REG(dev->regMap) & I2C_S_TCF_MASK)
        {
            timeoutResult = 1;
            break;
        }
    }

    if (timeoutResult == 0)
        return ERRORS_IIC_TX_TIMEOUT;
    
    /* Check ACK! */
    return (I2C_S_REG(dev->regMap) & I2C_S_RXAK_MASK) ? 
            ERRORS_IIC_TX_ACK_NOT_RECEIVED : ERRORS_IIC_TX_ACK_RECEIVED;
    
}

static System_Errors Iic_waitRxTransfer (Iic_DeviceHandle dev)
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
        return ERRORS_IIC_RX_TIMEOUT;
    
    /* Reset value */
    I2C_S_REG(dev->regMap) |= I2C_S_IICIF_MASK;
    
    return ERRORS_IIC_RX_OK;
}

static void Iic_sendNack (Iic_DeviceHandle dev)
{
    I2C_C1_REG(dev->regMap) |= I2C_C1_TXAK_MASK;
}

static void Iic_sendAck (Iic_DeviceHandle dev)
{
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TXAK_MASK;
}

System_Errors Iic_writeByte (Iic_DeviceHandle dev, uint8_t data)
{
    Iic_firstRead = 1;
    
    /* Set TX mode */
    I2C_C1_REG(dev->regMap) |= I2C_C1_TX_MASK;

    /* Write the data in the register */
    I2C_D_REG(dev->regMap) = data;
    
    /* Wait and return */
    return Iic_waitTxTransfer(dev);
    
}

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

    Iic_start(dev);
    
    /* Write address */
    I2C_D_REG(dev->regMap) = address;
    if (Iic_waitTxTransfer(dev) == ERRORS_IIC_TX_TIMEOUT)
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

System_Errors Iic_setSclTimeout (Iic_DeviceHandle dev, uint32_t usDelay)
{
    uint32_t ticks = usDelay/IIC_TIMEOUT_CLOCK_us;
    
    if (ticks > 65535)
        return ERRORS_IIC_SCLTIMEOUT_TOO_LARGE;
    
    I2C_SLTL_REG(dev->regMap) = ticks & 0x000000FF;
    I2C_SLTH_REG(dev->regMap) = ((ticks >> 8) & 0x000000FF);            

    /* Just for debug! */
    dev->sclTimeout = (0x0000FFFF & ticks);

    return ERRORS_NO_ERROR;
}

void Iic_resetSclTimeout (void)
{
    
}
