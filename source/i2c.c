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
#include "i2c.h"

#define IIC_DEF_BAUDRATE    100000 /* 100kbps */

#define IIC_PIN_ENABLED    1
#define IIC_PIN_DISABLED   0

typedef struct Iic_Device {
    I2C_MemMapPtr 		  regMap;

    uint32_t              baudRate;
    Iic_DeviceType        devType;
    Iic_AddressMode       addressMode;

    uint8_t               pinEnabled;
} Iic_Device;

#if defined(MKL15Z4) || defined(FRDMKL25Z)
static Iic_Device iic0 = {
        .regMap           = I2C0_BASE_PTR,

        .baudRate         = IIC_DEF_BAUDRATE,
        .devType          = IIC_MASTER_MODE,
        .addressMode      = IIC_SEVEN_BIT,
        .pinEnabled       = IIC_PIN_DISABLED,
};
Iic_DeviceHandle IIC0 = &iic0; 

static Iic_Device iic1 = {
        .regMap           = I2C1_BASE_PTR,

        .baudRate         = IIC_DEF_BAUDRATE,
        .devType          = IIC_MASTER_MODE,
        .addressMode      = IIC_SEVEN_BIT,
        .pinEnabled       = IIC_PIN_DISABLED
};
Iic_DeviceHandle IIC1 = &iic1; 
#elif defined(MK60DZ10)
static Iic_Device iic0 = {
        .regMap           = I2C0_BASE_PTR,
        .baudRate         = IIC_DEF_BAUDRATE,
        .devType          = IIC_MASTER_MODE,
        .addressMode      = IIC_SEVEN_BIT,
        .pinEnabled       = IIC_PIN_DISABLED,
};
Iic_DeviceHandle IIC0 = &iic0;
#elif defined(FRDMKL05Z)
static Iic_Device iic0 = {
        .regMap           = I2C0_BASE_PTR,
        .baudRate         = IIC_DEF_BAUDRATE,
        .devType          = IIC_MASTER_MODE,
        .addressMode      = IIC_SEVEN_BIT,
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
        I2C_F_REG(regmap) = (0x40 | 0x16);
#endif

        /* enable IIC */
        I2C_C1_REG(regmap) = I2C_C1_IICEN_MASK;
    }
    else
    {
        /* TODO: implement slave setup */
    }

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

/**
 * 
 * @param dev
 * @param slaveID
 * @param mode
 * @return Error code.
 */
//void Iic_startTransmission (Iic_DeviceHandle dev, uint8_t slaveID, Iic_TransmissionType mode)
//{
//    /* Shift ID in right position. */
//    slaveID <<= 1;
//    /* Set R/W bit at end of Slave Address. */
//    if (mode == IIC_MASTER_READ)
//        slaveID |= 0x01;
//    else
//        slaveID &= 0xFE;
//    
//    /* Send start signal. */
//    Iic_start(dev);
//    
//    /* Send ID with W/R bit. */
//    Iic_writeByte(dev, slaveID);
//}
//
//void Iic_disableAck (Iic_DeviceHandle dev)
//{
//    I2C_C1_REG(dev->regMap) |= I2C_C1_TXAK_MASK;
//}
//
//void Iic_repeatedStart (Iic_DeviceHandle dev)
//{
//    I2C_C1_REG(dev->regMap) |= I2C_C1_RSTA_MASK;
//}
//
//void Iic_enterRxMode (Iic_DeviceHandle dev)
//{
//    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TX_MASK;
//    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TXAK_MASK;
//}
//
//void Iic_wait (Iic_DeviceHandle dev)
//{
//    while ((I2C_S_REG(dev->regMap) & I2C_S_IICIF_MASK) == 0);
//    I2C_S_REG(dev->regMap) |= I2C_S_IICIF_MASK;
//}
//
//void Iic_writeByte (Iic_DeviceHandle dev, uint8_t data)
//{
//    I2C_D_REG(dev->regMap) = data;
//}
//
//void Iic_readByte (Iic_DeviceHandle dev, Iic_AcknoledgeType ackMode, uint8_t *data)
//{
//    Iic_enterRxMode(dev);
//    if (ackMode == IIC_NO_ACK)
//    	Iic_disableAck(dev);
//    *data = I2C_D_REG(dev->regMap);
//}

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
}

void Iic_stop (Iic_DeviceHandle dev)
{
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_MST_MASK;
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TX_MASK;
}

