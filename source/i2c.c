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
 */

/**
 * @file libohiboard/include/i2c.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C functions implementation
 */

#include "i2c.h"

#define IIC_DEF_BAUDRATE    100000 /* 100kbps */

typedef struct {
    I2C_MemMapPtr 		  regMap;

    uint32_t              baudRate;
    Iic_DeviceType        devType;
    Iic_AddressMode       addressMode;
    
    uint8_t               unsaved;
} Iic_Device;

static Iic_Device iic0 = {
        .regMap           = I2C0_BASE_PTR,
        .baudRate         = IIC_DEF_BAUDRATE,
        .devType          = IIC_MASTER_MODE,
        .addressMode      = IIC_SEVEN_BIT,
};
Iic_DeviceHandle IIC0 = &iic0; 

static Iic_Device iic1 = {
        .regMap           = I2C1_BASE_PTR,
        .baudRate         = IIC_DEF_BAUDRATE,
        .devType          = IIC_MASTER_MODE,
        .addressMode      = IIC_SEVEN_BIT,
};
Iic_DeviceHandle IIC1 = &iic1; 

#ifdef MK60DZ10
#endif

/**
 * 
 * @param dev
 */
System_Errors Iic_init(Iic_DeviceHandle dev)
{
    I2C_MemMapPtr regmap = dev->regMap;
    Iic_DeviceType devType = dev->devType;

    /* Turn on clock */
    if (regmap == I2C0_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;
    else if (regmap == I2C1_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK;
#ifdef MK60DZ10
#endif
    else
        SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;
        
    /* TODO: configure GPIO for I2C function */

    /* Select device type */
    if (devType == IIC_MASTER_MODE)
    {
            
    }
    else
    {
            
    }
///* set MULT and ICR */
//I2C0_F  = 0x14;
//
    /* enable IIC */
    if (regmap == I2C0_BASE_PTR)
        I2C0_C1 = I2C_C1_IICEN_MASK;
    else if (regmap == I2C1_BASE_PTR)
        I2C1_C1 = I2C_C1_IICEN_MASK;
#ifdef MK60DZ10
#endif
    else
        I2C0_C1 = I2C_C1_IICEN_MASK;

    dev->unsaved = 0;
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
        dev->unsaved = 1;
        return ERRORS_NO_ERROR;
    }
    else 
    {
        return ERRORS_PARAM_VALUE;
    }
}

System_Errors Iic_setDeviceType(Iic_DeviceHandle dev, Iic_DeviceType devType)
{
    dev->devType = devType;

    dev->unsaved = 1;
    return ERRORS_NO_ERROR;
}
