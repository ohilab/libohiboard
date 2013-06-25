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
 * @file libohiboard/include/i2c.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C definitions and prototypes
 */

#ifndef __I2C_H
#define __I2C_H

#include "libohiboard.h"
#include "errors.h"

typedef enum {
    IIC_MASTER_MODE,
    IIC_SLAVE_MODE
} Iic_DeviceType;

typedef enum {
    IIC_SEVEN_BIT,
    IIC_TEN_BIT
} Iic_AddressMode;

typedef struct Iic_Device* Iic_DeviceHandle;

System_Errors Iic_init(Iic_DeviceHandle dev);

System_Errors Iic_setBaudRate(Iic_DeviceHandle dev, uint32 br);
System_Errors Iic_setDeviceType(Iic_DeviceHandle dev, Iic_DeviceType devType);


#endif /* __I2C_H */
