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

typedef struct I2c_dev_ {
    I2C_MemMapPtr 		regMap;
} I2c_dev_;

/**
 * 
 * @param dev
 */
System_Errors I2c_init(I2c_dev dev)
{
    I2C_MemMapPtr regmap = dev->regMap;

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
//  PORTD_PCR9 = PORT_PCR_MUX(2);
//  PORTD_PCR8 = PORT_PCR_MUX(2);

///* set MULT and ICR */
//I2C0_F  = 0x14;
//
///* enable IIC */
//I2C0_C1 = I2C_C1_IICEN_MASK;
}
