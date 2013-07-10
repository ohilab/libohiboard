/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  
 * Project: libohiboard
 * Package: SPI
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
 * @file libohiboard/include/spi.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief SPI definitions and prototypes
 */

#include "platforms.h"
#include "spi.h"

typedef struct Spi_Device {
    SPI_MemMapPtr         regMap;
    
    uint32_t              baudRate;
    Spi_DeviceType        devType;

    uint8_t               unsaved;
} Spi_Device;

#if defined(MKL15Z4)
static Spi_Device spi0 = {
    .regMap           = SPI0_BASE_PTR,
    
    .devType          = SPI_MASTER_MODE,
};
Spi_DeviceHandle SPI0 = &spi0; 

static Spi_Device spi1 = {
    .regMap           = SPI1_BASE_PTR,

    .devType          = SPI_MASTER_MODE,
};
Spi_DeviceHandle SPI1 = &spi1;
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#endif

System_Errors Spi_init (Spi_DeviceHandle dev)
{
    SPI_MemMapPtr regmap = dev->regMap;

    /* Turn on clock */
#if defined(MKL15Z4)
    if (regmap == SPI0_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;
    else if (regmap == I2C1_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK;
    else
        SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#endif
    /* TODO */

    dev->unsaved = 0;
    return ERRORS_NO_ERROR;
}
