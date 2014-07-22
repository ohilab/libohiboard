/******************************************************************************
 * Copyright (C) 2012-2014 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 *  Nicola Orlandini <n.orlandini90@gmail.com>
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
 * @file libohiboard/include/spi.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 * @author Nicola Orlandini <n.orlandini90@gmail.com>
 * @brief SPI definitions and prototypes
 */

#ifndef __SPI_H
#define __SPI_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum {
    SPI_MASTER_MODE,
    SPI_SLAVE_MODE
} Spi_DeviceType;

typedef enum {
    SPI_LOW_SPEED, /**< Set to more or less 350kHz */
    SPI_HIGH_SPEED /**< Set to more or less 10MHz */
} Spi_SpeedType;

typedef struct Spi_Device* Spi_DeviceHandle;

System_Errors Spi_init (Spi_DeviceHandle dev);
System_Errors Spi_setDeviceType (Spi_DeviceHandle dev, Spi_DeviceType devType);
System_Errors Spi_setSpeedType (Spi_DeviceHandle dev, Spi_SpeedType speedType);
void Spi_pinEnabled (Spi_DeviceHandle dev);

System_Errors Spi_readByte (Spi_DeviceHandle dev, uint8_t * data);
System_Errors Spi_writeByte (Spi_DeviceHandle dev, uint8_t data);

#if defined(MKL15Z4) || defined(FRDMKL25Z)
extern Spi_DeviceHandle SPI0;
extern Spi_DeviceHandle SPI1;
#elif defined (MK10DZ10)|| defined(MK10D10)
extern Spi_DeviceHandle SPI0;
extern Spi_DeviceHandle SPI1;
extern Spi_DeviceHandle SPI2;
#elif defined(MK60DZ10)
extern Spi_DeviceHandle SPI0;
#elif defined(FRDMKL05Z)
extern Spi_DeviceHandle SPI0;
#endif

#endif /* __SPI_H */
