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
 * @file libohiboard/source/spi.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief SPI definitions and prototypes
 */

#include "platforms.h"
#include "utility.h"
#include "spi.h"

typedef struct Spi_Device {
    SPI_MemMapPtr         regMap;

    Spi_DeviceType        devType;
    Spi_SpeedType         speedType;
} Spi_Device;

#if defined(MKL15Z4)
static Spi_Device spi0 = {
    .regMap           = SPI0_BASE_PTR,
    
    .devType          = SPI_MASTER_MODE,
    .speedType        = SPI_LOW_SPEED,
};
Spi_DeviceHandle SPI0 = &spi0; 

static Spi_Device spi1 = {
    .regMap           = SPI1_BASE_PTR,

    .devType          = SPI_MASTER_MODE,
    .speedType        = SPI_LOW_SPEED,
};
Spi_DeviceHandle SPI1 = &spi1;
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#endif

System_Errors Spi_init (Spi_DeviceHandle dev)
{
    SPI_MemMapPtr regmap = dev->regMap;
    Spi_DeviceType devType = dev->devType;
    Spi_SpeedType speed = dev->speedType;

    /* Turn on clock */
#if defined(MKL15Z4)
    if (regmap == SPI0_BASE_PTR)
    {
        SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;
    }
    else if (regmap == SPI1_BASE_PTR)
    {
        SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK;
    }
    else
    {
        SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;
    }
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#endif
    
    /* TODO: configure GPIO for SPI function */
    /* WARNING: Current configurations is static!! */
#if defined(MKL15Z4)
    if (regmap == SPI0_BASE_PTR)
    {
        /* Enable clock on PORTE */
        SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
        /* Enable this pin as output */
        GPIOE_PDDR  = GPIO_PDDR_PDD(GPIO_PIN(16));
        /* Set CS as general purpose I/O */
        PORTE_PCR16 = PORT_PCR_MUX(1); 
        PORTE_PCR17 = PORT_PCR_MUX(2);
        PORTE_PCR18 = PORT_PCR_MUX(2);
        PORTE_PCR19 = PORT_PCR_MUX(2);
    }
    else
    {
        /* FIXME: Static pin definitions of SPI1 */
    }
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#endif

    /* Select device type */
    if (devType == SPI_MASTER_MODE)
    {
        /* Disable all and clear interrutps */
        regmap->C1 = 0;
        /* Match interrupt disabled, uses separate pins and DMA disabled. */
        regmap->C2 = 0;
        /* Set baud rate */
        if (speed == SPI_LOW_SPEED)
        {
            /* From 24MHz to 375kHz: set prescaler to 2 and divider to 32. */
            regmap->BR = 0x10 | 0x40;
        }
        else
        {
            /* From 24MHz to 12MHz: set prescaler to 1 and divider to 2. */
            regmap->BR = 0x00;            
        }
        /* Clear match register */
        regmap->M = 0;
        /* Enable master mode and SPI module. */
        regmap->C1 = SPI_C1_MSTR_MASK | SPI_C1_SPE_MASK;
    
    }
    else
    {
        /* TODO: implement slave setup */
    }

    return ERRORS_NO_ERROR;
}

/**
 * @brief
 * 
 * @param dev
 * @param devType
 * @return Error code.
 */
System_Errors Spi_setDeviceType (Spi_DeviceHandle dev, Spi_DeviceType devType)
{
    dev->devType = devType;

    return ERRORS_NO_ERROR;
}

/**
 * @brief
 * 
 * @param dev
 * @param speedType
 * @return Error code.
 */
System_Errors Spi_setSpeedType (Spi_DeviceHandle dev, Spi_SpeedType speedType)
{
    dev->speedType = speedType;

    return ERRORS_NO_ERROR;
}

System_Errors Spi_readByte (Spi_DeviceHandle dev, uint8_t * data)
{
    SPI_MemMapPtr regmap = dev->regMap;

    /* Copy dummy data in D register */
    regmap->D = 0xFF;
    /* Wait until slave replay */
    while (!(regmap->S & SPI_S_SPRF_MASK));
    /* Save data register */
    *data = regmap->D;

    return ERRORS_NO_ERROR;    
}

System_Errors Spi_writeByte (Spi_DeviceHandle dev, uint8_t data)
{
    SPI_MemMapPtr regmap = dev->regMap;
    
    /* Wait until SPTEF bit is 1 (transmit buffer is empty) */
    while (!(regmap->S & SPI_S_SPTEF_MASK));
    (void) regmap->S;
    /* Copy data in D register */
    regmap->D = data;
    /* Wait until slave replay */
    while (!(regmap->S & SPI_S_SPRF_MASK));
    /* Read data register */
    (void) regmap->D;
    
    return ERRORS_NO_ERROR;
}
