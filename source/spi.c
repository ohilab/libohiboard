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
 * @file libohiboard/source/spi.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 * @author Nicola Orlandini <n.orlandini90@gmail.com>
 * @brief SPI definitions and prototypes
 */

#include "platforms.h"
#include "utility.h"
#include "spi.h"

#if defined(MKL15Z4) || defined(FRDMK20D50M) || defined(FRDMKL05Z) || defined(MK60F15) || \
	  defined(FRDMKL02Z) || defined(MKL02Z4) || defined(MK10DZ10) || defined(MK10D10) || \
	  defined(MK60DZ10) || defined(MK60F15) || defined(MKL03Z4) || defined(FRDMKL03Z) || \
	  defined(OHIBOARD_R1)

#define SPI_PIN_ENABLED    1
#define SPI_PIN_DISABLED   0

typedef struct Spi_Device {
    SPI_MemMapPtr         regMap;

    Spi_DeviceType        devType;
    Spi_SpeedType         speedType;
	
	uint8_t               pinEnabled;
} Spi_Device;

#if defined(MKL15Z4) || defined(FRDMKL25Z)
static Spi_Device spi0 = {
    .regMap           = SPI0_BASE_PTR,
    
    .devType          = SPI_MASTER_MODE,
    .speedType        = SPI_LOW_SPEED,
    .pinEnabled       = SPI_PIN_DISABLED
};
Spi_DeviceHandle SPI0 = &spi0; 

static Spi_Device spi1 = {
    .regMap           = SPI1_BASE_PTR,

    .devType          = SPI_MASTER_MODE,
    .speedType        = SPI_LOW_SPEED,
    .pinEnabled       = SPI_PIN_DISABLED
};
Spi_DeviceHandle SPI1 = &spi1;

#elif defined(MK10DZ10) || defined(MK10D10)
static Spi_Device spi0 = {
    .regMap           = SPI0_BASE_PTR,
    
    .devType          = SPI_MASTER_MODE,
    .speedType        = SPI_LOW_SPEED,
    .pinEnabled       = SPI_PIN_DISABLED
};
Spi_DeviceHandle SPI0 = &spi0; 

static Spi_Device spi1 = {
    .regMap           = SPI1_BASE_PTR,

    .devType          = SPI_MASTER_MODE,
    .speedType        = SPI_LOW_SPEED,
    .pinEnabled       = SPI_PIN_DISABLED
};
Spi_DeviceHandle SPI1 = &spi1;

static Spi_Device spi2 = {
    .regMap           = SPI2_BASE_PTR,

    .devType          = SPI_MASTER_MODE,
    .speedType        = SPI_LOW_SPEED,
    .pinEnabled       = SPI_PIN_DISABLED
};
Spi_DeviceHandle SPI2 = &spi2;

#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#endif

System_Errors Spi_init (Spi_DeviceHandle dev)
{
    SPI_MemMapPtr regmap = dev->regMap;
    Spi_DeviceType devType = dev->devType;
    Spi_SpeedType speed = dev->speedType;
    
    if (dev->pinEnabled == SPI_PIN_DISABLED)
    	return ERRORS_HW_NOT_ENABLED;

    /* Turn on clock */
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    if (regmap == SPI0_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;
    else if (regmap == SPI1_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_SPI1_MASK;
    else
        return ERRORS_PARAM_VALUE;
#elif defined(MK10DZ10) || defined(MK10D10)
    if (regmap == SPI0_BASE_PTR)
        SIM_SCGC6 |= SIM_SCGC6_SPI0_MASK;
    else if (regmap == SPI1_BASE_PTR)
        SIM_SCGC6 |= SIM_SCGC6_SPI1_MASK;
    else if (regmap == SPI2_BASE_PTR)
    	SIM_SCGC3 |= SIM_SCGC3_SPI2_MASK;
    else
        return ERRORS_PARAM_VALUE;
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#endif

    /* Select device type */
    if (devType == SPI_MASTER_MODE)
    {
#if defined(FRDMKL05Z) || defined(MKL15Z4) || defined(FRDMKL25Z)
        /* Disable all and clear interrutps */
        SPI_C1_REG(regmap) = 0;
        /* Match interrupt disabled, uses separate pins and DMA disabled. */
        SPI_C2_REG(regmap) = 0;
#elif defined(MK10DZ10) || defined(MK10D10)
        SPI_MCR_REG(regmap) = SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(0xF) | 0;
#elif defined(MK60DZ10)
#elif defined(FRDMK20D50M)
#endif
        
        /* Set baud rate */
        if (speed == SPI_LOW_SPEED)
        {
#if defined(MKL15Z4)
            /* From 24MHz to 375kHz: set prescaler to 2 and divider to 32. */
            SPI_BR_REG(regmap) = 0x10 | 0x04;
#elif defined (MK10DZ10) || defined(MK10D10)
            /* From 24MHz to 375kHz: set prescaler to 2 and divider to 32. */
            SPI_CTAR_REG(regmap,0) = SPI_CTAR_FMSZ(0x7) | SPI_CTAR_CPOL_MASK | SPI_CTAR_BR(0x05) | SPI_CTAR_CPHA_MASK | 0;
            SPI_CTAR_REG(regmap,1) = SPI_CTAR_FMSZ(0x7) | SPI_CTAR_CPOL_MASK | SPI_CTAR_BR(0x05) | SPI_CTAR_CPHA_MASK | 0;
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#elif defined(FRDMKL25Z)
            /* From 20MHz to 357kHz: set prescaler to 7 and divider to 8. */
            SPI_BR_REG(regmap) = 0x60 | 0x02;
#elif defined(FRDMK20D50M)
#endif
        }
        else
        {
#if defined(MKL15Z4)
            /* From 24MHz to 12MHz: set prescaler to 1 and divider to 2. */
            SPI_BR_REG(regmap) = 0x00;
#elif defined(MK10DZ10) || defined(MK10D10)
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#elif defined(FRDMKL25Z)
            /* From 20MHz to 10MHz: set prescaler to 1 and divider to 2. */
            SPI_BR_REG(regmap) = 0x00;    
#elif defined(FRDMK20D50M)
#endif
        }

#if defined(FRDMKL05Z) || defined(MKL15Z4) || defined(FRDMKL25Z)
        /* Clear match register */
        SPI_M_REG(regmap) = 0;
        /* Enable master mode and SPI module. */
        SPI_C1_REG(regmap) = SPI_C1_MSTR_MASK | SPI_C1_SPE_MASK;
#elif defined(MK60DZ10)
#elif defined(FRDMK20D50M)
#endif    
    }
    else if (devType == SPI_SLAVE_MODE)
    {
        /* TODO: implement slave setup */
    }
    else
    {
        return ERRORS_PARAM_VALUE;
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

/**
 * @brief Indicate that device pin was selected.
 * @param dev Spi device.
 */
void Spi_pinEnabled (Spi_DeviceHandle dev)
{
	dev->pinEnabled = SPI_PIN_ENABLED;
}


System_Errors Spi_readByte (Spi_DeviceHandle dev, uint8_t *data)
{
    SPI_MemMapPtr regmap = dev->regMap;

#if defined(FRDMKL05Z) || defined(MKL15Z4) || defined(FRDMKL25Z)    
    /* Copy dummy data in D register */
    SPI_D_REG(regmap) = 0xFF;
    /* Wait until slave replay */
    while (!(SPI_S_REG(regmap) & SPI_S_SPRF_MASK));
    /* Save data register */
    *data = SPI_D_REG(regmap);
#elif defined(MK10DZ10) || defined(MK10D10)
    
	//wait till TX FIFO is Empty.
	while((SPI_SR_REG(regmap) & SPI_SR_TFFF_MASK) != SPI_SR_TFFF_MASK);
		
	// Transmit Byte on SPI 
//	SPI_PUSHR_REG(regmap) = //SPI_PUSHR_CONT_MASK |
//			//SPI_PUSHR_CTAS_MASK |
//			SPI_PUSHR_PCS(1);
	
//	SPI_PUSHR_REG(regmap) |= 0xFFFF;
	SPI_PUSHR_REG(regmap) = 0x0000;
//	
	SPI_MCR_REG(regmap) &= ~SPI_MCR_HALT_MASK;
		
//	//--> Wait till transmit complete
	while (((SPI_SR_REG(regmap) & SPI_SR_TCF_MASK)) != SPI_SR_TCF_MASK);
//		
//	//--> Clear Transmit Flag.
	SPI_SR_REG(regmap) |= SPI_SR_TFFF_MASK;
	
	SPI_MCR_REG(regmap) &= ~SPI_MCR_HALT_MASK;
	
	//wait till RX_FIFO is not empty
	while((SPI_SR_REG(regmap) & SPI_SR_RFDF_MASK) != SPI_SR_RFDF_MASK);

	*data = SPI_POPR_REG(regmap) & 0x000000FF;
	
	//-->Clear the RX FIFO Drain Flag
	SPI_SR_REG(regmap) |= SPI_SR_RFDF_MASK;
#elif defined(MK60DZ10)
#elif defined(FRDMK20D50M)
#endif
    
    return ERRORS_NO_ERROR;    
}

System_Errors Spi_writeByte (Spi_DeviceHandle dev, uint8_t data)
{
    SPI_MemMapPtr regmap = dev->regMap;
    
#if defined(FRDMKL05Z) || defined(MKL15Z4) || defined(FRDMKL25Z)    
    /* Wait until SPTEF bit is 1 (transmit buffer is empty) */
    while (!(SPI_S_REG(regmap) & SPI_S_SPTEF_MASK));
    (void) SPI_S_REG(regmap);
    /* Copy data in D register */
    SPI_D_REG(regmap) = data;
    /* Wait until slave replay */
    while (!(SPI_S_REG(regmap) & SPI_S_SPRF_MASK));
    /* Read data register */
    (void) SPI_D_REG(regmap);
#elif defined(MK10DZ10) || defined(MK10D10)
    uint16_t x = 0;
    
	//wait till TX FIFO is Empty.
	while((SPI_SR_REG(regmap)  & SPI_SR_TFFF_MASK) != 0x2000000U);
		
	// Transmit Byte on SPI 
//	SPI_PUSHR_REG(regmap) = //SPI_PUSHR_CONT_MASK |
//			//SPI_PUSHR_CTAS_MASK |
//			SPI_PUSHR_PCS(1);
	
//	SPI_PUSHR_REG(regmap) |= data;
	
	SPI_PUSHR_REG(regmap) = data;
	
	SPI_MCR_REG(regmap) &= ~SPI_MCR_HALT_MASK;
	
//	for (x = 0; x < 4; x++)
//	{			
		//--> Wait till transmit complete
		while (((SPI_SR_REG(regmap) & SPI_SR_TCF_MASK)) != SPI_SR_TCF_MASK) ;
		
		//--> Clear Transmit Flag.
//		SPI_SR_REG(regmap) |= SPI_SR_TFFF_MASK;
		SPI_SR_REG(regmap) |= SPI_SR_TCF_MASK;
		
		(void)SPI_POPR_REG(regmap);
//	}
#elif defined(MK60DZ10)
#elif defined(FRDMK20D50M)
#endif
    
    return ERRORS_NO_ERROR;
}

#endif
