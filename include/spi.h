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
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
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

#if defined(MK64F12) || defined(FRDMK64F)

typedef enum {
    SPI_CONTINUOUS_SCK, /**< Set to Continuous SCK */
    SPI_NOT_CONTINUOUS_SCK /**< Set to not Continuous SCK */
} Spi_ContinousSck;

typedef enum {
	SPI_SCK_INACTIVE_STATE_LOW,
	SPI_SCK_INACTIVE_STATE_HIGH,
} Spi_ClockPolarity;

typedef enum {
    SPI_SCK_LEADING_EDGE_DATA_CAPTURED,
    SPI_SCK_LEADING_EDGE_DATA_CHANGED,
} Spi_ClockPhase;

typedef enum
{
#if defined (MK64F12) || defined (FRDMK64F)
	SPI_PINS_PTA14,

	SPI_PINS_PTB9,
	SPI_PINS_PTB10,
	SPI_PINS_PTB20,

	SPI_PINS_PTC0,
	SPI_PINS_PTC1,
	SPI_PINS_PTC2,
	SPI_PINS_PTC3,
	SPI_PINS_PTC4,

	SPI_PINS_PTD0,
	SPI_PINS_PTD4,
	SPI_PINS_PTD5,
	SPI_PINS_PTD6,
	SPI_PINS_PTD11,
	SPI_PINS_PTD15,

	SPI_PINS_PTE0,
	SPI_PINS_PTE4,
	SPI_PINS_PTE5,
	SPI_PINS_PTE6,

#endif

	SPI_PINS_PCSNONE,
} Spi_PcsPins;

typedef enum
{
#if defined (MK64F12) || defined (FRDMK64F)
	SPI_PINS_PTA16,

	SPI_PINS_PTB16,
	SPI_PINS_PTB22,

	SPI_PINS_PTC6,

	SPI_PINS_PTD2,
	SPI_PINS_PTD13,

	SPI_PINS_PTE1,

#endif

	SPI_PINS_SOUTNONE,
} Spi_SoutPins;

typedef enum
{
#if defined (MK64F12) || defined (FRDMK64F)
	SPI_PINS_PTA17,

	SPI_PINS_PTB17,
	SPI_PINS_PTB23,

	SPI_PINS_PTC7,

	SPI_PINS_PTD3,
	SPI_PINS_PTD7,
	SPI_PINS_PTD14,

	SPI_PINS_PTE3,

#endif

	SPI_PINS_SINNONE,
} Spi_SinPins;

typedef enum
{
#if defined (MK64F12) || defined (FRDMK64F)
	SPI_PINS_PTA15,

	SPI_PINS_PTB11,
	SPI_PINS_PTB21,

	SPI_PINS_PTC5,

	SPI_PINS_PTD1,
	SPI_PINS_PTD12,

	SPI_PINS_PTE2,

#endif

	SPI_PINS_SCKNONE,
} Spi_SckPins;

typedef struct _Spi_Config
{
    Spi_PcsPins           pcs0Pin;
    Spi_PcsPins           pcs1Pin;
    Spi_PcsPins           pcs2Pin;
    Spi_PcsPins           pcs3Pin;
    Spi_PcsPins           pcs4Pin;
    Spi_SoutPins          soutPin;
    Spi_SinPins           sinPin;
    Spi_SckPins           sckPin;

    Spi_DeviceType        devType;
    uint32_t              baudrate;
    uint32_t              frameSize;

    Spi_ContinousSck      continuousSck;

    Spi_ClockPolarity     sckPolarity;
    Spi_ClockPhase        sckPhase;

} Spi_Config;

#endif

#if defined (MK64F12) || defined (FRDMK64F)
System_Errors Spi_init (Spi_DeviceHandle dev, Spi_Config *config);
System_Errors Spi_setBaudrate(Spi_DeviceHandle dev, uint32_t speed);
#else
System_Errors Spi_init (Spi_DeviceHandle dev);
#endif
System_Errors Spi_setDeviceType (Spi_DeviceHandle dev, Spi_DeviceType devType);
System_Errors Spi_setSpeedType (Spi_DeviceHandle dev, Spi_SpeedType speedType);
void Spi_pinEnabled (Spi_DeviceHandle dev);

System_Errors Spi_readByte (Spi_DeviceHandle dev, uint8_t * data);
System_Errors Spi_writeByte (Spi_DeviceHandle dev, uint8_t data);

#if defined(LIBOHIBOARD_KL15Z4) || defined(FRDMKL25Z)
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
#elif defined (MK64F12)|| defined(FRDMK64F)
extern Spi_DeviceHandle SPI0;
extern Spi_DeviceHandle SPI1;
extern Spi_DeviceHandle SPI2;
#endif

#endif /* __SPI_H */
