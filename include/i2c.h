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
 * @file libohiboard/include/i2c.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C definitions and prototypes
 */

#ifndef __I2C_H
#define __I2C_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum {
    IIC_MASTER_MODE,
    IIC_SLAVE_MODE
} Iic_DeviceType;

typedef enum {
    IIC_SEVEN_BIT,
    IIC_TEN_BIT
} Iic_AddressMode;

typedef enum {
    IIC_NO_STOP,
    IIC_STOP
} Iic_StopMode;

typedef enum {
    IIC_LAST_BYTE,
    IIC_NO_LAST_BYTE
} Iic_LastByteMode;

typedef struct Iic_Device* Iic_DeviceHandle;

#if defined(MK64F12) || defined(FRDMK64F)

typedef enum
{
#if defined (MK64F12) || defined (FRDMK64F)
	IIC_PINS_PTA12,
	IIC_PINS_PTA14,

	IIC_PINS_PTB0,
	IIC_PINS_PTB2,

	IIC_PINS_PTC10,

	IIC_PINS_PTD2,
	IIC_PINS_PTD8,

	IIC_PINS_PTE1,
	IIC_PINS_PTE24,

#endif

	IIC_PINS_SCLNONE,
} Iic_SclPins;

typedef enum
{
#if defined (MK64F12) || defined (FRDMK64F)
	IIC_PINS_PTA11,
	IIC_PINS_PTA13,

	IIC_PINS_PTB1,
	IIC_PINS_PTB3,

	IIC_PINS_PTC11,

	IIC_PINS_PTD3,
	IIC_PINS_PTD9,

	IIC_PINS_PTE0,
	IIC_PINS_PTE25,
#endif

	IIC_PINS_SDANONE,
} Iic_SdaPins;

typedef struct _Iic_Config
{
    Iic_SclPins           sclPin;
    Iic_SdaPins           sdaPin;

    uint32_t              baudRate;
    Iic_DeviceType        devType;
    Iic_AddressMode       addressMode;

    uint16_t              sclTimeout;

//    uint8_t               pinEnabled;
} Iic_Config;

#endif

#if defined (MK64F12) || defined (FRDMK64F)
System_Errors Iic_init (Iic_DeviceHandle dev, Iic_Config *config);
//#elif defined(MKL15Z4) || defined(FRDMK20D50M) || defined(FRDMKL05Z) || defined(MK60F15) || \
//	  defined(FRDMKL02Z) || defined(MKL02Z4) || defined(MK10DZ10) || defined(MK10D10) || \
//	  defined(MK60DZ10) || defined(MK60F15) || defined(MKL03Z4) || defined(FRDMKL03Z) || \
//	  defined(OHIBOARD_R1)
#elif !defined (MK64F12) && !defined (FRDMK64F)
System_Errors Iic_init (Iic_DeviceHandle dev);

System_Errors Iic_setBaudRate (Iic_DeviceHandle dev, uint32_t br);
System_Errors Iic_setDeviceType (Iic_DeviceHandle dev, Iic_DeviceType devType);
#endif

void Iic_pinEnabled (Iic_DeviceHandle dev);

void Iic_start (Iic_DeviceHandle dev);
void Iic_stop (Iic_DeviceHandle dev);

System_Errors Iic_writeByte (Iic_DeviceHandle dev, uint8_t data);
System_Errors Iic_writeBytes (Iic_DeviceHandle dev, uint8_t address, 
        const uint8_t *data, uint8_t length, Iic_StopMode stopRequest);
System_Errors Iic_readByte (Iic_DeviceHandle dev, uint8_t *data, 
        Iic_LastByteMode lastByte);
System_Errors Iic_readBytes (Iic_DeviceHandle dev, uint8_t address, 
        uint8_t *data, uint8_t length, Iic_StopMode stopRequest);

#if !defined(FRDMKL02Z) && !defined(MKL02Z4) && \
	!defined(FRDMKL03Z) && !defined(MKL03Z4)
System_Errors Iic_setSclTimeout (Iic_DeviceHandle dev, uint32_t usDelay);
void Iic_resetSclTimeout (Iic_DeviceHandle dev);
System_Errors Iic_isToggleSclTimeout (Iic_DeviceHandle dev);
#endif

#if defined(MKL15Z4) || defined(FRDMKL25Z) || defined(MK10D10)
extern Iic_DeviceHandle IIC0;
extern Iic_DeviceHandle IIC1;
#elif defined(MK60DZ10)
extern Iic_DeviceHandle IIC0;
#elif defined(FRDMKL05Z)
extern Iic_DeviceHandle IIC0;
#elif defined(MK64F12) || defined(FRDMK64F)
extern Iic_DeviceHandle IIC0;
extern Iic_DeviceHandle IIC1;
extern Iic_DeviceHandle IIC2;
#endif

#endif /* __I2C_H */
