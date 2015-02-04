/* Copyright (C) 2012-2015 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
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

typedef enum
{
#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

#elif defined (LIBOHIBOARD_KL15Z4)

    IIC_PINS_PTA3,

    IIC_PINS_PTB0,
    IIC_PINS_PTB2,

    IIC_PINS_PTC1,
    IIC_PINS_PTC8,
    IIC_PINS_PTC10,

    IIC_PINS_PTE1,
    IIC_PINS_PTE19,
    IIC_PINS_PTE24,

#elif defined (LIBOHIBOARD_KL25Z4) || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    IIC_PINS_PTA3,

    IIC_PINS_PTB0,
    IIC_PINS_PTB2,

    IIC_PINS_PTC1,
    IIC_PINS_PTC8,
    IIC_PINS_PTC10,

    IIC_PINS_PTE1,
    IIC_PINS_PTE24,

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

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

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

#elif defined (LIBOHIBOARD_KL15Z4)

    IIC_PINS_PTA4,

    IIC_PINS_PTB1,
    IIC_PINS_PTB3,

    IIC_PINS_PTC2,
    IIC_PINS_PTC9,
    IIC_PINS_PTC11,

    IIC_PINS_PTE0,
    IIC_PINS_PTE18,
    IIC_PINS_PTE25,

#elif defined (LIBOHIBOARD_KL25Z4) || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    IIC_PINS_PTA4,

    IIC_PINS_PTB1,
    IIC_PINS_PTB3,

    IIC_PINS_PTC2,
    IIC_PINS_PTC9,
    IIC_PINS_PTC11,

    IIC_PINS_PTE0,
    IIC_PINS_PTE25,

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

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

} Iic_Config;

typedef struct Iic_Device* Iic_DeviceHandle;

System_Errors Iic_init (Iic_DeviceHandle dev, Iic_Config *config);

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

#if 0
System_Errors Iic_setSclTimeout (Iic_DeviceHandle dev, uint32_t usDelay);
void Iic_resetSclTimeout (Iic_DeviceHandle dev);
System_Errors Iic_isToggleSclTimeout (Iic_DeviceHandle dev);
#endif

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

#elif defined (LIBOHIBOARD_KL15Z4)

extern Iic_DeviceHandle IIC0;
extern Iic_DeviceHandle IIC1;

#elif defined (LIBOHIBOARD_KL25Z4) || \
	  defined (LIBOHIBOARD_FRDMKL25Z)

extern Iic_DeviceHandle IIC0;
extern Iic_DeviceHandle IIC1;

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

extern Iic_DeviceHandle IIC0;
extern Iic_DeviceHandle IIC1;
extern Iic_DeviceHandle IIC2;

#endif

#endif /* __I2C_H */
