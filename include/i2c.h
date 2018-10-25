/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2018 A. C. Open Hardware Ideas Lab
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
 */

/**
 * @file libohiboard/include/i2c.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief I2C definitions and prototypes
 */

#ifdef LIBOHIBOARD_IIC

#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "system.h"

/**
 * Definition for possible device behavioral.
 */
typedef enum
{
    IIC_MASTER_MODE,
    IIC_SLAVE_MODE,

} Iic_DeviceType;

/**
 * Definition for possible address mode.
 */
typedef enum
{
    IIC_SEVEN_BIT,
    IIC_TEN_BIT,

} Iic_AddressMode;

#if defined (LIBOHIBOARD_NXP_KINETIS)

typedef enum
{
    IIC_NO_STOP,
    IIC_STOP

} Iic_StopMode;

typedef enum
{
    IIC_LAST_BYTE,
    IIC_NO_LAST_BYTE

} Iic_LastByteMode;

#endif // LIBOHIBOARD_NXP_KINETIS

#if defined (LIBOHIBOARD_ST_STM32)

typedef enum _Iic_ClockSource
{
    IIC_CLOCKSOURCE_PCLK    = 0x00U,                  /**< PCLK clock source */
    IIC_CLOCKSOURCE_SYSCLK  = 0x01U,                /**< SYSCLK clock source */
    IIC_CLOCKSOURCE_HSI     = 0x02U,                   /**< HSI clock source */

} Iic_ClockSource;

#endif // LIBOHIBOARD_ST_STM32

typedef enum
{
#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

    IIC_PINS_PTA3C,
    IIC_PINS_PTA4C,
    IIC_PINS_PTA8,

    IIC_PINS_PTB0,
    IIC_PINS_PTB3,

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

#elif defined (LIBOHIBOARD_K10D10) || \
      defined (LIBOHIBOARD_K10D7)  || \
      defined (LIBOHIBOARD_K12D5)

    IIC_PINS_PTB0,
    IIC_PINS_PTB2,

    IIC_PINS_PTC10,

    IIC_PINS_PTD8,

    IIC_PINS_PTE1,
    IIC_PINS_PTE19,

#elif defined (LIBOHIBOARD_K60DZ10) || \
	  defined (LIBOHIBOARD_OHIBOARD_R1)

    IIC_PINS_PTB0,
    IIC_PINS_PTB2,

    IIC_PINS_PTC10,

    IIC_PINS_PTD8,

    IIC_PINS_PTE1,

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

    IIC_PINS_PTA3D,
    IIC_PINS_PTA4D,
    IIC_PINS_PTA9,

    IIC_PINS_PTB1,
    IIC_PINS_PTB4,

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

#elif defined (LIBOHIBOARD_K10D10) || \
      defined (LIBOHIBOARD_K10D7)  || \
      defined (LIBOHIBOARD_K12D5)

    IIC_PINS_PTB1,
    IIC_PINS_PTB3,

    IIC_PINS_PTC11,

    IIC_PINS_PTD9,

    IIC_PINS_PTE0,
    IIC_PINS_PTE18,

#elif defined (LIBOHIBOARD_K60DZ10) || \
      defined (LIBOHIBOARD_OHIBOARD_R1)

    IIC_PINS_PTB1,
    IIC_PINS_PTB3,

    IIC_PINS_PTC11,

    IIC_PINS_PTD9,

    IIC_PINS_PTE0,

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

    bool                  pullupEnable;

    uint16_t              sclTimeout;

#if defined LIBOHIBOARD_ST_STM32

    Iic_ClockSource clockSource;

#endif

} Iic_Config;

#if (LIBOHIBOARD_VERSION >= 0x200)
typedef struct _Iic_Device* Iic_DeviceHandle;
#else
typedef struct Iic_Device* Iic_DeviceHandle;
#endif

/** @name Configuration functions
 *  Functions to open, close and configure a I2C peripheral.
 */

///@{

/**
  * @brief Initialize the I2C according to the specified parameters
  *         in the @ref Iic_Config and initialize the associated handle.
  * @param[in] dev I2C device handle
  * @param[in] config Configuration parameters list.
  * @return ERRORS_NO_ERROR The initialization is ok.
  */
System_Errors Iic_init (Iic_DeviceHandle dev, Iic_Config *config);

/**
 * This function de-initialize the selected peripheral.
 *
 * @param[in] dev I2C device handle
 */
System_Errors Iic_deInit (Iic_DeviceHandle dev);

///@}

#if defined (LIBOHIBOARD_KL25Z4)    || \
    defined (LIBOHIBOARD_FRDMKL25Z) || \
	defined (LIBOHIBOARD_KL15Z4)    || \
	defined (LIBOHIBOARD_K10D7)     || \
	defined (LIBOHIBOARD_K10D10)    || \
    defined (LIBOHIBOARD_K12D5)

void Iic_start (Iic_DeviceHandle dev);
void Iic_repeatedStart (Iic_DeviceHandle dev);
void Iic_stop (Iic_DeviceHandle dev);
void Iic_sendNack (Iic_DeviceHandle dev);
void Iic_sendAck (Iic_DeviceHandle dev);
bool Iic_getAck (Iic_DeviceHandle dev);
void Iic_setReceiveMode (Iic_DeviceHandle dev);
void Iic_writeByte (Iic_DeviceHandle dev, uint8_t data);
void Iic_readByte (Iic_DeviceHandle dev, uint8_t *data);
System_Errors Iic_waitTransfer (Iic_DeviceHandle dev);

void Iic_readRegister (Iic_DeviceHandle dev,
                       uint8_t writeAddress,
                       uint8_t readAddress,
                       uint8_t registerAddress,
                       uint8_t *data);

void Iic_writeRegister (Iic_DeviceHandle dev,
                        uint8_t writeAddress,
                        uint8_t registerAddress,
                        uint8_t data);

void Iic_readMultipleRegisters (Iic_DeviceHandle dev,
                       uint8_t writeAddress,
                       uint8_t readAddress,
                       uint8_t firstRegisterAddress,
                       uint8_t *data,
					   uint8_t length);

void Iic_writeMultipleRegisters (Iic_DeviceHandle dev,
                        uint8_t writeAddress,
                        uint8_t firstRegisterAddress,
                        uint8_t* data,
						uint8_t length);

#else

void Iic_start (Iic_DeviceHandle dev);
void Iic_stop (Iic_DeviceHandle dev);

System_Errors Iic_writeByte (Iic_DeviceHandle dev, uint8_t data);
System_Errors Iic_writeBytes (Iic_DeviceHandle dev, uint8_t address, 
        const uint8_t *data, uint8_t length, Iic_StopMode stopRequest);
System_Errors Iic_readByte (Iic_DeviceHandle dev, uint8_t *data, 
        Iic_LastByteMode lastByte);
System_Errors Iic_readBytes (Iic_DeviceHandle dev, uint8_t address, 
        uint8_t *data, uint8_t length, Iic_StopMode stopRequest);

#endif

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

#elif defined (LIBOHIBOARD_KL15Z4)

extern Iic_DeviceHandle OB_IIC0;
extern Iic_DeviceHandle OB_IIC1;

#elif defined (LIBOHIBOARD_KL25Z4) || \
	  defined (LIBOHIBOARD_FRDMKL25Z)

extern Iic_DeviceHandle OB_IIC0;
extern Iic_DeviceHandle OB_IIC1;

#elif defined (LIBOHIBOARD_K10D7)   || \
      defined (LIBOHIBOARD_K10D10)  || \
      defined (LIBOHIBOARD_K12D5)

extern Iic_DeviceHandle OB_IIC0;
extern Iic_DeviceHandle OB_IIC1;

#elif defined (LIBOHIBOARD_K60DZ10) || \
      defined (LIBOHIBOARD_OHIBOARD_R1)

extern Iic_DeviceHandle IIC0;
extern Iic_DeviceHandle IIC1;

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

extern Iic_DeviceHandle IIC0;
extern Iic_DeviceHandle IIC1;
extern Iic_DeviceHandle IIC2;

#elif defined (LIBOHIBOARD_STM32L4) // STM32 Microcontroller - L4 Series

#if defined (LIBOHIBOARD_STM32L476Jx) // WLCSP72 ballout

extern Iic_DeviceHandle OB_IIC1;
extern Iic_DeviceHandle OB_IIC2;
extern Iic_DeviceHandle OB_IIC3;

#endif // LIBOHIBOARD_STM32L476Jx

#endif

#ifdef __cplusplus
}
#endif

#endif // __I2C_H

#endif // LIBOHIBOARD_IIC
