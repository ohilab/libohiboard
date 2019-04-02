/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2019 A. C. Open Hardware Ideas Lab
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

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_IIC

/**
 * @defgroup I2C I2C
 * @brief I2C HAL driver
 * @{
 */

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
 * @defgroup I2C_Configuration_Functions I2C configuration functions and types
 * @brief Functions and types to open, close and configure a I2C peripheral.
 * @{
 */
    
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

/**
 * Useful definition to declare the register address dimension in bit during
 * read or write register operations.
 */
typedef enum _Iic_RegisterAddressSize
{
    IIC_REGISTERADDRESSSIZE_8BIT  = 1,
    IIC_REGISTERADDRESSSIZE_16BIT = 2,

} Iic_RegisterAddressSize;

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Iic_DeviceState
{
    IIC_DEVICESTATE_RESET,
    IIC_DEVICESTATE_READY,
    IIC_DEVICESTATE_BUSY,
    IIC_DEVICESTATE_ERROR,

} Iic_DeviceState;

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

/**
 * Useful define for choosing the dual address mode.
 */
typedef enum _Iic_DualAddress
{
    IIC_DUALADDRESS_ENABLE,
    IIC_DUALADDRESS_DISABLE,

} Iic_DualAddress;

/**
 * List of usable mask for second address.
 */
typedef enum _Iic_DualAddressMask
{
    IIC_DUALADDRESSMASK_NO_MASK = 0x00,
    IIC_DUALADDRESSMASK_MASK_01 = 0x01,
    IIC_DUALADDRESSMASK_MASK_02 = 0x02,
    IIC_DUALADDRESSMASK_MASK_03 = 0x03,
    IIC_DUALADDRESSMASK_MASK_04 = 0x04,
    IIC_DUALADDRESSMASK_MASK_05 = 0x05,
    IIC_DUALADDRESSMASK_MASK_06 = 0x06,
    IIC_DUALADDRESSMASK_MASK_07 = 0x07,

} Iic_DualAddressMask;

typedef enum _Iic_NoStretch
{
    IIC_NOSTRETCH_ENABLE,
    IIC_NOSTRETCH_DISABLE,

} Iic_NoStrech;

#endif // LIBOHIBOARD_ST_STM32

#if (LIBOHIBOARD_VERSION >= 0x20000u)
typedef struct _Iic_Device* Iic_DeviceHandle;
#else
typedef struct Iic_Device* Iic_DeviceHandle;
#endif

#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/STM32L4/i2c_STM32L4.h"

#elif defined (LIBOHIBOARD_PIC24FJ)

#include "hardware/PIC24FJ/i2c_PIC24FJ.h"

#else

typedef enum
{
	IIC_PINS_SCLNONE,

} Iic_SclPins;

typedef enum
{

	IIC_PINS_SDANONE,

} Iic_SdaPins;

#endif

typedef struct _Iic_Config
{
    Iic_SclPins           sclPin;
    Iic_SdaPins           sdaPin;

    uint32_t              baudrate;
    Iic_DeviceType        devType;
    Iic_AddressMode       addressMode;

    bool                  pullupEnable;

    uint16_t              sclTimeout;

#if defined LIBOHIBOARD_ST_STM32

    Iic_ClockSource clockSource;

    uint32_t address1;
    uint32_t address2;
    Iic_DualAddress dualAddressMode;
    Iic_DualAddressMask dualAddressMask;
    Iic_NoStrech noStretch;

#endif

} Iic_Config;

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

/**
 * @}
 */

#if (LIBOHIBOARD_VERSION >= 0x20000u)

/**
 * @defgroup I2C_Read_Write_Master_Functions I2C read/write master functions
 * @brief Functions to read and write to I2C peripheral by master device.
 * @{
 */

/**
 * This function send a specified number of bytes to a selected
 * slave device into I2C bus.
 * This function is blocking.
 *
 * @param[in] dev I2C device handle
 * @param[in] address Slave device address
 * @param[in] data Pointer to data buffer
 * @param[in] length the number of byte to write
 * @param[in] timeout Max timeout in millisecond
 */
System_Errors Iic_writeMaster (Iic_DeviceHandle dev,
                               uint16_t address,
                               const uint8_t* data,
                               uint16_t length,
                               uint32_t timeout);

/**
 * This function read a specified number of bytes from a selected
 * slave device into I2C bus.
 * This function is blocking.
 *
 * @param[in] dev I2C device handle
 * @param[in] address Slave device address
 * @param[out] data Pointer to data buffer
 * @param[in] length the number of byte to read
 * @param[in] timeout Max timeout in millisecond
 */
System_Errors Iic_readMaster (Iic_DeviceHandle dev,
                              uint16_t address,
                              uint8_t* data,
                              uint8_t length,
                              uint32_t timeout);

/**
 * This function write a register or memory location to the selected
 * slave device into I2C bus.
 * This function is blocking.
 *
 * @param[in] dev I2C device handle
 * @param[in] devAddress Slave device address 7bit or 10bit
 * @param[in] regAddress The address of slave register/memory location
 * @param[in] addressSize The dimension, in byte, of register address
 * @param[in] data Pointer to data buffer
 * @param[in] length the number of byte to write
 * @param[in] timeout Max timeout in millisecond
 * @return ERRORS_NO_ERROR in case of success, specific errors otherwise
 */
System_Errors Iic_writeRegister (Iic_DeviceHandle dev,
                                 uint16_t devAddress,
                                 uint16_t regAddress,
                                 Iic_RegisterAddressSize addressSize,
                                 const uint8_t* data,
                                 uint16_t length,
                                 uint32_t timeout);

/**
 * This function read a register or memory location from the selected
 * slave device into I2C bus.
 * This function is blocking.
 *
 * @param[in]  dev I2C device handle
 * @param[in]  devAddress Slave device address 7bit or 10bit
 * @param[in]  regAddress The address of slave register/memory location
 * @param[in]  addressSize The dimension, in byte, of register address
 * @param[out] data Pointer to data buffer
 * @param[in]  length the number of byte to read
 * @param[in]  timeout Max timeout in millisecond
 * @return ERRORS_NO_ERROR in case of success, specific errors otherwise
 */
System_Errors Iic_readRegister (Iic_DeviceHandle dev,
                                uint16_t devAddress,
                                uint16_t regAddress,
                                Iic_RegisterAddressSize addressSize,
                                uint8_t* data,
                                uint16_t length,
                                uint32_t timeout);

/**
 * This function send start condition.
 * 
 * @param[in] dev I2C device handle
 */
void Iic_start (Iic_DeviceHandle dev);

/**
 * This function send repeated start condition.
 * 
 * @param[in] dev I2C device handle
 */
void Iic_repeatedStart (Iic_DeviceHandle dev);

/**
 * This function send stop condition.
 * 
 * @param[in] dev I2C device handle
 */
void Iic_stop (Iic_DeviceHandle dev);

/**
 * @}
 */

#else

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
System_Errors Iic_getAck (Iic_DeviceHandle dev);
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

#endif // ELSE LIBOHIBOARD_VERSION >= 0x20000

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


#endif

#ifdef __cplusplus
}
#endif

#endif // __I2C_H

/**
 * @}
 */

#endif // LIBOHIBOARD_IIC

/**
 * @}
 */
