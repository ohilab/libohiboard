/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2019 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 *  Nicola Orlandini <n.orlandini90@gmail.com>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
 *  Leonardo Morichelli <leonardo.morichelli@live.com>
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
 * @file libohiboard/include/spi.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 * @author Nicola Orlandini <n.orlandini90@gmail.com>
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @author Leonardo Morichelli <leonardo.morichelli@live.com>
 * @brief SPI definitions and prototypes
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_SPI

/**
 * @defgroup SPI SPI
 * @brief SPI HAL driver
 * @{
 */

#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "system.h"

/**
 * @defgroup SPI_Configuration_Functions SPI configuration functions and types
 * @brief Functions and types to open, close and configure a SPI peripheral.
 * @{
 */

#define SPI_EMPTY_BYTE                           ((uint8_t)0x00)
#define SPI_EMPTY_WORD                           ((uint16_t)0x0000)

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Spi_DeviceState
{
    SPI_DEVICESTATE_RESET,
    SPI_DEVICESTATE_READY,
    SPI_DEVICESTATE_BUSY,
    SPI_DEVICESTATE_ERROR,

} Spi_DeviceState;

/**
 * SPI operating mode.
 */
typedef enum
{
    SPI_MASTER_MODE,
    SPI_SLAVE_MODE,

} Spi_DeviceType;

typedef enum
{
    SPI_TX_BUFFER,
    SPI_RX_BUFFER,

} Spi_BufferType;

#if defined (LIBOHIBOARD_K10D10)      || \
    defined (LIBOHIBOARD_K10D7)       || \
    defined (LIBOHIBOARD_K12D5)       || \
    defined (LIBOHIBOARD_K60DZ10)     || \
    defined (LIBOHIBOARD_OHIBOARD_R1) || \
    defined (LIBOHIBOARD_K64F12)      || \
    defined (LIBOHIBOARD_FRDMK64F)    || \
    defined (LIBOHIBOARD_KV46F)       || \
    defined (LIBOHIBOARD_TRWKV46F)

typedef enum {
    SPI_CONTINUOUS_SCK,                            /**< Set to Continuous SCK */
    SPI_NOT_CONTINUOUS_SCK                     /**< Set to not Continuous SCK */
} Spi_ContinousSck;

typedef enum {
    SPI_CS_NONE = 0x0,
    SPI_CS_0    = 0x1,
    SPI_CS_1    = 0x2,
    SPI_CS_2    = 0x4,
    SPI_CS_3    = 0x8,
    SPI_CS_4    = 0xF,
} Spi_ChipSelect;

typedef enum {
    SPI_CS_FORMAT_DISCONTINUOS = 0x0,
    SPI_CS_FORMAT_CONTINUOS    = 0x1,
} Spi_CSFormat;

#endif

/**
 * SPI serial clock steady state.
 */
typedef enum
{
    SPI_SCK_INACTIVE_STATE_LOW,
    SPI_SCK_INACTIVE_STATE_HIGH,

} Spi_ClockPolarity;

/**
 * SPI clock active edge for the bit capture.
 */
typedef enum
{
    SPI_SCK_LEADING_EDGE_DATA_CAPTURED,
    SPI_SCK_LEADING_EDGE_DATA_CHANGED,

} Spi_ClockPhase;

/**
 * SPI data size.
 */
typedef enum
{
#if defined (LIBOHIBOARD_STM32L4)
    SPI_DATASIZE_4BIT  = 4,
    SPI_DATASIZE_5BIT  = 5,
    SPI_DATASIZE_6BIT  = 6,
    SPI_DATASIZE_7BIT  = 7,
#endif // LIBOHIBOARD_STM32L4
	SPI_DATASIZE_8BIT  = 8,
#if defined (LIBOHIBOARD_STM32L4)
    SPI_DATASIZE_9BIT  = 9,
    SPI_DATASIZE_10BIT = 10,
    SPI_DATASIZE_11BIT = 11,
    SPI_DATASIZE_12BIT = 12,
    SPI_DATASIZE_13BIT = 13,
    SPI_DATASIZE_14BIT = 14,
    SPI_DATASIZE_15BIT = 15,
#endif // LIBOHIBOARD_STM32L4
	SPI_DATASIZE_16BIT = 16,
#if defined (LIBOHIBOARD_PIC24FJ)
    SPI_DATASIZE_32BIT = 32,
#endif

} Spi_DataSize;


/**
 * Specifies the first bit transmitted, that is whether data transfer starts
 * from MSB or LSB bit.
 */
typedef enum
{
    SPI_FIRSTBIT_MSB,
    SPI_FIRSTBIT_LSB,

} Spi_FirstBit;

#if defined (LIBOHIBOARD_ST_STM32)

#if defined (LIBOHIBOARD_STM32L4) || \
    defined (LIBOHIBOARD_STM32L0)
/**
 * SPI Direction Mode.
 */
typedef enum
{
    SPI_DIRECTION_FULL_DUPLEX,
    SPI_DIRECTION_HALF_DUPLEX,
    SPI_DIRECTION_RX_ONLY,
#if defined (LIBOHIBOARD_STM32L4)
    SPI_DIRECTION_TX_ONLY,
#endif

} Spi_Direction;

/**
 * SPI Frame format (Motorola or TI)
 */
typedef enum
{
    SPI_FRAMEFORMAT_MOTOROLA,
    SPI_FRAMEFORMAT_TI,

} Spi_FrameFormat;

#endif // LIBOHIBOARD_STM32L0 || LIBOHIBOARD_STM32L4

#endif // LIBOHIBOARD_ST_STM32

/**
 * Slave Select (SS) pins management type.
 */
typedef enum
{
    SPI_SSMANAGEMENT_SOFTWARE,
#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    SPI_SSMANAGEMENT_HARDWARE,
#endif // LIBOHIBOARD_MICROCHIP_PIC
#if defined (LIBOHIBOARD_ST_STM32)
    SPI_SSMANAGEMENT_HARDWARE_INPUT,
    SPI_SSMANAGEMENT_HARDWARE_OUTPUT,
#if defined (LIBOHIBOARD_STM32L4)
    SPI_SSMANAGEMENT_HARDWARE_OUTPUT_PULSE,
#endif
#endif // LIBOHIBOARD_ST_STM
} Spi_SSManagement;

#if (LIBOHIBOARD_VERSION >= 0x20000)
typedef struct _Spi_Device* Spi_DeviceHandle;
#else
typedef struct Spi_Device* Spi_DeviceHandle;
#endif

#if defined (LIBOHIBOARD_STM32L0)

#include "hardware/STM32L0/spi_STM32L0.h"

#elif defined (LIBOHIBOARD_STM32L4)

#include "hardware/STM32L4/spi_STM32L4.h"

#elif defined (LIBOHIBOARD_PIC24FJ)

#include "hardware/PIC24FJ/spi_PIC24FJ.h"

#endif

/**
 *
 */
typedef struct _Spi_Config
{
    Spi_PcsPins       pcs0Pin;

#if defined (LIBOHIBOARD_NXP_KINETIS)

    Spi_PcsPins       pcs1Pin;
    Spi_PcsPins       pcs2Pin;
    Spi_PcsPins       pcs3Pin;
    Spi_PcsPins       pcs4Pin;

#endif

    Spi_SoutPins      soutPin;
    Spi_SinPins       sinPin;
    Spi_SckPins       sckPin;

    Spi_DeviceType    devType;
    uint32_t          baudrate;

    Spi_DataSize      datasize;
#if !defined (LIBOHIBOARD_MICROCHIP_PIC)
    Spi_FirstBit      firstBit;
#endif
    Spi_ClockPolarity sckPolarity;
    Spi_ClockPhase    sckPhase;

#if defined (LIBOHIBOARD_ST_STM32)

    Spi_Direction     direction;
    Spi_FrameFormat   frameFormat;
#endif

    Spi_SSManagement  ssManagement;

#if defined (LIBOHIBOARD_K10D10)      || \
    defined (LIBOHIBOARD_K12D5)       || \
    defined (LIBOHIBOARD_K10D7)       || \
    defined (LIBOHIBOARD_K60DZ10)     || \
    defined (LIBOHIBOARD_OHIBOARD_R1) || \
    defined (LIBOHIBOARD_K64F12)      || \
    defined (LIBOHIBOARD_FRDMK64F)    || \
    defined (LIBOHIBOARD_KV46F)       || \
    defined (LIBOHIBOARD_TRWKV46F)

    uint32_t              frameSize;

    Spi_ContinousSck      continuousSck;

#endif

#if defined (LIBOHIBOARD_KV46F)       || \
	defined (LIBOHIBOARD_TRWKV46F)

    void (*callback)(void);
    bool mtfEnable;
    Spi_CSFormat csFormat;
    bool rxFifoEn;
    bool txFifoEn;
    bool rooeEn;

    Spi_PcsPins pcs5Pin;
    struct _csInactiveState
    {
        bool sc0;
        bool sc1;
        bool sc2;
        bool sc3;
        bool sc4;
        bool sc5;

    }csInactiveState;

    struct _intEventEn
    {
        bool TCF  ;
        bool TXRXS;
        bool EOQF ;
        bool TFUF ;
        bool TFFF ;
        bool RFOF ;
        bool RFDF ;
    }intEventEn;

#endif

} Spi_Config;

/**
  * Initialize the SPI according to the specified parameters
  * in the @ref Spi_Config and initialize the associated handle.
  *
  * @param[in] dev Spi device handle
  * @param[in] config Configuration parameters list.
  * @return ERRORS_NO_ERROR The initialization is ok.
  */
System_Errors Spi_init (Spi_DeviceHandle dev, Spi_Config *config);

/**
  * Deitialize the SPI.
  *
  * @param[in] dev Spi device handle
  * @return ERRORS_NO_ERROR The initialization is ok.
  */
System_Errors Spi_deInit (Spi_DeviceHandle dev);

/**
 * This function check the chosen baudrate for the SPI communication,
 * and set the registers to obtain this value.
 *
 * @param[in] dev  Spi device handle
 * @param[in] speed The speed for clock signal
 * @return The function return one of the following error:
 *   @arg @ref ERRORS_NO_ERROR when the function success
 *   @arg @ref ERRORS_SPI_CLOCKSOURCE_FREQUENCY_TOO_LOW when the clock source is 0
 *   @arg @ref ERRORS_SPI_BAUDRATE_NOT_FOUND no compute result for the speed request
 */
System_Errors Spi_setBaudrate (Spi_DeviceHandle dev, uint32_t speed);

bool Spi_isInit(Spi_DeviceHandle dev);

/**
 * @}
 */

#if defined (LIBOHIBOARD_KV46F)  ||\
    defined (LIBOHIBOARD_TRWKV46F)

System_Errors Spi_read (Spi_DeviceHandle dev, uint16_t * data);
System_Errors Spi_write (Spi_DeviceHandle dev, uint16_t data, Spi_ChipSelect cs);

System_Errors Spi_txEnDisable (Spi_DeviceHandle dev, bool enable);
System_Errors Spi_flushBuffer (Spi_DeviceHandle dev, Spi_BufferType buffer);

#elif defined (LIBOHIBOARD_K10D10)      || \
      defined (LIBOHIBOARD_K12D5)       || \
      defined (LIBOHIBOARD_K60DZ10)     || \
      defined (LIBOHIBOARD_OHIBOARD_R1) || \
      defined (LIBOHIBOARD_K64F12)      || \
      defined (LIBOHIBOARD_FRDMK64F)

System_Errors Spi_readByte (Spi_DeviceHandle dev, uint8_t * data);
System_Errors Spi_read (Spi_DeviceHandle dev, uint32_t* data);

System_Errors Spi_writeByte (Spi_DeviceHandle dev, uint8_t data);
System_Errors Spi_write (Spi_DeviceHandle dev, uint32_t data, Spi_ChipSelect cs);

#else

/**
 * @defgroup SPI_Read_Write_Functions SPI read/write functions
 * @brief Functions to read and write to SPI peripheral.
 * @{
 */

/**
 * This function wait (blocking mode) until a new byte was
 * received into selected SPI.
 *
 * @deprecated Use @ref Spi_read function
 *
 * @note The following microcontrollers no longer implements this function:
 * @li STM32L0 Series
 * @li STM32L4 Series
 * @li PIC24FJ Series
 *
 * @param[in] dev Spi device handle
 * @param[out] data Pointer where store the received byte
 * @return
 */
System_Errors Spi_readByte (Spi_DeviceHandle dev, uint8_t * data);

/**
 * This function send a byte into SPI bus.
 *
 * @deprecated Use @ref Spi_write function
 *
 * @note The following microcontrollers no longer implements this function:
 * @li STM32L0 Series
 * @li STM32L4 Series
 * @li PIC24FJ Series
 *
 * @param[in] dev Spi device handle
 * @param[in] data Byte to send
 */
System_Errors Spi_writeByte (Spi_DeviceHandle dev, uint8_t data);

/**
 * This function send a packet into SPI bus.
 *
 * @param[in] dev Spi device handle
 * @param[in] data Pointer to data buffer with only one element (two for 16bit message)
 * @param[in] timeout Max timeout in millisecond
 */
System_Errors Spi_write (Spi_DeviceHandle dev, const uint8_t* data, uint32_t timeout);

/**
 * This function wait (blocking mode) until a new packet was
 * received into selected Spi.
 *
 * @param[in] dev Spi device handle
 * @param[out] data Pointer to data buffer with only one element (two for 16bit message)
 * @param[in] timeout Max timeout in millisecond
 * @return
 */
System_Errors Spi_read (Spi_DeviceHandle dev, uint8_t* data, uint32_t timeout);

/**
 * @}
 */

#endif

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

#elif defined (LIBOHIBOARD_KL15Z4)

extern Spi_DeviceHandle OB_SPI0;
extern Spi_DeviceHandle OB_SPI1;

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

extern Spi_DeviceHandle OB_SPI0;
extern Spi_DeviceHandle OB_SPI1;

#elif defined (LIBOHIBOARD_K10D10)

extern Spi_DeviceHandle OB_SPI0;
extern Spi_DeviceHandle OB_SPI1;
extern Spi_DeviceHandle OB_SPI2;

#elif defined (LIBOHIBOARD_K12D5) || \
      defined (LIBOHIBOARD_K10D7)

extern Spi_DeviceHandle OB_SPI0;
extern Spi_DeviceHandle OB_SPI1;

#elif defined (LIBOHIBOARD_K60DZ10) || \
      defined (LIBOHIBOARD_OHIBOARD_R1)

extern Spi_DeviceHandle SPI0;
extern Spi_DeviceHandle SPI1;
extern Spi_DeviceHandle SPI2;

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

extern Spi_DeviceHandle OB_SPI0;
extern Spi_DeviceHandle OB_SPI1;
extern Spi_DeviceHandle OB_SPI2;

#elif defined (LIBOHIBOARD_KV46F)      || \
      defined (LIBOHIBOARD_TRWKV46F)

extern Spi_DeviceHandle OB_SPI0;

void  SPI0_IRQHandler(void);

#endif

#ifdef __cplusplus
}
#endif

#endif // __SPI_H

/**
 * @}
 */

#endif // LIBOHIBOARD_SPI

/**
 * @}
 */
