/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2019 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Edoardo Bezzeccheri <coolman3@gmail.com>
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
 *  Matteo Civale <matteo.civale@gmail.com>
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
 * @file libohiboard/include/uart.h
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief UART definitions and prototypes.
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_UART

/**
 * @defgroup UART UART
 * @brief UART HAL driver
 * @{
 */

#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "system.h"

/**
 * @defgroup UART_Configuration_Functions UART configuration functions and types
 * @brief Functions and types to open, close and configure a UART peripheral.
 * @{
 */

#ifdef LIBOHIBOARD_DMA
#include "dma.h"
#endif

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Uart_DeviceState
{
    UART_DEVICESTATE_RESET,
    UART_DEVICESTATE_READY,
    UART_DEVICESTATE_BUSY,
    UART_DEVICESTATE_ERROR,

} Uart_DeviceState;

typedef enum _Uart_ParityMode
{
    UART_PARITY_NONE,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} Uart_ParityMode;

typedef enum _Uart_DataBits
{
#if defined (LIBOHIBOARD_ST_STM32)
    UART_DATABITS_SEVEN,
#endif

    UART_DATABITS_EIGHT,
    UART_DATABITS_NINE,

#if defined (LIBOHIBOARD_NXP_KINETIS)
    UART_DATABITS_TEN,
#endif

} Uart_DataBits;

typedef enum _Uart_StopBits
{
    UART_STOPBITS_ONE,
    UART_STOPBITS_TWO,

#if defined (LIBOHIBOARD_ST_STM32)
    UART_STOPBITS_HALF,
    UART_STOPBITS_ONE_AND_HALF,
#endif

} Uart_StopBits;

typedef enum _Uart_FlowControl
{
    UART_FLOWCONTROL_NONE,
            
#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    UART_FLOWCONTROL_RTS,
    UART_FLOWCONTROL_CTS_RTS,
#endif

#if defined (LIBOHIBOARD_ST_STM32)
    UART_FLOWCONTROL_CTS,
    UART_FLOWCONTROL_RTS,
    UART_FLOWCONTROL_CTS_RTS,
#endif

} Uart_FlowControl;

typedef enum _Uart_Mode
{
    UART_MODE_TRANSMIT,
    UART_MODE_RECEIVE,
    UART_MODE_BOTH,

} Uart_Mode;

typedef enum _Uart_ClockSource
{

#if defined (LIBOHIBOARD_NXP_KINETIS)

#if defined (LIBOHIBOARD_KL15Z4)     || \
    defined (LIBOHIBOARD_KL25Z4)     || \
    defined (LIBOHIBOARD_FRDMKL25Z)  || \
    defined (LIBOHIBOARD_FRDMKL25Z)  || \
    defined (LIBOHIBOARD_KV31F12)    || \
    defined (LIBOHIBOARD_K10DZ10)    || \
    defined (LIBOHIBOARD_K10D10)     || \
    defined (LIBOHIBOARD_K12D5)      || \
    defined (LIBOHIBOARD_K60DZ10)    || \
    defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

    UART_CLOCKSOURCE_BUS,
    UART_CLOCKSOURCE_SYSTEM,

#elif defined (LIBOHIBOARD_KL03Z4)   || \
      defined (LIBOHIBOARD_FRDMKL03Z)

    UART_CLOCKSOURCE_IRC48,
    UART_CLOCKSOURCE_IRC8,
    UART_CLOCKSOURCE_IRC2,
    UART_CLOCKSOURCE_EXT,

#elif defined(LIBOHIBOARD_KV46F)     ||\
      defined(LIBOHIBOARD_TWRKV46F)

    UART_CLOCKSOURCE_FAST_PERIPHERALS,

#endif

#elif defined (LIBOHIBOARD_ST_STM32)

    UART_CLOCKSOURCE_PCLK    = 0x00u,                  /**< PCLK clock source */
    UART_CLOCKSOURCE_SYSCLK  = 0x01u,                /**< SYSCLK clock source */
    UART_CLOCKSOURCE_HSI     = 0x02u,                   /**< HSI clock source */
    UART_CLOCKSOURCE_LSE     = 0x03u,                   /**< LSE clock source */

#endif
            
#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    UART_CLOCKSOURCE_PERIPHERAL = 0x00u,
#endif

} Uart_ClockSource;

#if defined(LIBOHIBOARD_MICROCHIP_PIC)

/**
 * This list define the behaviour of UART peripheral during IDLE mode.
 */
typedef enum _Uart_StopIdleMode
{
    UART_STOPIDLEMODE_CONTINUES    = 0x0000,
    UART_STOPIDLEMODE_DISCONTINUES = _U1MODE_USIDL_MASK,
} Uart_StopIdleMode;

/**
 * This list define the behaviour of UART peripheral during SLEEP mode.
 * When enabled, the peripheral continues sample the UxRX pin. 
 */
typedef enum _Uart_WakeUpSleepMode
{
    UART_WAKEUPSLEEPMODE_ENABLE  = _U1MODE_WAKE_MASK,
    UART_WAKEUPSLEEPMODE_DISABLE = 0x0000,
} Uart_WakeUpSleepMode;

#endif

#if (LIBOHIBOARD_VERSION >= 0x20000u)
typedef struct _Uart_Device* Uart_DeviceHandle;
#else
typedef struct Uart_Device* Uart_DeviceHandle;
#endif

#if defined (LIBOHIBOARD_STM32L0)

#include "hardware/STM32L0/uart_STM32L0.h"

#elif defined (LIBOHIBOARD_STM32L4)

#include "hardware/STM32L4/uart_STM32L4.h"

#elif defined (LIBOHIBOARD_PIC24FJ)

#include "hardware/PIC24FJ/uart_PIC24FJ.h"

#else

typedef enum _Uart_RxPins
{

    UART_PINS_RXNONE,

} Uart_RxPins;

typedef enum _Uart_TxPins
{

    UART_PINS_TXNONE,

} Uart_TxPins;

#endif

typedef struct _Uart_Config
{
    Uart_RxPins rxPin;
    Uart_TxPins txPin;
    Uart_Mode   mode;
    
    Uart_ClockSource clockSource;
    
    Uart_DataBits dataBits;
    Uart_ParityMode parity;
    Uart_StopBits stop;
    Uart_FlowControl flowControl;
    
    uint32_t baudrate;
    
#if !defined(LIBOHIBOARD_MICROCHIP_PIC)
    uint8_t oversampling; /**< Into NXP microcontroller must be a value between 4 to 32,
                               otherwise, into ST microcontroller must be 16 or 8. If the value differ
                               from this two value, the default one is used. */
#endif

#if defined (LIBOHIBOARD_MICROCHIP_PIC)
    
    Uart_StopIdleMode    idleMode;
    Uart_WakeUpSleepMode sleepMode;
    
#endif
    
#if (LIBOHIBOARD_VERSION >= 0x20000u)
    /** Callback Function to handle RX Interrupt.*/
    void (*callbackRx)(struct _Uart_Device* dev, void* obj);
    /** Callback Function to handle TX Interrupt.*/
    void (*callbackTx)(struct _Uart_Device* dev, void* obj);
    /** Useful object added to callback when interrupt triggered. */
    void* callbackObj;
#else
    void (*callbackRx)(void);  /**< Callback Function to handle RX Interrupt.*/
    void (*callbackTx)(void);  /**< Callback Function to handle TX Interrupt.*/
#endif
    
    uint8_t isrTxPriority;
    uint8_t isrRxPriority;

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

    uint32_t extClk;      /* external frequency or crystal value if clockSource = UART_CLOCKSOURCE_EXT */

#elif defined (LIBOHIBOARD_KL15Z4)     || \
      defined (LIBOHIBOARD_KL25Z4)     || \
	  defined (LIBOHIBOARD_FRDMKL25Z)

    bool invertTx; /* invert Tx logical value */
#endif

} Uart_Config;

/**
 * This function initialize the selected peripheral.
 *
 * @deprecated Use @ref Uart_init function
 *
 * @note The following microcontrollers no longer implements this function:
 * @li STM32L0 Series
 * @li STM32L4 Series
 * @li PIC24FJ Series
 *
 * @param[in] dev Uart device handle
 * @param[in] config Configuration parameters for the Uart
 */
System_Errors Uart_open (Uart_DeviceHandle dev, Uart_Config *config);

/**
 * This function de-initialize the selected peripheral.
 *
 * @deprecated Use @ref Uart_deInit function
 *
 * @note The following microcontrollers no longer implements this function:
 * @li STM32L0 Series
 * @li STM32L4 Series
 * @li PIC24FJ Series
 *
 * @param[in] dev Uart device handle
 */
System_Errors Uart_close (Uart_DeviceHandle dev);

/**
 * This function initialize the selected peripheral.
 *
 * @param[in] dev Uart device handle
 * @param[in] config Configuration parameters for the Uart
 */
System_Errors Uart_init (Uart_DeviceHandle dev, Uart_Config *config);

/**
 * This function de-initialize the selected peripheral.
 *
 * @param[in] dev Uart device handle
 */
System_Errors Uart_deInit (Uart_DeviceHandle dev);

System_Errors Uart_setRxPin (Uart_DeviceHandle dev, Uart_RxPins rxPin);
System_Errors Uart_setTxPin (Uart_DeviceHandle dev, Uart_TxPins txPin);

/**
 * This function check the chosen baudrate for the UART peripheral,
 * and set the registers to obtain this value.
 *
 * @param[in] dev  Uart device handle
 * @param[in] baudrate The selected baudrate for the communication
 * @return The function return one of the following error:
 *   @arg @ref ERRORS_NO_ERROR when the function success
 *   @arg @ref ERRORS_UART_WRONG_BAUDRATE when the baudrate is out of the maximum range
 *   @arg @ref ERRORS_UART_NO_CLOCKSOURCE no compute result for the baudrate request
 *   @arg @ref ERRORS_UART_CLOCKSOURCE_FREQUENCY_TOO_LOW The clock source frequency is too low (0 maybe)
 */
System_Errors Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate);

/**
 * @}
 */

/**
 * @defgroup UART_Read_Write_Functions UART read/write functions
 * @brief Functions to read and write to UART peripheral.
 * @{
 */

/**
 * This function wait (blocking mode) until a new char was
 * received into selected Uart.
 *
 * @deprecated Use @ref Uart_read function
 *
 * @note The following microcontrollers no longer implements this function:
 * @li STM32L0 Series
 * @li STM32L4 Series
 * @li PIC24FJ Series
 *
 * @param[in] dev Uart device handle
 * @param[out] out Pointer where store the received char
 * @return
 */
System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out);

/**
 * This function send a character into serial bus.
 * In particular wait for space in the UART Tx FIFO and then send the character.
 *
 * @deprecated Use @ref Uart_write function
 *
 * @note The following microcontrollers no longer implements this function:
 * @li STM32L0 Series
 * @li STM32L4 Series
 * @li PIC24FJ Series
 *
 * @param[in] dev Uart device handle
 * @param[in] c Character to send
 */
void Uart_putChar (Uart_DeviceHandle dev, char c);

/**
 * Check the receiving queue to see if packet has arrived.
 *
 * @deprecated This functionality is included into @ref Uart_get function and
 * it is replaced by @ref Uart_isPresent function
 *
 * @note The following microcontrollers no longer implements this function:
 * @li STM32L0 Series
 * @li STM32L4 Series
 * @li PIC24FJ Series
 *
 * @param[in] dev Uart device handle
 * @return uint8_t
 *  0 No character received
 *  1 Character has been received
 */
uint8_t Uart_isCharPresent (Uart_DeviceHandle dev);

/**
 * Check the transmission queue to see if is empty and the last char
 * was send.
 *
 * @param[in] dev Uart device handle
 * @return uint8_t
 *  0 The transmission has not been completed
 *  1 The transmission is over
 */
uint8_t Uart_isTransmissionComplete (Uart_DeviceHandle dev);

/**
 * This function wait (blocking mode) until a new packet was
 * received into selected Uart.
 *
 * @param[in] dev Uart device handle
 * @param[out] data Pointer to data buffer with only one element (two for 9bit message)
 * @param[in] timeout Max timeout in millisecond
 * @return
 */
System_Errors Uart_read (Uart_DeviceHandle dev, uint8_t *data, uint32_t timeout);

/**
 * This function send a packet into serial bus.
 * In particular wait for empty space in the UART Tx FIFO and then send the data.
 *
 * @param[in] dev Uart device handle
 * @param[in] data Pointer to data buffer with only one element (two for 9bit message)
 * @param[in] timeout Max timeout in millisecond
 */
System_Errors Uart_write (Uart_DeviceHandle dev, const uint8_t* data, uint32_t timeout);

/**
 * Check the receiving queue to see if packet has arrived.
 *
 * @param[in] dev Uart device handle
 * @return TRUE if packet is present, FALSE otherwise.
 */
bool Uart_isPresent (Uart_DeviceHandle dev);

/**
 * @}
 */

/**
 * @defgroup UART_String_Functions UART common string management functions
 * @brief Functions to manage string over the UART peripheral.
 * @{
 */

void Uart_sendString (Uart_DeviceHandle dev, const char* text);
void Uart_sendStringln (Uart_DeviceHandle dev, const char* text);
void Uart_sendData (Uart_DeviceHandle dev, const char* data, uint8_t length);
void Uart_sendHex (Uart_DeviceHandle dev, const char* data, uint8_t length);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __UART_H

/**
 * @}
 */

#endif // LIBOHIBOARD_UART

/**
 * @}
 */
