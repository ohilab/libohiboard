/*
 * Copyright (C) 2012-2018 A. C. Open Hardware Ideas Lab
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

#ifdef LIBOHIBOARD_UART

#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "system.h"

#ifdef LIBOHIBOARD_DMA
#include "dma.h"
#endif

typedef enum
{
    UART_PARITY_NONE,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} Uart_ParityMode;

typedef enum
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

typedef enum
{
    UART_STOPBITS_ONE,
    UART_STOPBITS_TWO,

#if defined (LIBOHIBOARD_ST_STM32)
    UART_STOPBITS_HALF,
    UART_STOPBITS_ONE_AND_HALF,
#endif

} Uart_StopBits;

typedef enum
{
    UART_FLOWCONTROL_NONE,

#if defined (LIBOHIBOARD_ST_STM32)
    UART_FLOWCONTROL_CTS,
    UART_FLOWCONTROL_RTS,
    UART_FLOWCONTROL_CTS_RTS,
#endif

} Uart_FlowControl;

typedef enum
{
    UART_MODE_TRANSMIT,
    UART_MODE_RECEIVE,
    UART_MODE_BOTH,

} Uart_Mode;

typedef enum
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

    UART_CLOCKSOURCE_PCLK    = 0x00U,                 /**< PCLK clock source */
    UART_CLOCKSOURCE_SYSCLK  = 0x01U,               /**< SYSCLK clock source */
    UART_CLOCKSOURCE_HSI     = 0x02U,                  /**< HSI clock source */
    UART_CLOCKSOURCE_LSE     = 0x03U,                  /**< LSE clock source */

#endif

} Uart_ClockSource;


typedef enum
{
#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

    UART_PINS_PTA4,

    UART_PINS_PTB1R,
    UART_PINS_PTB2R,
    UART_PINS_PTB4,
    
#elif defined (LIBOHIBOARD_KL15Z4)

    UART_PINS_PTA1,
    UART_PINS_PTA15,
    UART_PINS_PTA18,

    UART_PINS_PTB16,

    UART_PINS_PTC3,
    
    UART_PINS_PTD2,
    UART_PINS_PTD4,
    UART_PINS_PTD6,
    
    UART_PINS_PTE1,
    UART_PINS_PTE17,
    UART_PINS_PTE21,
    UART_PINS_PTE23,

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

    UART_PINS_PTA1,
    UART_PINS_PTA15,
    UART_PINS_PTA18,
    UART_PINS_PTB16,

    UART_PINS_PTC3,
    
    UART_PINS_PTD2,
    UART_PINS_PTD4,
    UART_PINS_PTD6,
    
    UART_PINS_PTE1,
    UART_PINS_PTE21,
    UART_PINS_PTE23,

    
#elif defined (LIBOHIBOARD_K10D10)

    UART_PINS_PTA1,
    UART_PINS_PTA15,

    UART_PINS_PTB10,
    UART_PINS_PTB16,

    UART_PINS_PTC3,
    UART_PINS_PTC14,
    UART_PINS_PTC16,

    UART_PINS_PTD2,
    UART_PINS_PTD6,
    UART_PINS_PTD8,

    UART_PINS_PTE1,
    UART_PINS_PTE5,
    UART_PINS_PTE9,
    UART_PINS_PTE17,
    UART_PINS_PTE25,

#elif defined (LIBOHIBOARD_K12D5)

    UART_PINS_PTA1,
    UART_PINS_PTA15,

    UART_PINS_PTB10,
    UART_PINS_PTB16,

    UART_PINS_PTC3,
    UART_PINS_PTC16,

    UART_PINS_PTD2,
    UART_PINS_PTD6,

    UART_PINS_PTE1,
    UART_PINS_PTE5,
    UART_PINS_PTE17,

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)
    
    UART_PINS_PTA1,
    UART_PINS_PTA15,
    
    UART_PINS_PTB10,
    UART_PINS_PTB16,
    
    UART_PINS_PTC3,
    UART_PINS_PTC14,
    UART_PINS_PTC16,
    
    UART_PINS_PTD2,
    UART_PINS_PTD6,
    UART_PINS_PTD8,
    
    UART_PINS_PTE1,
    UART_PINS_PTE5,
    UART_PINS_PTE9,
    UART_PINS_PTE25,

#elif defined (LIBOHIBOARD_K60DZ10)     || \
      defined (LIBOHIBOARD_OHIBOARD_R1)

    UART_PINS_PTA1,
    UART_PINS_PTA15,

    UART_PINS_PTB10,
    UART_PINS_PTB16,

    UART_PINS_PTC3,
    UART_PINS_PTC14,
    UART_PINS_PTC16,

    UART_PINS_PTD2,
    UART_PINS_PTD6,

    UART_PINS_PTE1,
    UART_PINS_PTE5,
    UART_PINS_PTE25,

#elif defined(LIBOHIBOARD_KV31F12)

    /* UART0 */
    UART_PINS_PTA1,
    UART_PINS_PTA15,
    UART_PINS_PTB0,
    UART_PINS_PTB16,
    UART_PINS_PTD6,

    /* UART1 */
    UART_PINS_PTC3,
    UART_PINS_PTE1,

    /* UART2 */
    UART_PINS_PTD2,
    UART_PINS_PTE17,

#elif defined(LIBOHIBOARD_KV46F) ||\
	  defined(LIBOHIBOARD_TWRKV46F)

    /* UART0 */
    UART_PINS_PTA1,
    UART_PINS_PTA15,
    UART_PINS_PTB0,
    UART_PINS_PTB16,
    UART_PINS_PTC6,
    UART_PINS_PTD6,
    UART_PINS_PTE21,

    /* UART1*/
    UART_PINS_PTC3,
    UART_PINS_PTE1,
    UART_PINS_PTE17,

#elif defined (LIBOHIBOARD_STM32L476)

#if defined (LIBOHIBOARD_STM32L476Jx) // WLCSP72 ballout

    UART_PINS_PA1,
    UART_PINS_PA3,
    UART_PINS_PA10,

    UART_PINS_PB7,
    UART_PINS_PB10_RX,
    UART_PINS_PB11_RX,

    UART_PINS_PC0,
    UART_PINS_PC5,
    UART_PINS_PC11,

    UART_PINS_PD2,

    UART_PINS_PG10,

#endif // LIBOHIBOARD_STM32L476Jx

#endif

    UART_PINS_RXNONE,

} Uart_RxPins;

typedef enum
{
#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

    UART_PINS_PTA3,

    UART_PINS_PTB1T,
    UART_PINS_PTB2T,
    UART_PINS_PTB3,

#elif defined (LIBOHIBOARD_KL15Z4)

    UART_PINS_PTA2,
    UART_PINS_PTA14,
    UART_PINS_PTA19,

    UART_PINS_PTB17,

    UART_PINS_PTC4,
    
    UART_PINS_PTD3,
    UART_PINS_PTD5,
    UART_PINS_PTD7,
    
    UART_PINS_PTE0,
    UART_PINS_PTE16,
    UART_PINS_PTE20,
    UART_PINS_PTE22,
    
#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)
    
    UART_PINS_PTA2,
    UART_PINS_PTA14,
    UART_PINS_PTA19,

    UART_PINS_PTB17,

    UART_PINS_PTC4,
    
    UART_PINS_PTD3,
    UART_PINS_PTD5,
    UART_PINS_PTD7,
    
    UART_PINS_PTE0,
    UART_PINS_PTE20,
    UART_PINS_PTE22,
    
#elif defined (LIBOHIBOARD_K10D10)

    UART_PINS_PTA2,
    UART_PINS_PTA14,

    UART_PINS_PTB11,
    UART_PINS_PTB17,

    UART_PINS_PTC4,
    UART_PINS_PTC15,
    UART_PINS_PTC17,

    UART_PINS_PTD3,
    UART_PINS_PTD7,
    UART_PINS_PTD9,

    UART_PINS_PTE0,
    UART_PINS_PTE4,
    UART_PINS_PTE8,
    UART_PINS_PTE16,
    UART_PINS_PTE24,

#elif defined (LIBOHIBOARD_K12D5)

    UART_PINS_PTA2,
    UART_PINS_PTA14,

    UART_PINS_PTB11,
    UART_PINS_PTB17,

    UART_PINS_PTC4,
    UART_PINS_PTC17,

    UART_PINS_PTD3,
    UART_PINS_PTD7,

    UART_PINS_PTE0,
    UART_PINS_PTE4,
    UART_PINS_PTE16,
    
#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)
    
    UART_PINS_PTA2,
    UART_PINS_PTA14,

    UART_PINS_PTB11,
    UART_PINS_PTB17,

    UART_PINS_PTC4,
    UART_PINS_PTC15,
    UART_PINS_PTC17,

    UART_PINS_PTD3,
    UART_PINS_PTD7,
    UART_PINS_PTD9,

    UART_PINS_PTE0,
    UART_PINS_PTE4,
    UART_PINS_PTE8,
    UART_PINS_PTE24,

#elif defined (LIBOHIBOARD_K60DZ10)     || \
      defined (LIBOHIBOARD_OHIBOARD_R1)

    UART_PINS_PTA2,
    UART_PINS_PTA14,

    UART_PINS_PTB11,
    UART_PINS_PTB17,

    UART_PINS_PTC4,
    UART_PINS_PTC15,
    UART_PINS_PTC17,

    UART_PINS_PTD3,
    UART_PINS_PTD7,

    UART_PINS_PTE0,
    UART_PINS_PTE4,
    UART_PINS_PTE24,

#elif defined(LIBOHIBOARD_KV31F12)

    /* UART0 */
    UART_PINS_PTA2,
    UART_PINS_PTA14,
    UART_PINS_PTB1,
    UART_PINS_PTB17,
    UART_PINS_PTD7,

    /* UART1 */
    UART_PINS_PTC4,
    UART_PINS_PTE0,

    /* UART2 */
    UART_PINS_PTD3,
    UART_PINS_PTE16,

#elif defined(LIBOHIBOARD_KV46F) ||\
      defined(LIBOHIBOARD_TWRKV46F)

    /* UART0 */
    UART_PINS_PTA2,
    UART_PINS_PTA14,
    UART_PINS_PTB1,
    UART_PINS_PTB17,
    UART_PINS_PTC7,
    UART_PINS_PTD7,
    UART_PINS_PTE20,

    /* UART1*/
    UART_PINS_PTC4,
    UART_PINS_PTE0,
    UART_PINS_PTE16,

#elif defined (LIBOHIBOARD_STM32L476)

#if defined (LIBOHIBOARD_STM32L476Jx) // WLCSP72 ballout

    UART_PINS_PA0,
    UART_PINS_PA2,
    UART_PINS_PA9,

    UART_PINS_PB6,
    UART_PINS_PB10_TX,
    UART_PINS_PB11_TX,

    UART_PINS_PC1,
    UART_PINS_PC4,
    UART_PINS_PC10,
    UART_PINS_PC12,

    UART_PINS_PG9,

#endif // LIBOHIBOARD_STM32L476Jx

#endif

    UART_PINS_TXNONE,

} Uart_TxPins;

#if (LIBOHIBOARD_VERSION >= 0x200)
typedef struct _Uart_Device* Uart_DeviceHandle;
#else
typedef struct Uart_Device* Uart_DeviceHandle;
#endif

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

extern Uart_DeviceHandle UART0;

#elif defined(LIBOHIBOARD_KL15Z4)

void UART0_IRQHandler ();
void UART1_IRQHandler ();
void UART2_IRQHandler ();

extern Uart_DeviceHandle OB_UART0;
extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;

#elif defined (LIBOHIBOARD_KL25Z4)     || \
      defined (LIBOHIBOARD_FRDMKL25Z)

void UART0_IRQHandler ();
void UART1_IRQHandler ();
void UART2_IRQHandler ();

extern Uart_DeviceHandle OB_UART0;
extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;

#elif defined (LIBOHIBOARD_K10D10)

void UART0_RX_TX_IRQHandler ();
void UART1_RX_TX_IRQHandler ();
void UART2_RX_TX_IRQHandler ();
void UART3_RX_TX_IRQHandler ();
void UART4_RX_TX_IRQHandler ();
void UART5_RX_TX_IRQHandler ();

extern Uart_DeviceHandle OB_UART0;
extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;
extern Uart_DeviceHandle OB_UART3;
extern Uart_DeviceHandle OB_UART4;
extern Uart_DeviceHandle OB_UART5;

#elif defined (LIBOHIBOARD_K12D5)

void UART0_RX_TX_IRQHandler ();
void UART1_RX_TX_IRQHandler ();
void UART2_RX_TX_IRQHandler ();
void UART3_RX_TX_IRQHandler ();

extern Uart_DeviceHandle OB_UART0;
extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;
extern Uart_DeviceHandle OB_UART3;

#elif defined (LIBOHIBOARD_K60DZ10) || \
    defined (LIBOHIBOARD_OHIBOARD_R1)

extern Uart_DeviceHandle UART0;
extern Uart_DeviceHandle UART1;
extern Uart_DeviceHandle UART2;
extern Uart_DeviceHandle UART3;
extern Uart_DeviceHandle UART4;

#elif defined (LIBOHIBOARD_K64F12)     || \
      defined (LIBOHIBOARD_FRDMK64F)

void UART0_RX_TX_IRQHandler ();
void UART1_RX_TX_IRQHandler ();
void UART2_RX_TX_IRQHandler ();
void UART3_RX_TX_IRQHandler ();
void UART4_RX_TX_IRQHandler ();
void UART5_RX_TX_IRQHandler ();

extern Uart_DeviceHandle OB_UART0;
extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;
extern Uart_DeviceHandle OB_UART3;
extern Uart_DeviceHandle OB_UART4;
extern Uart_DeviceHandle OB_UART5;

#elif defined(LIBOHIBOARD_KV31F12)

void UART0_RX_TX_IRQHandler ();
void UART1_RX_TX_IRQHandler ();
void UART2_RX_TX_IRQHandler ();

extern Uart_DeviceHandle OB_UART0;
extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;

#elif defined(LIBOHIBOARD_KV46F) ||\
      defined(LIBOHIBOARD_TWRKV46F)

void UART0_RX_TX_IRQHandler ();
void UART1_RX_TX_IRQHandler ();

extern Uart_DeviceHandle OB_UART0;
extern Uart_DeviceHandle OB_UART1;

#elif defined (LIBOHIBOARD_STM32L476)

#if defined (LIBOHIBOARD_STM32L476Jx) // WLCSP72 ballout

void LPUART1_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);

extern Uart_DeviceHandle OB_UART1;
extern Uart_DeviceHandle OB_UART2;
extern Uart_DeviceHandle OB_UART3;
extern Uart_DeviceHandle OB_UART4;
extern Uart_DeviceHandle OB_UART5;
extern Uart_DeviceHandle OB_LPUART1;

#endif

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
    uint8_t oversampling; /**< Into NXP microcontroller must be a value between 4 to 32,
                               otherwise, into ST microcontroller must be 16 or 8. In the value differ
                               from this two value, the default is used. */

#if (LIBOHIBOARD_VERSION >= 0x200u)
    /** Callback Function to handle RX Interrupt.*/
    void (*callbackRx)(struct _Uart_Device* dev);
    /** Callback Function to handle TX Interrupt.*/
    void (*callbackTx)(struct _Uart_Device* dev);
#else
    void (*callbackRx)(void);  /**< Callback Function to handle RX Interrupt.*/
    void (*callbackTx)(void);  /**< Callback Function to handle TX Interrupt.*/
#endif

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

    uint32_t extClk;      /* external frequency or crystal value if clockSource = UART_CLOCKSOURCE_EXT */

#elif defined (LIBOHIBOARD_KL15Z4)     || \
      defined (LIBOHIBOARD_KL25Z4)     || \
	  defined (LIBOHIBOARD_FRDMKL25Z)

    bool invertTx; /* invert Tx logical value */
#endif

} Uart_Config;

/** @name Packet Management
 *  All the functions useful for one packet managing.
 */
///@{

/**
 * This function wait (blocking mode) until a new char was
 * received into selected Uart.
 *
 * @deprecated Use @ref Uart_get function
 *
 * @note The following microcontrollers no longer implements this function:
 * @li STM32L476
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
 * @deprecated Use @ref Uart_put function
 *
 * @note The following microcontrollers no longer implements this function:
 * @li STM32L476
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
 * @li STM32L476
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
System_Errors Uart_get (Uart_DeviceHandle dev, uint8_t *data, uint32_t timeout);

/**
 * This function send a packet into serial bus.
 * In particular wait for empty space in the UART Tx FIFO and then send the data.
 *
 * @param[in] dev Uart device handle
 * @param[in] data Pointer to data buffer with only one element (two for 9bit message)
 * @param[in] timeout Max timeout in millisecond
 */
System_Errors Uart_put (Uart_DeviceHandle dev, const uint8_t* data, uint32_t timeout);

/**
 * Check the receiving queue to see if packet has arrived.
 *
 * @param[in] dev Uart device handle
 * @return TRUE if packet is present, FALSE otherwise.
 */
bool Uart_isPresent (Uart_DeviceHandle dev);

///@}

/** @name Configuration functions
 *  Functions to open, close and configure a UART peripheral.
 */
///@{

System_Errors Uart_open (Uart_DeviceHandle dev, Uart_Config *config);

System_Errors Uart_close (Uart_DeviceHandle dev);

System_Errors Uart_setRxPin (Uart_DeviceHandle dev, Uart_RxPins rxPin);
System_Errors Uart_setTxPin (Uart_DeviceHandle dev, Uart_TxPins txPin);
#if defined (LIBOHIBOARD_KL15Z4)     || \
    defined (LIBOHIBOARD_KL25Z4)     || \
    defined (LIBOHIBOARD_FRDMKL25Z)
void Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate, uint8_t oversampling);
#elif defined (LIBOHIBOARD_STM32L476)
System_Errors Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate);
#else
void Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate);
#endif

///@}

/** @name String management functions
 *  Functions to string over the UART peripheral.
 */
///@{

void Uart_sendString (Uart_DeviceHandle dev, const char* text);
void Uart_sendStringln (Uart_DeviceHandle dev, const char* text);
void Uart_sendData (Uart_DeviceHandle dev, const char* data, uint8_t length);
void Uart_sendHex (Uart_DeviceHandle dev, const char* data, uint8_t length);

///@}

#ifdef LIBOHIBOARD_DMA
uint8_t Uart_enableDmaTrigger (Uart_DeviceHandle dev, Dma_RequestSource request);
uint32_t* Uart_getRxRegisterAddress (Uart_DeviceHandle dev);
#endif

#ifdef __cplusplus
}
#endif

#endif // __UART_H

#endif // LIBOHIBOARD_UART
