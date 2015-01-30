/******************************************************************************
 * Copyright (C) 2012-2014 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Edoardo Bezzeccheri <coolman3@gmail.com>
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 *  Alessio Paolucci <a.paolucci89@gmail.com>
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
 * @file libohiboard/include/uart.h
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 * @author ALessio Paolucci <a.paolucci89@gmail.com>
 * @brief UART definitions and prototypes.
 */

#ifdef LIBOHIBOARD_UART

#ifndef __UART_H
#define __UART_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum {
    UART_PARITY_NONE,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} Uart_ParityMode;

typedef enum {
    UART_DATABITS_EIGHT,
    UART_DATABITS_NINE,
    UART_DATABITS_TEN,
} Uart_DataBits;

typedef enum {
#if defined(MKL15Z4) || defined(FRDMK20D50M) || defined(FRDMKL05Z) || defined(MK60F15) || \
	  defined(FRDMKL02Z) || defined(MKL02Z4) || defined(MK10DZ10) || defined(MK10D10) || \
	  defined(MK60DZ10) || defined(MK60F15) || defined(MK64F12) || defined(FRDMK64F) || \
	  defined(OHIBOARD_R1)
    UART_CLOCKSOURCE_BUS,
    UART_CLOCKSOURCE_SYSTEM,
#elif defined(MKL03Z4) || defined(FRDMKL03Z)
    UART_CLOCKSOURCE_IRC48,
    UART_CLOCKSOURCE_IRC8,
    UART_CLOCKSOURCE_IRC2,
    UART_CLOCKSOURCE_EXT,
#endif
} Uart_ClockSource;


typedef enum
{
#if defined (MK60DZ10)
    
#elif defined (OHIBOARD_R1)
    
#elif defined (FRDMKL25Z)

    UART_PINS_PTA1,
    UART_PINS_PTA15,
//    UART_PINS_PTA18, // Connected to crystal

    UART_PINS_PTB16,

    UART_PINS_PTC3,
    
    UART_PINS_PTD2,
    UART_PINS_PTD4,
    UART_PINS_PTD6,
    
    UART_PINS_PTE1,
    UART_PINS_PTE21,
    UART_PINS_PTE23,

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
    
#elif defined (MK10DZ10) || defined (MK10D10)

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

#elif defined (FRDMKL03Z) || defined (MKL03Z4)

    UART_PINS_PTA4,

    UART_PINS_PTB1R,
    UART_PINS_PTB2R,
    UART_PINS_PTB4,

#elif defined (MK64F12) || defined (FRDMK64F)
    
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

#endif

    UART_PINS_RXNONE,

} Uart_RxPins;

typedef enum
{
#if defined (MK60DZ10)
    
#elif defined (OHIBOARD_R1)
    
#elif defined (FRDMKL25Z)
    
    UART_PINS_PTA2,
    UART_PINS_PTA14,
//    UART_PINS_PTA19, // Connected to crystal

    UART_PINS_PTB17,

    UART_PINS_PTC4,
    
    UART_PINS_PTD3,
    UART_PINS_PTD5,
    UART_PINS_PTD7,
    
    UART_PINS_PTE0,
    UART_PINS_PTE20,
    UART_PINS_PTE22,
    
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
    
#elif defined (MK10DZ10) || defined (MK10D10)

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
    
#elif defined (MK64F12) || defined (FRDMK64F)
    
    UART_PINS_PTA2,
    UART_PINS_PTA14,

    UART_PINS_PTB11,
    UART_PINS_PTB17,

    UART_PINS_PTC4,
    UART_PINS_PTC15,
    UART_PINS_PTC17,

    UART_PINS_PTD3,
    UART_PINS_PTD7,
    UART_PINS_PTD8,
    UART_PINS_PTD9,

    UART_PINS_PTE0,
    UART_PINS_PTE4,
    UART_PINS_PTE8,
    UART_PINS_PTE9,
    UART_PINS_PTE24,

#elif defined (FRDMKL03Z) || defined (MKL03Z4)

    UART_PINS_PTA3,

    UART_PINS_PTB1T,
    UART_PINS_PTB2T,
    UART_PINS_PTB3,

#endif

    UART_PINS_TXNONE,

} Uart_TxPins;

typedef struct Uart_Device* Uart_DeviceHandle;

#if defined (MK60DZ10)
    
extern Uart_DeviceHandle UART0;
extern Uart_DeviceHandle UART1;
extern Uart_DeviceHandle UART2;
extern Uart_DeviceHandle UART3;
extern Uart_DeviceHandle UART4;
extern Uart_DeviceHandle UART5;

#elif defined (OHIBOARD_R1)
    
#elif defined (FRDMKL25Z)

extern Uart_DeviceHandle UART0;
extern Uart_DeviceHandle UART1;
extern Uart_DeviceHandle UART2;

#elif defined(LIBOHIBOARD_KL15Z4)

extern Uart_DeviceHandle UART0;
extern Uart_DeviceHandle UART1;
extern Uart_DeviceHandle UART2;

#elif defined (MK10DZ10) || defined (MK10D10)

extern Uart_DeviceHandle UART0;
extern Uart_DeviceHandle UART1;
extern Uart_DeviceHandle UART2;
extern Uart_DeviceHandle UART3;
extern Uart_DeviceHandle UART4;
extern Uart_DeviceHandle UART5;

#elif defined (FRDMKL03Z) || defined (MKL03Z4)

extern Uart_DeviceHandle UART0;

#elif defined (MK64F12)

extern Uart_DeviceHandle UART0;
extern Uart_DeviceHandle UART1;
extern Uart_DeviceHandle UART2;
extern Uart_DeviceHandle UART3;
extern Uart_DeviceHandle UART4;
extern Uart_DeviceHandle UART5;

#endif


typedef struct _Uart_Config
{
    Uart_RxPins rxPin;
    Uart_TxPins txPin;
    
    Uart_ClockSource clockSource;
    
    Uart_DataBits dataBits;
    Uart_ParityMode parity;
    
    uint32_t baudrate;

#if defined (FRDMKL03Z) || defined (MKL03Z4)
    uint8_t oversampling; /* 4 to 32 */
    uint32_t extClk;      /* external frequency or crystal value if clockSource = UART_CLOCKSOURCE_EXT */
#endif

} Uart_Config;

System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out);
void Uart_putChar (Uart_DeviceHandle dev, char c);
uint8_t Uart_isCharPresent (Uart_DeviceHandle dev);
uint8_t Uart_isTransmissionComplete (Uart_DeviceHandle dev);

System_Errors Uart_open (Uart_DeviceHandle dev, void *callback, Uart_Config *config);
System_Errors Uart_close (Uart_DeviceHandle dev);

System_Errors Uart_setRxPin (Uart_DeviceHandle dev, Uart_RxPins rxPin);
System_Errors Uart_setTxPin (Uart_DeviceHandle dev, Uart_TxPins txPin);

void Uart_sendString (Uart_DeviceHandle dev, const char* text);
void Uart_sendData (Uart_DeviceHandle dev, const char* data, uint8_t length);
void Uart_sendHex (Uart_DeviceHandle dev, const char* data, uint8_t length);

#endif /* __UART_H */

#endif
