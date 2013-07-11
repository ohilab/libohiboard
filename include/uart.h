/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Edoardo Bezzeccheri <coolman3@gmail.com>
 *	
 * Project: libohiboard
 * Package: UART
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
 * @file libohiboard/include/uart.h
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @brief UART definitions and prototypes
 */

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
	UART_EIGHT_BIT,
	UART_NINE_BIT
} Uart_DataBits;

typedef struct Uart_Device* Uart_DeviceHandle;

System_Errors Uart_init (Uart_DeviceHandle dev);

System_Errors Uart_setBaudRate (Uart_DeviceHandle dev, uint32 br);
System_Errors Uart_enable (Uart_DeviceHandle dev);
System_Errors Uart_disable (Uart_DeviceHandle dev);

System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out);
void Uart_putChar (Uart_DeviceHandle dev, char c);
int Uart_getCharPresent (Uart_DeviceHandle dev);

#if defined(MKL15Z4)
extern Uart_DeviceHandle UART1;
extern Uart_DeviceHandle UART2;

#elif defined(MK60DZ10)
extern Uart_DeviceHandle UART0;
extern Uart_DeviceHandle UART1;
extern Uart_DeviceHandle UART2;
extern Uart_DeviceHandle UART3;

#elif defined(FRDMKL05Z)

#elif defined(FRDMKL25Z)
extern Uart_DeviceHandle UART1;
extern Uart_DeviceHandle UART2;
#endif


#endif /* __UART_H */
     
