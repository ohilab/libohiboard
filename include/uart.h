/**
 * @file libohiboard/include/uart.h
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @brief UART definitions and prototypes
 */

#ifndef UART_H_
#define UART_H_

#include "libohiboard.h"

/*typedef enum BaudRate_t	{
	br9600,
	br19200,
	br38400,
	br57600,
	br115200
} BaudRate_t;*/

typedef enum {
	None,
	Even,
	Odd
} ParityMode_t;

typedef enum {
	bit8,
	bit9
} DataBits_t;

typedef struct uart_dev {
	UART_MemMapPtr 		regMap;
	
	uint32				baudRate;
	ParityMode_t 		parityMode;
	DataBits_t			dataBits;
} uart_dev;

int uart_init(uart_dev *dev,int sysclk, int baud);
void uart_enable(uart_dev *dev);
void uart_disable(uart_dev *dev);

char uart_getChar (uart_dev *dev);
void uart_putChar (uart_dev *dev, char c);
int uart_getCharPresent (uart_dev *dev);

extern struct uart_dev *UART2;

/*
#define UART0_DEV_DEFAULT	{	UART0_BASE_PTR \
								}
*/

#endif /* UART_H_ */
     
