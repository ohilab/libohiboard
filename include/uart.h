/**
 * @file libohiboard/include/uart.h
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @brief UART definitions and prototypes
 */

#ifndef UART_H_
#define UART_H_

#include "libohiboard.h"

typedef struct uart_dev_ *uart_dev;

typedef enum {
	None,
	Even,
	Odd
} ParityMode_t;

typedef enum {
	bit8,
	bit9
} DataBits_t;

error_t uart_init(uart_dev dev);

error_t uart_setBaudRate(uart_dev dev, uint32 br);
error_t uart_enable(uart_dev dev);
error_t uart_disable(uart_dev dev);

error_t uart_getChar (uart_dev dev, char *out);
void uart_putChar (uart_dev dev, char c);
int uart_getCharPresent (uart_dev dev);

extern uart_dev UART2;
extern uart_dev UART3;

#endif /* UART_H_ */
     
