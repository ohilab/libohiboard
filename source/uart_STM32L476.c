/*
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/source/uart_STM32L476.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART implementations for STM32L476.
 */

#ifdef LIBOHIBOARD_UART

#ifdef __cplusplus
extern "C" {
#endif

#if defined (LIBOHIBOARD_STM32L476)

#include "uart.h"

#include "interrupt.h"
#include "clock.h"

#define UART_DEVICE_ENABLE(REGMAP)        (REGMAP->CR1 |= USART_CR1_UE)
#define UART_DEVICE_DISABLE(REGMAP)       (REGMAP->CR1 &= ~USART_CR1_UE)

#define UART_MAX_PINS                     10

typedef struct Uart_Device
{
    USART_TypeDef* regmap;                         /**< Device memory pointer */

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    Uart_RxPins rxPins[UART_MAX_PINS];
    Uart_TxPins txPins[UART_MAX_PINS];

    volatile uint32_t* rxPinsPtr[UART_MAX_PINS];
    volatile uint32_t* txPinsPtr[UART_MAX_PINS];
    uint8_t rxPinsMux[UART_MAX_PINS];
    uint8_t txPinsMux[UART_MAX_PINS];

    Uart_ClockSource clockSource;
//
//    void (*isr)(void);                     /**< The function pointer for ISR. */
    void (*callbackRx)(void); /**< The function pointer for user Rx callback. */
    void (*callbackTx)(void); /**< The function pointer for user Tx callback. */
//    Interrupt_Vector isrNumber;                       /**< ISR vector number. */
//
//    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Uart_Device;

static Uart_Device uart1 = {
        .regmap            = USART1,

        .rccRegisterPtr    = &RCC->APB2ENR,
        .rccRegisterEnable = RCC_APB2ENR_USART1EN,
};
Uart_DeviceHandle OB_UART1 = &uart1;

System_Errors Uart_open (Uart_DeviceHandle dev, Uart_Config *config)
{
    if (dev == NULL)
    {
        return ERRORS_UART_NO_DEVICE;
    }

    // FIXME: FLOW CONTROL=???

    // ENABLE PIN e CLOCK

    // Disable the peripheral
    UART_DEVICE_DISABLE(dev->regmap);

    // FIXME: Config peripheral

    // Enable the peripheral
    UART_DEVICE_ENABLE(dev->regmap);
}

#endif // LIBOHIBOARD_STM32L476

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_UART
