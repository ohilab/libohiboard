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
#include "utility.h"

#define UART_DEVICE_ENABLE(REGMAP)        (REGMAP->CR1 |= USART_CR1_UE)
#define UART_DEVICE_DISABLE(REGMAP)       (REGMAP->CR1 &= ~USART_CR1_UE)

#define UART_MAX_PINS                     10
#define UART_MAX_BAUDRATE                 10000000U

#define UART_CLOCK_ENABLE(REG,MASK)       do { \
                                            UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                          } while (0)

/**
  * @brief Check that number of stop bits is valid for UART.
  * @param STOPBITS Number of stop bits.
  * @retval TRUE if is valid, FALSE otherwise
  */
#define UART_VALID_STOPBITS(STOPBITS) (((STOPBITS) == UART_STOPBITS_HALF)         || \
                                       ((STOPBITS) == UART_STOPBITS_ONE)          || \
                                       ((STOPBITS) == UART_STOPBITS_ONE_AND_HALF) || \
                                       ((STOPBITS) == UART_STOPBITS_TWO))

/**
  * @brief Check that number of stop bits is valid for LPUART.
  * @param STOPBITS Number of stop bits.
  * @retval TRUE if is valid, FALSE otherwise
  */
#define UART_VALID_STOPBITS_LP(STOPBITS) (((STOPBITS) == UART_STOPBITS_ONE) || \
                                          ((STOPBITS) == UART_STOPBITS_TWO))

/**
  * @brief Check that UART parity is valid.
  * @param PARITY UART parity type.
  * @retval TRUE if is valid, FALSE otherwise
  */
#define UART_VALID_PARITY(PARITY) (((PARITY) == UART_PARITY_NONE) || \
                                   ((PARITY) == UART_PARITY_EVEN) || \
                                   ((PARITY) == UART_PARITY_ODD))

#define UART_VALID_DATABITS(DATABITS) (((DATABITS) == UART_DATABITS_SEVEN) || \
                                       ((DATABITS) == UART_DATABITS_EIGHT) || \
                                       ((DATABITS) == UART_DATABITS_NINE))

#define UART_VALID_FLOWCONTROL(FLOWCONTROL) (((FLOWCONTROL) == UART_FLOWCONTROL_NONE) || \
                                             ((FLOWCONTROL) == UART_FLOWCONTROL_CTS)  || \
                                             ((FLOWCONTROL) == UART_FLOWCONTROL_RTS)  || \
                                             ((FLOWCONTROL) == UART_FLOWCONTROL_CTS_RTS))

#define UART_VALID_MODE(MODE) (((MODE) == UART_MODE_TRANSMIT) || \
                               ((MODE) == UART_MODE_RECEIVE)  || \
                               ((MODE) == UART_MODE_BOTH))

#define UART_IS_LOWPOWER_DEVICE(DEVICE) ((DEVICE) == OB_LPUART1)

/**
 * @brief Check the baudrate value
 */
#define UART_VALID_BAUDRATE(BAUDRATE) ((BAUDRATE) <= UART_MAX_BAUDRATE)

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

static Uart_Device lpuart1 = {
        .regmap            = LPUART1,

        .rccRegisterPtr    = &RCC->APB2ENR,//FIXME
        .rccRegisterEnable = RCC_APB2ENR_USART1EN,//FIXME
};
Uart_DeviceHandle OB_LPUART1 = &lpuart1;

System_Errors Uart_config (Uart_DeviceHandle dev, Uart_Config * config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check all parameters with asserts
    // The OR is to detect an error: it is not important where is, the important is that there is!
    err = ohiassert(UART_VALID_DATABITS(config->dataBits));
    err |= ohiassert(UART_VALID_BAUDRATE(config->baudrate));
    if (UART_IS_LOWPOWER_DEVICE(dev))
    {
        err |= ohiassert(UART_VALID_STOPBITS_LP(config->stop));
    }
    else
    {
        err |= ohiassert(UART_VALID_STOPBITS(config->stop));
    }
    err |= ohiassert(UART_VALID_PARITY(config->parity));
    err |= ohiassert(UART_VALID_FLOWCONTROL(config->flowControl));
    err |= ohiassert(UART_VALID_MODE(config->mode));

    if (err != ERRORS_NO_ERROR)
        return ERRORS_UART_WRONG_PARAM;

    // Disable the peripheral
    UART_DEVICE_DISABLE(dev->regmap);

    // Config peripheral
    // Configure data bits with the M bits
    dev->regmap->CR1 = dev->regmap->CR1 & (~(USART_CR1_M_Msk));
    switch (config->dataBits)
    {
    case UART_DATABITS_SEVEN:
        dev->regmap->CR1 |= USART_CR1_M1_Msk;
        break;
    case UART_DATABITS_EIGHT:
        dev->regmap->CR1 |= 0x00U;
        break;
    case UART_DATABITS_NINE:
        dev->regmap->CR1 |= USART_CR1_M0_Msk;
        break;
    }

    // Configure parity type with PCE and PS bit
    dev->regmap->CR1 = dev->regmap->CR1 & (~(USART_CR1_PCE_Msk | USART_CR1_PS_Msk));
    switch (config->parity)
    {
    case UART_PARITY_NONE:
        dev->regmap->CR1 |= 0x00U;
        break;
    case UART_PARITY_ODD:
        dev->regmap->CR1 |= USART_CR1_PCE_Msk | USART_CR1_PS_Msk;
        break;
    case UART_PARITY_EVEN:
        dev->regmap->CR1 |= USART_CR1_PCE_Msk;
        break;
    }

    // Configure peripheral mode with TE and RE bits into CR1
    dev->regmap->CR1 = dev->regmap->CR1 & (~(USART_CR1_TE_Msk | USART_CR1_RE_Msk));
    switch (config->stop)
    {
    case UART_MODE_TRANSMIT:
        dev->regmap->CR1 |= USART_CR1_TE_Msk;
        break;
    case UART_MODE_RECEIVE:
        dev->regmap->CR1 |= USART_CR1_RE_Msk;
        break;
    case UART_MODE_BOTH:
        dev->regmap->CR1 |= USART_CR1_TE_Msk | USART_CR1_RE_Msk;
        break;
    }

    // Set oversampling: if value differ from 8, use default value.
    dev->regmap->CR1 = dev->regmap->CR1 & (~(USART_CR1_OVER8_Msk));
    if (config->oversampling == 8)
    {
        dev->regmap->CR1 |= USART_CR1_OVER8_Msk;
    }

    // Configure stop bits with STOP[13:12] bits into CR2
    dev->regmap->CR2 = dev->regmap->CR2 & (~(USART_CR2_STOP_Msk));
    switch (config->stop)
    {
    case UART_STOPBITS_HALF:
        dev->regmap->CR2 |= (USART_CR2_STOP_Msk & (0x01UL << USART_CR2_STOP_Pos));
        break;
    case UART_STOPBITS_ONE:
        dev->regmap->CR2 |= (USART_CR2_STOP_Msk & (0x00UL << USART_CR2_STOP_Pos));
        break;
    case UART_STOPBITS_ONE_AND_HALF:
        dev->regmap->CR2 |= (USART_CR2_STOP_Msk & (0x03UL << USART_CR2_STOP_Pos));
        break;
    case UART_STOPBITS_TWO:
        dev->regmap->CR2 |= (USART_CR2_STOP_Msk & (0x02UL << USART_CR2_STOP_Pos));
        break;
    }

    // Configure hardware flow control with CTS and RTS bits into CR3
    dev->regmap->CR3 = dev->regmap->CR3 & (~(USART_CR3_CTSE_Msk | USART_CR3_RTSE_Msk));
    switch (config->stop)
    {
    case UART_FLOWCONTROL_NONE:
        dev->regmap->CR3 |= 0x0U;
        break;
    case UART_FLOWCONTROL_CTS:
        dev->regmap->CR3 |= USART_CR3_CTSE_Msk;
        break;
    case UART_FLOWCONTROL_RTS:
        dev->regmap->CR3 |= USART_CR3_RTSE_Msk;
        break;
    case UART_FLOWCONTROL_CTS_RTS:
        dev->regmap->CR3 |= USART_CR3_CTSE_Msk | USART_CR3_RTSE_Msk;
        break;
    }

    // TODO: ONEBIT One sample bit method enable

    // Configure Baudrate
    Uart_setBaudrate(dev,config->baudrate);

    // Enable the peripheral
    UART_DEVICE_ENABLE(dev->regmap);

    return ERRORS_NO_ERROR;
}

void Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate)
{
//    switch ()
}

System_Errors Uart_open (Uart_DeviceHandle dev, Uart_Config *config)
{
    if (dev == NULL)
    {
        return ERRORS_UART_NO_DEVICE;
    }

    // FIXME: FLOW CONTROL=???

    // Enable peripheral clock
    UART_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);
    // ENABLE PIN e CLOCK

    // Configure the peripheral
    Uart_config(dev,config);

    return ERRORS_NO_ERROR;
}

#endif // LIBOHIBOARD_STM32L476

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_UART
