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
#include "gpio.h"
#include "system.h"

#define UART_DEVICE_ENABLE(REGMAP)        (REGMAP->CR1 |= USART_CR1_UE)
#define UART_DEVICE_DISABLE(REGMAP)       (REGMAP->CR1 &= ~USART_CR1_UE)

#define UART_MAX_PINS                     10
#define UART_MAX_BAUDRATE                 10000000U

#define UART_LP_BRR_MIN                   0x00000300U
#define UART_LP_BRR_MAX                   0x000FFFFFU

#define UART_BRR_MIN                      0x10U
#define UART_BRR_MAX                      0x0000FFFFU

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

#if defined (LIBOHIBOARD_STM32L476Jx) // WLCSP72 ballout

#define UART_IS_DEVICE(DEVICE) (((DEVICE) == OB_UART1)  || \
                                ((DEVICE) == OB_UART2)  || \
                                ((DEVICE) == OB_UART3)  || \
                                ((DEVICE) == OB_UART4)  || \
                                ((DEVICE) == OB_UART5))
#endif

#define UART_IS_LOWPOWER_DEVICE(DEVICE) ((DEVICE) == OB_LPUART1)

#define UART_IS_VALID_CLOCK_SOURCE(DEVICE,CLOCKSOURCE) (((CLOCKSOURCE) == UART_CLOCKSOURCE_PCLK)  || \
                                                        ((CLOCKSOURCE) == UART_CLOCKSOURCE_LSE)   || \
                                                        ((CLOCKSOURCE) == UART_CLOCKSOURCE_HSI)   || \
                                                        ((CLOCKSOURCE) == UART_CLOCKSOURCE_SYSCLK))

/**
 * @brief Check the baudrate value
 */
#define UART_VALID_BAUDRATE(BAUDRATE) ((BAUDRATE) <= UART_MAX_BAUDRATE)

typedef struct Uart_Device
{
    USART_TypeDef* regmap;                         /**< Device memory pointer */

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    volatile uint32_t* rccTypeRegisterPtr;  /**< Register for clock enabling. */
    uint32_t rccTypeRegisterMask;      /**< Register mask for user selection. */

    Uart_RxPins rxPins[UART_MAX_PINS];
    Uart_TxPins txPins[UART_MAX_PINS];

    Gpio_Pins rxPinsGpio[UART_MAX_PINS];
    Gpio_Pins txPinsGpio[UART_MAX_PINS];
    Gpio_Alternate rxPinsMux[UART_MAX_PINS];
    Gpio_Alternate txPinsMux[UART_MAX_PINS];

    Uart_ClockSource clockSource;
    bool oversmpling8;

    Uart_DataBits databits;
    Uart_ParityMode parity;

//
//    void (*isr)(void);                     /**< The function pointer for ISR. */
    void (*callbackRx)(void); /**< The function pointer for user Rx callback. */
    void (*callbackTx)(void); /**< The function pointer for user Tx callback. */
//    Interrupt_Vector isrNumber;                       /**< ISR vector number. */
//
//    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Uart_Device;

#if defined (LIBOHIBOARD_STM32L476Jx) // WLCSP72 ballout

static Uart_Device uart1 = {
        .regmap              = USART1,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_USART1EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_USART1SEL,

        .rxPins              =
        {
                               UART_PINS_PA10,
                               UART_PINS_PB7,
                               UART_PINS_PG10,
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA10,
                               GPIO_PINS_PB7,
                               GPIO_PINS_PG10,
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_7,
                               GPIO_ALTERNATE_7,
                               GPIO_ALTERNATE_7,
        },

        .txPins              =
        {
                               UART_PINS_PA9,
                               UART_PINS_PB6,
                               UART_PINS_PG9,
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA9,
                               GPIO_PINS_PB6,
                               GPIO_PINS_PG9,
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_7,
                               GPIO_ALTERNATE_7,
                               GPIO_ALTERNATE_7,
        },

};
Uart_DeviceHandle OB_UART1 = &uart1;

static Uart_Device uart2 = {
        .regmap            = USART2,

        .rccRegisterPtr    = &RCC->APB1ENR1,
        .rccRegisterEnable = RCC_APB1ENR1_USART2EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_USART2SEL,

        .rxPins              =
        {
                               UART_PINS_PA3,
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA3,
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_7,
        },

        .txPins              =
        {
                               UART_PINS_PA2,
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA2,
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_7,
        },
};
Uart_DeviceHandle OB_UART2 = &uart2;

static Uart_Device uart3 = {
        .regmap            = USART3,

        .rccRegisterPtr    = &RCC->APB1ENR1,
        .rccRegisterEnable = RCC_APB1ENR1_USART3EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_USART3SEL,

        .rxPins              =
        {
                               UART_PINS_PB11_RX,
                               UART_PINS_PC5,
                               UART_PINS_PC11,
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PB11,
                               GPIO_PINS_PC5,
                               GPIO_PINS_PC11,
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_7,
                               GPIO_ALTERNATE_7,
                               GPIO_ALTERNATE_7,
        },

        .txPins              =
        {
                               UART_PINS_PB10_TX,
                               UART_PINS_PC4,
                               UART_PINS_PC10,
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PB10,
                               GPIO_PINS_PC4,
                               GPIO_PINS_PC10,
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_7,
                               GPIO_ALTERNATE_7,
                               GPIO_ALTERNATE_7,
        },
};
Uart_DeviceHandle OB_UART3 = &uart3;

static Uart_Device uart4 = {
        .regmap            = UART4,

        .rccRegisterPtr    = &RCC->APB1ENR1,
        .rccRegisterEnable = RCC_APB1ENR1_UART4EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_UART4SEL,

        .rxPins              =
        {
                               UART_PINS_PA1,
                               UART_PINS_PC11,
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA1,
                               GPIO_PINS_PC11,
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_8,
                               GPIO_ALTERNATE_8,
        },

        .txPins              =
        {
                               UART_PINS_PA0,
                               UART_PINS_PC10,
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA0,
                               GPIO_PINS_PC10,
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_8,
                               GPIO_ALTERNATE_8,
        },
};
Uart_DeviceHandle OB_UART4 = &uart4;

static Uart_Device uart5 = {
        .regmap            = UART5,

        .rccRegisterPtr    = &RCC->APB1ENR1,
        .rccRegisterEnable = RCC_APB1ENR1_UART5EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_UART5SEL,

        .rxPins              =
        {
                               UART_PINS_PD2,
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PD2,
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_8,
        },

        .txPins              =
        {
                               UART_PINS_PC12,
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PC12,
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_8,
        },
};
Uart_DeviceHandle OB_UART5 = &uart5;

static Uart_Device lpuart1 = {
        .regmap            = LPUART1,

        .rccRegisterPtr    = &RCC->APB1ENR2,
        .rccRegisterEnable = RCC_APB1ENR2_LPUART1EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_LPUART1SEL,

        .rxPins              =
        {
                               UART_PINS_PB10_RX,
                               UART_PINS_PC0,
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PB10,
                               GPIO_PINS_PC0,
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_8,
                               GPIO_ALTERNATE_8,
        },

        .txPins              =
        {
                               UART_PINS_PB11_TX,
                               UART_PINS_PC1,
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PB11,
                               GPIO_PINS_PC1,
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_8,
                               GPIO_ALTERNATE_8,
        },
};
Uart_DeviceHandle OB_LPUART1 = &lpuart1;

#endif

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
    dev->databits = config->dataBits;
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
    dev->parity = config->parity;
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
    switch (config->mode)
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
        dev->oversmpling8 = TRUE;
    }
    {
        dev->oversmpling8 = FALSE;
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
    switch (config->flowControl)
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

System_Errors Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate)
{
    uint32_t frequency = 0;

    // Get current parent clock
    switch (dev->clockSource)
    {
    case UART_CLOCKSOURCE_HSI:
        frequency = (uint32_t)CLOCK_FREQ_HSI;
        break;
    case UART_CLOCKSOURCE_LSE:
        frequency = (uint32_t)CLOCK_FREQ_LSE;
        break;
    case UART_CLOCKSOURCE_SYSCLK:
        frequency = Clock_getOutputClock(CLOCK_OUTPUT_SYSCLK);
        break;
    case UART_CLOCKSOURCE_PCLK:
        if (dev == OB_UART1)
            frequency = Clock_getOutputClock(CLOCK_OUTPUT_PCLK2);
        else
            frequency = Clock_getOutputClock(CLOCK_OUTPUT_PCLK1);
        break;
    default:
        ohiassert(0);
        return ERRORS_UART_NO_CLOCKSOURCE;
    }

    // Current clock is different from 0
    if (frequency != 0u)
    {
        if (UART_IS_LOWPOWER_DEVICE(dev))
        {
            // Check the range of current clock
            // The range is [3 * baudrate, 4096 * baudrate]
            if (((frequency < ( 3 * baudrate))) && (frequency > ( 4096 * baudrate)))
            {
                return ERRORS_UART_WRONG_BAUDRATE;
            }
            else
            {
                uint32_t uartdiv = ((uint64_t)frequency * 256u) / baudrate;
                if ((uartdiv >= UART_LP_BRR_MIN) && (uartdiv <= UART_LP_BRR_MAX))
                {
                    dev->regmap->BRR = uartdiv;
                }
                else
                {
                    return ERRORS_UART_WRONG_BAUDRATE;
                }

            }
        }
        else if (dev->oversmpling8)
        {
            uint32_t uartdiv = (frequency * 2u) / baudrate;
            if ((uartdiv >= UART_BRR_MIN) && (uartdiv <= UART_BRR_MAX))
            {
                // Fix BRR value
                // see page 1348 of RM0351 (Rev6)
                uint32_t brr = uartdiv & 0xFFF0U;
                brr |= (uint16_t)((uartdiv & (uint16_t)0x000FU) >> 1U);
                brr &= 0xFFF7; // Clear BRR3
                dev->regmap->BRR = brr;
            }
            else
            {
                return ERRORS_UART_WRONG_BAUDRATE;
            }
        }
        else
        {
            uint32_t uartdiv = frequency / baudrate;
            if ((uartdiv >= UART_BRR_MIN) && (uartdiv <= UART_BRR_MAX))
            {
                dev->regmap->BRR = uartdiv;
            }
            else
            {
                return ERRORS_UART_WRONG_BAUDRATE;
            }
        }
    }
    {
        return ERRORS_UART_CLOCKSOURCE_FREQUENCY_TOO_LOW;
    }

    return ERRORS_NO_ERROR;
}

System_Errors Uart_open (Uart_DeviceHandle dev, Uart_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the UART device
    if (dev == NULL)
    {
        return ERRORS_UART_NO_DEVICE;
    }
    // Check the UART instance
    err = ohiassert((UART_IS_DEVICE(dev)) || (UART_IS_LOWPOWER_DEVICE(dev)));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_UART_WRONG_DEVICE;
    }
    // Check clock source selections
    err = ohiassert(UART_IS_VALID_CLOCK_SOURCE(dev,config->clockSource));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_UART_WRONG_PARAM;
    }

    // Select clock source
    UTILITY_MODIFY_REGISTER(*dev->rccTypeRegisterPtr,dev->rccTypeRegisterMask,config->clockSource);
    // Enable peripheral clock
    UART_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

    // Enable pins
    if (config->rxPin != UART_PINS_RXNONE)
        Uart_setRxPin(dev, config->rxPin);

    if (config->txPin != UART_PINS_TXNONE)
        Uart_setTxPin(dev, config->txPin);

    // Configure the peripheral
    err = Uart_config(dev,config);
    if (err != ERRORS_NO_ERROR)
    {
        return err;
    }

    // TODO: Configure interrupt!

    return ERRORS_NO_ERROR;
}

System_Errors Uart_close (Uart_DeviceHandle dev)
{
    // TODO
}

System_Errors Uart_setRxPin (Uart_DeviceHandle dev, Uart_RxPins rxPin)
{
    uint8_t devPinIndex;

//    if (dev->devInitialized == 0)
//        return ERRORS_UART_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < UART_MAX_PINS; ++devPinIndex)
    {
        if (dev->rxPins[devPinIndex] == rxPin)
        {
            Gpio_configAlternate(dev->rxPinsGpio[devPinIndex],
                                 dev->rxPinsMux[devPinIndex]);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_UART_NO_PIN_FOUND;
}

System_Errors Uart_setTxPin (Uart_DeviceHandle dev, Uart_TxPins txPin)
{
    uint8_t devPinIndex;

//    if (dev->devInitialized == 0)
//        return ERRORS_UART_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < UART_MAX_PINS; ++devPinIndex)
    {
        if (dev->txPins[devPinIndex] == txPin)
        {
            Gpio_configAlternate(dev->txPinsGpio[devPinIndex],
                                 dev->txPinsMux[devPinIndex]);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_UART_NO_PIN_FOUND;
}

System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out)
{
    // deprecated
    ohiassert(0);
}

void Uart_putChar (Uart_DeviceHandle dev, char c)
{
    // deprecated
    ohiassert(0);
}

uint8_t Uart_isCharPresent (Uart_DeviceHandle dev)
{

}

uint8_t Uart_isTransmissionComplete (Uart_DeviceHandle dev)
{

}

System_Errors Uart_get (Uart_DeviceHandle dev, uint8_t *data, uint32_t timeout)
{
    uint16_t* temp;

    uint32_t timeoutEnd = System_currentTick() + timeout;

    while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,USART_ISR_RXNE) == 0)
    {
        if (System_currentTick() > timeoutEnd)
        {
            return ERRORS_UART_TIMEOUT_RX;
        }
    }

    // In case of 9B and parity NONE, the message is split into two byte
    if ((dev->databits == UART_DATABITS_NINE) && (dev->parity == UART_PARITY_NONE))
    {
        // Cast the pointer
        temp = (uint16_t *) data;
        *temp = (uint16_t) (dev->regmap->RDR);// FIXME: Add mask?!
    }
    else
    {
        *data = (uint8_t)(dev->regmap->RDR);// FIXME: add mask?!
    }

    return ERRORS_NO_ERROR;
}

System_Errors Uart_put (Uart_DeviceHandle dev, uint8_t data, uint32_t timeout)
{
    uint16_t* temp;

    uint32_t timeoutEnd = System_currentTick() + timeout;

    // Wait until the buffer is empty
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,USART_ISR_TXE) == 0)
    {
        if (System_currentTick() > timeoutEnd)
        {
            return ERRORS_UART_TIMEOUT_RX;
        }
    }

    // In case of 9B and parity NONE, the message is split into two byte
    if ((dev->databits == UART_DATABITS_NINE) && (dev->parity == UART_PARITY_NONE))
    {
        // Cast the pointer
        temp = (uint16_t *) data;
        dev->regmap->TDR = (*temp & 0x01FFu);
    }
    else
    {
        dev->regmap->TDR = (*temp & 0x00FFu);
    }
    // Start-up new timeout
    timeoutEnd = System_currentTick() + timeout;
    // Wait until the transmission is complete
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,USART_ISR_TC) == 0)
    {
        if (System_currentTick() > timeoutEnd)
        {
            return ERRORS_UART_TIMEOUT_RX;
        }
    }

    return ERRORS_NO_ERROR;
}

#endif // LIBOHIBOARD_STM32L476

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_UART
