/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/STM32L0/uart_STM32L0.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART implementations for STM32L0 Series.
 */

#ifdef LIBOHIBOARD_UART

#ifdef __cplusplus
extern "C" {
#endif

#include "uart.h"

#include "interrupt.h"
#include "clock.h"
#include "utility.h"
#include "gpio.h"
#include "system.h"

#if defined (LIBOHIBOARD_STM32L0)

#define UART_DEVICE_IS_ENABLED(REGMAP)    (REGMAP->CR1 & USART_CR1_UE)
#define UART_DEVICE_ENABLE(REGMAP)        (REGMAP->CR1 |= USART_CR1_UE)
#define UART_DEVICE_DISABLE(REGMAP)       (REGMAP->CR1 &= ~USART_CR1_UE)

#define UART_MAX_PINS                     8

#define UART_LP_BRR_MIN                   0x00000300U
#define UART_LP_BRR_MAX                   0x000FFFFFU

#define UART_BRR_MIN                      0x10U
#define UART_BRR_MAX                      0x0000FFFFU

#define UART_CLOCK_ENABLE(REG,MASK)       do { \
                                            UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                          } while (0)

#define UART_CLOCK_DISABLE(REG,MASK)      do { \
		                                    UTILITY_CLEAR_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                          } while (0)

/**
  * @brief Check that number of stop bits is valid for UART.
  * @param STOPBITS Number of stop bits.
  * @retval TRUE if is valid, FALSE otherwise
  *
  * @hideinitializer
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

#define UART_IS_VALID_CLOCK_SOURCE(CLOCKSOURCE) (((CLOCKSOURCE) == UART_CLOCKSOURCE_PCLK)  || \
                                                 ((CLOCKSOURCE) == UART_CLOCKSOURCE_LSE)   || \
                                                 ((CLOCKSOURCE) == UART_CLOCKSOURCE_HSI)   || \
                                                 ((CLOCKSOURCE) == UART_CLOCKSOURCE_SYSCLK))

/**
 * @brief Check the baudrate value
 */
#define UART_VALID_BAUDRATE(BAUDRATE) ((BAUDRATE) <= UART_MAX_BAUDRATE)

#define UART_VALID_OVERSAMPLING(OVERSAMPLING) (((OVERSAMPLING) == 8) || \
                                               ((OVERSAMPLING) == 16))

typedef struct _Uart_Device
{
    USART_TypeDef* regmap;                         /**< Device memory pointer */

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    volatile uint32_t* rccTypeRegisterPtr;  /**< Register for clock enabling. */
    uint32_t rccTypeRegisterMask;      /**< Register mask for user selection. */
    uint32_t rccTypeRegisterPos;       /**< Mask position for user selection. */

    Uart_RxPins rxPins[UART_MAX_PINS];
    Uart_TxPins txPins[UART_MAX_PINS];
    Uart_CtsPins ctsPins[UART_MAX_PINS];
    Uart_RtsPins rtsPins[UART_MAX_PINS];

    Gpio_Pins rxPinsGpio[UART_MAX_PINS];
    Gpio_Pins txPinsGpio[UART_MAX_PINS];
    Gpio_Pins ctsPinsGpio[UART_MAX_PINS];
    Gpio_Pins rtsPinsGpio[UART_MAX_PINS];
    Gpio_Alternate rxPinsMux[UART_MAX_PINS];
    Gpio_Alternate txPinsMux[UART_MAX_PINS];
    Gpio_Alternate ctsPinsMux[UART_MAX_PINS];
    Gpio_Alternate rtsPinsMux[UART_MAX_PINS];

    Uart_Config config;

    uint16_t mask;              /**< Computed mask to use with received data. */

    /** The function pointer for user Rx callback. */
    void (*callbackRx)(struct _Uart_Device* dev, void* obj);
    /** The function pointer for user Tx callback. */
    void (*callbackTx)(struct _Uart_Device* dev, void* obj);
    /** Callback Function to handle Error Interrupt. */
    void (*callbackError)(struct _Uart_Device* dev, void* obj);
    /** Useful object added to callback when interrupt triggered. */
    void* callbackObj;

    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Uart_DeviceState state;

} Uart_Device;

#if defined (LIBOHIBOARD_STM32L0x2)

#define UART_IS_DEVICE(DEVICE) (((DEVICE) == OB_UART1)  || \
                                ((DEVICE) == OB_UART2)  || \
                                ((DEVICE) == OB_UART4)  || \
                                ((DEVICE) == OB_UART5))

#define UART_IS_LOWPOWER_DEVICE(DEVICE) ((DEVICE) == OB_LPUART1)

#define UART_IS_DUAL_CLOCK_DOMAIN(DEVICE) (((DEVICE) == OB_UART1)  || \
                                           ((DEVICE) == OB_UART2)  || \
                                           ((DEVICE) == OB_LPUART1))

static Uart_Device uart1 =
{
        .regmap              = USART1,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_USART1EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_USART1SEL,
        .rccTypeRegisterPos  = RCC_CCIPR_USART1SEL_Pos,

        .rxPins              =
        {
                               UART_PINS_PA10,
                               UART_PINS_PB7_RX,
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA10,
                               GPIO_PINS_PB7,
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_0,
        },

        .txPins              =
        {
                               UART_PINS_PA9,
                               UART_PINS_PB6,
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA9,
                               GPIO_PINS_PB6,
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_0,
        },

        .ctsPins             =
        {
                               UART_PINS_PA11,
                               UART_PINS_PB4_CTS,
        },
        .ctsPinsGpio         =
        {
                               GPIO_PINS_PA11,
                               GPIO_PINS_PB4,
        },
        .ctsPinsMux          =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_5,
        },

        .rtsPins             =
        {
                               UART_PINS_PA12,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PB3_RTS,
#endif
        },
        .rtsPinsGpio         =
        {
                               GPIO_PINS_PA12,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PB3,
#endif
        },
        .rtsPinsMux          =
        {
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_5,
#endif
        },

        .isrNumber           = INTERRUPT_USART1,

        .state               = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART1 = &uart1;

static Uart_Device uart2 =
{
        .regmap              = USART2,

        .rccRegisterPtr      = &RCC->APB1ENR,
        .rccRegisterEnable   = RCC_APB1ENR_USART2EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_USART2SEL,
        .rccTypeRegisterPos  = RCC_CCIPR_USART2SEL_Pos,

        .rxPins              =
        {
                               UART_PINS_PA3,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PA15_RX,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PD6,
#endif
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA3,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PA15,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PD6,
#endif
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_4,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .txPins              =
        {
                               UART_PINS_PA2,
                               UART_PINS_PA14,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PD5,
#endif
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA2,
                               GPIO_PINS_PA14,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PD5,
#endif
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .ctsPins             =
        {
                               UART_PINS_PA0_CTS,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PD3,
#endif

        },
        .ctsPinsGpio         =
        {
                               GPIO_PINS_PA0,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PD3,
#endif
        },
        .ctsPinsMux          =
        {
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .rtsPins             =
        {
                               UART_PINS_PA1_RTS,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PD4,
#endif

        },
        .rtsPinsGpio         =
        {
                               GPIO_PINS_PA1,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PD4,
#endif

        },
        .rtsPinsMux          =
        {
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .isrNumber           = INTERRUPT_USART2,

        .state               = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART2 = &uart2;

static Uart_Device uart4 =
{
        .regmap              = USART4,

        .rccRegisterPtr      = &RCC->APB1ENR,
        .rccRegisterEnable   = RCC_APB1ENR_USART4EN,

        .rccTypeRegisterPtr  = 0,
        .rccTypeRegisterMask = 0,
        .rccTypeRegisterPos  = 0,

        .txPins              =
        {
                               UART_PINS_PA0_TX,
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PC10,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PE8,
#endif
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA0,
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PC10,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PE8,
#endif
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
        },

        .rxPins              =
        {
                               UART_PINS_PA1_RX,
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PC11,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PE9,
#endif
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA1,
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PC11,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PE9,
#endif
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
        },

        .ctsPins             =
        {
                               UART_PINS_PB7_CTS,
        },
        .ctsPinsGpio         =
        {
                               GPIO_PINS_PB7,
        },
        .ctsPinsMux          =
        {
                               GPIO_ALTERNATE_6,
        },

        .rtsPins             =
        {
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PA15_RTS,
#else
                               UART_PINS_RTSNONE,
#endif
        },
        .rtsPinsGpio         =
        {
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PA15,
#else
                               GPIO_PINS_NONE,
#endif
        },
        .rtsPinsMux          =
        {
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#else
                               GPIO_ALTERNATE_ANALOG,
#endif
        },

        .isrNumber           = INTERRUPT_USART4_5,

        .state               = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART4 = &uart4;

static Uart_Device uart5 =
{
        .regmap              = USART5,

        .rccRegisterPtr      = &RCC->APB1ENR,
        .rccRegisterEnable   = RCC_APB1ENR_USART5EN,

        .rccTypeRegisterPtr  = 0,
        .rccTypeRegisterMask = 0,
        .rccTypeRegisterPos  = 0,

        .txPins              =
        {
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PB3_TX,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PC12,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PE10,
#endif
        },
        .txPinsGpio          =
        {
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PB3,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PC12,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PE10,
#endif
        },
        .txPinsMux           =
        {
#if defined (LIBOHIBOARD_STM32L072KxT) || \
    defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_2,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
        },

        .rxPins              =
        {
                               UART_PINS_PB4_RX,
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PD2,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PE11,
#endif
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PB4,
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PD2,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PE11,
#endif
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
        },

        .ctsPins             =
        {
                               UART_PINS_CTSNONE,
        },
        .ctsPinsGpio         =
        {
                               GPIO_PINS_NONE,
        },
        .ctsPinsMux          =
        {
                               GPIO_ALTERNATE_ANALOG,
        },

        .rtsPins             =
        {
                               UART_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PE7,
#endif
        },
        .rtsPinsGpio         =
        {
                               GPIO_PINS_PB5,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PE7,
#endif
        },
        .rtsPinsMux          =
        {
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
        },

        .isrNumber           = INTERRUPT_USART4_5,

        .state               = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART5 = &uart5;

static Uart_Device lpuart1 =
{
        .regmap              = LPUART1,

        .rccRegisterPtr      = &RCC->APB1ENR,
        .rccRegisterEnable   = RCC_APB1ENR_LPUART1EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_LPUART1SEL,
        .rccTypeRegisterPos  = RCC_CCIPR_LPUART1SEL_Pos,

        .rxPins              =
        {
                               UART_PINS_PA3,
                               UART_PINS_PA13,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PB10_RX,
                               UART_PINS_PB11_RX,
#endif
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PC0,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PC5,
                               UART_PINS_PC11,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PD9,
#endif
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA3,
                               GPIO_PINS_PA13,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PB10,
                               GPIO_PINS_PB11,
#endif
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PC0,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PC5,
                               GPIO_PINS_PC11,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PD9,
#endif
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_7,
                               GPIO_ALTERNATE_4,
#endif
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_0,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .txPins              =
        {
                               UART_PINS_PA2,
                               UART_PINS_PA14,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PB10_TX,
                               UART_PINS_PB11_TX,
#endif
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PC1,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PC4,
                               UART_PINS_PC10,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PD8,
#endif
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA2,
                               GPIO_PINS_PA14,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PB10,
                               GPIO_PINS_PB11,
#endif
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PC1,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PC4,
                               GPIO_PINS_PC10,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PD8,
#endif
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L072CxT) || \
    defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_7,
#endif
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_6,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_0,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .rtsPins             =
        {
                               UART_PINS_PB1,
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PB12,
                               UART_PINS_PB14,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PD2,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PD12,
#endif
        },
        .rtsPinsGpio         =
        {
                               GPIO_PINS_PB1,
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PB12,
                               GPIO_PINS_PB14,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PD2,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PD12,
#endif
        },
        .rtsPinsMux          =
        {
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_4,
#endif
#if defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_0,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .ctsPins             =
        {
                               UART_PINS_PA6_CTS,
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PB13,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               UART_PINS_PD11,
#endif
        },
        .ctsPinsGpio         =
        {
                               GPIO_PINS_PA6,
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PB13,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_PINS_PD11,
#endif
        },
        .ctsPinsMux          =
        {
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L072CxY) || \
    defined (LIBOHIBOARD_STM32L072RxT) || \
    defined (LIBOHIBOARD_STM32L072RxH) || \
    defined (LIBOHIBOARD_STM32L072RxI) || \
    defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_4,
#endif
#if defined (LIBOHIBOARD_STM32L072VxT) || \
    defined (LIBOHIBOARD_STM32L072VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .isrNumber           = INTERRUPT_LPUART1,

        .state               = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_LPUART1 = &lpuart1;

#elif defined (LIBOHIBOARD_STM32L0x3)

#define UART_IS_DEVICE(DEVICE) (((DEVICE) == OB_UART1)  || \
                                ((DEVICE) == OB_UART2)  || \
                                ((DEVICE) == OB_UART4)  || \
                                ((DEVICE) == OB_UART5))

#define UART_IS_LOWPOWER_DEVICE(DEVICE) ((DEVICE) == OB_LPUART1)

#define UART_IS_DUAL_CLOCK_DOMAIN(DEVICE) (((DEVICE) == OB_UART1)  || \
                                           ((DEVICE) == OB_UART2)  || \
                                           ((DEVICE) == OB_LPUART1))

static Uart_Device uart1 = {
        .regmap              = USART1,

        .rccRegisterPtr      = &RCC->APB2ENR,
        .rccRegisterEnable   = RCC_APB2ENR_USART1EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_USART1SEL,
        .rccTypeRegisterPos  = RCC_CCIPR_USART1SEL_Pos,

        .rxPins              =
        {
                               UART_PINS_PA10,
                               UART_PINS_PB7,
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA10,
                               GPIO_PINS_PB7,
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_0,
        },

        .txPins              =
        {
                               UART_PINS_PA9,
                               UART_PINS_PB6,
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA9,
                               GPIO_PINS_PB6,
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_0,
        },

        .isrNumber           = INTERRUPT_USART1,

        .state               = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART1 = &uart1;

static Uart_Device uart2 = {
        .regmap              = USART2,

        .rccRegisterPtr      = &RCC->APB1ENR,
        .rccRegisterEnable   = RCC_APB1ENR_USART2EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_USART2SEL,
        .rccTypeRegisterPos  = RCC_CCIPR_USART2SEL_Pos,

        .rxPins              =
        {
                               UART_PINS_PA3,
                               UART_PINS_PA15,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PD6,
#endif
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA3,
                               GPIO_PINS_PA15,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PD6,
#endif
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .txPins              =
        {
                               UART_PINS_PA2,
                               UART_PINS_PA14,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PD5,
#endif
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA2,
                               GPIO_PINS_PA14,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PD5,
#endif
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .isrNumber           = INTERRUPT_USART2,

        .state               = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART2 = &uart2;

static Uart_Device uart4 = {
        .regmap              = USART4,

        .rccRegisterPtr      = &RCC->APB1ENR,
        .rccRegisterEnable   = RCC_APB1ENR_USART4EN,

        .rccTypeRegisterPtr  = 0,
        .rccTypeRegisterMask = 0,
        .rccTypeRegisterPos  = 0,

        .rxPins              =
        {
                               UART_PINS_PA1,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PC11,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PE9,
#endif
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA1,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PC11,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PE9,
#endif
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_6,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_6,
#endif
        },

        .txPins              =
        {
                               UART_PINS_PA0,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PC10,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PE8,
#endif
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA0,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PC10,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PE8,
#endif
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_6,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_6,
#endif
        },

        .isrNumber           = INTERRUPT_USART4_5,

        .state               = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART4 = &uart4;

static Uart_Device uart5 = {
        .regmap              = USART5,

        .rccRegisterPtr      = &RCC->APB1ENR,
        .rccRegisterEnable   = RCC_APB1ENR_USART5EN,

        .rccTypeRegisterPtr  = 0,
        .rccTypeRegisterMask = 0,
        .rccTypeRegisterPos  = 0,

        .rxPins              =
        {
                               UART_PINS_PB4,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PD2,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PE11,
#endif
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PB4,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PD2,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PE11,
#endif
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_6,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_6,
#endif
        },

        .txPins              =
        {
                               UART_PINS_PB3,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PC12,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PE10,
#endif
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PB3,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PC12,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PE10,
#endif
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_6,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_2,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_6,
#endif
        },

        .isrNumber           = INTERRUPT_USART4_5,

        .state               = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART5 = &uart5;

static Uart_Device lpuart1 = {
        .regmap              = LPUART1,

        .rccRegisterPtr      = &RCC->APB1ENR,
        .rccRegisterEnable   = RCC_APB1ENR_LPUART1EN,

        .rccTypeRegisterPtr  = &RCC->CCIPR,
        .rccTypeRegisterMask = RCC_CCIPR_LPUART1SEL,
        .rccTypeRegisterPos  = RCC_CCIPR_LPUART1SEL_Pos,

        .rxPins              =
        {
                               UART_PINS_PA3,
                               UART_PINS_PA13,
                               UART_PINS_PB10_RX,
                               UART_PINS_PB11_RX,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PC0,
                               UART_PINS_PC5,
                               UART_PINS_PC11,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PD9,
#endif
        },
        .rxPinsGpio          =
        {
                               GPIO_PINS_PA3,
                               GPIO_PINS_PA13,
                               GPIO_PINS_PB10,
                               GPIO_PINS_PB11,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PC0,
                               GPIO_PINS_PC5,
                               GPIO_PINS_PC11,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PD9,
#endif
        },
        .rxPinsMux           =
        {
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_7,
                               GPIO_ALTERNATE_4,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_0,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_0,
#endif
        },

        .txPins              =
        {
                               UART_PINS_PA2,
                               UART_PINS_PA14,
                               UART_PINS_PB10_TX,
                               UART_PINS_PB11_TX,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PC1,
                               UART_PINS_PC4,
                               UART_PINS_PC10,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               UART_PINS_PD8,
#endif
        },
        .txPinsGpio          =
        {
                               GPIO_PINS_PA2,
                               GPIO_PINS_PA14,
                               GPIO_PINS_PB10,
                               GPIO_PINS_PB11,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PC1,
                               GPIO_PINS_PC4,
                               GPIO_PINS_PC10,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_PINS_PD8,
#endif
        },
        .txPinsMux           =
        {
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_4,
                               GPIO_ALTERNATE_7,
#if defined (LIBOHIBOARD_STM32L073RxT) || \
    defined (LIBOHIBOARD_STM32L073RxI) || \
    defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_6,
                               GPIO_ALTERNATE_2,
                               GPIO_ALTERNATE_0,
#endif
#if defined (LIBOHIBOARD_STM32L073VxT) || \
    defined (LIBOHIBOARD_STM32L073VxI)
                               GPIO_ALTERNATE_0,
#endif
        },
        .isrNumber           = INTERRUPT_LPUART1,

        .state               = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_LPUART1 = &lpuart1;

#endif

static inline __attribute__((always_inline)) void Uart_computeRxMask (Uart_DeviceHandle dev)
{
    switch (dev->config.dataBits)
    {
    case UART_DATABITS_SEVEN:
        if (dev->config.parity == UART_PARITY_NONE)
            dev->mask = 0x007Fu;
        else
            dev->mask = 0x003Fu;
        break;

    case UART_DATABITS_EIGHT:
        if (dev->config.parity == UART_PARITY_NONE)
            dev->mask = 0x00FFu;
        else
            dev->mask = 0x007Fu;
        break;

    case UART_DATABITS_NINE:
        if (dev->config.parity == UART_PARITY_NONE)
            dev->mask = 0x01FFu;
        else
            dev->mask = 0x00FFu;
        break;
    }
}

static System_Errors Uart_config (Uart_DeviceHandle dev, Uart_Config * config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check all parameters with asserts
    // The OR is to detect an error: it is not important where is, the important is that there is!
    err = ohiassert(UART_VALID_DATABITS(config->dataBits));
    err |= ohiassert(UART_VALID_BAUDRATE(config->baudrate));
    err |= ohiassert(UART_VALID_OVERSAMPLING(config->oversampling));
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

    // Save configurations
    dev->config = *config;

    // Disable the peripheral
    UART_DEVICE_DISABLE(dev->regmap);

    // Config peripheral
    // Configure data bits with the M bits
    dev->regmap->CR1 = dev->regmap->CR1 & (~(USART_CR1_M));
    switch (dev->config.dataBits)
    {
    case UART_DATABITS_SEVEN:
        dev->regmap->CR1 |= USART_CR1_M1;
        break;
    case UART_DATABITS_EIGHT:
        dev->regmap->CR1 |= 0x00U;
        break;
    case UART_DATABITS_NINE:
        dev->regmap->CR1 |= USART_CR1_M0;
        break;
    }

    // Configure parity type with PCE and PS bit
    dev->regmap->CR1 = dev->regmap->CR1 & (~(USART_CR1_PCE | USART_CR1_PS));
    switch (dev->config.parity)
    {
    case UART_PARITY_NONE:
        dev->regmap->CR1 |= 0x00U;
        break;
    case UART_PARITY_ODD:
        dev->regmap->CR1 |= USART_CR1_PCE | USART_CR1_PS;
        break;
    case UART_PARITY_EVEN:
        dev->regmap->CR1 |= USART_CR1_PCE;
        break;
    }

    // Configure peripheral mode with TE and RE bits into CR1
    dev->regmap->CR1 = dev->regmap->CR1 & (~(USART_CR1_TE | USART_CR1_RE));
    switch (dev->config.mode)
    {
    case UART_MODE_TRANSMIT:
        dev->regmap->CR1 |= USART_CR1_TE;
        break;
    case UART_MODE_RECEIVE:
        dev->regmap->CR1 |= USART_CR1_RE;
        break;
    case UART_MODE_BOTH:
        dev->regmap->CR1 |= USART_CR1_TE | USART_CR1_RE;
        break;
    }

    // Set oversampling: if value differ from 8, use default value.
    dev->regmap->CR1 = dev->regmap->CR1 & (~(USART_CR1_OVER8));
    if (dev->config.oversampling == 8)
    {
        dev->regmap->CR1 |= USART_CR1_OVER8;
    }

    // Configure stop bits with STOP[13:12] bits into CR2
    dev->regmap->CR2 = dev->regmap->CR2 & (~(USART_CR2_STOP));
    switch (dev->config.stop)
    {
    case UART_STOPBITS_HALF:
        dev->regmap->CR2 |= (USART_CR2_STOP & (0x01ul << USART_CR2_STOP_Pos));
        break;
    case UART_STOPBITS_ONE:
        dev->regmap->CR2 |= (USART_CR2_STOP & (0x00ul << USART_CR2_STOP_Pos));
        break;
    case UART_STOPBITS_ONE_AND_HALF:
        dev->regmap->CR2 |= (USART_CR2_STOP & (0x03ul << USART_CR2_STOP_Pos));
        break;
    case UART_STOPBITS_TWO:
        dev->regmap->CR2 |= (USART_CR2_STOP & (0x02ul << USART_CR2_STOP_Pos));
        break;
    }

    // Configure hardware flow control with CTS and RTS bits into CR3
    dev->regmap->CR3 = dev->regmap->CR3 & (~(USART_CR3_CTSE | USART_CR3_RTSE));
    switch (dev->config.flowControl)
    {
    case UART_FLOWCONTROL_NONE:
        dev->regmap->CR3 |= 0x0U;
        break;
    case UART_FLOWCONTROL_CTS:
        dev->regmap->CR3 |= USART_CR3_CTSE;
        break;
    case UART_FLOWCONTROL_RTS:
        dev->regmap->CR3 |= USART_CR3_RTSE;
        break;
    case UART_FLOWCONTROL_CTS_RTS:
        dev->regmap->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
        break;
    }

    // Compute RX mask
    Uart_computeRxMask(dev);

    // TODO: ONEBIT One sample bit method enable

    // Configure Baudrate
    err = Uart_setBaudrate(dev,dev->config.baudrate);
    if (err != ERRORS_NO_ERROR)
        return ERRORS_UART_WRONG_PARAM;

    // In asynchronous mode, the following bits must be kept cleared:
    // LINEN and CLKEN bits into CR2
    // SCEN, HDSEL and IREN  bits into CR3
    // TODO: in other configuration, these bit must be change!
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

    // Enable the peripheral
    UART_DEVICE_ENABLE(dev->regmap);

    return ERRORS_NO_ERROR;
}

System_Errors Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate)
{
    uint32_t frequency = 0;

    // TODO: This clock check into a define divided for microcontroller type?
    if (UART_IS_DUAL_CLOCK_DOMAIN(dev))
    {
        // Get current parent clock
        switch (dev->config.clockSource)
        {
        case UART_CLOCKSOURCE_HSI:
            frequency = (uint32_t)CLOCK_FREQ_HSI;
            break;
        case UART_CLOCKSOURCE_LSE:
            frequency = (uint32_t)CLOCK_FREQ_LSE;
            break;
        case UART_CLOCKSOURCE_SYSCLK:
            frequency = Clock_getOutputValue(CLOCK_OUTPUT_SYSCLK);
            break;
        case UART_CLOCKSOURCE_PCLK:
            if (dev == OB_UART1)
                frequency = Clock_getOutputValue(CLOCK_OUTPUT_PCLK2);
            else
                frequency = Clock_getOutputValue(CLOCK_OUTPUT_PCLK1);
            break;
        default:
            ohiassert(0);
            return ERRORS_UART_NO_CLOCKSOURCE;
        }
    }
    else
    {
        frequency = Clock_getOutputValue(CLOCK_OUTPUT_PCLK1);
    }

    // Current clock is different from 0
    if (frequency != 0u)
    {
        if (UART_IS_LOWPOWER_DEVICE(dev))
        {
            // Check the range of current clock
            // The range is [3 * baudrate, 4096 * baudrate]
            if (((frequency < ( 3u * baudrate))) && (frequency > ( 4096u * baudrate)))
            {
                return ERRORS_UART_WRONG_BAUDRATE;
            }
            else
            {
                uint32_t uartdiv = (uint32_t)(((uint64_t)frequency * 256u) / baudrate);
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
        else if (dev->config.oversampling == 8)
        {
            uint32_t uartdiv = (frequency * 2u) / baudrate;
            if ((uartdiv >= UART_BRR_MIN) && (uartdiv <= UART_BRR_MAX))
            {
                // Fix BRR value
                // see page 771 of RM0367 (Rev6)
                uint32_t brr = uartdiv & 0xFFF0u;
                brr |= (uint16_t)((uartdiv & (uint16_t)0x000Fu) >> 1u);
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
    else
    {
        return ERRORS_UART_CLOCKSOURCE_FREQUENCY_TOO_LOW;
    }

    return ERRORS_NO_ERROR;
}

System_Errors Uart_open (Uart_DeviceHandle dev, Uart_Config *config)
{
    return ohiassert(0);
}

System_Errors Uart_init (Uart_DeviceHandle dev, Uart_Config *config)
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
    err = ohiassert(UART_IS_VALID_CLOCK_SOURCE(config->clockSource));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_UART_WRONG_PARAM;
    }

    // Enable peripheral clock if needed
    if (dev->state == UART_DEVICESTATE_RESET)
    {
        // Select clock source (Only for some peripheral)
        if (UART_IS_DUAL_CLOCK_DOMAIN(dev))
            UTILITY_MODIFY_REGISTER(*dev->rccTypeRegisterPtr,dev->rccTypeRegisterMask,(config->clockSource << dev->rccTypeRegisterPos));

        // Enable peripheral clock
        UART_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

        // Enable pins
        if (config->rxPin != UART_PINS_RXNONE)
            Uart_setRxPin(dev, config->rxPin);

        if (config->txPin != UART_PINS_TXNONE)
            Uart_setTxPin(dev, config->txPin);
    }
    dev->state = UART_DEVICESTATE_BUSY;

    // Configure the peripheral
    err = Uart_config(dev,config);
    if (err != ERRORS_NO_ERROR)
    {
        return err;
    }

    // Configure interrupt
    if (config->callbackRx)
    {
        dev->callbackRx = config->callbackRx;
        if (config->callbackError)
        {
            dev->callbackError = config->callbackError;
        }
        // Enable NVIC interrupt
        Interrupt_enable(dev->isrNumber);
        // Enable the UART Error Interrupt
        UTILITY_SET_REGISTER_BIT(dev->regmap->CR3, USART_CR3_EIE);
        // Enable UART Data Register Not Empty interrupt
        UTILITY_SET_REGISTER_BIT(dev->regmap->CR1, USART_CR1_RXNEIE);
        // FIXME: Enable UART Parity Error, only when parity was request!
    }

    if (config->callbackTx)
    {
        dev->callbackTx = config->callbackTx;
        if (config->callbackError)
        {
            dev->callbackError = config->callbackError;
        }
        // Enable NVIC interrupt
        Interrupt_enable(dev->isrNumber);
        UTILITY_SET_REGISTER_BIT(dev->regmap->CR1,USART_CR1_TXEIE);
    }

    if (dev->config.callbackObj != NULL)
    {
        dev->callbackObj = dev->config.callbackObj;
    }
    else
    {
        dev->callbackObj = NULL;
    }

    dev->state = UART_DEVICESTATE_READY;

    return ERRORS_NO_ERROR;
}

System_Errors Uart_close (Uart_DeviceHandle dev)
{
    return ohiassert(0);
}

System_Errors Uart_deInit (Uart_DeviceHandle dev)
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

    // The device is busy...
    dev->state = UART_DEVICESTATE_BUSY;
    // Disable the peripheral
    UART_DEVICE_DISABLE(dev->regmap);

    // Clear all registers
    dev->regmap->CR1 = 0x0u;
    dev->regmap->CR2 = 0x0u;
    dev->regmap->CR3 = 0x0u;

    // Delete interrrupt callback
    dev->callbackRx = NULL;
    dev->callbackTx = NULL;

    // Disable peripheral clock
    UART_CLOCK_DISABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

    // Set reset state
    dev->state = UART_DEVICESTATE_RESET;

    return err;
}

bool Uart_isEnabled(Uart_DeviceHandle dev)
{
    return(UART_DEVICE_IS_ENABLED(dev->regmap) != 0)?(true):(false);
}

System_Errors Uart_resume(Uart_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the UART device
    if (dev == NULL)
    {
        return ERRORS_UART_NO_DEVICE;
    }
    // Check the UART instance
    if (ohiassert(UART_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_UART_WRONG_DEVICE;
    }
    UART_DEVICE_ENABLE(dev->regmap);

    return err;
}

System_Errors Uart_suspend(Uart_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the UART device
    if (dev == NULL)
    {
        return ERRORS_UART_NO_DEVICE;
    }
    // Check the UART instance
    if (ohiassert(UART_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_UART_WRONG_DEVICE;
    }
    UART_DEVICE_DISABLE(dev->regmap);

    return err;
}

System_Errors Uart_setRxPin (Uart_DeviceHandle dev, Uart_RxPins rxPin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < UART_MAX_PINS; ++devPinIndex)
    {
        if (dev->rxPins[devPinIndex] == rxPin)
        {
            Gpio_configAlternate(dev->rxPinsGpio[devPinIndex],
                                 dev->rxPinsMux[devPinIndex],
                                 0);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_UART_NO_PIN_FOUND;
}

System_Errors Uart_setTxPin (Uart_DeviceHandle dev, Uart_TxPins txPin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < UART_MAX_PINS; ++devPinIndex)
    {
        if (dev->txPins[devPinIndex] == txPin)
        {
            Gpio_configAlternate(dev->txPinsGpio[devPinIndex],
                                 dev->txPinsMux[devPinIndex],
                                 0);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_UART_NO_PIN_FOUND;
}

System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out)
{
    // deprecated
    return ohiassert(0);
}

void Uart_putChar (Uart_DeviceHandle dev, char c)
{
    // deprecated
    ohiassert(0);
}

uint8_t Uart_isCharPresent (Uart_DeviceHandle dev)
{
    // deprecated
    ohiassert(0);
    return 0;
}

uint8_t Uart_isTransmissionComplete (Uart_DeviceHandle dev)
{
    // deprecated
    ohiassert(0);
    return 0;
}

System_Errors Uart_read (Uart_DeviceHandle dev, uint8_t* data, uint32_t timeout)
{
    uint16_t* temp;

    uint32_t timeoutEnd = System_currentTick() + timeout;

    if (dev->state == UART_DEVICESTATE_READY)
    {
        dev->state = UART_DEVICESTATE_BUSY;
        while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,USART_ISR_RXNE) == 0)
        {
            if (System_currentTick() > timeoutEnd)
            {
                return ERRORS_UART_TIMEOUT_RX;
            }
        }

        // In case of 9B and parity NONE, the message is split into two byte and aligned!
        if ((dev->config.dataBits == UART_DATABITS_NINE) &&
            (dev->config.parity == UART_PARITY_NONE))
        {
            // Cast the pointer
            temp = (uint16_t *) data;
            *temp = (uint16_t) (dev->regmap->RDR & dev->mask);
        }
        else
        {
            *data = (uint8_t)(dev->regmap->RDR & (uint8_t)dev->mask);
        }
    }
    else
    {
        return ERRORS_UART_DEVICE_BUSY;
    }
    dev->state = UART_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Uart_write (Uart_DeviceHandle dev, const uint8_t* data, uint32_t timeout)
{
    uint16_t* temp = 0;

    uint32_t timeoutEnd = System_currentTick() + timeout;

    if (dev->state == UART_DEVICESTATE_READY)
    {
        // Wait until the buffer is empty
        while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,USART_ISR_TXE) == 0)
        {
            if (System_currentTick() > timeoutEnd)
            {
                return ERRORS_UART_TIMEOUT_TX;
            }
        }

        // In case of 9B and parity NONE, the message is split into two byte
        if ((dev->config.dataBits == UART_DATABITS_NINE) &&
            (dev->config.parity == UART_PARITY_NONE))
        {
            // Cast the pointer
            temp = (uint16_t *) data;
            dev->regmap->TDR = (*temp & 0x01FFu);
        }
        else
        {
            dev->regmap->TDR = (*data & 0x00FFu);
        }
        // Start-up new timeout
        timeoutEnd = System_currentTick() + timeout;
        // Wait until the transmission is complete
        while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,USART_ISR_TC) == 0)
        {
            if (System_currentTick() > timeoutEnd)
            {
                return ERRORS_UART_TIMEOUT_TX;
            }
        }
    }
    else
    {
        return ERRORS_UART_DEVICE_BUSY;
    }
    dev->state = UART_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

bool Uart_isPresent (Uart_DeviceHandle dev)
{
    return (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,USART_ISR_RXNE) == 0) ? FALSE : TRUE;
}

void Uart_setCallbackObject (Uart_DeviceHandle dev, void* obj)
{
    ohiassert(obj != NULL);

    if (obj != NULL)
    {
        dev->callbackObj = obj;
    }
}

static inline void __attribute__((always_inline)) Uart_callbackInterrupt (Uart_DeviceHandle dev)
{
    uint32_t isrreg = dev->regmap->ISR;
    uint32_t cr1reg = dev->regmap->CR1;

    // Check errors:
    //  - Parity Error
    //  - Framing Error
    //  - START bit Noise detection
    //  - Overrun Error
    uint32_t errorFlag = isrreg & (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
    // No Errors...
    if (errorFlag == 0)
    {
        // Check if the interrupt is in reception
        if (((isrreg & USART_ISR_RXNE) != 0) && ((cr1reg & USART_CR1_RXNEIE) > 0))
        {
            if (dev->callbackRx != NULL)
            {
                dev->callbackRx(dev,dev->callbackObj);
                // Clear flag, just to increase the safety
                UTILITY_SET_REGISTER_BIT(dev->regmap->RQR,USART_RQR_RXFRQ);
            }
            return;
        }
    }
    else
    {
        if (dev->callbackError != NULL)
        {
            dev->callbackError(dev,dev->callbackObj);
            // Clear ORE flag
            UTILITY_SET_REGISTER_BIT(dev->regmap->ICR,USART_ICR_ORECF);
        }
    }

    // Check if the interrupt is in transmission
    if (((isrreg & USART_ISR_TXE) != 0) && ((cr1reg & USART_CR1_TXEIE) > 0))
    {
        if (dev->callbackTx != NULL)
        {
            dev->callbackTx(dev,dev->callbackObj);
        }
    }
}

void LPUART1_IRQHandler(void)
{
    Uart_callbackInterrupt(OB_LPUART1);
}

void USART1_IRQHandler(void)
{
    Uart_callbackInterrupt(OB_UART1);
}

void USART2_IRQHandler(void)
{
    Uart_callbackInterrupt(OB_UART2);
}

#if defined (LIBOHIBOARD_STM32L0x2)
void USART4_5_IRQHandler(void)
#elif defined (LIBOHIBOARD_STM32L0x3)
void USART4_USART5_IRQHandler (void)
#endif
{
    if(Uart_isEnabled(OB_UART4))
    {
        Uart_callbackInterrupt(OB_UART4);
    }
    if(Uart_isEnabled(OB_UART5))
    {
        Uart_callbackInterrupt(OB_UART5);
    }
}

#endif // LIBOHIBOARD_STM32L0

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_UART
