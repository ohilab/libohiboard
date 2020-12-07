/*
 * Copyright (C) 2012-2020 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/uart_MKL.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief UART implementations for MKL series.
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

#if defined (LIBOHIBOARD_MKL)

#define UART_DEVICE_ENABLE(REGMAP)        (REGMAP->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK))
#define UART_DEVICE_ENABLE0(REGMAP)       (REGMAP->C2 |= (UART0_C2_TE_MASK | UART0_C2_RE_MASK))

#define UART_DEVICE_DISABLE(REGMAP)       (REGMAP->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK))
#define UART_DEVICE_DISABLE0(REGMAP)      (REGMAP->C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK))

#define UART_DEVICE_IS_ENABLED(REGMAP)    ((REGMAP->C1 & UART_C2_TE_MASK) || (REGMAP->C1 & UART_C2_RE_MASK))
#define UART_DEVICE_IS_ENABLED0(REGMAP)   ((REGMAP->C1 & UART0_C2_TE_MASK) || (REGMAP->C1 & UART0_C2_RE_MASK))

#define UART_MAX_PINS                     10

#if !defined(UART_MAX_CALLBACK_NUMBER)
#define UART_MAX_CALLBACK_NUMBER          5
#endif

typedef struct _Uart_Device
{
    UART_Type* regMap;                             /**< Device memory pointer */
    UART0_Type* regMap0;                 /**< Device memory pointer for UART0 */

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Uart_RxPins rxPins[UART_MAX_PINS];
    Uart_TxPins txPins[UART_MAX_PINS];
    Uart_CtsPins ctsPins[UART_MAX_PINS];
    Uart_RtsPins rtsPins[UART_MAX_PINS];

    Gpio_Pins rxPinsGpio[UART_MAX_PINS];
    Gpio_Pins txPinsGpio[UART_MAX_PINS];

    volatile uint32_t* rxPinsPtr[UART_MAX_PINS];
    volatile uint32_t* txPinsPtr[UART_MAX_PINS];
    volatile uint32_t* ctsPinsPtr[UART_MAX_PINS];
    volatile uint32_t* rtsPinsPtr[UART_MAX_PINS];
    uint8_t rxPinsMux[UART_MAX_PINS];
    uint8_t txPinsMux[UART_MAX_PINS];
    uint8_t ctsPinsMux[UART_MAX_PINS];
    uint8_t rtsPinsMux[UART_MAX_PINS];

    Uart_Config config;

    //void (*isr)(void);                     /**< The function pointer for ISR. */
    //void (*callbackRx)(void); /**< The function pointer for user Rx callback. */
    //void (*callbackTx)(void); /**< The function pointer for user Tx callback. */

    /** The array of function pointers for user Rx callback. */
    void (*callbackRx[UART_MAX_CALLBACK_NUMBER])(struct _Uart_Device* dev, void* obj);
    /** The array of function pointers for user Tx callback. */
    void (*callbackTx[UART_MAX_CALLBACK_NUMBER])(struct _Uart_Device* dev, void* obj);
    /** The array of function pointers to handle Error Interrupt. */
    void (*callbackError[UART_MAX_CALLBACK_NUMBER])(struct _Uart_Device* dev, void* obj);
    /** Useful array of object added to callback when interrupt triggered. */
    void* callbackObj[UART_MAX_CALLBACK_NUMBER];

    bool isRxInterruptEnabled;
    bool isTxInterruptEnabled;
    bool isErrorInterruptEnabled;

    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Uart_DeviceState state;

} Uart_Device;

#define UART_IS_DEVICE(DEVICE) (((DEVICE) == OB_UART1)  || \
                                ((DEVICE) == OB_UART2))

#define UART_IS_LOWPOWER_DEVICE(DEVICE) ((DEVICE) == OB_UART0)

#define UART_IS_VALID_CLOCK_SOURCE(CLOCKSOURCE) (((CLOCKSOURCE) == UART_CLOCKSOURCE_BUS)    || \
                                                 ((CLOCKSOURCE) == UART_CLOCKSOURCE_SYSTEM))

/**
  * @brief Check that number of stop bits is valid for UART.
  * @param STOPBITS Number of stop bits.
  * @retval TRUE if is valid, FALSE otherwise
  *
  * @hideinitializer
  */
#define UART_VALID_STOPBITS(STOPBITS) (((STOPBITS) == UART_STOPBITS_ONE))

/**
  * @brief Check that UART parity is valid.
  * @param PARITY UART parity type.
  * @retval TRUE if is valid, FALSE otherwise
  */
#define UART_VALID_PARITY(PARITY) (((PARITY) == UART_PARITY_NONE) || \
                                   ((PARITY) == UART_PARITY_EVEN) || \
                                   ((PARITY) == UART_PARITY_ODD))

#define UART_VALID_DATABITS(DATABITS) (((DATABITS) == UART_DATABITS_TEN) || \
                                       ((DATABITS) == UART_DATABITS_EIGHT) || \
                                       ((DATABITS) == UART_DATABITS_NINE))

// FIXME: Implement flow control!
#if 0
#define UART_VALID_FLOWCONTROL(FLOWCONTROL) (((FLOWCONTROL) == UART_FLOWCONTROL_NONE) || \
                                             ((FLOWCONTROL) == UART_FLOWCONTROL_CTS)  || \
                                             ((FLOWCONTROL) == UART_FLOWCONTROL_RTS)  || \
                                             ((FLOWCONTROL) == UART_FLOWCONTROL_CTS_RTS))
#endif
#define UART_VALID_FLOWCONTROL(FLOWCONTROL) (((FLOWCONTROL) == UART_FLOWCONTROL_NONE))

static Uart_Device uart0 =
{
        .regMap0          = UART0,
        .regMap           = 0,

        .simScgcPtr       = &SIM->SCGC4,
        .simScgcBitEnable = SIM_SCGC4_UART0_MASK,

        .rxPins           =
        {
                             UART_PINS_PTA1,
#if defined (LIBOHIBOARD_MKL15ZxLK)
                             UART_PINS_PTA15,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             UART_PINS_PTB16,
#endif
                             UART_PINS_PTD6,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             UART_PINS_PTE21,
#endif
        },
        .rxPinsGpio       =
        {
                             GPIO_PINS_PTA1,
#if defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTA15,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             GPIO_PINS_PTB16,
#endif
                             GPIO_PINS_PTD6,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             GPIO_PINS_PTE21,
#endif
        },
        .rxPinsPtr        =
        {
                             &PORTA->PCR[1],
#if defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTA->PCR[15],
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             &PORTB->PCR[16],
#endif
                             &PORTD->PCR[6],
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             &PORTE->PCR[21],
#endif
        },
        .rxPinsMux        =
        {
                             2,
#if defined (LIBOHIBOARD_MKL15ZxLK)
                             3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             3,
#endif
                             3,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             4,
#endif
        },

        .txPins           =
        {
                             UART_PINS_PTA2,
#if defined (LIBOHIBOARD_MKL15ZxLK)
                             UART_PINS_PTA14,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             UART_PINS_PTB17,
#endif
                             UART_PINS_PTD7,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             UART_PINS_PTE20,
#endif
        },
        .txPinsGpio       =
        {
                             GPIO_PINS_PTA2,
#if defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTA14,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             GPIO_PINS_PTB17,
#endif
                             GPIO_PINS_PTD7,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             GPIO_PINS_PTE20,
#endif
        },
        .txPinsPtr        =
        {
                             &PORTA->PCR[2],
#if defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTA->PCR[14],
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             &PORTB->PCR[17],
#endif
                             &PORTD->PCR[7],
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             &PORTE->PCR[20],
#endif
        },
        .txPinsMux        =
        {
                             2,
#if defined (LIBOHIBOARD_MKL15ZxLK)
                             3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             3,
#endif
                             3,
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             4,
#endif
        },

        .isrNumber        = INTERRUPT_UART0,
        .callbackRx       = 0,
        .callbackTx       = 0,

        .state            = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART0 = &uart0;

static Uart_Device uart1 =
{
        .regMap0          = 0,
        .regMap           = UART1,

        .simScgcPtr       = &SIM->SCGC4,
        .simScgcBitEnable = SIM_SCGC4_UART1_MASK,

        .rxPins           =
        {
                             UART_PINS_PTA18,
                             UART_PINS_PTC3,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             UART_PINS_PTE1,
#endif
        },
        .rxPinsGpio       =
        {
                             GPIO_PINS_PTA18,
                             GPIO_PINS_PTC3,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             GPIO_PINS_PTE1,
#endif
        },
        .rxPinsPtr        =
        {
                             &PORTA->PCR[18],
                             &PORTC->PCR[3],
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             &PORTE->PCR[1],
#endif
        },
        .rxPinsMux        =
        {
                             3,
                             3,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             3,
#endif
        },

        .txPins           =
        {
                             UART_PINS_PTA19,
                             UART_PINS_PTC4,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
	defined (LIBOHIBOARD_MKL25ZxFM) || \
	defined (LIBOHIBOARD_MKL25ZxLH) || \
	defined (LIBOHIBOARD_MKL25ZxLK)
                             UART_PINS_PTE0,
#endif
        },
        .txPinsGpio       =
        {
                             GPIO_PINS_PTA19,
                             GPIO_PINS_PTC4,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFM) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             GPIO_PINS_PTE0,
#endif
        },
        .txPinsPtr        =
        {
                             &PORTA->PCR[19],
                             &PORTC->PCR[4],
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
	defined (LIBOHIBOARD_MKL25ZxFM) || \
	defined (LIBOHIBOARD_MKL25ZxLH) || \
	defined (LIBOHIBOARD_MKL25ZxLK)
                             &PORTE->PCR[0],
#endif
        },
        .txPinsMux        =
        {
                             3,
                             3,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
	defined (LIBOHIBOARD_MKL25ZxFM) || \
	defined (LIBOHIBOARD_MKL25ZxLH) || \
	defined (LIBOHIBOARD_MKL25ZxLK)
                             3,
#endif
        },

        .isrNumber        = INTERRUPT_UART1,
        .callbackRx       = 0,
        .callbackTx       = 0,

        .state            = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART1 = &uart1;

static Uart_Device uart2 =
{
        .regMap0          = 0,
        .regMap           = UART2,

        .simScgcPtr       = &SIM->SCGC4,
        .simScgcBitEnable = SIM_SCGC4_UART2_MASK,

        .rxPins           =
        {
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             UART_PINS_PTD2,
#endif
                             UART_PINS_PTD4,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             UART_PINS_PTE17,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             UART_PINS_PTE23,
#endif
        },
        .rxPinsGpio       =
        {
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             GPIO_PINS_PTD2,
#endif
                             GPIO_PINS_PTD4,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTE17,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             GPIO_PINS_PTE23,
#endif
        },
        .rxPinsPtr        =
        {
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             &PORTD->PCR[2],
#endif
                             &PORTD->PCR[4],
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTE->PCR[17],
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             &PORTE->PCR[23],
#endif
        },
        .rxPinsMux        =
        {
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             3,
#endif
                             3,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             4,
#endif
        },

        .txPins           =
        {
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             UART_PINS_PTD3,
#endif
                             UART_PINS_PTD5,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             UART_PINS_PTE16,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             UART_PINS_PTE22,
#endif
        },
        .txPinsGpio       =
        {
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             GPIO_PINS_PTD3,
#endif
                             GPIO_PINS_PTD5,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             GPIO_PINS_PTE16,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             GPIO_PINS_PTE22,
#endif
        },
        .txPinsPtr        =
        {
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             &PORTD->PCR[3],
#endif
                             &PORTD->PCR[5],
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             &PORTE->PCR[16],
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             &PORTE->PCR[22],
#endif
        },
        .txPinsMux        =
        {
#if defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxFT) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             3,
#endif
                             3,
#if defined (LIBOHIBOARD_MKL15ZxFM) || \
    defined (LIBOHIBOARD_MKL15ZxFT) || \
    defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK)
                             3,
#endif
#if defined (LIBOHIBOARD_MKL15ZxLH) || \
    defined (LIBOHIBOARD_MKL15ZxLK) || \
    defined (LIBOHIBOARD_MKL25ZxLH) || \
    defined (LIBOHIBOARD_MKL25ZxLK)
                             4,
#endif
        },

        .isrNumber        = INTERRUPT_UART2,
        .callbackRx       = 0,
        .callbackTx       = 0,

        .state            = UART_DEVICESTATE_RESET,
};
Uart_DeviceHandle OB_UART2 = &uart2;

System_Errors Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate)
{
    register uint16_t sbr;
    uint32_t temp;
    uint32_t clockHz;
    uint8_t osr;

    uint8_t myoversampling = dev->config.oversampling;

    if (myoversampling < 4)
        myoversampling = 16;

    osr = myoversampling - 1;


    // Save off the current value of the UARTx_BDH except for the SBR field
    if (dev == OB_UART0)
    {
        SIM->SOPT2 &= ~(SIM_SOPT2_UART0SRC_MASK);
        SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);

        switch(Clock_getCurrentState())
        {
        case CLOCK_STATE_FBE:
        case CLOCK_STATE_FBI:
        case CLOCK_STATE_FEI:
        case CLOCK_STATE_FEE:
            clockHz = Clock_getOutputValue(CLOCK_OUTPUT_SYSCLK);
            break;

        case CLOCK_STATE_PBE:
        case CLOCK_STATE_PEE:
            clockHz = Clock_getOutputValue(CLOCK_OUTPUT_SYSCLK)/2;
            break;
        default:
            ohiassert(0);
        }

        /* Calculate baud settings */
        sbr = (uint16_t)((clockHz)/(baudrate * myoversampling));

        temp = dev->regMap0->C4 & ~(UART0_C4_OSR_MASK);
        dev->regMap0->C4 = temp | UART0_C4_OSR(osr);

        temp = dev->regMap0->BDH & ~(UART0_BDH_SBR(0x1F));

        dev->regMap0->BDH = temp |  UART0_BDH_SBR(((sbr & 0x1F00) >> 8));
        dev->regMap0->BDL = (uint8_t)(sbr & UART0_BDL_SBR_MASK);
    }
    else
    {
        clockHz = Clock_getOutputValue(CLOCK_OUTPUT_BUS);

        /* Calculate baud settings */
        sbr = (uint16_t)((clockHz)/(baudrate * 16));


        temp = dev->regMap->BDH & ~(UART_BDH_SBR(0x1F));

        dev->regMap->BDH = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
        dev->regMap->BDL = (uint8_t)(sbr & UART_BDL_SBR_MASK);
    }

    return ERRORS_NO_ERROR;
}

System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out)
{
    // deprecated
    return ohiassert(0);
}

System_Errors Uart_read (Uart_DeviceHandle dev, uint8_t* data, uint32_t timeout)
{
    uint32_t timeoutEnd = System_currentTick() + timeout;

    if (dev->state == UART_DEVICESTATE_READY)
    {
        dev->state = UART_DEVICESTATE_BUSY;

        if (dev == OB_UART0)
        {
            // Wait until character has been received
            while (UTILITY_READ_REGISTER_BIT(dev->regMap0->S1,UART0_S1_TDRE_MASK) == 0)
            {
                if (System_currentTick() > timeoutEnd)
                {
                    return ERRORS_UART_TIMEOUT_RX;
                }
            }

            *data = (uint8_t)(dev->regMap0->D);
        }
        else
        {
            // Wait until character has been received
            while (UTILITY_READ_REGISTER_BIT(dev->regMap->S1,UART_S1_TDRE_MASK) == 0)
            {
                if (System_currentTick() > timeoutEnd)
                {
                    return ERRORS_UART_TIMEOUT_RX;
                }
            }

            *data = (uint8_t)(dev->regMap->D);
        }
    }
    else
    {
        return ERRORS_UART_DEVICE_BUSY;
    }
    dev->state = UART_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

void Uart_putChar (Uart_DeviceHandle dev, char c)
{
    // deprecated
    ohiassert(0);
}

System_Errors Uart_write (Uart_DeviceHandle dev, const uint8_t* data, uint32_t timeout)
{
    uint32_t timeoutEnd = System_currentTick() + timeout;

    if (dev->state == UART_DEVICESTATE_READY)
    {
        dev->state = UART_DEVICESTATE_BUSY;

        if (dev == OB_UART0)
        {
            // Wait until space is available in the FIFO
            while (UTILITY_READ_REGISTER_BIT(dev->regMap0->S1,UART0_S1_TDRE_MASK) == 0)
            {
                if (System_currentTick() > timeoutEnd)
                {
                    return ERRORS_UART_TIMEOUT_TX;
                }
            }
            // Send the character
            dev->regMap0->D = *data;
        }
        else
        {
            // Wait until space is available in the FIFO
            while (UTILITY_READ_REGISTER_BIT(dev->regMap->S1,UART_S1_TDRE_MASK) == 0)
            {
                if (System_currentTick() > timeoutEnd)
                {
                    return ERRORS_UART_TIMEOUT_TX;
                }
            }
            // Send the character
            dev->regMap->D = *data;
        }
    }
    else
    {
        return ERRORS_UART_DEVICE_BUSY;
    }
    dev->state = UART_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
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

System_Errors Uart_open (Uart_DeviceHandle dev, Uart_Config *config)
{
    return ohiassert(0);
}

static System_Errors Uart_config (Uart_DeviceHandle dev, Uart_Config * config)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Check all parameters with asserts
    // The OR is to detect an error: it is not important where is, the important is that there is!
    err = ohiassert(UART_VALID_DATABITS(config->dataBits));
    //err |= ohiassert(UART_VALID_BAUDRATE(config->baudrate));
    //err |= ohiassert(UART_VALID_OVERSAMPLING(config->oversampling));
    err |= ohiassert(UART_VALID_STOPBITS(config->stop));
    err |= ohiassert(UART_VALID_PARITY(config->parity));
    err |= ohiassert(UART_VALID_FLOWCONTROL(config->flowControl));
    //err |= ohiassert(UART_VALID_MODE(config->mode));

    if (err != ERRORS_NO_ERROR)
        return ERRORS_UART_WRONG_PARAM;

    // Save configurations
    dev->config = *config;

    // Make sure that the transmitter and receiver are disabled while we change settings.
    if (dev == OB_UART0)
    {
        UART_DEVICE_DISABLE0(dev->regMap0);
    }
    else
    {
        UART_DEVICE_DISABLE(dev->regMap);
    }


    // Set data bits
    if (dev == OB_UART0)
    {
        switch (dev->config.dataBits)
        {
        case UART_DATABITS_EIGHT:
            dev->regMap0->C1 &= ~(UART_C1_M_MASK);
            break;
        case UART_DATABITS_NINE:
            break;
        case UART_DATABITS_TEN:
            break;
        }
    }
    else
    {
        switch (config->dataBits)
        {
        case UART_DATABITS_EIGHT:
            dev->regMap->C1 &= ~(UART_C1_M_MASK);
            break;
        case UART_DATABITS_NINE:
            break;
        case UART_DATABITS_TEN:
            break;
        }
    }

    // Set parity type
    if (dev == OB_UART0)
    {
        switch (dev->config.parity)
        {
        case UART_PARITY_NONE:
            dev->regMap0->C1 &= ~(UART0_C1_PE_MASK | UART0_C1_PT_MASK);
            break;
        case UART_PARITY_ODD:
            dev->regMap0->C1 |= UART0_C1_PE_MASK | UART0_C1_PT_MASK | 0;
            break;
        case UART_PARITY_EVEN:
            dev->regMap0->C1 |= UART0_C1_PE_MASK | 0;
            break;
        }
    }
    else
    {
        switch (config->parity)
        {
        case UART_PARITY_NONE:
            dev->regMap->C1 &= ~(UART_C1_PE_MASK | UART_C1_PT_MASK);
            break;
        case UART_PARITY_ODD:
            dev->regMap->C1 |= UART_C1_PE_MASK | UART_C1_PT_MASK | 0;
            break;
        case UART_PARITY_EVEN:
            dev->regMap->C1 |= UART_C1_PE_MASK | 0;
            break;
        }
    }

    // FIXME: add flow control!
    // FIXME: Configure the UART just for 8-bit mode, fix with 9 and 10

    Uart_setBaudrate(dev,config->baudrate);

    // Enable receiver and transmitter
    if (dev == OB_UART0)
    {
        UART_DEVICE_ENABLE0(dev->regMap0);
    }
    else
    {
        UART_DEVICE_ENABLE(dev->regMap);
    }

    return err;
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
        // Enable peripheral clock
        *dev->simScgcPtr |= dev->simScgcBitEnable;

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
        dev->callbackRx[0] = config->callbackRx;
        dev->isRxInterruptEnabled = TRUE;
        if (config->callbackError)
        {
            dev->callbackError[0] = config->callbackError;
            dev->isErrorInterruptEnabled = TRUE;
        }
        // Enable NVIC interrupt
        Interrupt_enable(dev->isrNumber);
        if (dev == OB_UART0)
        {
            dev->regMap0->C2 |= UART0_C2_RIE_MASK;
        }
        else
        {
            dev->regMap->C2 |= UART_C2_RIE_MASK;
        }
    }

    if (config->callbackTx)
    {
        dev->callbackTx[0] = config->callbackTx;
        dev->isTxInterruptEnabled = TRUE;
        if (config->callbackError)
        {
            dev->callbackError[0] = config->callbackError;
            dev->isErrorInterruptEnabled = TRUE;
        }
        // Enable NVIC interrupt
        Interrupt_enable(dev->isrNumber);
        if (dev == OB_UART0)
        {
            dev->regMap0->C2 |= UART0_C2_TIE_MASK;
        }
        else
        {
            dev->regMap->C2 |= UART_C2_TIE_MASK;
        }
    }

    if (dev->config.callbackObj != NULL)
    {
        dev->callbackObj[0] = dev->config.callbackObj;
    }
    else
    {
        dev->callbackObj[0] = NULL;
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

    // Whether is not initialized, return!
    if (dev->state == UART_DEVICESTATE_RESET)
    {
        return ERRORS_NO_ERROR;
    }

    // The device is busy...
    dev->state = UART_DEVICESTATE_BUSY;

    // Disable the peripheral
    if (dev == OB_UART0)
    {
        UART_DEVICE_DISABLE0(dev->regMap0);
    }
    else
    {
        UART_DEVICE_DISABLE(dev->regMap);
    }

    // Delete interrupt callback
    for (uint8_t i = 0; i < UART_MAX_CALLBACK_NUMBER; ++i)
    {
        dev->callbackRx[i] = NULL;
        dev->callbackTx[i] = NULL;
        dev->callbackError[i] = NULL;
        dev->callbackObj[i] = 0;
    }

    // Disable peripheral clock
    *dev->simScgcPtr &= ~dev->simScgcBitEnable;

    // Set reset state
    dev->state = UART_DEVICESTATE_RESET;

    return err;
}

bool Uart_isEnabled (Uart_DeviceHandle dev)
{
    if (dev == OB_UART0)
    {
        return (UART_DEVICE_IS_ENABLED0(dev->regMap0) != 0) ? (true) : (false);
    }
    else
    {
        return (UART_DEVICE_IS_ENABLED(dev->regMap) != 0) ? (true) : (false);
    }
}

System_Errors Uart_resume(Uart_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;

    // TODO

    return err;
}

System_Errors Uart_suspend(Uart_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;

    // TODO

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

static inline void __attribute__((always_inline)) Uart_callbackInterrupt (Uart_DeviceHandle dev)
{
    uint8_t s1reg = 0;
    if (dev == OB_UART0)
    {
        s1reg = dev->regMap0->S1;
    }
    else
    {
        s1reg = dev->regMap->S1;
    }

    if ((s1reg & (UART_S1_OR_MASK | UART_S1_PF_MASK | UART_S1_FE_MASK)) != 0)
    {
        if (dev == OB_UART0)
        {
            // Clear all flags
            dev->regMap0->S1 |= (UART_S1_OR_MASK | UART_S1_PF_MASK | UART_S1_FE_MASK);
        }
        else
        {
            // Clear all flags by reading the data
            (void)dev->regMap->D;
        }
        // TODO: Call callback error?
        return;
    }

    // Check if the interrupt is in reception
    if ((s1reg & UART_S1_RDRF_MASK) != 0)
    {
        if (dev->callbackRx[0] != NULL)
        {
            for (uint8_t i = 0; i < UART_MAX_CALLBACK_NUMBER; ++i)
            {

                if (dev->callbackRx[i] != NULL)
                {
                    dev->callbackRx[i](dev,dev->callbackObj[i]);
                }
            }
            // FIXME: flag read again?
        }
        return;
    }

    // Check if the interrupt is in transmission
    if ((s1reg & UART_S1_TDRE_MASK) != 0)
    {
        if (dev->callbackTx[0] != NULL)
        {
            for (uint8_t i = 0; i < UART_MAX_CALLBACK_NUMBER; ++i)
            {

                if (dev->callbackTx[i] != NULL)
                {
                    dev->callbackTx[i](dev,dev->callbackObj[i]);
                }
            }
            // FIXME: flag read again?
        }
        return;
    }
}

void Uart_setCallbackObject (Uart_DeviceHandle dev, void* obj)
{
    ohiassert(obj != NULL);

    if (obj != NULL)
    {
        dev->callbackObj[0] = obj;
    }
}

void Uart_addRxCallback (Uart_DeviceHandle dev, Uart_callback callback)
{
    ohiassert(callback != NULL);

    if (callback != NULL)
    {
        dev->callbackRx[0] = callback;
        dev->isRxInterruptEnabled = TRUE;
        // Enable NVIC interrupt
        Interrupt_enable(dev->isrNumber);

        if (dev == OB_UART0)
        {
            dev->regMap0->C2 |= UART0_C2_RIE_MASK;
        }
        else
        {
            dev->regMap->C2 |= UART_C2_RIE_MASK;
        }
    }
}

void Uart_addTxCallback (Uart_DeviceHandle dev, Uart_callback callback)
{
    ohiassert(callback != NULL);

    if (callback != NULL)
    {
        dev->callbackTx[0] = callback;
        dev->isTxInterruptEnabled = TRUE;
        // Enable NVIC interrupt
        Interrupt_enable(dev->isrNumber);

        if (dev == OB_UART0)
        {
            dev->regMap0->C2 |= UART0_C2_TIE_MASK;
        }
        else
        {
            dev->regMap->C2 |= UART_C2_TIE_MASK;
        }
    }
}

void Uart_addErrorCallback (Uart_DeviceHandle dev, Uart_callback callback)
{
    // Not used in this microcontroller
    ohiassert(0);
}

void UART0_IRQHandler (void)
{
    Uart_callbackInterrupt(OB_UART0);
}

void UART1_IRQHandler (void)
{
    Uart_callbackInterrupt(OB_UART1);
}

void UART2_IRQHandler (void)
{
    Uart_callbackInterrupt(OB_UART2);
}

#endif // LIBOHIBOARD_MKL

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_UART
