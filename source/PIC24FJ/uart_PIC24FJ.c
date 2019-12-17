/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/source/PIC24FJ/uart_PIC24FJ.c
 * @author Leonardo Morichelli
 * @brief UART implementations for PIC24FJ Series.
 */

#include "platforms.h"

#if defined(LIBOHIBOARD_UART)

#ifdef __cplusplus
extern "C" {
#endif

#include "uart.h"

#include "platforms.h"
#include "utility.h"
#include "gpio.h"
#include "clock.h"
#include "interrupt.h"

#if defined (LIBOHIBOARD_PIC24FJ)

/**
 * @brief Enable the UART peripheral
 */
#define UART_DEVICE_ENABLE(REGMAP)        (UTILITY_SET_REGISTER_BIT(REGMAP->UMODE,_U1MODE_UARTEN_MASK))
/**
 * @brief Disable the UART peripheral
 */
#define UART_DEVICE_DISABLE(REGMAP)       (UTILITY_CLEAR_REGISTER_BIT(REGMAP->UMODE,_U1MODE_UARTEN_MASK))

/**
  * @brief Check that number of stop bits is valid for UART.
  * @param STOPBITS Number of stop bits.
  * @retval TRUE if is valid, FALSE otherwise
  */
#define UART_VALID_STOPBITS(STOPBITS) (((STOPBITS) == UART_STOPBITS_ONE)          || \
                                       ((STOPBITS) == UART_STOPBITS_TWO))

/**
  * @brief Check that UART parity is valid.
  * @param PARITY UART parity type.
  * @retval TRUE if is valid, FALSE otherwise
  */
#define UART_VALID_PARITY(PARITY) (((PARITY) == UART_PARITY_NONE) || \
                                   ((PARITY) == UART_PARITY_EVEN) || \
                                   ((PARITY) == UART_PARITY_ODD))

#define UART_VALID_DATABITS(DATABITS) (((DATABITS) == UART_DATABITS_EIGHT) || \
                                       ((DATABITS) == UART_DATABITS_NINE))

#define UART_VALID_FLOWCONTROL(FLOWCONTROL) (((FLOWCONTROL) == UART_FLOWCONTROL_NONE) || \
                                             ((FLOWCONTROL) == UART_FLOWCONTROL_RTS)  || \
                                             ((FLOWCONTROL) == UART_FLOWCONTROL_CTS_RTS))

#define UART_VALID_MODE(MODE) (((MODE) == UART_MODE_TRANSMIT) || \
                               ((MODE) == UART_MODE_RECEIVE)  || \
                               ((MODE) == UART_MODE_BOTH))

#define UART_VALID_SLEEPMODE(MODE) (((MODE) == UART_WAKEUPSLEEPMODE_ENABLE)  || \
                                    ((MODE) == UART_WAKEUPSLEEPMODE_DISABLE))

#define UART_VALID_IDLEMODE(MODE) (((MODE) == UART_STOPIDLEMODE_CONTINUES)    || \
                                   ((MODE) == UART_STOPIDLEMODE_DISCONTINUES))

/**
 * @brief Check the baudrate value
 */
#define UART_VALID_BAUDRATE(BAUDRATE) ((BAUDRATE) <= (Clock_getOutputValue(CLOCK_OUTPUT_PERIPHERAL) / 4))

typedef struct _Uart_Device
{
    UART_TypeDef* regmap;                          /**< Device memory pointer */

    volatile uint16_t* pmdRegisterPtr;     /**< Register for device enabling. */
    uint16_t pmdRegisterEnable;        /**< Register mask for current device. */

    volatile uint16_t* ppsRxRegisterPtr;
    uint16_t ppsRxRegisterMask;
    uint16_t ppsRxRegisterPosition;

    volatile uint16_t* ppsCtsRegisterPtr;
    uint16_t ppsCtsRegisterMask;
    uint16_t ppsCtsRegisterPosition;

    Gpio_PpsOutputFunction ppsTxRegisterValue;

    Gpio_PpsOutputFunction ppsRtsRegisterValue;

    Uart_Config config;

    /** The function pointer for user Rx callback. */
    void (*callbackRx)(struct _Uart_Device* dev, void* obj);
    /** The function pointer for user Tx callback. */
    void (*callbackTx)(struct _Uart_Device* dev, void* obj);
    /** The function pointer for user Errore callback. */
    void (*callbackErr)(struct _Uart_Device* dev, void* obj);
    /** Useful object added to callback when interrupt triggered. */
    void* callbackObj;

    Interrupt_Vector isrNumberTx;/**< ISR vector number for transmit interrupt. */
    Interrupt_Vector isrNumberRx;/**< ISR vector number for receive interrupt. */
    Interrupt_Vector isrNumberErr;/**< ISR vector number for error interrupt. */

    Uart_DeviceState state;                    /**< Current peripheral state. */

} Uart_Device;

#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)

#define UART_IS_DEVICE(DEVICE) (((DEVICE) == OB_UART1)  || \
                                ((DEVICE) == OB_UART2)  || \
                                ((DEVICE) == OB_UART3)  || \
                                ((DEVICE) == OB_UART4)  || \
                                ((DEVICE) == OB_UART5)  || \
                                ((DEVICE) == OB_UART6))

#define UART_IS_PIN_REMAPPABLE(DEVICE) (((DEVICE) == OB_UART1) || \
                                        ((DEVICE) == OB_UART2) || \
                                        ((DEVICE) == OB_UART3) || \
                                        ((DEVICE) == OB_UART4))

static Uart_Device uart1 =
{
        .regmap                 = UART1B,

        .pmdRegisterPtr         = &PMD->PMD1,
        .pmdRegisterEnable      = _PMD1_U1MD_MASK,

        .ppsRxRegisterPtr       = &PPS->RPINR[18],
        .ppsRxRegisterMask      = _RPINR18_U1RXR_MASK,
        .ppsRxRegisterPosition  = _RPINR18_U1RXR_POSITION,

        .ppsCtsRegisterPtr      = &PPS->RPINR[18],
        .ppsCtsRegisterMask     = _RPINR18_U1CTSR_MASK,
        .ppsCtsRegisterPosition = _RPINR18_U1CTSR_POSITION,

        .ppsTxRegisterValue     = GPIO_PPSOUTPUTFUNCTION_U1TX,

        .ppsRtsRegisterValue    = GPIO_PPSOUTPUTFUNCTION_U1RTS,

        .isrNumberTx            = INTERRUPT_UART1_TX,
        .isrNumberRx            = INTERRUPT_UART1_RX,
        .isrNumberErr           = INTERRUPT_UART1_ERROR,
};
Uart_DeviceHandle OB_UART1 = &uart1;

static Uart_Device uart2 =
{
        .regmap                 = UART2B,

        .pmdRegisterPtr         = &PMD->PMD1,
        .pmdRegisterEnable      = _PMD1_U2MD_MASK,

        .ppsRxRegisterPtr       = &PPS->RPINR[19],
        .ppsRxRegisterMask      = _RPINR19_U2RXR_MASK,
        .ppsRxRegisterPosition  = _RPINR19_U2RXR_POSITION,

        .ppsCtsRegisterPtr      = &PPS->RPINR[19],
        .ppsCtsRegisterMask     = _RPINR19_U2CTSR_MASK,
        .ppsCtsRegisterPosition = _RPINR19_U2CTSR_POSITION,

        .ppsTxRegisterValue     = GPIO_PPSOUTPUTFUNCTION_U2TX,

        .ppsRtsRegisterValue    = GPIO_PPSOUTPUTFUNCTION_U2RTS,

        .isrNumberTx            = INTERRUPT_UART2_TX,
        .isrNumberRx            = INTERRUPT_UART2_RX,
        .isrNumberErr           = INTERRUPT_UART2_ERROR,
};
Uart_DeviceHandle OB_UART2 = &uart2;

static Uart_Device uart3 =
{
        .regmap                 = UART3B,

        .pmdRegisterPtr         = &PMD->PMD3,
        .pmdRegisterEnable      = _PMD3_U3MD_MASK,

        .ppsRxRegisterPtr       = &PPS->RPINR[17],
        .ppsRxRegisterMask      = _RPINR17_U3RXR_MASK,
        .ppsRxRegisterPosition  = _RPINR17_U3RXR_POSITION,

        .ppsCtsRegisterPtr      = &PPS->RPINR[21],
        .ppsCtsRegisterMask     = _RPINR21_U3CTSR_MASK,
        .ppsCtsRegisterPosition = _RPINR21_U3CTSR_POSITION,

        .ppsTxRegisterValue     = GPIO_PPSOUTPUTFUNCTION_U3TX,

        .ppsRtsRegisterValue    = GPIO_PPSOUTPUTFUNCTION_U3RTS,

        .isrNumberTx            = INTERRUPT_UART3_TX,
        .isrNumberRx            = INTERRUPT_UART3_RX,
        .isrNumberErr           = INTERRUPT_UART3_ERROR,
};
Uart_DeviceHandle OB_UART3 = &uart3;

static Uart_Device uart4 =
{
        .regmap                 = UART4B,

        .pmdRegisterPtr         = &PMD->PMD4,
        .pmdRegisterEnable      = _PMD4_U4MD_MASK,

        .ppsRxRegisterPtr       = &PPS->RPINR[27],
        .ppsRxRegisterMask      = _RPINR27_U4RXR_MASK,
        .ppsRxRegisterPosition  = _RPINR27_U4RXR_POSITION,

        .ppsCtsRegisterPtr      = &PPS->RPINR[27],
        .ppsCtsRegisterMask     = _RPINR27_U4CTSR_MASK,
        .ppsCtsRegisterPosition = _RPINR27_U4CTSR_POSITION,

        .ppsTxRegisterValue     = GPIO_PPSOUTPUTFUNCTION_U4TX,

        .ppsRtsRegisterValue    = GPIO_PPSOUTPUTFUNCTION_U4RTS,

        .isrNumberTx            = INTERRUPT_UART4_TX,
        .isrNumberRx            = INTERRUPT_UART4_RX,
        .isrNumberErr           = INTERRUPT_UART4_ERROR,
};
Uart_DeviceHandle OB_UART4 = &uart4;

// This peripheral have all interface pin not configurable, but fixed!
static Uart_Device uart5 =
{
        .regmap                 = UART5B,

        .pmdRegisterPtr         = &PMD->PMD8,
        .pmdRegisterEnable      = _PMD8_U5MD_MASK,

        .ppsRxRegisterPtr       = 0,
        .ppsRxRegisterMask      = 0,
        .ppsRxRegisterPosition  = 0,

        .ppsCtsRegisterPtr      = 0,
        .ppsCtsRegisterMask     = 0,
        .ppsCtsRegisterPosition = 0,

        .ppsTxRegisterValue     = 0,

        .ppsRtsRegisterValue    = 0,

        .isrNumberTx            = INTERRUPT_UART5_TX,
        .isrNumberRx            = INTERRUPT_UART5_RX,
        .isrNumberErr           = INTERRUPT_UART5_ERROR,
};
Uart_DeviceHandle OB_UART5 = &uart5;

// This peripheral have all interface pin not configurable, but fixed!
static Uart_Device uart6 =
{
        .regmap                 = UART6B,

        .pmdRegisterPtr         = &PMD->PMD8,
        .pmdRegisterEnable      = _PMD8_U6MD_MASK,

        .ppsRxRegisterPtr       = 0,
        .ppsRxRegisterMask      = 0,
        .ppsRxRegisterPosition  = 0,

        .ppsCtsRegisterPtr      = 0,
        .ppsCtsRegisterMask     = 0,
        .ppsCtsRegisterPosition = 0,

        .ppsTxRegisterValue     = 0,

        .ppsRtsRegisterValue    = 0,

        .isrNumberTx            = INTERRUPT_UART6_TX,
        .isrNumberRx            = INTERRUPT_UART6_RX,
        .isrNumberErr           = INTERRUPT_UART6_ERROR,
};
Uart_DeviceHandle OB_UART6 = &uart6;

#endif

static System_Errors Uart_config (Uart_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Disable the peripheral
    UART_DEVICE_DISABLE(dev->regmap);

    // Clear all registers
    dev->regmap->UMODE = 0;
    dev->regmap->UBRG  = 0;
    dev->regmap->USTA  = 0;

    // Config peripheral
    // Configure data bits and parity control
    dev->regmap->UMODE = dev->regmap->UMODE & (~(_U1MODE_PDSEL_MASK));
    switch (dev->config.dataBits)
    {
    case UART_DATABITS_EIGHT:
        if (dev->config.parity == UART_PARITY_EVEN)
        {
            dev->regmap->UMODE |= _U1MODE_PDSEL0_MASK;
        }
        else if (dev->config.parity == UART_PARITY_ODD)
        {
            dev->regmap->UMODE |= _U1MODE_PDSEL1_MASK;
        }
        else
        {
            dev->regmap->UMODE |= 0x0000u;
        }
        break;
    case UART_DATABITS_NINE:
        dev->regmap->UMODE |= _U1MODE_PDSEL_MASK;
        break;
    }

    // Configure stop bits
    dev->regmap->UMODE = dev->regmap->UMODE & (~(_U1MODE_STSEL_MASK));
    switch (dev->config.stop)
    {
    case UART_STOPBITS_ONE:
        dev->regmap->UMODE |= 0x0000u;
        break;
    case UART_STOPBITS_TWO:
        dev->regmap->UMODE |= _U1MODE_STSEL_MASK;
        break;
    }

    // Configure hardware flow control
    dev->regmap->UMODE = dev->regmap->UMODE & (~(_U1MODE_UEN_MASK));
    switch (dev->config.flowControl)
    {
    case UART_FLOWCONTROL_NONE:
        dev->regmap->UMODE |= 0x0000u;
        break;
    case UART_FLOWCONTROL_RTS:
        dev->regmap->UMODE |= _U1MODE_UEN0_MASK;
        break;
    case UART_FLOWCONTROL_CTS_RTS:
        dev->regmap->UMODE |= _U1MODE_UEN1_MASK;
        break;
    }

    // Configure behaviour during sleep and idle mode
    dev->regmap->UMODE = dev->regmap->UMODE & (~(_U1MODE_WAKE_MASK | _U1MODE_USIDL_MASK));
    dev->regmap->UMODE |= dev->config.sleepMode;
    dev->regmap->UMODE |= dev->config.idleMode;

    // Configure Baudrate
    err = Uart_setBaudrate(dev,dev->config.baudrate);
    if (err != ERRORS_NO_ERROR)
        return ERRORS_UART_WRONG_PARAM;

    // Check callback and save it
    if (dev->config.callbackRx != 0)
    {
        dev->callbackRx = dev->config.callbackRx;
        Interrupt_setPriority(dev->isrNumberRx,dev->config.isrRxPriority);
        Interrupt_clearFlag(dev->isrNumberRx);
        Interrupt_enable(dev->isrNumberRx);

        Interrupt_clearFlag(dev->isrNumberErr);
        Interrupt_enable(dev->isrNumberErr);
        dev->callbackErr = dev->config.callbackError;
    }
    if (dev->config.callbackTx != 0)
    {
        dev->callbackTx = dev->config.callbackTx;
        Interrupt_setPriority(dev->isrNumberTx,dev->config.isrTxPriority);
        Interrupt_clearFlag(dev->isrNumberTx);
        Interrupt_enable(dev->isrNumberTx);
    }
    if (dev->config.callbackObj != NULL)
    {
        dev->callbackObj = dev->config.callbackObj;
    }
    else
    {
        dev->callbackObj = NULL;
    }

    UART_DEVICE_ENABLE(dev->regmap);

    // Configure peripheral mode
    dev->regmap->USTA = dev->regmap->USTA & (~(_U1STA_URXEN_MASK | _U1STA_UTXEN_MASK));
    switch (dev->config.mode)
    {
    case UART_MODE_TRANSMIT:
        dev->regmap->USTA |= _U1STA_UTXEN_MASK;
        break;
    case UART_MODE_RECEIVE:
        dev->regmap->USTA |= _U1STA_URXEN_MASK;
        break;
    case UART_MODE_BOTH:
        dev->regmap->USTA |= _U1STA_URXEN_MASK | _U1STA_UTXEN_MASK;
        break;
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
    if (ohiassert(UART_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_UART_WRONG_DEVICE;
    }

    err = ohiassert(UART_VALID_DATABITS(config->dataBits));
    err |= ohiassert(UART_VALID_BAUDRATE(config->baudrate));
    err |= ohiassert(UART_VALID_STOPBITS(config->stop));
    err |= ohiassert(UART_VALID_PARITY(config->parity));
    err |= ohiassert(UART_VALID_FLOWCONTROL(config->flowControl));
    err |= ohiassert(UART_VALID_SLEEPMODE(config->sleepMode));
    err |= ohiassert(UART_VALID_IDLEMODE(config->idleMode));
    if (config->callbackRx != 0)
    {
        err |= ohiassert(INTERRUPT_IS_VALID_PRIORITY(config->isrRxPriority));
    }
    if (config->callbackTx != 0)
    {
        err |= ohiassert(INTERRUPT_IS_VALID_PRIORITY(config->isrTxPriority));
    }
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_UART_WRONG_PARAM;
    }
    // Save configuration
    dev->config = *config;

    // Enable peripheral clock if needed
    if (dev->state == UART_DEVICESTATE_RESET)
    {
        // Enable peripheral
        UTILITY_CLEAR_REGISTER_BIT(*dev->pmdRegisterPtr, dev->pmdRegisterEnable);

        // Whether the device have remappable pins, configure it!
        if (UART_IS_PIN_REMAPPABLE(dev))
        {
            // Enable pins
            if (config->rxPin != UART_PINS_RXNONE)
            {
                Uart_setRxPin(dev, config->rxPin);
            }

            if (config->txPin != UART_PINS_TXNONE)
            {
                Uart_setTxPin(dev, config->txPin);
            }
        }
    }
    dev->state = UART_DEVICESTATE_BUSY;

    // Configure the peripheral
    err = Uart_config(dev);
    if (err != ERRORS_NO_ERROR)
    {
        // FIXME: Call deInit?
        dev->state = UART_DEVICESTATE_ERROR;
        return err;
    }

    dev->state = UART_DEVICESTATE_READY;
    return err;
}

System_Errors Uart_deInit (Uart_DeviceHandle dev)
{
    System_Errors error = ERRORS_NO_ERROR;

    Uart_flushRx(dev);

    // Disable the peripheral
    UART_DEVICE_DISABLE(dev->regmap);

    // Clear all registers
    dev->regmap->UMODE = 0;
    dev->regmap->UBRG  = 0;
    dev->regmap->USTA  = 0;

    // Check callback and delete it
    if (dev->callbackRx != 0)
    {
        dev->callbackRx = 0;
        Interrupt_clearFlag(dev->isrNumberRx);
        Interrupt_disable(dev->isrNumberRx);

        dev->callbackErr = 0;
        Interrupt_clearFlag(dev->isrNumberErr);
        Interrupt_disable(dev->isrNumberErr);
    }
    if (dev->callbackTx != 0)
    {
        dev->callbackTx = 0;
        Interrupt_clearFlag(dev->isrNumberTx);
        Interrupt_disable(dev->isrNumberTx);
    }
    dev->callbackObj = NULL;

    return error;
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
    UTILITY_SET_REGISTER_BIT(dev->regmap->USTA,_U1STA_URXEN_MASK);
    UTILITY_SET_REGISTER_BIT(dev->regmap->USTA,_U1STA_UTXEN_MASK);    
    
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
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->USTA,_U1STA_URXEN_MASK);
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->USTA,_U1STA_UTXEN_MASK);
    
    return err;
}

System_Errors Uart_setRxPin (Uart_DeviceHandle dev, Uart_RxPins rxPin)
{
    // unlock PPS
    __builtin_write_OSCCONL(OSCCON & 0xbf);

    UTILITY_MODIFY_REGISTER(*dev->ppsRxRegisterPtr, dev->ppsRxRegisterMask, (rxPin << dev->ppsRxRegisterPosition));

    // lock   PPS
    __builtin_write_OSCCONL(OSCCON | 0x40);

    return ERRORS_NO_ERROR;
}

System_Errors Uart_setTxPin (Uart_DeviceHandle dev, Uart_TxPins txPin)
{
    uint8_t regIndex = txPin / 2;
    uint8_t regPosIndex = ((txPin % 2 ) * 8);

    // unlock PPS
    __builtin_write_OSCCONL(OSCCON & 0xbf);

    UTILITY_MODIFY_REGISTER(PPS->RPOR[regIndex], (0x3F << regPosIndex),(dev->ppsTxRegisterValue << regPosIndex));

    // lock   PPS
    __builtin_write_OSCCONL(OSCCON | 0x40);

    return ERRORS_NO_ERROR;
}

System_Errors Uart_setBaudrate (Uart_DeviceHandle dev, uint32_t baudrate)
{
    uint32_t frequency = Clock_getOutputValue(CLOCK_OUTPUT_PERIPHERAL);
    uint16_t brg = 0;

    if (frequency != 0)
    {
        if (baudrate <= (frequency / 16))
        {
            uint16_t brgLow  = (uint16_t) ((float)(((frequency / (16 * baudrate)) + 0.5f) - 1));
            uint16_t brgHigh = (uint16_t) ((float)(((frequency / (4  * baudrate)) + 0.5f) - 1));

            // Compute baudrate
            uint32_t baudrateComputeLow  = frequency / (16 * (brgLow  + 1));
            uint32_t baudrateComputeHigh = frequency / (4 * (brgHigh + 1));

            uint32_t diffLow  = 0xFFFFFFFFul;
            uint32_t diffHigh = 0xFFFFFFFFul;

            if (baudrateComputeLow > baudrate) diffLow = baudrateComputeLow - baudrate;
            else                               diffLow = baudrate - baudrateComputeLow;

            if (baudrateComputeHigh > baudrate) diffHigh = baudrateComputeHigh - baudrate;
            else                                diffHigh = baudrate - baudrateComputeHigh;

            if (diffLow < diffHigh)
            {
                brg = brgLow;
                dev->regmap->UMODE &= ~(_U1MODE_BRGH_MASK);
            }
            else
            {
                brg = brgHigh;
                dev->regmap->UMODE |= (_U1MODE_BRGH_MASK);
            }
        }
        else
        {
            brg = (uint16_t) ((float)(((frequency / (4 * baudrate)) + 0.5f) - 1));
            dev->regmap->UMODE |= _U1MODE_BRGH_MASK;
        }
    }
    else
    {
        return ERRORS_UART_NO_CLOCKSOURCE;
    }

    dev->regmap->UBRG = brg;

    return ERRORS_NO_ERROR;
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

System_Errors Uart_read (Uart_DeviceHandle dev, uint8_t *data, uint32_t timeout)
{
    uint32_t timeoutEnd = System_currentTick() + timeout;

    // Wait until the buffer is empty
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->USTA,_U1STA_URXDA_MASK) == 0)
    {
        if (System_currentTick() > timeoutEnd)
        {
            return ERRORS_UART_TIMEOUT_RX;
        }
    }

    // In case of 9B
    if (dev->config.dataBits == UART_DATABITS_NINE)
    {
        // Cast the pointer
        uint16_t* temp = (uint16_t *) data;
        *temp = (uint16_t) (dev->regmap->URXREG);
    }
    else
    {
        *data = (uint8_t) (dev->regmap->URXREG & 0x00FF);
    }

    return ERRORS_NO_ERROR;
}

System_Errors Uart_write (Uart_DeviceHandle dev, const uint8_t* data, uint32_t timeout)
{
    System_Errors error = ERRORS_NO_ERROR;

    uint32_t timeoutEnd = System_currentTick() + timeout;

    // Wait until the buffer is empty
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->USTA,_U1STA_UTXBF_MASK) > 0)
    {
        if (System_currentTick() > timeoutEnd)
        {
            return ERRORS_UART_TIMEOUT_TX;
        }
    }

    if (dev->config.dataBits == UART_DATABITS_NINE)
    {
        uint16_t* temp = (uint16_t *) data;
        dev->regmap->UTXREG = (*temp & 0x01FFu);
    }
    else
    {
        dev->regmap->UTXREG = (*data & 0x00FFu);
    }

    return error;
}

void Uart_flushRx (Uart_DeviceHandle dev)
{
    uint16_t dummy = 0;
    for(uint16_t i = 0; i < 4; i++)
    {
        (void)dummy;
        dummy = (uint16_t) (dev->regmap->URXREG);
    }
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->USTA, _U1STA_OERR_MASK);
}

bool Uart_isPresent (Uart_DeviceHandle dev)
{
    return (UTILITY_READ_REGISTER_BIT(dev->regmap->USTA,_U1STA_URXDA_MASK) == 0) ? FALSE : TRUE;
}

static inline void Uart_isrTxHandler (Uart_DeviceHandle dev)
{
    if (dev->callbackTx)
    {
        dev->callbackTx(dev,dev->callbackObj);
    }
    Interrupt_clearFlag(dev->isrNumberTx);
}

static inline void Uart_isrRxHandler (Uart_DeviceHandle dev)
{
    if (dev->callbackRx)
    {
        dev->callbackRx(dev,dev->callbackObj);
    }
    Interrupt_clearFlag(dev->isrNumberRx);
}

static inline void Uart_isrErrorHandler (Uart_DeviceHandle dev)
{
    if (dev->callbackErr)
    {
        dev->callbackErr(dev,dev->callbackObj);
    }
    Interrupt_clearFlag(dev->isrNumberErr);
}

void __isr_noautopsv _U1RXInterrupt(void)
{
    Uart_isrRxHandler(OB_UART1);
}

void __isr_noautopsv _U1TXInterrupt(void)
{
    Uart_isrTxHandler(OB_UART1);
}

void __isr_noautopsv _U1ErrInterrupt (void)
{
    if (UTILITY_READ_REGISTER_BIT(OB_UART1->regmap->USTA, _U1STA_OERR_MASK) != 0)
    {
        UTILITY_CLEAR_REGISTER_BIT(OB_UART1->regmap->USTA, _U1STA_OERR_MASK);
    }

    Uart_isrErrorHandler(OB_UART1);

    Interrupt_clearFlag(INTERRUPT_UART1_ERROR);
}

void __isr_noautopsv _U2RXInterrupt(void)
{
    Uart_isrRxHandler(OB_UART2);
}

void __isr_noautopsv _U2TXInterrupt(void)
{
    Uart_isrTxHandler(OB_UART2);
}

void __isr_noautopsv _U2ErrInterrupt (void)
{
    if (UTILITY_READ_REGISTER_BIT(OB_UART2->regmap->USTA, _U2STA_OERR_MASK) != 0)
    {
        UTILITY_CLEAR_REGISTER_BIT(OB_UART2->regmap->USTA, _U2STA_OERR_MASK);
    }

    Uart_isrErrorHandler(OB_UART2);

    Interrupt_clearFlag(INTERRUPT_UART2_ERROR);
}

void __isr_noautopsv _U3RXInterrupt(void)
{
    Uart_isrRxHandler(OB_UART3);
}

void __isr_noautopsv _U3TXInterrupt(void)
{
    Uart_isrTxHandler(OB_UART3);
}

void __isr_noautopsv _U3ErrInterrupt (void)
{
    if (UTILITY_READ_REGISTER_BIT(OB_UART3->regmap->USTA, _U3STA_OERR_MASK) != 0)
    {
        UTILITY_CLEAR_REGISTER_BIT(OB_UART3->regmap->USTA, _U3STA_OERR_MASK);
    }

    Uart_isrErrorHandler(OB_UART3);

    Interrupt_clearFlag(INTERRUPT_UART3_ERROR);
}

void __isr_noautopsv _U4RXInterrupt(void)
{
    Uart_isrRxHandler(OB_UART4);
}

void __isr_noautopsv _U4TXInterrupt(void)
{
    Uart_isrTxHandler(OB_UART4);
}

void __isr_noautopsv _U4ErrInterrupt (void)
{
    if (UTILITY_READ_REGISTER_BIT(OB_UART4->regmap->USTA, _U4STA_OERR_MASK) != 0)
    {
        UTILITY_CLEAR_REGISTER_BIT(OB_UART4->regmap->USTA, _U4STA_OERR_MASK);
    }

    Uart_isrErrorHandler(OB_UART4);

    Interrupt_clearFlag(INTERRUPT_UART4_ERROR);
}

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_UART
