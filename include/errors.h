/******************************************************************************
 * Copyright (C) 2012-2014 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Edoardo Bezzeccheri <coolman3@gmail.com>
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
 ******************************************************************************/

/**
 * @file libohiboard/include/errors.h
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Errors definition
 */

#ifndef __ERRORS_H
#define __ERRORS_H

typedef enum _System_Errors
{
	ERRORS_NO_ERROR,                        /**< There is no error. */
	ERRORS_PARAM_VALUE,                     /**< Invalid value. */
	ERRORS_EXT_OSC_NOT_SELECT,              /**< External oscillator not selected. */
	
	ERRORS_HW_NOT_ENABLED,                  /**< Hardware pin of the device was not enabled. */
	
	ERRORS_IRQ_NUM_VECTOR_WRONG,
	
	ERRORS_GPIO_WRONG_PORT,
	ERRORS_GPIO_WRONG_CONFIG,
	
    ERRORS_UART_DEVICE_NOT_INIT,
    ERRORS_UART_DEVICE_JUST_INIT,
    ERRORS_UART_NO_PIN_FOUND,
	
	ERRORS_IIC_TX_OK,
	ERRORS_IIC_TX_ERROR,
	ERRORS_IIC_TX_TIMEOUT,
	ERRORS_IIC_TX_ACK_RECEIVED,
	ERRORS_IIC_TX_ACK_NOT_RECEIVED,
    ERRORS_IIC_RX_OK,
	ERRORS_IIC_RX_TIMEOUT,
	ERRORS_IIC_SCLTIMEOUT_TOO_LARGE,
	ERRORS_IIC_SCLTIMEOUT,
	ERRORS_IIC_NO_SCLTIMEOUT,
	
	ERRORS_ADC_CHANNEL_WRONG,
	
    ERRORS_DAC_DEVICE_NOT_INIT,
    ERRORS_DAC_DEVICE_JUST_INIT,
	
	ERRORS_FTM_OK,
	ERRORS_FTM_CHANNEL_NOT_FOUND,
	ERRORS_FTM_DEVICE_NOT_INIT,
	
	ERRORS_UTILITY_ILLEGAL_CHAR,
	ERRORS_UTILITY_EMPTY_STRING,
	ERRORS_UTILITY_LONG_STRING,
    ERRORS_UTILITY_CONVERSION_OK,
    ERRORS_UTILITY_FLOAT_WRONG_PRECISION,
    
    ERRORS_MCG_OUT_OF_RANGE, //Frequency outside of the allowed range
    ERRORS_MCG_NO_FREQUENCY, //Impossible to obtain the request fout_SYS with this setup
    ERRORS_MCG_NO_STATE, //Not valid state
    ERRORS_MCG_EXTERNAL_REFERENCE_OUT_OF_RANGE,
    ERRORS_MCG_UNDER_100khz, //There is a strange behavior if foutSYS < 100Khz
    ERRORS_MCG_JUST_INIT,
    ERRORS_MCG_NOT_INIT,
    ERRORS_MCG_ERRATA_DIVIDER,
    ERRORS_MCG_48M_REF //Ther is a strange behavior when I use IRC48M like MCG source in MK64F

} System_Errors;

void Errors_assert (const char* file, const int line);
#define assert(condition) ((condition) ? (void)0 : Errors_assert(__FILE__,__LINE__))

#define ERR_OK                          0x00U /* OK */
#define ERR_SPEED                       0x01U /* This device does not work in the active speed mode. */
#define ERR_RANGE                       0x02U /* Parameter out of range. */
#define ERR_VALUE                       0x03U /* Parameter of incorrect value. */
#define ERR_OVERFLOW                    0x04U /* Timer overflow. */
#define ERR_MATH                        0x05U /* Overflow during evaluation. */
#define ERR_ENABLED                     0x06U /* Device is enabled. */
#define ERR_DISABLED                    0x07U /* Device is disabled. */
#define ERR_BUSY                        0x08U /* Device is busy. */
#define ERR_NOTAVAIL                    0x09U /* Requested value or method not available. */
#define ERR_RXEMPTY                     0x0AU /* No data in receiver. */
#define ERR_TXFULL                      0x0BU /* Transmitter is full. */
#define ERR_BUSOFF                      0x0CU /* Bus not available. */
#define ERR_OVERRUN                     0x0DU /* Overrun error is detected. */
#define ERR_FRAMING                     0x0EU /* Framing error is detected. */
#define ERR_PARITY                      0x0FU /* Parity error is detected. */
#define ERR_NOISE                       0x10U /* Noise error is detected. */
#define ERR_IDLE                        0x11U /* Idle error is detected. */
#define ERR_FAULT                       0x12U /* Fault error is detected. */
#define ERR_BREAK                       0x13U /* Break char is received during communication. */
#define ERR_CRC                         0x14U /* CRC error is detected. */
#define ERR_ARBITR                      0x15U /* A node losts arbitration. This error occurs if two nodes start transmission at the same time. */
#define ERR_PROTECT                     0x16U /* Protection error is detected. */
#define ERR_UNDERFLOW                   0x17U /* Underflow error is detected. */
#define ERR_UNDERRUN                    0x18U /* Underrun error is detected. */
#define ERR_COMMON                      0x19U /* Common error of a device. */
#define ERR_LINSYNC                     0x1AU /* LIN synchronization error is detected. */
#define ERR_FAILED                      0x1BU /* Requested functionality or process failed. */
#define ERR_QFULL                       0x1CU /* Queue is full. */
#define ERR_PARAM_MASK                  0x80U /* Invalid mask. */
#define ERR_PARAM_MODE                  0x81U /* Invalid mode. */
#define ERR_PARAM_INDEX                 0x82U /* Invalid index. */
#define ERR_PARAM_DATA                  0x83U /* Invalid data. */
#define ERR_PARAM_SIZE                  0x84U /* Invalid size. */
#define ERR_PARAM_VALUE                 0x85U /* Invalid value. */
#define ERR_PARAM_RANGE                 0x86U /* Invalid parameter's range or parameters' combination. */
#define ERR_PARAM_LOW_VALUE             0x87U /* Invalid value (LOW part). */
#define ERR_PARAM_HIGH_VALUE            0x88U /* Invalid value (HIGH part). */
#define ERR_PARAM_ADDRESS               0x89U /* Invalid address. */
#define ERR_PARAM_PARITY                0x8AU /* Invalid parity. */
#define ERR_PARAM_WIDTH                 0x8BU /* Invalid width. */
#define ERR_PARAM_LENGTH                0x8CU /* Invalid length. */
#define ERR_PARAM_ADDRESS_TYPE          0x8DU /* Invalid address type. */
#define ERR_PARAM_COMMAND_TYPE          0x8EU /* Invalid command type. */
#define ERR_PARAM_COMMAND               0x8FU /* Invalid command. */
#define ERR_PARAM_RECIPIENT             0x90U /* Invalid recipient. */
#define ERR_PARAM_BUFFER_COUNT          0x91U /* Invalid buffer count. */
#define ERR_PARAM_ID                    0x92U /* Invalid ID. */
#define ERR_PARAM_GROUP                 0x93U /* Invalid group. */
#define ERR_PARAM_CHIP_SELECT           0x94U /* Invalid chip select. */
#define ERR_PARAM_ATTRIBUTE_SET         0x95U /* Invalid set of attributes. */
#define ERR_PARAM_SAMPLE_COUNT          0x96U /* Invalid sample count. */
#define ERR_PARAM_CONDITION             0x97U /* Invalid condition. */
#define ERR_PARAM_TICKS                 0x98U /* Invalid ticks parameter. */

#endif /* __ERRORS_H */
