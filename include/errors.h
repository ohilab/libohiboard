/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2012-2018 A. C. Open Hardware Ideas Lab
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
 */

/**
 * @file libohiboard/include/errors.h
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief Errors definition
 */

#ifndef __ERRORS_H
#define __ERRORS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _System_Errors
{
	ERRORS_NO_ERROR = 0,                              /**< There is no error. */
	ERRORS_PARAM_VALUE,                                   /**< Invalid value. */
	ERRORS_EXT_OSC_NOT_SELECT,         /**< External oscillator not selected. */
	
	ERRORS_SYSTEM_TICK_INIT_FAILED,
	ERRORS_SYSTEM_NO_CLOCK,

	ERRORS_HW_NOT_ENABLED,   /**< Hardware pin of the device was not enabled. */
	
	ERRORS_IRQ_NUM_VECTOR_WRONG,
	ERRORS_IRQ_PRIORITY_LEVEL_WRONG,
	
	ERRORS_GPIO_WRONG_PORT,
	ERRORS_GPIO_WRONG_CONFIG,
    ERRORS_GPIO_WRONG_PIN,
	
    ERRORS_UART_DEVICE_NOT_INIT,
    ERRORS_UART_DEVICE_JUST_INIT,
    ERRORS_UART_NO_PIN_FOUND,
    ERRORS_UART_LIRC_SOURCE_CONFLICT_MCG,
    ERRORS_UART_NO_CLOCKSOURCE,
    ERRORS_UART_CLOCKSOURCE_FREQUENCY_TOO_LOW,
    ERRORS_UART_PARITY,                            /**< Parity error occured. */
    ERRORS_UART_NO_DEVICE,
    ERRORS_UART_WRONG_DEVICE,
    ERRORS_UART_WRONG_PARAM,
    ERRORS_UART_WRONG_BAUDRATE,
    ERRORS_UART_TIMEOUT_RX,
    ERRORS_UART_TIMEOUT_TX,
	
	ERRORS_IIC_OK,
	ERRORS_IIC_TX_OK,
	ERRORS_IIC_TX_ERROR,
	ERRORS_IIC_TX_TIMEOUT,
	ERRORS_IIC_TX_ACK_RECEIVED,
	ERRORS_IIC_TX_ACK_NOT_RECEIVED,
    ERRORS_IIC_RX_OK,
	ERRORS_IIC_RX_TIMEOUT,
	ERRORS_IIC_TIMEOUT,
	ERRORS_IIC_SCLTIMEOUT_TOO_LARGE,
	ERRORS_IIC_SCLTIMEOUT,
	ERRORS_IIC_NO_SCLTIMEOUT,
    ERRORS_IIC_DEVICE_NOT_INIT,
    ERRORS_IIC_DEVICE_JUST_INIT,
    ERRORS_IIC_NO_PIN_FOUND,
    ERRORS_IIC_NO_DEVICE,
    ERRORS_IIC_WRONG_DEVICE,
    ERRORS_IIC_WRONG_BAUDRATE,
    ERRORS_IIC_NO_CLOCKSOURCE,
    ERRORS_IIC_CLOCKSOURCE_FREQUENCY_TOO_LOW,
    ERRORS_IIC_WRONG_PARAM,

    ERRORS_SPI_BAUDRATE_NOT_FOUND,
    ERRORS_SPI_DEVICE_NOT_INIT,
    ERRORS_SPI_DEVICE_JUST_INIT,
    ERRORS_SPI_NO_PIN_FOUND,
    ERRORS_SPI_NO_DEVICE,
    ERRORS_SPI_WRONG_DEVICE,
    ERRORS_SPI_WRONG_PARAM,
    ERRORS_SPI_CLOCKSOURCE_FREQUENCY_TOO_LOW,
    ERRORS_SPI_TIMEOUT_RX,
    ERRORS_SPI_TIMEOUT_TX,
	
	ERRORS_ADC_CHANNEL_WRONG,
    ERRORS_ADC_CHANNEL_BUSY,
	
    ERRORS_DAC_DEVICE_NOT_INIT,
    ERRORS_DAC_DEVICE_JUST_INIT,
    ERRORS_ADC_ERRATA_DIVIDERS,
    ERRORS_DAC_WRONG_PARAMETER,
    ERRORS_ADC_PIN_WRONG,

    ERRORS_ADC_DEVICE_JUST_INIT,
    ERRORS_ADC_DEVICE_NOT_INIT,                  /**< Device not initialized. */
    ERRORS_ADC_DIVIDER_NOT_FOUND,
    ERRORS_ADC_NUMCH_WRONG,
    ERRORS_ADC_CALIBRATION,        /**< Error during ADC calibration process. */
	
	ERRORS_FTM_OK,
	ERRORS_FTM_CHANNEL_NOT_FOUND,
	ERRORS_FTM_DEVICE_NOT_INIT,
	ERRORS_FTM_FAULT_PIN_WRONG,
	
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
    ERRORS_MCG_48M_REF, //Ther is a strange behavior when I use IRC48M like MCG source in MK64F

    ERRORS_CLOCK_NO_CONFIG,
    ERRORS_CLOCK_WRONG_CONFIGURATION,
    ERRORS_CLOCK_NOT_READY,
#if defined (LIBOHIBOARD_ST_STM32)
    ERRORS_CLOCK_PLL_NOT_READY,
    ERRORS_CLOCK_HSE_NOT_READY,
    ERRORS_CLOCK_HSI_NOT_READY,
    ERRORS_CLOCK_MSI_NOT_READY,
#endif

    ERRORS_PIT_NOT_EXIST,                   /**< The requested PIT not exist. */
    ERRORS_PIT_WRONG_VALUE,                       /**< Wrong frequency value. */
    ERRORS_PIT_NOT_INITIALIZED,    /**< The selected PIT was not initialized. */

    ERRORS_PDB_DEVICE_WRONG,                  /**< Device triggered not exist. */
    ERRORS_PDB_DEVICE_NOT_INIT,

	ERRORS_DMA_OK,                                 /**< DMA initialization ok */

    ERRORS_ETHERNET_OK,                     /**< No Ethernet errors occurred. */
    ERRORS_ETHERNET_TIMEOUT,         /**< Generic timeout of ethernet device. */
    ERRORS_ETHERNET_SMI_TIMEOUT,    /**< Error during communication with PHY. */
    ERRORS_ETHERNETIF_WRONG_DEVICE,                 /**< Wrong device number. */
    ERRORS_ETHERNETIF_NO_MAC_ADDRESS,         /**< No MAC address configured. */
    ERRORS_ETHERNETIF_RX_BUFFERDESCRIPTOR_FULL,
    ERRORS_ETHERNETIF_RX_FRAME_TRUNCATED,
    ERRORS_ETHERNETIF_RX_GENERIC_ERROR,
    ERRORS_ETHERNETIF_RX_SMALL_BUFFERDESCRIPTOR_NUMBER,
    ERRORS_ETHERNETIF_TX_BUFFERDESCRIPTOR_FULL,

    ERRORS_XBAR_JUST_INIT,
    ERRORS_XBAR_NOT_INIT,
    ERRORS_XBAR_IN_WRONG,
    ERRORS_XBAR_OUT_WRONG,

	ERRORS_LLWU_WRONG_EXTPIN,
	ERRORS_LLWU_WRONG_WAKEUPMODULE,

	ERRORS_SMC_NO_ERROR,
	ERRORS_SMC_STATUS_NOT_ALLOWED,
	ERRORS_SMC_STATUS_NOT_ENABLED,
	ERRORS_SMC_STATUS_ALREADY_SET,
	ERRORS_SMC_ERROR,

	ERRORS_FLASH_READ_COLLISION,
    ERRORS_FLASH_ACCESS,
    ERRORS_FLASH_PROTECTION_VIOLATION,
    ERRORS_FLASH_COMPLETION_STATUS,
    ERRORS_FLASH_JUST_INIT,

    ERRORS_ASSERT,
} System_Errors;

System_Errors Errors_assert (const char* file, const int line);
#define ohiassert(condition) ((condition) ? ERRORS_NO_ERROR : Errors_assert(__FILE__,__LINE__))

#ifdef __cplusplus
}
#endif

#endif // __ERRORS_H
