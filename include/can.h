/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2021 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/can.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief CAN definitions and prototypes.
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

#ifdef LIBOHIBOARD_CAN

/**
 * @defgroup CAN CAN
 * @brief CAN HAL driver
 * @{
 */

#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "utility.h"

/**
 * @defgroup CAN_Configuration_Functions CAN configuration functions and types
 * @brief Functions and types to open, close and configure a CAN peripheral.
 * @{
 */

/**
 * The list of the possible peripheral HAL state.
 */
typedef enum _Can_DeviceState
{
    CAN_DEVICESTATE_RESET,
    CAN_DEVICESTATE_READY,
    CAN_DEVICESTATE_BUSY,
    CAN_DEVICESTATE_ERROR,

} Can_DeviceState;

#if (LIBOHIBOARD_VERSION >= 0x20000u)
typedef struct _Can_Device* Can_DeviceHandle;

//typedef void (*Can_callback)(Can_DeviceHandle dev, void* obj);

#else
#error "[LIBOHIBOARD ERROR] Not implemented in the previous version!"
#endif

#if defined (LIBOHIBOARD_STM32L4)

#include "hardware/STM32L4/can_STM32L4.h"

#else
#error "[LIBOHIBOARD ERROR] Not implemented for the current platform!"
#endif

typedef struct _Can_Config
{
//    Uart_RxPins rxPin;
//    Uart_TxPins txPin;

    uint32_t baudrate;

} Can_Config;

/**
 * This function initialize the selected peripheral.
 *
 * @param[in] dev Can device handle
 * @param[in] config Configuration parameters for the Uart
 */
System_Errors Can_init (Can_DeviceHandle dev, Can_Config *config);

/**
 * This function de-initialize the selected peripheral.
 *
 * @param[in] dev Can device handle
 */
System_Errors Can_deInit (Can_DeviceHandle dev);

/**
 * Enable and start CAN peripheral.
 *
 * @param[in] dev CAN device handle
 * @return
 */
System_Errors Can_start (Can_DeviceHandle dev);

/**
 * Stop CAN peripheral.
 *
 * @param[in] dev CAN device handle
 * @return
 */
System_Errors Can_stop (Can_DeviceHandle dev);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __CAN_H

/**
 * @}
 */

#endif // LIBOHIBOARD_CAN

/**
 * @}
 */

