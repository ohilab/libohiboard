/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2014-2019 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Matteo Civale <m.civale@gmail.com>
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
 * @file libohiboard/include/gpio.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Matteo Civale <m.civale@gmail.com>
 * @brief GPIO definitions and prototypes.
 */

/**
 * @addtogroup LIBOHIBOARD_Driver
 * @{
 */

/**
 * @defgroup GPIO GPIO
 * @brief GPIO HAL driver
 * @{
 */

#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"
#include "errors.h"
#include "types.h"
#include "interrupt.h"

/**
 * @defgroup GPIO_Configuration_Functions GPIO configuration functions and types
 * @brief Functions and types useful to manage GPIO pins.
 * @{
 */

/**
 * @defgroup GPIO_Pin_Option GPIO Pins configuration options
 * @{
 */
#define GPIO_PINS_OUTPUT                    0x0001
#define GPIO_PINS_INPUT                     0x0002
#define GPIO_PINS_PULL                      0x0004
#define GPIO_PINS_ENABLE_PULLUP             0x0008
#define GPIO_PINS_ENABLE_PULLDOWN           0x0010
#if defined LIBOHIBOARD_NXP_KINETIS
#define GPIO_PINS_ENABLE_DRIVE_STRENGTH     0x0020
#define GPIO_PINS_ENABLE_SLEW_RATE          0x0040
#define GPIO_PINS_ENABLE_PASSIVE_FILTER     0x0080
#elif defined LIBOHIBOARD_ST_STM32
#define GPIO_PINS_ENABLE_OUTPUT_PUSHPULL    0x0020
#define GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN   0x0040
#define GPIO_PINS_SPEED_LOW                 0x0080
#define GPIO_PINS_SPEED_MEDIUM              0x0100
#define GPIO_PINS_SPEED_HIGH                0x0200
#define GPIO_PINS_SPEED_VERY_HIGH           0x0400
#if defined (LIBOHIBOARD_STM32L4)
#define GPIO_PINS_ADC_CONNECTED             0x0800
#endif
#elif defined LIBOHIBOARD_MICROCHIP_PIC
#if defined LIBOHIBOARD_PIC24FJ
#define GPIO_PINS_ENABLE_OUTPUT_PUSHPULL    0x0020
#define GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN   0x0040
#define GPIO_PINS_ANALOG                    0x0080
#endif
#endif
/**
 * @}
 */

// Useful define
#if defined LIBOHIBOARD_NXP_KINETIS
#define GPIO_PIN_MASK_NUMBER                0x1Fu
#elif defined LIBOHIBOARD_ST_STM32
#define GPIO_PIN_MASK_NUMBER                0x0Fu
#elif defined LIBOHIBOARD_MICROCHIP_PIC
#define GPIO_PIN_MASK_NUMBER                0x01u
#endif
#define GPIO_PIN(x)                         (((1)<<(x & GPIO_PIN_MASK_NUMBER)))

/**
 * List of possible Gpio status.
 */
typedef enum _Gpio_Level
{
    GPIO_LOW    = 0x00,
    GPIO_HIGH   = 0x01,
    GPIO_TOGGLE = 0xFF,
} Gpio_Level;


#if defined (LIBOHIBOARD_STM32L0)

#include "hardware/STM32L0/gpio_STM32L0.h"

#elif defined (LIBOHIBOARD_STM32L4)

#include "hardware/STM32L4/gpio_STM32L4.h"

#elif defined (LIBOHIBOARD_MKL)

#include "hardware/NXPMKL/gpio_MKL.h"

#elif defined (LIBOHIBOARD_PIC24FJ)

#include "hardware/PIC24FJ/gpio_PIC24FJ.h"

#else

typedef enum _Gpio_Pins
{
    GPIO_PINS_NONE = 0,

} Gpio_Pins;

typedef enum _Gpio_Ports
{
    GPIO_PORTS_NONE,

} Gpio_Ports;

#endif

typedef enum
{
    GPIO_EVENT_NONE            = 0x00,

#if defined (LIBOHIBOARD_NXP_KINETIS)

    GPIO_EVENT_WHEN_0          = 0x08,
    GPIO_EVENT_ON_RISING       = 0x09,
    GPIO_EVENT_ON_FALLING      = 0x0A,
    GPIO_EVENT_WHEN_1          = 0x0C,
    GPIO_EVENT_ON_BOOTH        = 0x0B,

#elif defined (LIBOHIBOARD_ST_STM32)

    GPIO_EVENT_ON_RISING       = 0x01,
    GPIO_EVENT_ON_FALLING      = 0x02,

    GPIO_EVENT_USE_INTERRUPT   = 0x04,
    GPIO_EVENT_USE_EVENT       = 0x08,

#elif defined (LIBOHIBOARD_MICROCHIP_PIC)

    GPIO_EVENT_ON_RISING       = 0x01,
    GPIO_EVENT_ON_FALLING      = 0x02,

#endif
} Gpio_EventType;

/**
 * Enumeration for all possible alternate function of each pin
 *
 * @note For NXP microcontroller, Alternate 0 means pin disabled.
 */
typedef enum _Gpio_Alternate
{
#if defined(LIBOHIBOARD_PIC24FJ)
    GPIO_ALTERNATE_DIGITAL = -2,
#endif
    GPIO_ALTERNATE_ANALOG  = -1,
#if (defined (LIBOHIBOARD_STM32L0) || defined (LIBOHIBOARD_STM32L4))
    GPIO_ALTERNATE_0       = 0,
    GPIO_ALTERNATE_1       = 1,
    GPIO_ALTERNATE_2       = 2,
    GPIO_ALTERNATE_3       = 3,
    GPIO_ALTERNATE_4       = 4,
    GPIO_ALTERNATE_5       = 5,
    GPIO_ALTERNATE_6       = 6,
    GPIO_ALTERNATE_7       = 7,
#endif
#if defined (LIBOHIBOARD_STM32L4)
    GPIO_ALTERNATE_8       = 8,
    GPIO_ALTERNATE_9       = 9,
    GPIO_ALTERNATE_10      = 10,
    GPIO_ALTERNATE_11      = 11,
    GPIO_ALTERNATE_12      = 12,
    GPIO_ALTERNATE_13      = 13,
    GPIO_ALTERNATE_14      = 14,
    GPIO_ALTERNATE_15      = 15,
#endif

} Gpio_Alternate;

/**
 * Configure the selected pin with digital input or digital
 * output function.
 *
 * @param[in] pin The selected pins
 * @param[in] options Particular configurations for the pins,
 *                    check @ref GPIO_Pin_Option for all options.
 * @return ERRORS_NO_ERROR in case the configuration success.
 */
System_Errors Gpio_config (Gpio_Pins pin, uint16_t options);

/**
 * Configure the selected pin with alternate function.
 *
 * @param[in] pin The selected pins
 * @param[in] alternate The alternate functions
 * @param[in] options Particular configurations for the pins
 *   @arg @ref GPIO_PINS_PULL Enable pull-up/down
 *   @arg @ref GPIO_PINS_ENABLE_PULLUP Select pull-up
 *   @arg @ref GPIO_PINS_ENABLE_PULLDOWN Select pull-down
 *   @arg @ref GPIO_PINS_ENABLE_OUTPUT_PUSHPULL Configure output with push-pull
 *   @arg @ref GPIO_PINS_ENABLE_OUTPUT_OPENDRAIN Configure output with open-drain
 */
void Gpio_configAlternate (Gpio_Pins pin, Gpio_Alternate alternate, uint16_t options);

/**
 * Enable clock of specific port
 *
 * @param[in] port The Gpio port
 */
void Gpio_enablePortClock (Gpio_Ports port);

/**
 * Disable clock of specific port
 *
 * @param[in] port The Gpio port
 */
void Gpio_disablePortClock (Gpio_Ports port);

/**
 * @}
 */

/**
 * @defgroup GPIO_Manage_Functions GPIO read/write functions
 * @brief Functions to read and write on GPIO pins.
 * @{
 */

/**
 * This function configure the output pin to high-level value.
 *
 * @param[in] pin The selected pins
 */
void Gpio_set (Gpio_Pins pin);

/**
 * This function configure the output pin to low-level value.
 *
 * @param[in] pin The selected pins
 */
void Gpio_clear (Gpio_Pins pin);

/**
 * This function toggle the output pin level.
 *
 * @param[in] pin The selected pins
 */
void Gpio_toggle (Gpio_Pins pin);

/**
 * This function read the input pin level value.
 *
 * @param[in] pin The selected pins
 * @return @ref GPIO_LOW in case the input pis is low, @ref GPIO_HIGH otherwise.
 */
Gpio_Level Gpio_get (Gpio_Pins pin);

/**
 * @}
 */

/**
 * @defgroup GPIO_Interrupt_Functions GPIO interrupt functions
 * @brief Functions to manage on GPIO pins interrupt.
 * @{
 */

/**
 * This function configure the ISR function for a specific pin.
 *
 * @param[in] pin The selected pin
 * @param[in] priority The selected priority for interrupt
 * @param[in] callback The selected callback for the specific event on the pin.
 * @return The function return these possible value:
 *         @arg @ref ERRORS_GPIO_WRONG_PIN when the selected pin doesn't exist.
 *         @arg @ref ERRORS_GPIO_WRONG_PORT when the selected port doesn't support interrupt (only for NXP).
 *         @arg @ref ERRORS_NO_ERROR in case of success.
 */
System_Errors Gpio_configInterrupt (Gpio_Pins pin, Interrupt_Priority priority, void* callback);

/**
 * This function enable interrupt on selected pin at the specified event.
 *
 * @param[in] pin The selected pin
 * @param[in] event The selected event that generate interrupt.
 * @return The function return these possible value:
 *         @arg @ref ERRORS_GPIO_WRONG_PIN when the selected pin doesn't exist.
 *         @arg @ref ERRORS_GPIO_WRONG_PORT when the selected port doesn't support interrupt (only for NXP).
 *         @arg @ref ERRORS_GPIO_WRONG_CONFIG when the configuration is wrong (only for STM).
 *         @arg @ref ERRORS_NO_ERROR in case of success.
 */
System_Errors Gpio_enableInterrupt (Gpio_Pins pin, Gpio_EventType event);

Gpio_EventType Gpio_getEnabledInterrupt (Gpio_Pins pin);

/**
 * This function enable interrupt on selected pin at the specified event.
 *
 * @param[in] pin The selected pin
 * @return The function return these possible value:
 *         @arg @ref ERRORS_GPIO_WRONG_PIN when the selected pin doesn't exist.
 *         @arg @ref ERRORS_GPIO_WRONG_PORT when the selected port doesn't support interrupt (only for NXP).
 *         @arg @ref ERRORS_NO_ERROR in case of success.
 */
System_Errors Gpio_disableInterrupt (Gpio_Pins pin);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __GPIO_H

/**
 * @}
 */

/**
 * @}
 */
