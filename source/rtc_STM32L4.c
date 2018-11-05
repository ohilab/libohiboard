/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2018 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
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
 * @file libohiboard/source/rtc_STM32L4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief RTC implementations for STM32L4 series.
 */

#ifdef LIBOHIBOARD_RTC

#ifdef __cplusplus
extern "C" {
#endif

#include "rtc.h"

#include "interrupt.h"
#include "clock.h"
#include "utility.h"
#include "system.h"

#if defined (LIBOHIBOARD_STM32L4)

#define RTC_CLOCK_ENABLE(REG,MASK)        do { \
                                            UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                          } while (0)

// FIXME: Clock disable!

/**
 * This define enable the write-protection system
 */
#define RTC_WRITE_PROTECTION_ENABLE(REGMAP)  do { \
                                               (REGMAP)->WPR = 0xFF;   \
                                             } while (0)

/**
 * This define disable the write-protection system
 */
#define RTC_WRITE_PROTECTION_DISABLE(REGMAP) do { \
                                               (REGMAP)->WPR = 0xCA;   \
                                               (REGMAP)->WPR = 0x53;   \
                                             } while (0)

#define RTC_IS_VALID_CLOCK_SOURCE(CLOCKSOURCE) (((CLOCKSOURCE) == RTC_CLOCK_HSE_RTC) || \
                                                ((CLOCKSOURCE) == RTC_CLOCK_LSE)     || \
                                                ((CLOCKSOURCE) == RTC_CLOCK_LSI))

#define RTC_IS_VALID_HOUR_FORMAT(FORMAT) (((FORMAT) == RTC_HOURFORMAT_12H) || \
                                          ((FORMAT) == RTC_HOURFORMAT_24H))

#define RTC_IS_VALID_OUTPUT_MODE(MODE) (((MODE) == RTC_OUTPUTMODE_DISABLE) || \
                                        ((MODE) == RTC_OUTPUTMODE_ALARM_A) || \
                                        ((MODE) == RTC_OUTPUTMODE_ALARM_B) || \
                                        ((MODE) == RTC_OUTPUTMODE_WAKEUP))

#define RTC_IS_VALID_OUTPUT_TYPE(TYPE) (((TYPE) == RTC_OUTPUTTYPE_PUSH_PULL)  || \
                                        ((TYPE) == RTC_OUTPUTTYPE_OPEN_DRAIN))

#define RTC_IS_VALID_OUTPUT_REMAP(REMAP) (((REMAP) == RTC_OUTPUTREMAP_DEFAULT)  || \
                                          ((REMAP) == RTC_OUTPUTREMAP_REMAP))

#define RTC_IS_VALID_OUTPUT_POLARITY(POL) (((POL) == GPIO_HIGH) || \
                                           ((POL) == GPIO_LOW))

#define RTC_IS_DEVICE(DEVICE) (((DEVICE) == OB_RTC0))

typedef struct _Rtc_Device
{
    RTC_TypeDef* regmap;                           /**< Device memory pointer */

    volatile uint32_t* rccRegisterPtr;      /**< Register for clock enabling. */
    uint32_t rccRegisterEnable;        /**< Register mask for current device. */

    volatile uint32_t* rccTypeRegisterPtr; /**< Register for clock selection. */
    uint32_t rccTypeRegisterMask;      /**< Register mask for user selection. */
    uint32_t rccTypeRegisterPos;       /**< Mask position for user selection. */

//    void (*isrAlarm)(void);          /**< The function pointer for Alarm ISR. */
//    void (*callbackAlarm)(void);    /**< The pointer for user alarm callback. */
//    Interrupt_Vector isrAlarmNumber;            /**< ISR Alarm vector number. */
//
//    void (*isrSecond)(void);        /**< The function pointer for Second ISR. */
//    void (*callbackSecond)(void);  /**< The pointer for user second callback. */
//    Interrupt_Vector isrSecondNumber;          /**< ISR Second vector number. */

    Rtc_ClockSource clockSource;          /**< Selected clock source from user. */

    Rtc_HourFormat hourFormat;

    Rtc_OutputMode outputMode;
    Rtc_OutputType outputType;
    Rtc_OutputRemap outputRemap;
    Gpio_Level outputPolarity;

    Rtc_DeviceState state;                       /**< Current peripheral state. */

} Rtc_Device;

static Rtc_Device rtc0 =
{
    .regmap              = RTC,

    .rccRegisterPtr      = &RCC->BDCR,
    .rccRegisterEnable   = RCC_BDCR_RTCEN,

    .rccTypeRegisterPtr  = &RCC->BDCR,
    .rccTypeRegisterMask = RCC_BDCR_RTCSEL,
    .rccTypeRegisterPos  = RCC_BDCR_RTCSEL_Pos,

//        .isrAlarm         = RTC_IRQHandler,
//        .callbackAlarm    = 0,
//        .isrAlarmNumber   = INTERRUPT_RTC_ALARM,
//
//        .isrSecond        = RTC_Seconds_IRQHandler,
//        .callbackSecond   = 0,
//        .isrSecondNumber  = INTERRUPT_RTC_SECOND,

    .state               = RTC_DEVICESTATE_RESET,
};
Rtc_DeviceHandle OB_RTC0 = &rtc0;

static inline System_Errors __attribute__((always_inline)) Rtc_setInitMode (Rtc_DeviceHandle dev)
{
    uint32_t timeout = 0;

    // Check if the device is just in initialization mode
    if ((dev->regmap->ISR & RTC_ISR_INITF) == 0)
    {
        // Setup the timeout value
        timeout = System_currentTick() + 1000;
        // Write the initialization bit
        UTILITY_SET_REGISTER_BIT(dev->regmap->ISR,RTC_ISR_INIT);

        // Wait until the RTC is initialization mode
        while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,RTC_ISR_INITF) == 0)
        {
            if (System_currentTick() > timeout)
            {
                return ERRORS_RTC_TIMEOUT;
            }
        }
    }

    return ERRORS_NO_ERROR;
}

System_Errors Rtc_init (Rtc_DeviceHandle dev, Rtc_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the UART device
    if (dev == NULL)
    {
        return ERRORS_RTC_NO_DEVICE;
    }
    // Check the UART instance
    err = ohiassert(RTC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_RTC_WRONG_DEVICE;
    }
    // Check clock source selections and other configuration parameters
    err  = ohiassert(RTC_IS_VALID_CLOCK_SOURCE(config->clockSource));
    err |= ohiassert(RTC_IS_VALID_HOUR_FORMAT(config->hourFormat));
    err |= ohiassert(RTC_IS_VALID_OUTPUT_MODE(config->outputMode));
    err |= ohiassert(RTC_IS_VALID_OUTPUT_TYPE(config->outputType));
    err |= ohiassert(RTC_IS_VALID_OUTPUT_REMAP(config->outputRemap));
    err |= ohiassert(RTC_IS_VALID_OUTPUT_POLARITY(config->outputPolarity));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_RTC_WRONG_PARAM;
    }
    // Save configuration parameters
    dev->clockSource = config->clockSource;
    dev->hourFormat = config->hourFormat;
    dev->outputMode = config->outputMode;
    dev->outputType = config->outputType;
    dev->outputRemap = config->outputRemap;
    dev->outputPolarity = config->outputPolarity;

    // In reset condition, enable clock
    if (dev->state == RTC_DEVICESTATE_RESET)
    {
        // Select clock source
        UTILITY_MODIFY_REGISTER(*dev->rccTypeRegisterPtr,dev->rccTypeRegisterMask,(config->clockSource << dev->rccTypeRegisterPos));
        // Enable peripheral clock
        RTC_CLOCK_ENABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);
    }

    // Now the peripheral is busy
    dev->state = RTC_DEVICESTATE_BUSY;

    // Disable write-protection
    RTC_WRITE_PROTECTION_DISABLE(dev->regmap);

    // Launch configuration sequence
    err = Rtc_setInitMode(dev);
    // Configuration problems...
    if (err != ERRORS_NO_ERROR)
    {
        // Restore write-protection
        RTC_WRITE_PROTECTION_ENABLE(dev->regmap);
        dev->state = RTC_DEVICESTATE_ERROR;

        return ERRORS_RTC_INIT_FAILED;

    }
    // Configure the peripheral
    else
    {

    }

    return ERRORS_NO_ERROR;
}

System_Errors Rtc_deInit (Rtc_DeviceHandle dev)
{

    return ERRORS_NO_ERROR;
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_RTC
