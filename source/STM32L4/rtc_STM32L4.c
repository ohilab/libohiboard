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

#include "timeday.h"

#include "interrupt.h"
#include "clock.h"
#include "utility.h"
#include "system.h"

#if defined (LIBOHIBOARD_STM32L4)

/**
 * Maximum timeout for RTC peripheral operation.
 * It is expressed in milli-second.
 */
#define RTC_MAX_TIMEOUT                   1000

#define RTC_MAX_APREDIV                   128
#define RTC_MAX_SPREDIV                   32768

#define RTC_TR_RESERVED_MASK              0x007F7F7FU
#define RTC_DR_RESERVED_MASK              0x00FFFF3FU

#define RTC_CLOCK_ENABLE(REG,MASK)        do { \
                                            UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                          } while (0)

#define RTC_CLOCK_DISABLE(REG,MASK)       do { \
                                            UTILITY_CLEAR_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                          } while (0)

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

/**
 * This function enter into initialization mode.
 *
 * From RM0351 rev6 document, page 1226, the sequence is:
 * @li Set INIT bit to 1 in the RTC_ISR register to enter initialization mode.
 * In this mode, the calendar counter is stopped and its value can be updated.
 * @li Poll INITF bit of in the RTC_ISR register. The initialization phase mode
 * is entered when INITF is set to 1. It takes around 2 RTCCLK clock cycles
 * (due to clock synchronization).
 *
 * @param[in] dev Rtc device handle
 *
 */
static inline System_Errors __attribute__((always_inline)) Rtc_enterInitialization (Rtc_DeviceHandle dev)
{
    uint32_t timeout = 0;

    // Check if the device is just in initialization mode
    if ((dev->regmap->ISR & RTC_ISR_INITF) == 0)
    {
        // Setup the timeout value
        timeout = System_currentTick() + RTC_MAX_TIMEOUT;
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

/**
 * This function exit from initialization mode.
 *
 * From RM0351 rev6 document, page 1226, the exit from initialization is done by
 * clearing the INIT bit. The actual calendar counter value is then automatically
 * loaded and the counting restarts after 4 RTCCLK clock cycles.
 *
 * @param[in] dev Rtc device handle
 */
static inline void __attribute__((always_inline)) Rtc_exitInitialization (Rtc_DeviceHandle dev)
{
    // Clear the initialization bit
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->ISR,RTC_ISR_INIT);
}

/**
 * @param[in] dev Rtc device handle
 */
static inline System_Errors __attribute__((always_inline)) Rtc_waitSynchronization (Rtc_DeviceHandle dev)
{
    uint32_t timeout = 0;

    // Clear RSF flag
    dev->regmap->ISR &= RTC_ISR_RSF;

    // Setup the timeout value
    timeout = System_currentTick() + RTC_MAX_TIMEOUT;

    // Wait until hardware set this bit
    // This bit was set each time the calendar registers are copied into the shadow registers
    while (UTILITY_READ_REGISTER_BIT(dev->regmap->ISR,RTC_ISR_RSF) == 0)
    {
        if (System_currentTick() > timeout)
        {
            return ERRORS_RTC_TIMEOUT;
        }
    }

    return ERRORS_NO_ERROR;
}

/**
 * RTC configuration procedure.
 *
 * @li Exit the initialization mode by clearing the INIT bit. The actual
 * calendar counter value is then automatically loaded and the counting
 * restarts after 4 RTCCLK clock cycles.
 *
 * @param[in] dev Rtc device handle
 */
static System_Errors Rtc_config (Rtc_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Disable write-protection
    RTC_WRITE_PROTECTION_DISABLE(dev->regmap);

    err = Rtc_enterInitialization(dev);
    // Initialization problems...
    if (err != ERRORS_NO_ERROR)
    {
        return err;
    }

    // Clear output configuration for ALARM pin
    // Clear hour format configuration
    dev->regmap->CR = dev->regmap->CR & (~(RTC_CR_FMT_Msk | RTC_CR_OSEL_Msk | RTC_CR_POL_Msk));
    dev->regmap->OR = dev->regmap->OR & (~(RTC_OR_ALARMOUTTYPE_Msk | RTC_OR_OUT_RMP_Msk));

    // Setup hour format
    if (dev->hourFormat == RTC_HOURFORMAT_12H)
    {
        dev->regmap->CR |= RTC_CR_FMT;
    }

    // Setup output pin
    if (dev->outputMode != RTC_OUTPUTMODE_DISABLE)
    {
        dev->regmap->CR |= ((dev->outputMode << RTC_CR_OSEL_Pos) | ((dev->outputPolarity == GPIO_LOW) ? RTC_CR_POL : 0 ));
    }
    dev->regmap->OR = ((dev->outputRemap << RTC_OR_OUT_RMP_Pos) | (dev->outputType << RTC_OR_ALARMOUTTYPE_Pos));

    // TODO: Compute and setup clock prescaler
    uint32_t frequency = 0;
    // Get current parent clock
    switch (dev->clockSource)
    {
    case RTC_CLOCK_LSI:
        frequency = (uint32_t)CLOCK_FREQ_LSI;
        break;
    case RTC_CLOCK_LSE:
        frequency = (uint32_t)CLOCK_FREQ_LSE;
        break;
    case RTC_CLOCK_HSE_RTC:
        frequency = (uint32_t)(Clock_getOscillatorValue() / 32u);
        break;
    }
    // Check if frequency is grather than 32000Hz, in this case put
    // asynchronous divider to the maximum value (128)
    if (frequency >= CLOCK_FREQ_LSI)
    {
        // Compute divided frequency
        frequency = frequency / (RTC_MAX_APREDIV - 1);

        if ((frequency > RTC_MAX_SPREDIV) || (frequency == 0))
        {
            // FIXME: Manage error
        }

        dev->regmap->PRER = (uint32_t)(frequency - 1);
        dev->regmap->PRER |= (uint32_t)((RTC_MAX_APREDIV - 1) << 16);
    }
    else
    {
        // FIXME!
    }

    // Exit from configuration mode
    Rtc_exitInitialization(dev);

    // Check Bypass-shadow bit to wait or not the synchronization
    // Question: who set this bit?
    if (UTILITY_READ_REGISTER_BIT(dev->regmap->CR,RTC_CR_BYPSHAD) == 0)
    {
        err = Rtc_waitSynchronization(dev);
        if (err != ERRORS_NO_ERROR)
        {
            return err;
        }
    }

    return ERRORS_NO_ERROR;
}

System_Errors Rtc_init (Rtc_DeviceHandle dev, Rtc_Config *config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the RTC device
    if (dev == NULL)
    {
        return ERRORS_RTC_NO_DEVICE;
    }
    // Check the RTC instance
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

    // Enable RTC and backup domain write
    bool isPwrChanged = FALSE;
    if (CLOCK_IS_ENABLE_PWR() == FALSE)
    {
        CLOCK_ENABLE_PWR();
        isPwrChanged = TRUE;
    }

    // Disable write protection
    CLOCK_BACKUP_DISABLE_WRITE_PROTECTION();
    // Wait some cycle...
    asm("NOP");
    asm("NOP");

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

    // configuration sequence
    err = Rtc_config(dev);
    if (err != ERRORS_NO_ERROR)
    {
        // Restore write-protection
        RTC_WRITE_PROTECTION_ENABLE(dev->regmap);
        dev->state = RTC_DEVICESTATE_ERROR;

        return ERRORS_RTC_INIT_FAILED;
    }

    // Enable the write-protection
    RTC_WRITE_PROTECTION_ENABLE(dev->regmap);

    // Now the peripheral is ready
    dev->state = RTC_DEVICESTATE_READY;

    // Restore PWR status
    if (isPwrChanged == TRUE)
    {
        CLOCK_DISABLE_PWR();
    }

    return ERRORS_NO_ERROR;
}

System_Errors Rtc_deInit (Rtc_DeviceHandle dev)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the RTC device
    if (dev == NULL)
    {
        return ERRORS_RTC_NO_DEVICE;
    }
    // Check the RTC instance
    err = ohiassert(RTC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_RTC_WRONG_DEVICE;
    }

    // Now the peripheral is busy
    dev->state = RTC_DEVICESTATE_BUSY;

    // TODO!

    // Disable peripheral clock
    RTC_CLOCK_DISABLE(*dev->rccRegisterPtr,dev->rccRegisterEnable);

    // Now the peripheral is in the reset state
    dev->state = RTC_DEVICESTATE_RESET;
    return ERRORS_NO_ERROR;
}

System_Errors Rtc_setTime (Rtc_DeviceHandle dev, Rtc_Time time)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the RTC device
    if (dev == NULL)
    {
        return ERRORS_RTC_NO_DEVICE;
    }
    // Check the RTC instance
    err = ohiassert(RTC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_RTC_WRONG_DEVICE;
    }
    // Check the time value
    err = ohiassert(time > 0);
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_RTC_WRONG_PARAM;
    }

    dev->state = RTC_DEVICESTATE_BUSY;

    Time_DateType mydate;
    Time_TimeType mytime;
    bool isPM = FALSE;
    uint32_t tmpregTime = 0, tmpregDate = 0;

    // Convert unix timestamp to Time and Date Structure
    Time_unixtimeToTime(time,&mydate,&mytime);

    if (dev->hourFormat == RTC_HOURFORMAT_12H)
    {
        if (mytime.hours > 12)
        {
            mytime.hours -= 12;
            isPM = TRUE;
        }
    }

    tmpregTime = (((uint32_t)Utility_byteToBcd2(mytime.hours)   << 16) | \
                  ((uint32_t)Utility_byteToBcd2(mytime.minutes) << 8)  | \
                  ((uint32_t)Utility_byteToBcd2(mytime.seconds))       | \
                  (((dev->hourFormat == RTC_HOURFORMAT_12H) && (isPM)) ? RTC_TR_PM_Msk : 0 ));

    // Only dozen and unit for year
    mydate.year -= 2000;

    tmpregDate = (((uint32_t)Utility_byteToBcd2(mydate.year)  << 16) | \
                  ((uint32_t)Utility_byteToBcd2(mydate.month) << 8)  | \
                  ((uint32_t)Utility_byteToBcd2(mydate.day))         | \
                  ((uint32_t)Utility_byteToBcd2(mydate.wday)  << 13));

    // The values are ready, now enter in initialization mode
    // Disable protection mode
    RTC_WRITE_PROTECTION_DISABLE(dev->regmap);

    err = Rtc_enterInitialization(dev);
    // Initialization problems...
    if (err != ERRORS_NO_ERROR)
    {
        // Restore write-protection
        RTC_WRITE_PROTECTION_ENABLE(dev->regmap);
        dev->state = RTC_DEVICESTATE_ERROR;

        return ERRORS_RTC_TIMEOUT;
    }

    // Set DR and TR registers
    dev->regmap->DR = (uint32_t)(tmpregDate & RTC_DR_RESERVED_MASK);
    dev->regmap->TR = (uint32_t)(tmpregTime & RTC_TR_RESERVED_MASK);

    // Clear BCK bit into CR register... No day-light saving operation here.
    dev->regmap->CR &= ((uint32_t)~RTC_CR_BCK);

    // Exit from configuration mode
    Rtc_exitInitialization(dev);

    // Check Bypass-shadow bit to wait or not the synchronization
    // Question: who set this bit?
    if (UTILITY_READ_REGISTER_BIT(dev->regmap->CR,RTC_CR_BYPSHAD) == 0)
    {
        err = Rtc_waitSynchronization(dev);
        if (err != ERRORS_NO_ERROR)
        {
            // Restore write-protection
            RTC_WRITE_PROTECTION_ENABLE(dev->regmap);
            dev->state = RTC_DEVICESTATE_ERROR;

            return ERRORS_RTC_TIMEOUT;
        }
    }

    // Restore write-protection
    RTC_WRITE_PROTECTION_ENABLE(dev->regmap);
    dev->state = RTC_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

Rtc_Time Rtc_getTime (Rtc_DeviceHandle dev)
{
    Time_DateType mydate;
    Time_TimeType mytime;

    System_Errors err = ERRORS_NO_ERROR;
    // Check the RTC device
    if (dev == NULL)
    {
        return 0;
    }
    // Check the RTC instance
    err = ohiassert(RTC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return 0;
    }

    // Read TR and DR registers
    uint32_t tmpregTime = (uint32_t)(dev->regmap->TR & RTC_TR_RESERVED_MASK);
    uint32_t tmpregDate = (uint32_t)(dev->regmap->DR & RTC_DR_RESERVED_MASK);

    // Save values
    mytime.hours   = (uint8_t)((tmpregTime & (RTC_TR_HT_Msk  | RTC_TR_HU_Msk))  >> 16);
    mytime.minutes = (uint8_t)((tmpregTime & (RTC_TR_MNT_Msk | RTC_TR_MNU_Msk)) >> 8);
    mytime.seconds = (uint8_t)(tmpregTime  & (RTC_TR_ST_Msk  | RTC_TR_SU_Msk));
    // Convert BCD to binary
    mytime.hours   = Utility_bcd2ToByte(mytime.hours);
    mytime.minutes = Utility_bcd2ToByte(mytime.minutes);
    mytime.seconds = Utility_bcd2ToByte(mytime.seconds);

    // Check AM/PM status
    if (((tmpregTime & RTC_TR_PM) == RTC_TR_PM) && (dev->hourFormat == RTC_HOURFORMAT_12H))
    {
        mytime.hours += 12;
    }

    mydate.year  = (uint8_t)((tmpregDate & (RTC_DR_YT_Msk | RTC_DR_YU_Msk)) >> 16);
    mydate.month = (uint8_t)((tmpregDate & (RTC_DR_MT_Msk | RTC_DR_MU_Msk)) >> 8);
    mydate.day   = (uint8_t)(tmpregDate  & (RTC_DR_DT_Msk | RTC_DR_DU_Msk));
    // Convert BCD to binary
    mydate.year  = (uint16_t)Utility_bcd2ToByte((uint8_t)mydate.year) + 2000;
    mydate.month = Utility_bcd2ToByte(mydate.month);
    mydate.day   = Utility_bcd2ToByte(mydate.day);

    return (Rtc_Time)Time_getUnixTime(&mydate,&mytime);
}

void Rtc_enableAlam(Rtc_DeviceHandle dev, void *callback, Rtc_Time alarm)
{

}

void Rtc_disableAlarm (Rtc_DeviceHandle dev)
{

}

void Rtc_enableWakeUp(Rtc_DeviceHandle dev, void *callback, uint32_t seconds)
{

}

void Rtc_disableWakeUp (Rtc_DeviceHandle dev)
{

}


#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_RTC
