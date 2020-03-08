/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
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
 * @file libohiboard/source/PIC24FJ/rtc_PIC24FJ.c
 * @author Leonardo Morichelli <leonardo.morichelli@gruppofilippetti.it>
 * @brief RTC implementations for PIC24FJ series.
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

#if defined (LIBOHIBOARD_PIC24FJ)

/**
 * Maximum timeout for RTC peripheral operation.
 * It is expressed in milli-second.
 */
#define RTC_MAX_TIMEOUT                 1000

#define RTC_MAX_APREDIV                 128
#define RTC_MAX_SPREDIV                 32768

#define RTC_TR_RESERVED_MASK            0x007F7F7FU
#define RTC_DR_RESERVED_MASK            0x00FFFF3FU

#define RTC_CLOCK_ENABLE(REG,MASK)      do { \
                                            UTILITY_CLEAR_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                        } while (0)

#define RTC_CLOCK_DISABLE(REG,MASK)     do { \
                                            UTILITY_SET_REGISTER_BIT(REG,MASK); \
                                            asm("nop"); \
                                            (void) UTILITY_READ_REGISTER_BIT(REG,MASK); \
                                        } while (0)

#define RTC_ENABLE()                    do { \
                                            UTILITY_SET_REGISTER_BIT(RTC->RTCCON1L,_RTCCON1L_RTCEN_MASK);\
                                        } while (0)

#define RTC_DISABLE()                   do { \
                                            UTILITY_CLEAR_REGISTER_BIT(RTC->RTCCON1L,_RTCCON1L_RTCEN_MASK);\
                                        } while (0)

#define RTC_RESET()                     do { \
                                            UTILITY_CLEAR_REGISTER_BIT(RTC->RTCCON1L,0xFFFF);\
                                        } while (0)

#define RTC_ENABLE_WRITE_PROTECTION()   do { \
                                            asm volatile("DISI #6");\
                                            asm volatile("MOV #NVMKEY, W1");\
                                            asm volatile("MOV #0x55, W2");\
                                            asm volatile("MOV W2, [W1]");\
                                            asm volatile("MOV #0xAA, W3");\
                                            asm volatile("MOV W3, [W1]");\
                                            asm volatile("BSET RTCCON1L, #11");\
                                        } while (0)

#define RTC_DISABLE_WRITE_PROTECTION()  do { \
                                            asm volatile("DISI #6");\
                                            asm volatile("MOV #NVMKEY, W1");\
                                            asm volatile("MOV #0x55, W2");\
                                            asm volatile("MOV W2, [W1]");\
                                            asm volatile("MOV #0xAA, W3");\
                                            asm volatile("MOV W3, [W1]");\
                                            asm volatile("BCLR RTCCON1L, #11");\
                                        } while (0)

#define RTC_IS_VALID_CLOCK_SOURCE(CLOCKSOURCE)                                     \
                                        (((CLOCKSOURCE) == RTC_CLOCK_SOSC)      || \
                                         ((CLOCKSOURCE) == RTC_CLOCK_LPRC)      || \
                                         ((CLOCKSOURCE) == RTC_CLOCK_PWRLCLK)   || \
                                         ((CLOCKSOURCE) == RTC_CLOCK_SYSCLK))

#define RTC_IS_VALID_OUTPUT(OUTPUT)                                                \
                                        (((OUTPUT) == RTC_OUTPUT_DISABLED)      || \
                                         ((OUTPUT) == RTC_OUTPUT_TIMESTAMP_A)   || \
                                         ((OUTPUT) == RTC_OUTPUT_POWER_CONTROL) || \
                                         ((OUTPUT) == RTC_OUTPUT_RTC_INPUT_CLK) || \
                                         ((OUTPUT) == RTC_OUTPUT_SECOND_CLK)    || \
                                         ((OUTPUT) == RTC_OUTPUT_ALARM_EVT))

#define RTC_IS_DEVICE(DEVICE)           (((DEVICE) == OB_RTC0))

typedef enum _Rtc_ValueType
{
    RTC_VALUE_TYPE_CALENDAR = 0,
    RTC_VALUE_TYPE_ALARM,
    RTC_VALUE_TYPE_TIMESTAMP,
} Rtc_ValueType;

typedef union
{
    struct
    {
        uint32_t time;
        uint32_t date;
    } __packed values;

    struct
    {
        //time
        uint16_t timel;
        uint16_t timeh;
        //date
        uint16_t datel;
        uint16_t dateh;
    } __packed registers;

    struct
    {
        //timel
        uint8_t __padding;
        uint8_t seconds;
        //timeh
        uint8_t minutes;
        uint8_t hours;
        //datel
        uint8_t wday;
        uint8_t mday;
        //dateh
        uint8_t month;
        uint8_t years;
    } __packed fields;

} __packed Rtc_Value;

typedef struct _Rtc_Device
{
    RTC_TypeDef* regmap;                           /**< Device memory pointer */
    volatile uint16_t* pmdRegister;        /**< Register for device enabling. */
    uint16_t pmdRegisterMask;          /**< Register mask for current device. */
    volatile uint16_t* intFlagRegister;
    uint16_t intFlagRegisterMask;
    volatile uint16_t* intEnRegister;
    uint16_t intEnRegisterMask;

    Rtc_Config config;                  /**< Selected clock source from user. */
    Rtc_DeviceState state;                     /**< Current peripheral state. */
} Rtc_Device;

static Rtc_Device rtc0 =
{
    .regmap                = RTC,
    .pmdRegister           = &PMD->PMD3,
    .pmdRegisterMask       = _PMD3_RTCCMD_MASK,
    .intFlagRegister       = &INTERRUPT->IFS[3],
    .intFlagRegisterMask   = _IFS3_RTCIF_MASK,
    .intEnRegister         = &INTERRUPT->IEC[3],
    .intEnRegisterMask     = _IEC3_RTCIE_MASK,

    .config = 
    {
        .clockSource       = RTC_CLOCK_LPRC,
        .outputSignal      = RTC_OUTPUT_DISABLED,
        .alarmMask         = RTC_MASK_ONCE_A_YEAR,
        .callbackAlarm     = NULL,
    },
    .state                 = RTC_DEVICESTATE_RESET,
};
Rtc_DeviceHandle OB_RTC0 = &rtc0;

static uint8_t Rtc_convertHexToBCD(uint8_t hexvalue)
{
    uint8_t bcdvalue;
    bcdvalue = (hexvalue / 10) << 4;
    bcdvalue = bcdvalue | (hexvalue % 10);
    return (bcdvalue);
}

static uint8_t Rtc_convertBCDToHex(uint8_t bcdvalue)
{
    uint8_t hexvalue;
    hexvalue = (((bcdvalue & 0xF0) >> 4)* 10) + (bcdvalue & 0x0F);
    return hexvalue;
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
static System_Errors Rtc_config (Rtc_DeviceHandle dev, Rtc_Config* config)
{
    System_Errors err = ERRORS_NO_ERROR;

    // Save current configuration
    dev->config = *config;

    // setup ClockSource
    UTILITY_MODIFY_REGISTER(dev->regmap->RTCCON2L, _RTCCON2L_CLKSEL_MASK, ((uint16_t)dev->config.clockSource << _RTCCON2L_CLKSEL_POSITION));

    // Setup Output
    UTILITY_MODIFY_REGISTER(dev->regmap->RTCCON1L, _RTCCON1L_OUTSEL_MASK, ((uint16_t)dev->config.outputSignal << _RTCCON1L_OUTSEL_POSITION));

    // Setup Alarm Mask
    UTILITY_MODIFY_REGISTER(dev->regmap->RTCCON1H, _RTCCON1H_AMASK_MASK, ((uint16_t)dev->config.alarmMask << _RTCCON1H_AMASK_POSITION));

    return err;
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
    err |= ohiassert(RTC_IS_VALID_OUTPUT(config->outputSignal));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_RTC_WRONG_PARAM;
    }

    // In reset condition, enable clock
    if (dev->state == RTC_DEVICESTATE_RESET)
    {
        // Enable peripheral clock
        RTC_CLOCK_ENABLE(*dev->pmdRegister, dev->pmdRegisterMask);
    }

    // Now the peripheral is disabled
    RTC_DISABLE_WRITE_PROTECTION();
    RTC_RESET();
    dev->state = RTC_DEVICESTATE_BUSY;    

    // configuration sequence
    err = Rtc_config(dev, config);
    Rtc_disableAlarm(dev);
    if (err != ERRORS_NO_ERROR)
    {
        dev->state = RTC_DEVICESTATE_ERROR;
        return ERRORS_RTC_INIT_FAILED;
    }

    // Now the peripheral is ready
    RTC_ENABLE();
    RTC_ENABLE_WRITE_PROTECTION();
    dev->state = RTC_DEVICESTATE_READY;

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

    // Disable peripheral clock
    RTC_CLOCK_DISABLE(*dev->pmdRegister,dev->pmdRegisterMask);

    // Now the peripheral is in the reset state
    dev->state = RTC_DEVICESTATE_RESET;
    return ERRORS_NO_ERROR;
}

static void Rtc_writeValue(Rtc_DeviceHandle dev, Rtc_ValueType type, Rtc_Time time)
{
    // Convert unix timestamp to Time and Date Structure
    Time_DateType mDate = {0};
    Time_TimeType mTime = {0};
    Time_unixtimeToTime(time,&mDate,&mTime);

    // Only dozen and unit for year
    uint16_t year = (mDate.year >= 2000) ? (mDate.year -= 2000) : (mDate.year -= 1900);
    mDate.year = year;

    Rtc_Value value =
    {
        .fields.seconds = Rtc_convertHexToBCD((uint8_t)mTime.seconds),
        .fields.hours = Rtc_convertHexToBCD((uint8_t)mTime.hours),
        .fields.minutes = Rtc_convertHexToBCD((uint8_t)mTime.minutes),
        .fields.mday = Rtc_convertHexToBCD((uint8_t)mDate.day),
        .fields.wday = Rtc_convertHexToBCD((uint8_t)mDate.wday),
        .fields.years = Rtc_convertHexToBCD((uint8_t)mDate.year),
        .fields.month = Rtc_convertHexToBCD((uint8_t)mDate.month),
    };

    // Write value
    switch(type)
    {
        case RTC_VALUE_TYPE_CALENDAR:
            UTILITY_WRITE_REGISTER(dev->regmap->TIMEL, value.registers.timel);
            UTILITY_WRITE_REGISTER(dev->regmap->TIMEH, value.registers.timeh);
            UTILITY_WRITE_REGISTER(dev->regmap->DATEL, value.registers.datel);
            UTILITY_WRITE_REGISTER(dev->regmap->DATEH, value.registers.dateh);
            break;
            
        case RTC_VALUE_TYPE_ALARM:
            UTILITY_WRITE_REGISTER(dev->regmap->ALMTIMEL, value.registers.timel);
            UTILITY_WRITE_REGISTER(dev->regmap->ALMTIMEH, value.registers.timeh);
            UTILITY_WRITE_REGISTER(dev->regmap->ALMDATEL, value.registers.datel);
            UTILITY_WRITE_REGISTER(dev->regmap->ALMDATEH, value.registers.dateh);
            break;
            
        default:            
        case RTC_VALUE_TYPE_TIMESTAMP:
            break;
    }
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

    // Now the peripheral is disabled
    RTC_DISABLE_WRITE_PROTECTION();
    RTC_DISABLE();
    dev->state = RTC_DEVICESTATE_BUSY;   

    Rtc_writeValue(dev, RTC_VALUE_TYPE_CALENDAR, time);

    // Initialization problems...
    if (err != ERRORS_NO_ERROR)
    {
        // Restore write-protection
        RTC_ENABLE_WRITE_PROTECTION();
        dev->state = RTC_DEVICESTATE_ERROR;

        return ERRORS_RTC_TIMEOUT;
    }

    // Now the peripheral is ready
    RTC_ENABLE();
    RTC_ENABLE_WRITE_PROTECTION();
    dev->state = RTC_DEVICESTATE_SET;

    return ERRORS_NO_ERROR;
}

Rtc_Time Rtc_getTime (Rtc_DeviceHandle dev)
{
    Time_DateType mDate = {0};
    Time_TimeType mTime = {0};

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

    // check the Synchronization Status bit
    if(UTILITY_READ_REGISTER_BIT(dev->regmap->RTCSTATL, _RTCSTATL_SYNC_MASK) != 0)
    {
        return 0;
    }

    // Read value
    Rtc_Value value =
    {
        .registers.timel = dev->regmap->TIMEL,
        .registers.timeh = dev->regmap->TIMEH,
        .registers.datel = dev->regmap->DATEL,
        .registers.dateh = dev->regmap->DATEH,
    };

    mTime.seconds = Rtc_convertBCDToHex(value.fields.seconds);
    mTime.hours = Rtc_convertBCDToHex(value.fields.hours);
    mTime.minutes = Rtc_convertBCDToHex(value.fields.minutes);
    mDate.day = Rtc_convertBCDToHex(value.fields.mday);
    mDate.wday = Rtc_convertBCDToHex(value.fields.wday);
    uint16_t year = Rtc_convertBCDToHex(value.fields.years);
    mDate.year = (year >= 70) ? (year + 1900) : (year + 2000);
    mDate.month = Rtc_convertBCDToHex(value.fields.month);

    return (Rtc_Time)Time_getUnixTime(&mDate,&mTime);
}

Rtc_DeviceState Rtc_getState (Rtc_DeviceHandle dev)
{
    return dev->state;
}

void Rtc_enableAlarm(Rtc_DeviceHandle dev, void *callback, Rtc_Time alarm)
{
    // Disable Alarm
    Rtc_disableAlarm(dev);
    
    if(callback != NULL)
    {
        // Save CallBack
        dev->config.callbackAlarm = callback;

        // Write Alarm Value and Enable Alarm
        RTC_DISABLE_WRITE_PROTECTION();
        Rtc_writeValue(dev, RTC_VALUE_TYPE_ALARM, alarm);        
        UTILITY_SET_REGISTER_BIT(dev->regmap->RTCCON1H, _RTCCON1H_ALRMEN_MASK);
        RTC_ENABLE_WRITE_PROTECTION();

        // Clear Interrupt Flag
        UTILITY_CLEAR_REGISTER_BIT(*dev->intFlagRegister, dev->intFlagRegisterMask);

        // Enable Interrupt
        UTILITY_SET_REGISTER_BIT(*dev->intEnRegister, dev->intEnRegisterMask);
    }
}

void Rtc_disableAlarm (Rtc_DeviceHandle dev)
{
    dev->config.callbackAlarm = NULL;

    //Enable Alarm
    RTC_DISABLE_WRITE_PROTECTION();
    UTILITY_CLEAR_REGISTER_BIT(dev->regmap->RTCCON1H, _RTCCON1H_ALRMEN_MASK);
    RTC_ENABLE_WRITE_PROTECTION();

    // Disable Interrupt
    UTILITY_CLEAR_REGISTER_BIT(*dev->intEnRegister, dev->intEnRegisterMask);

    // Clear Interrupt Flag
    UTILITY_CLEAR_REGISTER_BIT(*dev->intFlagRegister, dev->intFlagRegisterMask);
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _RTCCInterrupt( void )
{
	// RTCC callback function 
    if(rtc0.config.callbackAlarm != NULL)
    {
        rtc0.config.callbackAlarm();
    }

    // Clear Interrupt Flag
    UTILITY_CLEAR_REGISTER_BIT(*rtc0.intFlagRegister, rtc0.intFlagRegisterMask);
}

#endif // LIBOHIBOARD_STM32L4

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_RTC
