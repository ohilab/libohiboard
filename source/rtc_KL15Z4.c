/******************************************************************************
 * Copyright (C) 2012-2016 A. C. Open Hardware Ideas Lab
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
 ******************************************************************************/

/**
 * @file libohiboard/source/rtc_KL15Z4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief RTC implementations for KL15Z4.
 */

#ifdef LIBOHIBOARD_RTC

#include "platforms.h"
#include "rtc.h"
#include "clock.h"

#if defined (LIBOHIBOARD_KL15Z4)     || \
      defined (LIBOHIBOARD_KL25Z4)   || \
      defined (LIBOHIBOARD_FRDMKL25Z)

typedef struct Rtc_Device {
    RTC_MemMapPtr regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    void (*isrAlarm)(void);          /**< The function pointer for Alarm ISR. */
    void (*callbackAlarm)(void);    /**< The pointer for user alarm callback. */
    Interrupt_Vector isrAlarmNumber;            /**< ISR Alarm vector number. */

    void (*isrSecond)(void);        /**< The function pointer for Second ISR. */
    void (*callbackSecond)(void);  /**< The pointer for user second callback. */
    Interrupt_Vector isrSecondNumber;          /**< ISR Second vector number. */

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Rtc_Device;

static Rtc_Device rtc0 = {
        .regMap           = RTC_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_RTC_MASK,

        .isrAlarm         = RTC_IRQHandler,
        .callbackAlarm    = 0,
        .isrAlarmNumber   = INTERRUPT_RTC_ALARM,

        .isrSecond        = RTC_Seconds_IRQHandler,
        .callbackSecond   = 0,
        .isrSecondNumber  = INTERRUPT_RTC_SECOND,
};
Rtc_DeviceHandle OB_RTC0 = &rtc0;

void RTC_IRQHandler (void)
{
    OB_RTC0->callbackAlarm();
    /* Clear alarm flag */
    RTC_TAR_REG(OB_RTC0->regMap) = 0;
}

void RTC_Seconds_IRQHandler (void)
{
    OB_RTC0->callbackSecond();
}

System_Errors Rtc_init (Rtc_DeviceHandle dev, Rtc_Config *config)
{
    RTC_MemMapPtr regmap = dev->regMap;

    /* Turn on clock */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    switch(config->clockSource)
    {
    case RTC_CLOCK_CLKIN:
        /* Activate mux for this pin */
        PORTC_PCR1 &= ~PORT_PCR_MUX_MASK;
        PORTC_PCR1 = PORT_PCR_MUX(1);
        /* Select this clock source */
        SIM_SOPT1 &= ~SIM_SOPT1_OSC32KSEL_MASK;
        SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(2);
        break;
    case RTC_CLOCK_SYSTEM:
        /* Select this clock source */
        SIM_SOPT1 &= ~SIM_SOPT1_OSC32KSEL_MASK;
        SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(0);
        break;
    case RTC_CLOCK_LPO:
        SIM_SOPT1 &= ~SIM_SOPT1_OSC32KSEL_MASK;
        SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(3);
        break;
    }

    /* Configure TSR value to default */
    /* From RM: "TSR with zero is supported, but not recommended" */
    RTC_TSR_REG(regmap) = 1;

    if (config->callbackAlarm)
    {
        Rtc_enableAlarm(dev,config->callbackAlarm,config->alarm);
    }

    if (config->callbackSecond)
    {
        Rtc_enableSecond(dev,config->callbackSecond);
    }

    /* Enable counter */
    RTC_SR_REG(regmap) |= RTC_SR_TCE_MASK;

    return ERRORS_NO_ERROR;
}

/**
 * Set current time with Unix system format.
 * Unix format is the number of seconds since the start of the Unix epoch:
 * midnight UTC of January 1, 1970 (not counting leap seconds).
 */
void Rtc_setTime (Rtc_DeviceHandle dev, Rtc_Time time)
{
    /* Disable counter */
    RTC_SR_REG(dev->regMap) &= ~RTC_SR_TCE_MASK;

    if (time == 0)
        RTC_TSR_REG(dev->regMap) = 1;
    else
        RTC_TSR_REG(dev->regMap) = time;

    /* Enable counter */
    RTC_SR_REG(dev->regMap) |= RTC_SR_TCE_MASK;
}

Rtc_Time Rtc_getTime (Rtc_DeviceHandle dev)
{
    return RTC_TSR_REG(dev->regMap);
}

void Rtc_enableAlarm (Rtc_DeviceHandle dev, void *callback, Rtc_Time alarm)
{
    /* save callback */
    dev->callbackAlarm = callback;

    /* Set alarm value */
    RTC_TAR_REG(dev->regMap) = alarm;

    RTC_IER_REG(dev->regMap) |= RTC_IER_TAIE_MASK;
    Interrupt_enable(dev->isrAlarmNumber);
}

void Rtc_disableAlarm (Rtc_DeviceHandle dev)
{
    /* Set alarm value */
    RTC_TAR_REG(dev->regMap) = 0;
    RTC_IER_REG(dev->regMap) &= ~RTC_IER_TAIE_MASK;
    Interrupt_disable(dev->isrAlarmNumber);
}

void Rtc_enableSecond (Rtc_DeviceHandle dev, void *callback)
{
    /* save callback */
    dev->callbackSecond = callback;

    RTC_IER_REG(dev->regMap) |= RTC_IER_TSIE_MASK;
    Interrupt_enable(dev->isrSecondNumber);
}

void Rtc_disableSecond (Rtc_DeviceHandle dev)
{
    RTC_IER_REG(dev->regMap) &= ~RTC_IER_TSIE_MASK;
    Interrupt_disable(dev->isrSecondNumber);
}

#endif /* LIBOHIBOARD_KL15Z4 */

#endif /* LIBOHIBOARD_RTC */
