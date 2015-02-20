/******************************************************************************
 * Copyright (C) 2015 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/rtc_KL03Z4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief RTC implementations for KL03Z4 and FRDMKL03Z.
 */

#ifdef LIBOHIBOARD_RTC

#include "platforms.h"
#include "rtc.h"
#include "clock.h"

#if defined (LIBOHIBOARD_KL03Z4)     || \
    defined (LIBOHIBOARD_FRDMKL03Z)

typedef struct Rtc_Device {
    RTC_MemMapPtr regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Rtc_Device;

static Rtc_Device rtc0 = {
        .regMap           = RTC_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_RTC_MASK,
};
Rtc_DeviceHandle RTC0 = &rtc0;

System_Errors Rtc_init (Rtc_DeviceHandle dev, Rtc_Config *config)
{
    RTC_MemMapPtr regmap = dev->regMap;

    /* Turn on clock */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    switch(config->clockSource)
    {
    case RTC_CLKIN:
        /* Activate mux for this pin */
        PORTA_PCR5 &= ~PORT_PCR_MUX_MASK;
        PORTA_PCR5 = PORT_PCR_MUX(1);
        /* Select this clock source */
        SIM_SOPT1 &= ~SIM_SOPT1_OSC32KSEL_MASK;
        SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(2);
        break;
    case RTC_SYSTEM_OSCILLATOR:
        /* Select this clock source */
        SIM_SOPT1 &= ~SIM_SOPT1_OSC32KSEL_MASK;
        SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(0);
        break;
    case RTC_LPO_1kHz:
        /* TODO */
        break;
    }

    /* Configure TSR value to default */
    /* From RM: "TSR with zero is supported, but not recommended" */
    RTC_TSR_REG(regmap) = 1;

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

void Rtc_enableAlarm (Rtc_DeviceHandle dev, Rtc_Time alarm)
{
    /* Set alarm value */
    RTC_TAR_REG(dev->regMap) = alarm;

    RTC_IER_REG(dev->regMap) |= RTC_IER_TAIE_MASK;
    Interrupt_enable(INTERRUPT_RTC_ALARM);
}

void Rtc_disableAlarm (Rtc_DeviceHandle dev)
{
    /* Set alarm value */
    RTC_TAR_REG(dev->regMap) = 0;
    RTC_IER_REG(dev->regMap) &= ~RTC_IER_TAIE_MASK;
    Interrupt_disable(INTERRUPT_RTC_ALARM);
}

void Rtc_enableSecond (Rtc_DeviceHandle dev)
{
    RTC_IER_REG(dev->regMap) |= RTC_IER_TAIE_MASK;
    Interrupt_enable(INTERRUPT_RTC_SECOND);
}

void Rtc_disableSecond (Rtc_DeviceHandle dev)
{
    RTC_IER_REG(dev->regMap) &= ~RTC_IER_TSIE_MASK;
    Interrupt_disable(INTERRUPT_RTC_SECOND);
}

#endif /* LIBOHIBOARD_KL03Z4 || LIBOHIBOARD_FRDMKL03Z */

#endif /* LIBOHIBOARD_RTC */
