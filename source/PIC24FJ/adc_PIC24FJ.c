/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/PIC24FJ/adc_PIC24FJ.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief ADC implementations for PIC24FJ series.
 */

#ifdef LIBOHIBOARD_ADC

#ifdef __cplusplus
extern "C" {
#endif

#include "platforms.h"

#if defined (LIBOHIBOARD_PIC24FJ)

#include "adc.h"

#include "utility.h"
#include "gpio.h"
#include "clock.h"
#include "interrupt.h"

/**
 * @brief Enable the ADC peripheral
 */
#define ADC_DEVICE_ENABLE(REGMAP)        (REGMAP->AD1CON1 |= _AD1CON1_ADON_MASK)
/**
 * @brief Disable the ADC peripheral
 */
#define ADC_DEVICE_DISABLE(REGMAP)       (REGMAP->AD1CON1 &= ~_AD1CON1_ADON_MASK)

#define ADC_VALID_CLOCK_SOURCE(SOURCE) (((SOURCE) == ADC_CLOCKSOURCE_DEDICATED) || \
                                        ((SOURCE) == ADC_CLOCKSOURCE_SYSCLOCK))

#define ADC_VALID_RESOLUTION(RESOLUTION) (((RESOLUTION) == ADC_RESOLUTION_12BIT) || \
                                          ((RESOLUTION) == ADC_RESOLUTION_10BIT))

#define ADC_VALID_VOLTAGE_REFERENCE(REFERENCE) (((REFERENCE) == ADC_VOLTREFERENCE_VREF) || \
                                                ((REFERENCE) == ADC_VOLTREFERENCE_AVDD))
    
#define ADC_VALID_CONVERSION_TYPE(TYPE) ((TYPE) == ADC_INPUTTYPE_SINGLE_ENDED)
    
#define ADC_VALID_INTERNAL_CHANNEL(CHANNEL) (((CHANNEL) == ADC_CHANNELS_BANDGAP) || \
                                             ((CHANNEL) == ADC_CHANNELS_AVSS)    || \
                                             ((CHANNEL) == ADC_CHANNELS_AVDD))

#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGB606)
    
#define ADC_MAX_PINS                     16
    
#elif defined (LIBOHIBOARD_PIC24FJxGA610) || \
      defined (LIBOHIBOARD_PIC24FJxGB610)

#define ADC_MAX_PINS                     24

#endif

typedef struct _Adc_Device
{
    ADC_TypeDef* regmap;                           /**< Device memory pointer */
    ANCFG_TypeDef* regmapANCFG;               /**< ANCFG block memory pointer */

    volatile uint16_t* pmdRegisterPtr;     /**< Register for device enabling. */
    uint16_t pmdRegisterEnable;        /**< Register mask for current device. */
    
    Adc_Pins pins[ADC_MAX_PINS];      /**< List of the pin for the peripheral */
    Adc_Channels pinsChannel[ADC_MAX_PINS];
    Gpio_Pins pinsGpio[ADC_MAX_PINS];

    uint32_t samplingTime;
    
    Interrupt_Vector isrNumber;

    Adc_DeviceState state;                      /**< Current peripheral state */
    Adc_Config config;                                /**< User configuration */

} Adc_Device;

#if defined (LIBOHIBOARD_PIC24FJxGA606) || \
    defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB606) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)


#define ADC_IS_DEVICE(DEVICE) (((DEVICE) == OB_ADC1))

static Adc_Device adc1 =
{
        .regmap              = ADC1,
        .regmapANCFG         = ANCFG1,

        .pmdRegisterPtr      = &PMD->PMD1,
        .pmdRegisterEnable   = _PMD1_AD1MD_MASK,

        .pins                =
        {
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
                               ADC_PINS_PA6,
                               ADC_PINS_PA7,
#endif
                               ADC_PINS_PB0,
                               ADC_PINS_PB1,
                               ADC_PINS_PB2,
                               ADC_PINS_PB3,
                               ADC_PINS_PB4,
                               ADC_PINS_PB5,
                               ADC_PINS_PB6,
                               ADC_PINS_PB7,
                               ADC_PINS_PB8,
                               ADC_PINS_PB9,
                               ADC_PINS_PB10,
                               ADC_PINS_PB11,
                               ADC_PINS_PB12,
                               ADC_PINS_PB13,
                               ADC_PINS_PB14,
                               ADC_PINS_PB15,
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
                               ADC_PINS_PC4,

                               ADC_PINS_PE9,

                               ADC_PINS_PG6,
                               ADC_PINS_PG7,
                               ADC_PINS_PG8,
                               ADC_PINS_PG9,
#endif
        },
        .pinsChannel         =
        {
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
                               ADC_CHANNELS_CH23,
                               ADC_CHANNELS_CH22,
#endif
                               ADC_CHANNELS_CH0,
                               ADC_CHANNELS_CH1,
                               ADC_CHANNELS_CH2,
                               ADC_CHANNELS_CH3,
                               ADC_CHANNELS_CH4,
                               ADC_CHANNELS_CH5,
                               ADC_CHANNELS_CH6,
                               ADC_CHANNELS_CH7,
                               ADC_CHANNELS_CH8,
                               ADC_CHANNELS_CH9,
                               ADC_CHANNELS_CH10,
                               ADC_CHANNELS_CH11,
                               ADC_CHANNELS_CH12,
                               ADC_CHANNELS_CH13,
                               ADC_CHANNELS_CH14,
                               ADC_CHANNELS_CH15,
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
                               ADC_CHANNELS_CH16,

                               ADC_CHANNELS_CH21,

                               ADC_CHANNELS_CH17,
                               ADC_CHANNELS_CH18,
                               ADC_CHANNELS_CH19,
                               ADC_CHANNELS_CH20,
#endif
        },
        .pinsGpio            =
        {
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
                               GPIO_PINS_PA6,
                               GPIO_PINS_PA7,
#endif
                               GPIO_PINS_PB0,
                               GPIO_PINS_PB1,
                               GPIO_PINS_PB2,
                               GPIO_PINS_PB3,
                               GPIO_PINS_PB4,
                               GPIO_PINS_PB5,
                               GPIO_PINS_PB6,
                               GPIO_PINS_PB7,
                               GPIO_PINS_PB8,
                               GPIO_PINS_PB9,
                               GPIO_PINS_PB10,
                               GPIO_PINS_PB11,
                               GPIO_PINS_PB12,
                               GPIO_PINS_PB13,
                               GPIO_PINS_PB14,
                               GPIO_PINS_PB15,
#if defined (LIBOHIBOARD_PIC24FJxGA610) || \
    defined (LIBOHIBOARD_PIC24FJxGB610)
                               GPIO_PINS_PC4,

                               GPIO_PINS_PE9,

                               GPIO_PINS_PG6,
                               GPIO_PINS_PG7,
                               GPIO_PINS_PG8,
                               GPIO_PINS_PG9,
#endif
        },
        
        .isrNumber           = INTERRUPT_ADC1,

        .state               = ADC_DEVICESTATE_RESET,
};
Adc_DeviceHandle OB_ADC1 = &adc1;

#endif

System_Errors Adc_init (Adc_DeviceHandle dev, Adc_Config* config)
{
    System_Errors err = ERRORS_NO_ERROR;
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    err = ohiassert(ADC_IS_DEVICE(dev));
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    err  = ohiassert(ADC_VALID_RESOLUTION(config->resolution));
    err |= ohiassert(ADC_VALID_CLOCK_SOURCE(config->clockSource));
    err |= ohiassert(ADC_VALID_VOLTAGE_REFERENCE(config->voltReference));
    err |= ohiassert(config->prescaler < 0x0100);
    if (err != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_PARAM;
    }
    // Save configuration
    dev->config = *config;

    // Enable peripheral clock if needed
    if (dev->state == ADC_DEVICESTATE_RESET)
    {
        // Enable peripheral
        UTILITY_CLEAR_REGISTER_BIT(*dev->pmdRegisterPtr, dev->pmdRegisterEnable);
    }
    dev->state = ADC_DEVICESTATE_BUSY;
    
    // Select voltage reference
    dev->regmap->AD1CON2 &= ~(_AD1CON2_PCVFG0_MASK | _AD1CON2_PCVFG1_MASK | _AD1CON2_NVCFG0_MASK);
    dev->regmap->AD1CON2 |= config->voltReference;
    
    // Select clock source
    dev->regmap->AD1CON3 &= ~(_AD1CON3_ADRC_MASK);
    dev->regmap->AD1CON3 |= config->clockSource;
    
    // Configure bit resolution
    dev->regmap->AD1CON1 &= ~(_AD1CON1_MODE12_MASK);
    dev->regmap->AD1CON1 |= config->resolution;
    
    // Set T_AD value
    // Clear LSB part of the register
    dev->regmap->AD1CON3 &= 0xFF00;
    dev->regmap->AD1CON3 |= config->prescaler;
    
    // Clear other registers
    dev->regmap->AD1CSSL = 0u;
    dev->regmap->AD1CHS = 0;
    dev->regmapANCFG->ANCFG = 0;
    
    dev->state = ADC_DEVICESTATE_READY;
    return ERRORS_NO_ERROR;
}

System_Errors Adc_deInit (Adc_DeviceHandle dev)
{
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

//    // Stop on-going conversion...
//    Adc_stop(dev);

    // TODO: disable interrupt

    // TODO: Clear interrupt flags

    // clear registers
    dev->regmap->AD1CON1 = 0;
    dev->regmap->AD1CON2 = 0;
    dev->regmap->AD1CON3 = 0;
    //dev->regmap->AD1CON4 = 0;
    dev->regmap->AD1CON5 = 0;
    dev->regmap->AD1CSSL = 0u;
    dev->regmap->AD1CHS = 0;
    dev->regmapANCFG->ANCFG = 0;
    
    // Disable peripheral clock
    UTILITY_SET_REGISTER_BIT(*dev->pmdRegisterPtr, dev->pmdRegisterEnable);

    dev->state = ADC_DEVICESTATE_RESET;
    return ERRORS_NO_ERROR;
}

System_Errors Adc_configPin (Adc_DeviceHandle dev, Adc_ChannelConfig* config, Adc_Pins pin)
{
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    ohiassert(ADC_VALID_CONVERSION_TYPE(config->type));

    // Configure Pin
    Adc_Channels channel;
    if ((pin == ADC_PINS_INTERNAL) && (config->isInternal == TRUE))
    {
        ohiassert(ADC_VALID_INTERNAL_CHANNEL(config->channel));
        channel = config->channel;
    }
    else
    {
        // Search channel
        bool isPinFound = FALSE;
        for (uint16_t i = 0; i < ADC_MAX_PINS; ++i)
        {
            if (dev->pins[i] == pin)
            {
                Gpio_configAlternate(dev->pinsGpio[i],
                                     GPIO_ALTERNATE_ANALOG,
                                     0);
                channel = dev->pinsChannel[i];
                isPinFound = TRUE;
                break;
            }
        }
        if (isPinFound == FALSE)
        {
            dev->state = ADC_DEVICESTATE_ERROR;
            return ERRORS_ADC_NO_PIN_FOUND;
        }
    }

    // Save sampling time
    dev->samplingTime = config->samplingTime;
    // Clear channel bits, and set new channel number
    dev->regmap->AD1CHS &= ~(_AD1CHS_CH0SA_MASK);
    dev->regmap->AD1CHS |= channel;
    
    return ERRORS_NO_ERROR;
}

System_Errors Adc_start (Adc_DeviceHandle dev)
{
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    // Enable device
    ADC_DEVICE_ENABLE(dev->regmap);

    // FIXME: Right now the follow one it is the only version implemented!!
    if (dev->config.clockSource == ADC_CLOCKSOURCE_SYSCLOCK)
    {
        // Wait sampling time
        UTILITY_SET_REGISTER_BIT(dev->regmap->AD1CON1,_AD1CON1_SAMP_MASK);
        System_delay(dev->samplingTime);
        UTILITY_CLEAR_REGISTER_BIT(dev->regmap->AD1CON1,_AD1CON1_SAMP_MASK);
    }

    // The conversion starts...

    return ERRORS_NO_ERROR;
}

System_Errors Adc_stop (Adc_DeviceHandle dev)
{
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    // Disable device
    ADC_DEVICE_DISABLE(dev->regmap);
    
    return ERRORS_NO_ERROR;
}

uint32_t Adc_read (Adc_DeviceHandle dev)
{
    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    return dev->regmap->ADC1BUF[0];
}

System_Errors Adc_poll (Adc_DeviceHandle dev, uint32_t timeout)
{
    uint32_t tickstart = 0u;

    // Check the ADC device
    if (dev == NULL)
    {
        return ERRORS_ADC_NO_DEVICE;
    }
    // Check the ADC instance
    if (ohiassert(ADC_IS_DEVICE(dev)) != ERRORS_NO_ERROR)
    {
        return ERRORS_ADC_WRONG_DEVICE;
    }

    // check conversion status
    tickstart = System_currentTick();
    while ((dev->regmap->AD1CON1 & _AD1CON1_DONE_MASK) == 0u)
    {
        if ((System_currentTick() - tickstart) > timeout)
        {
            return ERRORS_ADC_TIMEOUT;
        }
    }

    // Single conversion or sequence conversions flag is up!
    return ERRORS_ADC_CONVERSION_DONE;
}

int32_t Adc_getTemperature (Adc_DeviceHandle dev, uint32_t data, uint32_t vref)
{
    // NOT USED
    return 0;
}

#endif // LIBOHIBOARD_PIC24FJ

#ifdef __cplusplus
}
#endif

#endif // LIBOHIBOARD_ADC
