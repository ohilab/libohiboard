/* Copyright (C) 2012-2015 A. C. Open Hardware Ideas Lab
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
 ******************************************************************************/

/**
 * @file libohiboard/source/adc_KL15Z4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief ADC functions implementation.
 */

#ifdef LIBOHIBOARD_ADC

#include "platforms.h"
#include "interrupt.h"
#include "clock.h"
#include "adc.h"

#if defined (LIBOHIBOARD_KL15Z4)

#define ADC_PIN_ENABLED                  1
#define ADC_PIN_DISABLED                 0

#define ADC_MAX_PINS                     20

typedef struct Adc_Device {
    ADC_MemMapPtr regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    void (*isr)(void);                     /**< The function pointer for ISR. */
    void (*callback)(void);      /**< The function pointer for user callback. */
    Interrupt_Vector isrNumber;                       /**< ISR vector number. */

    Adc_Pins pins[ADC_MAX_PINS];    /**< List of the pin for the FTM channel. */
    volatile uint32_t* pinsPtr[ADC_MAX_PINS];
    Adc_ChannelNumber channelNumber[ADC_MAX_PINS];
    Adc_ChannelMux channelMux[ADC_MAX_PINS];
    uint8_t pinMux[ADC_MAX_PINS];     /**< Mux of the pin of the FTM channel. */

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
    /** Indicate that device require calibration on Adc_readValue. */
    uint8_t devCalibration;
} Adc_Device;

static Adc_Device adc0 = {
        .regMap           = ADC0_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_ADC0_MASK,

        .pins             = {ADC_PINS_PTE16,
                             ADC_PINS_PTE17,
                             ADC_PINS_PTE18,
                             ADC_PINS_PTE19,
        		             ADC_PINS_PTE20,
                             ADC_PINS_PTE21,
                             ADC_PINS_PTE22,
                             ADC_PINS_PTE23,
                             ADC_PINS_PTE29,
                             ADC_PINS_PTE30,
                             ADC_PINS_PTB0,
                             ADC_PINS_PTB1,
                             ADC_PINS_PTB2,
                             ADC_PINS_PTB3,
                             ADC_PINS_PTC0,
                             ADC_PINS_PTC1,
                             ADC_PINS_PTC2,
                             ADC_PINS_PTD1,
                             ADC_PINS_PTD5,
                             ADC_PINS_PTD6,
        },
        .pinsPtr          = {&PORTE_PCR16,
                             &PORTE_PCR17,
                             &PORTE_PCR18,
                             &PORTE_PCR19,
                             &PORTE_PCR20,
                             &PORTE_PCR21,
                             &PORTE_PCR22,
                             &PORTE_PCR23,
                             &PORTE_PCR29,
                             &PORTE_PCR30,
                             &PORTB_PCR0,
                             &PORTB_PCR1,
                             &PORTB_PCR2,
                             &PORTB_PCR3,
                             &PORTC_PCR0,
                             &PORTC_PCR1,
                             &PORTC_PCR2,
                             &PORTD_PCR1,
                             &PORTD_PCR5,
                             &PORTD_PCR6,
        },
        .pinMux           = {0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
        },
        .channelNumber    = {ADC_CH_SE1,
                             ADC_CH_SE5a,
                             ADC_CH_SE2,
                             ADC_CH_SE6a,
                             ADC_CH_SE0,
                             ADC_CH_SE4a,
                             ADC_CH_SE3,
                             ADC_CH_SE7a,
                             ADC_CH_SE4b,
                             ADC_CH_SE23,
                             ADC_CH_SE8,
                             ADC_CH_SE9,
                             ADC_CH_SE12,
                             ADC_CH_SE13,
                             ADC_CH_SE14,
                             ADC_CH_SE15,
                             ADC_CH_SE11,
                             ADC_CH_SE5b,
                             ADC_CH_SE6b,
                             ADC_CH_SE7b,
        },
        .channelMux       = {ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_B,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_A,
                             ADC_CHL_B,
                             ADC_CHL_B,
                             ADC_CHL_B,
        },

        .isr              = ADC0_IRQHandler,
        .isrNumber        = INTERRUPT_ADC0,

	    .devInitialized = 0,
	    .devCalibration = 0,
};
Adc_DeviceHandle OB_ADC0 = &adc0;

/**
 * This function initialize the ADC device and setup operational mode.
 *
 * @param dev Adc device handle to be synchronize.
 * @return A System_Errors elements that indicate the status of initialization.
 */
System_Errors Adc_init (Adc_DeviceHandle dev, void* callback, Adc_Config *config)
{
    ADC_MemMapPtr regmap = dev->regMap;
    System_Errors errore = ERRORS_NO_ERROR;
    uint8_t clkdiv = 0;

    if (dev->devInitialized) return ERRORS_ADC_DEVICE_JUST_INIT;

    /* Enable the clock to the selected ADC */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* Disable conversion */
    ADC_SC1_REG(regmap,0) = ADC_SC1_ADCH(ADC_CH_DISABLE);

    /* If call back exist save it */
    if (callback)
    {
        dev->callback = callback;
        /* Enable interrupt */
        Interrupt_enable(dev->isrNumber);
    }

    /*setting clock source and divider*/
    ADC_CFG1_REG(regmap) &= ~ADC_CFG1_ADIV_MASK;
    switch (config->clkDiv)
    {
    case 1:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADIV(0);
    	break;
    case 2:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADIV(1);
    	break;
    case 4:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADIV(2);
    	break;
    case 8:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADIV(3);
    	break;
    default:
    	return errore = ERRORS_ADC_DIVIDER_NOT_FOUND;
    }

    ADC_CFG1_REG(regmap) &= ~ADC_CFG1_ADICLK_MASK;
    switch (config->clkSource)
    {
    case ADC_BUS_CLOCK:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADICLK(0);
    	break;
    case ADC_BUS_CLOCK_DIV2:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADICLK(1);
    	break;
    case ADC_ALTERNATE_CLOCK:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADICLK(2);
    	break;
    case ADC_ASYNCHRONOUS_CLOCK:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADICLK(3);
    	break;
    }

    /*Setting Sample Time*/
    ADC_CFG1_REG(regmap) &= ~(ADC_CFG1_ADLSMP_MASK | ADC_CFG2_ADLSTS_MASK);
    switch (config->sampleLength)
    {
    case ADC_SHORT_SAMPLE:
    	ADC_CFG1_REG(regmap) &= ~(ADC_CFG1_ADLSMP_MASK);
    	break;
    case ADC_LONG_SAMPLE_20:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADLSMP_MASK;
    	ADC_CFG2_REG(regmap) |= ADC_CFG2_ADLSTS(0);
    	break;
    case ADC_LONG_SAMPLE_12:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADLSMP_MASK;
    	ADC_CFG2_REG(regmap) |= ADC_CFG2_ADLSTS(1);
    	break;
    case ADC_LONG_SAMPLE_6:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADLSMP_MASK;
    	ADC_CFG2_REG(regmap) |= ADC_CFG2_ADLSTS(2);
    	break;
    case ADC_LONG_SAMPLE_2:
    	ADC_CFG1_REG(regmap) |= ADC_CFG1_ADLSMP_MASK;
    	ADC_CFG2_REG(regmap) |= ADC_CFG2_ADLSTS(3);
    	break;
    }

    /*setting convertion speed*/
    switch (config->covertionSpeed)
    {
    case ADC_NORMAL_CONVERTION:
    	ADC_CFG2_REG(regmap) &= ~(ADC_CFG2_ADHSC_MASK);
    	break;
    case ADC_HIGH_SPEED_CONVERTION:
    	ADC_CFG2_REG(regmap) |= ADC_CFG2_ADHSC_MASK;
    	break;
    }

    /*setting single or continuous convertion*/
    switch (config->contConv)
    {
    case ADC_SINGLE_CONVERTION:
    	ADC_SC3_REG(regmap) &= ~(ADC_SC3_ADCO_MASK);
    	break;
    case ADC_CONTINUOUS_CONVERTION:
    	ADC_SC3_REG(regmap) |= ADC_SC3_ADCO_MASK;
    	break;
    }

    /*setting resoluton*/
    ADC_CFG1_REG(regmap) &= ~ADC_CFG1_MODE_MASK;
    switch (config->resolution)
    {
    case ADC_RESOLUTION_8BIT:
        ADC_CFG1_REG(regmap) |= ADC_CFG1_MODE(0);
        break;
    case ADC_RESOLUTION_10BIT:
        ADC_CFG1_REG(regmap) |= ADC_CFG1_MODE(2);
        break;
    case ADC_RESOLUTION_12BIT:
        ADC_CFG1_REG(regmap) |= ADC_CFG1_MODE(1);
        break;
    case ADC_RESOLUTION_16BIT:
        ADC_CFG1_REG(regmap) |= ADC_CFG1_MODE(3);
        break;
	}

    /* Select voltage reference*/
    ADC_SC2_REG(regmap) &= ~ADC_SC2_REFSEL_MASK;
    switch (config->voltRef)
    {
    case ADC_VREF:
    	ADC_SC2_REG(regmap) |= ADC_SC2_REFSEL(0);
        break;
    case ADC_VALT:
    	ADC_SC2_REG(regmap) |= ADC_SC2_REFSEL(1);
        break;
    }

    /* Select the average */
    ADC_SC3_REG(regmap) &= ~(ADC_SC3_AVGE_MASK | ADC_SC3_AVGS_MASK);
    switch (config->average)
    {
    case ADC_AVERAGE_1_SAMPLES:
        /* Nothing to do! */
    	ADC_SC3_REG(regmap) &= ~ADC_SC3_AVGE_MASK;
        break;
    case ADC_AVERAGE_4_SAMPLES:
        ADC_SC3_REG(regmap) |= ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0);
        break;
    case ADC_AVERAGE_8_SAMPLES:
        ADC_SC3_REG(regmap) |= ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(1);
        break;
    case ADC_AVERAGE_16_SAMPLES:
        ADC_SC3_REG(regmap) |= ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(2);
        break;
    case ADC_AVERAGE_32_SAMPLES:
        ADC_SC3_REG(regmap) |= ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);
        break;
    }

    if (config->enableHwTrigger)
        ADC_SC2_REG(dev->regMap) |= ADC_SC2_ADTRG_MASK;
    else
        ADC_SC2_REG(dev->regMap) &= ~ADC_SC2_ADTRG_MASK;

    /* Calibration flag */
    if (config->doCalibration)
        Adc_calibration(dev);

    dev->devInitialized = 1;

	return ERRORS_NO_ERROR;
}

void Adc_enablePin (Adc_DeviceHandle dev, Adc_Pins pin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < ADC_MAX_PINS; ++devPinIndex)
    {
        if (dev->pins[devPinIndex] == pin)
        {
            *(dev->pinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->pinMux[devPinIndex]) | PORT_PCR_IRQC(0);
            break;
        }
    }

    /* TODO: It's all? */
}

System_Errors Adc_readValue (Adc_DeviceHandle dev,
                             Adc_ChannelNumber channel,
                             uint16_t *value,
                             Adc_InputType type)
{
    ADC_MemMapPtr regmap = dev->regMap;
    uint8_t channelIndex;
    Adc_ChannelMux channelMux;

    if (!dev->devInitialized) return ERRORS_ADC_DEVICE_NOT_INIT;

    if (channel != ADC_CH_DISABLE)
    {
        for (channelIndex = 0; channelIndex < ADC_MAX_PINS; ++channelIndex)
        {
            if (dev->channelNumber[channelIndex] == channel)
            {
                channelMux = dev->channelMux[channelIndex];
                break;
            }
        }

        if (channel > 0x1F)
            channel -= 0x20;

        if (channelMux == ADC_CHL_A)
            ADC_CFG2_REG(regmap) &= ~ADC_CFG2_MUXSEL_MASK;
        else
            ADC_CFG2_REG(regmap) |= ADC_CFG2_MUXSEL_MASK;

        /* Set single-ended or differential input mode! */
        ADC_SC1_REG(regmap,0) |= ((type << ADC_SC1_DIFF_SHIFT) & ADC_SC1_DIFF_MASK);

        /*
         * If is there a callback, enable interrupt and go out!
         */
        if(dev->callback)
        {
            ADC_SC1_REG(regmap,0) &= ~(ADC_SC1_AIEN_MASK | ADC_SC1_ADCH_MASK);
            ADC_SC1_REG(regmap,0) |= ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(channel);
            *value = 0;
            return ERRORS_NO_ERROR;
        }

        /* Start conversion */
        ADC_SC1_REG(regmap,0) = ADC_SC1_ADCH(channel);

        /* wait until conversion ended */
        while ((ADC_SC1_REG(regmap,0) & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK);

        *value = (uint16_t) ADC_R_REG(regmap,0);

        /* Disable conversion */
        ADC_SC1_REG(regmap,0) = ADC_SC1_ADCH(ADC_CH_DISABLE);

        return ERRORS_NO_ERROR;
    }
    else
    {
        *value = 0;
        return ERRORS_ADC_CHANNEL_WRONG;
    }
}

System_Errors Adc_readValueFromInterrupt (Adc_DeviceHandle dev, uint16_t *value)
{
    *value = (uint16_t) ADC_R_REG(dev->regMap,0);
}

System_Errors Adc_calibration (Adc_DeviceHandle dev)
{
    (void)dev;
    return ERRORS_NO_ERROR;
}

#endif // LIBOHIBOARD_KL15Z4

#endif // LIBOHIBOARD_ADC
