/******************************************************************************
 * Copyright (C) 2012-2014 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  Francesco Piunti <francesco.piunti89@gmail.com>
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
 * @file libohiboard/source/adc.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Francesco Piunti <francesco.piunti89@gmail.com>
 * @brief ADC functions implementation.
 */

#if  0

#include "platforms.h"
#include "system.h"

#include "adc.h"

/* TODO:
 * Gestione dell'adc per gli altri microcontrollori
 * Togliere i set a favore del config come per uart e ftm
 */

#define ADC_PIN_ENABLED                  1
#define ADC_PIN_DISABLED                 0

#define ADC_MAX_PINS                     28

typedef struct Adc_Device {
    ADC_MemMapPtr regMap;
    
    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Adc_Pins pins[ADC_MAX_PINS];    /**< List of the pin for the FTM channel. */
    volatile uint32_t* pinsPtr[ADC_MAX_PINS];
    Adc_ChannelNumber channelNumber[ADC_MAX_PINS];
    Adc_ChannelMux channelMux[ADC_MAX_PINS];
    uint8_t pinMux[ADC_MAX_PINS];     /**< Mux of the pin of the FTM channel. */
    
    Adc_Resolution resolution;
    Adc_Average average;
} Adc_Device;

#if defined(MK60DZ10)

static Adc_Device adc0 = {
        .regMap           = ADC0_BASE_PTR,
        .resolution       = ADC_RESOLUTION_8BIT,
        .average          = ADC_AVERAGE_1_SAMPLES
};
Adc_DeviceHandle ADC0 = &adc0; 

static Adc_Device adc1 = {
        .regMap           = ADC1_BASE_PTR,
        .resolution       = ADC_RESOLUTION_8BIT,
        .average          = ADC_AVERAGE_1_SAMPLES
};
Adc_DeviceHandle ADC1 = &adc1;

#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)
#elif defined(MK10DZ10) 
static Adc_Device adc0 = {
		.regMap           = ADC0_BASE_PTR,
		.resolution       = ADC_RESOLUTION_8BIT,
		.average          = ADC_AVERAGE_1_SAMPLES
		};
Adc_DeviceHandle ADC0 = &adc0; 

static Adc_Device adc1 = {
	 .regMap           = ADC1_BASE_PTR,
	 .resolution       = ADC_RESOLUTION_8BIT,
	 .average          = ADC_AVERAGE_1_SAMPLES
	 };
Adc_DeviceHandle ADC1 = &adc1;

#endif

/**
 * @brief
 * 
 * Fixed conditions:
 *   - Long sample enabled.
 *   - High speed disabled.
 *   - Bus clock selected and divided-by-2.
 *   - Single conversion.
 * 
 * TODO: Pin manage!  
 * 
 * @param dev Adc device handle to be synchronize.
 * @return A System_Errors elements that indicate the status of initialization.
 */
System_Errors Adc_init (Adc_DeviceHandle dev)
{
    ADC_MemMapPtr regmap = dev->regMap;

    /* Turn on clock */
#if defined(LIBOHIBOARD_KL15Z4) || defined(FRDMKL25Z) || defined (MK10D10)

    *dev->simScgcPtr |= dev->simScgcBitEnable;
    
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)
#elif defined(MK10DZ10) 
    if (regmap == ADC0_BASE_PTR)
        SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
    else if (regmap == ADC1_BASE_PTR)
    	SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK;
    else
        return ERRORS_PARAM_VALUE;

#endif

    /* Long sample time, busclock/2 and divide by-1*/
    ADC_CFG1_REG(regmap) = ADC_CFG1_ADIV(0) | 
            ADC_CFG1_ADLSMP_MASK | ADC_CFG1_ADICLK(1);
    
    switch (dev->resolution)
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
#if defined (OHIBOARD_R1) || defined (MK60DZ10) ||           \
    defined (FRDMKL25Z) ||                                   \
    defined (MK10DZ10) || defined (MK10D10)
        
    case ADC_RESOLUTION_16BIT:
        ADC_CFG1_REG(regmap) |= ADC_CFG1_MODE(3);
        break;
        
#endif
	}
    
    /* 24 ADCK cycles total: Long sample time */
    ADC_CFG2_REG(regmap) = ADC_CFG2_ADLSTS(0);
    
    /* Select voltage reference: default */
    ADC_SC2_REG(regmap) = ADC_SC2_REFSEL(0);
    
    /* Select the average */
    switch (dev->average)
    {
    case ADC_AVERAGE_1_SAMPLES:
        /* Nothing to do! */
    	ADC_SC3_REG(regmap) &= ~ADC_SC3_AVGE_MASK;
        break;
    case ADC_AVERAGE_4_SAMPLES:
        ADC_SC3_REG(regmap) = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0);
        break;
    case ADC_AVERAGE_8_SAMPLES:
        ADC_SC3_REG(regmap) = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(1);
        break;
    case ADC_AVERAGE_16_SAMPLES:
        ADC_SC3_REG(regmap) = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(2);
        break;
    case ADC_AVERAGE_32_SAMPLES:
        ADC_SC3_REG(regmap) = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);
        break;
    }

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
                             uint16_t *value)
{
    ADC_MemMapPtr regmap = dev->regMap;
    uint8_t channelIndex;
    Adc_ChannelMux channelMux;
    
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

void Adc_setResolution (Adc_DeviceHandle dev, Adc_Resolution resolution)
{
	dev->resolution = resolution;
}

void Adc_setAverage (Adc_DeviceHandle dev, Adc_Average average)
{
    dev->average = average;
}

#endif
