/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Marco Giammarini <m.giammarini@warcomeb.it>
 *	
 * Project: libohiboard
 * Package: ADC
 * Version: 0.0
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>
 ******************************************************************************/

/**
 * @file libohiboard/source/adc.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief ADC functions implementation.
 */


#include "platforms.h"
#include "system.h"

#include "adc.h"

#define ADC_PIN_ENABLED        1
#define ADC_PIN_DISABLED       0

typedef struct Adc_Device {
    ADC_MemMapPtr 		  regMap;
    
    Adc_Resolution        resolution;
    Adc_Avarage           avarage;
    
//    uint8_t               pinEnabled;
} Adc_Device;

#if defined(MKL15Z4) || defined(FRDMKL25Z)
static Adc_Device adc0 = {
        .regMap           = ADC0_BASE_PTR,
        .resolution       = ADC_RESOLUTION_8BIT,
        .avarage          = ADC_AVARAGE_1_SAMPLES
};
Adc_DeviceHandle ADC0 = &adc0; 
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)
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

//    if (dev->pinEnabled == IIC_PIN_DISABLED)
//    	return ERRORS_HW_NOT_ENABLED;
    
    /* Turn on clock */
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    if (regmap == ADC0_BASE_PTR)
        SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
    else
        return ERRORS_PARAM_VALUE;
#elif defined(MK60DZ10)
#elif defined(FRDMKL05Z)
#elif defined(FRDMK20D50M)
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
        ADC_CFG1_REG(regmap) |= ADC_CFG1_MODE(1);
        break;
    case ADC_RESOLUTION_12BIT:
        ADC_CFG1_REG(regmap) |= ADC_CFG1_MODE(2);
        break;
    case ADC_RESOLUTION_16BIT:
        ADC_CFG1_REG(regmap) |= ADC_CFG1_MODE(3);
        break;
	}
    
    /* 24 ADCK cycles total: Long sample time */
    ADC_CFG2_REG(regmap) = ADC_CFG2_MUXSEL_MASK | ADC_CFG2_ADLSTS(0);
    
    /* Select voltage reference: default */
    ADC_SC2_REG(regmap) = ADC_SC2_REFSEL(0);
    
    /* Select the avarage */
    switch (dev->avarage)
    {
    case ADC_AVARAGE_1_SAMPLES:
        /* Nothing to do! */
        break;
    case ADC_AVARAGE_4_SAMPLES:
        ADC_SC3_REG(regmap) = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0);
        break;
    case ADC_AVARAGE_8_SAMPLES:
        ADC_SC3_REG(regmap) = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(1);
        break;
    case ADC_AVARAGE_16_SAMPLES:
        ADC_SC3_REG(regmap) = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(2);
        break;
    case ADC_AVARAGE_32_SAMPLES:
        ADC_SC3_REG(regmap) = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);
        break;
    }

	return ERRORS_NO_ERROR;
}

System_Errors Adc_readValue (Adc_DeviceHandle dev, Adc_ChannelNumber channel, uint16_t *value)
{
    ADC_MemMapPtr regmap = dev->regMap;

    if (channel != ADC_CH_DISABLE)
    {
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

void Adc_setAvarage (Adc_DeviceHandle dev, Adc_Avarage avarage)
{
    dev->avarage = avarage;
}
