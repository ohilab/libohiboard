/* Copyright (C) 2012-2015 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
 *   Francesco Piunti <francesco.piunti89@gmail.com>
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
 * @file libohiboard/source/adc_K10D10.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Francesco Piunti <francesco.piunti89@gmail.com>
 * @brief ADC functions implementation.
 */

#ifdef LIBOHIBOARD_ADC

#include "platforms.h"
#include "system.h"
#include "adc.h"
#include "clock.h"

#if defined (LIBOHIBOARD_K10D10)

#define ADC_MAX_PINS  52

typedef struct Adc_Device {
    ADC_MemMapPtr regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

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
        .regMap        = ADC0_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC6,
        .simScgcBitEnable = SIM_SCGC6_ADC0_MASK,

        .pins          = {ADC_PINS_ADC0_DP0,
                          ADC_PINS_ADC0_DP1,
                          ADC_PINS_PGA0_DP,
                          ADC_PINS_ADC0_DP3,
                          ADC_PINS_PTE16,
                          ADC_PINS_PTE17,
                          ADC_PINS_PTE18,
                          ADC_PINS_PTE19,
                          ADC_PINS_PTC2,
                          ADC_PINS_PTD1,
                          ADC_PINS_PTD5,
                          ADC_PINS_PTD6,
                          ADC_PINS_PTB0,
                          ADC_PINS_PTB1,
                          ADC_PINS_PTA7,
                          ADC_PINS_PTA8,
                          ADC_PINS_PTB2,
                          ADC_PINS_PTB3,
                          ADC_PINS_PTC0,
                          ADC_PINS_PTC1,
                          ADC_PINS_PTE24,
                          ADC_PINS_PTE25,
                          ADC_PINS_ADC0_DM0,
                          ADC_PINS_ADC0_DM1,
                          ADC_PINS_ADC0_SE21,
                          ADC_PINS_ADC0_SE22,
                          ADC_PINS_ADC0_SE23,

        },

        .pinsPtr       = {0,//&ADC_PINS_ADC0_DP0,
                          0,//&ADC_PINS_ADC0_DP1,
                          0,//&ADC_PINS_PGA0_DP,
                          0,//&ADC_PINS_ADC0_DP3,
                          &PORTE_PCR16,
                          &PORTE_PCR17,
                          &PORTE_PCR18,
                          &PORTE_PCR19,
                          &PORTC_PCR2,
                          &PORTD_PCR1,
                          &PORTD_PCR5,
                          &PORTD_PCR6,
                          &PORTB_PCR0,
                          &PORTB_PCR1,
                          &PORTA_PCR7,
                          &PORTA_PCR8,
                          &PORTB_PCR2,
                          &PORTB_PCR3,
                          &PORTC_PCR0,
                          &PORTC_PCR1,
				 	 	  &PORTE_PCR24,
				 	 	  &PORTE_PCR25,
                          0,//&ADC_PINS_ADC0_DM0,
                          0,//&ADC_PINS_ADC0_DM1,
                          0,//&ADC_PINS_ADC0_SE21
                          0,//&ADC_PINS_ADC0_SE22,
                          0,//&ADC_PINS_ADC0_SE23,
		},

        .pinMux        = {0,
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
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
        },

        .channelNumber = {ADC_CH_DP0,
                          ADC_CH_DP1,
                          ADC_CH_PGA_DP,
                          ADC_CH_DP3,
                          ADC_CH_SE4a,
                          ADC_CH_SE5a,
                          ADC_CH_SE6a,
                          ADC_CH_SE7a,
                          ADC_CH_SE4b,
                          ADC_CH_SE5b,
                          ADC_CH_SE6b,
                          ADC_CH_SE7b,
                          ADC_CH_SE8,
                          ADC_CH_SE9,
                          ADC_CH_SE10,
                          ADC_CH_SE11,
                          ADC_CH_SE12,
                          ADC_CH_SE13,
                          ADC_CH_SE14,
                          ADC_CH_SE15,
                          ADC_CH_SE16,
                          ADC_CH_SE17,
                          ADC_CH_SE18,
                          ADC_CH_DM0,
                          ADC_CH_DM1,
                          ADC_CH_SE21,
                          ADC_CH_SE22,
                          ADC_CH_SE23,
		},

        .channelMux    = {ADC_CHL_A,
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
                          ADC_CHL_B,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_A,
     	},

	    .devInitialized = 0,
};
Adc_DeviceHandle OB_ADC0 = &adc0;

static Adc_Device adc1 = {
        .regMap          = ADC1_BASE_PTR,

        .simScgcPtr      = &SIM_SCGC3,
        .simScgcBitEnable= SIM_SCGC3_ADC1_MASK,

        .pins            = {ADC_PINS_ADC1_DP0,
                            ADC_PINS_ADC1_DP1,
                            ADC_PINS_PGA_DP,
                            ADC_PINS_ADC0_DP3,
                            ADC_PINS_PTE0,
                            ADC_PINS_PTE1,
                            ADC_PINS_PTE2,
                            ADC_PINS_PTE3,
                            ADC_PINS_PTC7,
                            ADC_PINS_PTC8,
                            ADC_PINS_PTC9,
                            ADC_PINS_PTC10,
                            ADC_PINS_PTB0,
                            ADC_PINS_PTB1,
                            ADC_PINS_PTB4,
                            ADC_PINS_PTB5,
                            ADC_PINS_PTB6,
                            ADC_PINS_PTB7,
                            ADC_PINS_PTB10,
                            ADC_PINS_PTB11,
                            ADC_PINS_ADC1_SE16,
                            ADC_PINS_PTA17,
                            ADC_PINS_ADC1_SE18,
                            ADC_PINS_ADC1_DM0,
                            ADC_PINS_ADC1_DM1,
                            ADC_PINS_ADC1_SE23,

        },

		.pinsPtr         = {0,//&ADC_PINS_ADC1_DP0
                            0,//&ADC_PINS_ADC1_DP1,
                            0,//&ADC_PINS_PGA_DP,
                            0,//&ADC_PINS_ADC0_DP3,
                            &PORTE_PCR0,
                            &PORTE_PCR1,
                            &PORTE_PCR2,
                            &PORTE_PCR3,
                            &PORTC_PCR7,
                            &PORTC_PCR8,
                            &PORTC_PCR9,
                            &PORTC_PCR10,
                            &PORTB_PCR0,
                            &PORTB_PCR1,
                            &PORTB_PCR4,
                            &PORTB_PCR5,
                            &PORTB_PCR6,
                            &PORTB_PCR7,
                            &PORTB_PCR10,
                            &PORTB_PCR11,
                            0,//&ADC_PINS_ADC1_SE16,
                            &PORTA_PCR17,
                            0,//&ADC_PINS_ADC1_SE18,
                            0,//&ADC_PINS_ADC1_DM0,
                            0,//&ADC_PINS_ADC1_DM1,
                            0,//&ADC_PIN_ADC1_SE23,
        },

        .pinMux          = {0,
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
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
		},

		.channelNumber   = {ADC_CH_DP0,
                            ADC_CH_DP1,
                            ADC_CH_PGA_DP,
                            ADC_CH_DP3,
                            ADC_CH_SE4a,
                            ADC_CH_SE5a,
                            ADC_CH_SE6a,
                            ADC_CH_SE7a,
                            ADC_CH_SE4b,
                            ADC_CH_SE5b,
                            ADC_CH_SE6b,
                            ADC_CH_SE7b,
                            ADC_CH_SE8,
                            ADC_CH_SE9,
                            ADC_CH_SE10,
                            ADC_CH_SE11,
                            ADC_CH_SE12,
                            ADC_CH_SE13,
                            ADC_CH_SE14,
                            ADC_CH_SE15,
                            ADC_CH_SE16,
                            ADC_CH_SE17,
                            ADC_CH_SE18,
                            ADC_CH_DM0,
                            ADC_CH_DM1,
                           //ADC_CH_SE21,
                           //ADC_CH_SE22,
                            ADC_CH_SE23,
        },

        .channelMux      = {ADC_CHL_A,
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
                            ADC_CHL_B,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                            ADC_CHL_A,
                           //ADC_CHL_A,
                           //ADC_CHL_A,
                            ADC_CHL_A,
        },

        .devInitialized = 0,
};
Adc_DeviceHandle OB_ADC1 = &adc1;


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

    /*setting clock source and divider*/
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

    /*setting single or continuous conversion*/
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
    switch (config->voltRef)
    {
    case ADC_VREF:
    	ADC_SC2_REG(regmap) = ADC_SC2_REFSEL(0);
        break;
    case ADC_VALT:
    	ADC_SC2_REG(regmap) = ADC_SC2_REFSEL(1);
        break;
    }

    /* Select the average */
    switch (config->average)
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

//    Adc_enablePin (dev, config->adcPin);
    /* Calibration flag */
    if(config->doCalibration)
        Adc_calibration(dev);

    dev->devInitialized = 1;

	return ERRORS_NO_ERROR;
}

void Adc_enablePin (Adc_DeviceHandle dev, Adc_Pins pin)
{
    uint8_t devPinIndex;

    for (devPinIndex = 0; devPinIndex < ADC_MAX_PINS; ++devPinIndex)
    {
        if (dev->pins[devPinIndex] == ADC_PINS_NONE) break;

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

System_Errors Adc_calibration (Adc_DeviceHandle dev)
{
    ADC_MemMapPtr regmap = dev->regMap;
    uint16_t calibration;
    uint32_t busClock = Clock_getFrequency(CLOCK_BUS);

    uint32_t regSC3  = ADC_SC3_REG(regmap);
    uint32_t regSC2  = ADC_SC2_REG(regmap);
    uint32_t regCFG1 = ADC_CFG1_REG(regmap);

    if (!dev->devInitialized) return ERRORS_ADC_DEVICE_NOT_INIT;

    /* Set registers and clock for a better calibration */
    ADC_SC3_REG(regmap) &= ~(ADC_SC3_AVGE_MASK | ADC_SC3_AVGS_MASK | ADC_SC3_ADCO_MASK);
    ADC_SC3_REG(regmap) |= ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3)|ADC_SC3_CAL_MASK;

    ADC_SC2_REG(regmap) &= ~ADC_SC2_REFSEL_MASK;
    ADC_SC2_REG(regmap) |= ADC_SC2_REFSEL(0);

    ADC_CFG1_REG(regmap) &= ~(ADC_CFG1_ADIV_MASK | ADC_CFG1_ADICLK_MASK);
    if (busClock < 32000000)
        ADC_CFG1_REG(regmap) |= ADC_CFG1_ADIV(3);
    else
        ADC_CFG1_REG(regmap) |= ADC_CFG1_ADIV(3) | ADC_CFG1_ADICLK_MASK;

    /* Start calibration */
    ADC_SC3_REG(regmap) |= ADC_SC3_CAL_MASK;
    /* wait calibration ended */
    while ((ADC_SC1_REG(regmap,0) & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK);
    /* Check error */
    if(ADC_SC3_REG(regmap) & ADC_SC3_CALF_MASK)
    {
        /* Restore register */
        ADC_SC3_REG(regmap) = regSC3;
        ADC_SC2_REG(regmap) = regSC2;
        ADC_CFG1_REG(regmap) = regCFG1;
        return ERRORS_ADC_CALIBRATION;
    }

    calibration = 0;
    calibration += ADC_CLP0_REG(regmap);
    calibration += ADC_CLP1_REG(regmap);
    calibration += ADC_CLP2_REG(regmap);
    calibration += ADC_CLP3_REG(regmap);
    calibration += ADC_CLP4_REG(regmap);
    calibration += ADC_CLPS_REG(regmap);
    calibration = (calibration >> 1U) | 0x8000U; // divide by 2
    ADC_PG_REG(regmap) = calibration;

    calibration = 0;
    calibration += ADC_CLM0_REG(regmap);
    calibration += ADC_CLM1_REG(regmap);
    calibration += ADC_CLM2_REG(regmap);
    calibration += ADC_CLM3_REG(regmap);
    calibration += ADC_CLM4_REG(regmap);
    calibration += ADC_CLMS_REG(regmap);
    calibration = (calibration >> 1U) | 0x8000U; // divide by 2
    ADC_MG_REG(regmap) = calibration;

    ADC_SC1_REG(regmap,0) = ADC_SC1_ADCH(ADC_CH_DISABLE);

    /* Restore register */
    ADC_SC3_REG(regmap) = regSC3;
    ADC_SC2_REG(regmap) = regSC2;
    ADC_CFG1_REG(regmap) = regCFG1;

    dev->devCalibration = 1;

    return ERRORS_NO_ERROR;
}

#endif // LIBOHIBOARD_K10D10

#endif // LIBOHIBOARD_ADC
