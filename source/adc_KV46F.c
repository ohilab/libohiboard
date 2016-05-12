/* Copyright (C) 2014-2016 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Matteo Civale <matteo.civale@gmail.com>
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
 * @file libohiboard/source/adc_KV46F.c
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief ADC functions implementation for KV46f or TWRKV46F.
 */

#include "adc.h"
#include "platforms.h"
#include "system.h"
#include "interrupt.h"
#include "adc.h"
#include "clock.h"

#define ADC_MAX_PINS        37
#define ADC_MAX_CHANNEL     16


typedef struct Adc_Device {
    ADC_MemMapPtr regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */
    volatile uint32_t *channelmuxPtr;
    void (*isrADCA)(void);                     /**< The function pointer for ISR. */
    void (*isrADCB)(void);                     /**< The function pointer for ISR. */

    void (*callbackADCA)(void);      /**< The function pointer for user callback. */
    void (*callbackADCB)(void);

    Interrupt_Vector isrNumberADCA;                       /**< ISR vector number. */
    Interrupt_Vector isrNumberADCB;

    Adc_Pins pins[ADC_MAX_PINS+1];  /**< List of the pin for the ADC channel. */
    volatile uint32_t* pinsPtr[ADC_MAX_PINS];

    Adc_ChannelNumber channelNumber[ADC_MAX_PINS];

    Adc_ChannelMux channelMux[ADC_MAX_PINS]; /*<< mux for the channel */

    uint8_t pinMux[ADC_MAX_PINS];     /**< Mux of the pin of the FTM channel. */

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
    /** Indicate that device require calibration on Adc_readValue. */
    uint8_t devCalibration;
} Adc_Device;

static Adc_Device adc0 = {
     .regMap           = ADC_BASE_PTR,
     .channelmuxPtr    = &SIM_ADCOPT,

     .simScgcPtr       = &SIM_SCGC5,
     .simScgcBitEnable = SIM_SCGC5_ADC_MASK,

     .pins             = {ADC_PINS_PTA17,

                          ADC_PINS_PTB0,
                          ADC_PINS_PTB1,
                          ADC_PINS_PTB2,
                          ADC_PINS_PTB3,
                          ADC_PINS_PTB10,
                          ADC_PINS_PTB11,

                          ADC_PINS_PTC0,
                          ADC_PINS_PTC1,
                          ADC_PINS_PTC2,
                          ADC_PINS_PTC10,
                          ADC_PINS_PTC11,

                          ADC_PINS_PTD1,
                          ADC_PINS_PTD5,
                          ADC_PINS_PTD6,

                          ADC_PINS_PTE0,
                          ADC_PINS_PTE1,
                          ADC_PINS_PTE2,
                          ADC_PINS_PTE3,
                          ADC_PINS_PTE16,
                          ADC_PINS_PTE17,
                          ADC_PINS_PTE18,
                          ADC_PINS_PTE19,
                          ADC_PINS_PTE20,
                          ADC_PINS_PTE21,
                          ADC_PINS_PTE24,
                          ADC_PINS_PTE25,
                          ADC_PINS_PTE29,
                          ADC_PINS_PTE30,


                          ADC_PINS_ACH6a,
                          ADC_PINS_ACH7a,
                          ADC_PINS_ACH2,
                          ADC_PINS_ACH3,
                          ADC_PINS_ACH6c,
                          ADC_PINS_ACH7c,
                          ADC_PINS_ACH6d,
                },
      .pinsPtr         = {&PORTA_PCR17,

                          &PORTB_PCR0,
                          &PORTB_PCR1,
                          &PORTB_PCR2,
                          &PORTB_PCR3,
                          &PORTB_PCR10,
                          &PORTB_PCR11,

                          &PORTC_PCR0,
                          &PORTC_PCR1,
                          &PORTC_PCR2,
                          &PORTC_PCR10,
                          &PORTC_PCR11,

                          &PORTD_PCR1,
                          &PORTD_PCR5,
                          &PORTD_PCR6,

                          &PORTE_PCR0,
                          &PORTE_PCR1,
                          &PORTE_PCR2,
                          &PORTE_PCR3,
                          &PORTE_PCR16,
                          &PORTE_PCR17,
                          &PORTE_PCR18,
                          &PORTE_PCR19,
                          &PORTE_PCR20,
                          &PORTE_PCR21,
                          &PORTE_PCR24,
                          &PORTE_PCR25,
                          &PORTE_PCR29,
                          &PORTE_PCR30,

                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,

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
               },

         .channelMux   = {ADC_CHL_E,        //pte17

                          ADC_CHL_RESERVED, //ptb0
                          ADC_CHL_RESERVED, //ptb1
                          ADC_CHL_E,        //ptb2
                          ADC_CHL_E,        //ptb3
                          ADC_CHL_A,        //ptb10
                          ADC_CHL_A,        //ptb11

                          ADC_CHL_B,        //ptc0
                          ADC_CHL_B,        //ptc1
                          ADC_CHL_C,        //ptc2
                          ADC_CHL_D,        //ptc10
                          ADC_CHL_E,        //ptc11

                          ADC_CHL_F,        //ptd1
                          ADC_CHL_G,        //ptd5
                          ADC_CHL_G,        //ptd6

                          ADC_CHL_F,        //pte0
                          ADC_CHL_F,        //pte1
                          ADC_CHL_G,        //pte2
                          ADC_CHL_G,        //pte3
                          ADC_CHL_RESERVED, //pte16
                          ADC_CHL_RESERVED, //pte17
                          ADC_CHL_RESERVED, //pte18
                          ADC_CHL_RESERVED, //pte19
                          ADC_CHL_B,        //pte20
                          ADC_CHL_B,        //pte21
                          ADC_CHL_RESERVED, //pte24
                          ADC_CHL_RESERVED, //pte25
                          ADC_CHL_RESERVED, //pte29
                          ADC_CHL_RESERVED, //pte30

                          ADC_CHL_A,
                          ADC_CHL_A,
                          ADC_CHL_RESERVED,
                          ADC_CHL_RESERVED,
                          ADC_CHL_C,
                          ADC_CHL_C,
                          ADC_CHL_D,



         },
         .channelNumber={ADC_CHANNEL_CHA7,
                         //port B pins
                         ADC_CHANNEL_CHB2,
                         ADC_CHANNEL_CHB3,
                         ADC_CHANNEL_CHA6,
                         ADC_CHANNEL_CHB7,
                         ADC_CHANNEL_CHB6,
                         ADC_CHANNEL_CHB7,
                         //port C pins
                         ADC_CHANNEL_CHB6,
                         ADC_CHANNEL_CHB7,
                         ADC_CHANNEL_CHB6,
                         ADC_CHANNEL_CHB7,
                         ADC_CHANNEL_CHB6,
                         //port D pins
                         ADC_CHANNEL_CHA7,
                         ADC_CHANNEL_CHA6,
                         ADC_CHANNEL_CHA7,
                         //port E pins
                         ADC_CHANNEL_CHB6,
                         ADC_CHANNEL_CHB7,
                         ADC_CHANNEL_CHB6,
                         ADC_CHANNEL_CHB7,
                         ADC_CHANNEL_CHA0,
                         ADC_CHANNEL_CHA1,
                         ADC_CHANNEL_CHB0,
                         ADC_CHANNEL_CHB1,
                         ADC_CHANNEL_CHA6,
                         ADC_CHANNEL_CHA7,
                         ADC_CHANNEL_CHB4,
                         ADC_CHANNEL_CHB5,
                         ADC_CHANNEL_CHA4,
                         ADC_CHANNEL_CHA5,
                         //no port
                         ADC_CHANNEL_CHA6,
                         ADC_CHANNEL_CHA7,
                         ADC_CHANNEL_CHA2,
                         ADC_CHANNEL_CHA3,
                         ADC_CHANNEL_CHA6,
                         ADC_CHANNEL_CHA7,
                         ADC_CHANNEL_CHB6,


         },

         .isrADCA          = ADCA_IRQHandler,
         .isrADCB          = ADCB_IRQHandler,

         .isrNumberADCA    = INTERRUPT_ADCA,
         .isrNumberADCB    = INTERRUPT_ADCA,

         .devInitialized = 0,
         .devCalibration = 0,
};

Adc_DeviceHandle OB_ADC0 = &adc0;

void ADCA_IRQHandler(void)
{
    OB_ADC0->callbackADCA();
}
void ADCB_IRQHandler(void)
{
    OB_ADC0->callbackADCB();
}


System_Errors Adc_init (Adc_DeviceHandle dev, Adc_Config *config)
{
    uint16_t regapp;
    uint32_t busfrequency;

    /* Enable ADC Clock */
    *dev->simScgcPtr|=dev->simScgcBitEnable;

    busfrequency=Clock_getFrequency(CLOCK_FAST_PERIPHERALS);
    if(
       (busfrequency/(config->clkDiv0+1)>(config->ADCAspeed+1)*6.26e6)||
       (busfrequency/(config->clkDiv1+1)>(config->ADCBspeed+1)*6.26e6)
       )
        return ERRORS_ADC_ERRATA_DIVIDERS;

    ADC_CAL_REG(dev->regMap)|= ADC_CAL_SEL_VREFH_A(config->ADCAvrefH)|
                               ADC_CAL_SEL_VREFH_B(config->ADCBvrefH)|
                               ADC_CAL_SEL_VREFLO_A(config->ADCAvrefL)|
                               ADC_CAL_SEL_VREFLO_B(config->ADCBvrefL);

    /* Set DIV0 */
    regapp=ADC_CTRL2_REG(dev->regMap);
    regapp&=~ADC_CTRL2_DIV0_MASK;
    regapp|=ADC_CTRL2_DIV0(config->clkDiv0);
    ADC_CTRL2_REG(dev->regMap)=regapp;

    /* Set ABS and PUDELAY and  APD */
    regapp=ADC_PWR_REG(dev->regMap);
    regapp&=~ADC_PWR_PUDELAY_MASK;
    regapp|= ADC_PWR_ASB(config->autoStbEn)|ADC_PWR_PUDELAY(config->pwrUpDelay)|
             ADC_PWR_APD(config->autoPwrDownEn);
    ADC_PWR_REG(dev->regMap) =regapp;


    /* Set DIV1 SPEEDB and SPEEDA  */

    ADC_PWR2_REG(dev->regMap)= ADC_PWR2_SPEEDA(config->ADCAspeed)|
                               ADC_PWR2_SPEEDB(config->ADCBspeed)|
                               ADC_PWR2_DIV1(config->clkDiv1);

    /* Power UP converter */
    ADC_PWR_REG(dev->regMap) &=~(ADC_PWR_PD0_MASK|ADC_PWR_PD1_MASK);

    /* wait for ADC to power up */
    while(ADC_PWR_REG(dev->regMap) & (ADC_PWR_PSTS0_MASK|ADC_PWR_PSTS1_MASK)){};

    /* Disable all channel */
    ADC_SDIS_REG(dev->regMap)=0xFFFF;

    dev->devInitialized=TRUE;
    /* Disable all channel */
    ADC_SDIS_REG(dev->regMap)=0xFFFF;


    return ERRORS_NO_ERROR;
}


System_Errors Adc_acquireConfig (Adc_DeviceHandle dev,  Adc_acqConfig *config)
{
    uint8_t regapp;
    if (!dev->devInitialized)
    return ERRORS_DAC_DEVICE_NOT_INIT;

    /* Set SMODE=?, SYNC0=?, EOSIE0=?, ZCIE=?, LLMTIE=?, HLMTIE=? */
    regapp=0;
    regapp |= ADC_CTRL1_SMODE(config->scanmode)|
             ADC_CTRL1_SYNC0(config->sync0)|
             ADC_CTRL1_EOSIE0(config->interrToEnable.eos0)|
             ADC_CTRL1_ZCIE(config->interrToEnable.zc)|
             ADC_CTRL1_LLMTIE(config->interrToEnable.llmt)|
             ADC_CTRL1_HLMTIE(config->interrToEnable.hlmt);

    ADC_CTRL1_REG(dev->regMap)=regapp;

    /* Set SYNC1=?, EOSIE1=?, SIMULT=? */
    regapp= ADC_CTRL2_REG(dev->regMap);
    regapp &= (~ADC_CTRL2_SYNC1_MASK)|(~ADC_CTRL2_EOSIE1_MASK)|(~ADC_CTRL2_SIMULT_MASK);
    regapp |= ADC_CTRL2_SYNC1(config->sync1)|ADC_CTRL2_EOSIE1(config->interrToEnable.eos1)|
              ADC_CTRL2_SIMULT(config->simultEn);
    ADC_CTRL2_REG(dev->regMap)=regapp;

    if(config->isrADCA)
    {
        dev->callbackADCA=config->isrADCA;
        Interrupt_enable(INTERRUPT_ADCA);
    }

    if(config->isrADCB)
    {
        dev->callbackADCB=config->isrADCB;
        Interrupt_enable(INTERRUPT_ADCB);
    }


    return ERRORS_NO_ERROR;

}

System_Errors Adc_setChannel (Adc_DeviceHandle dev, uint8_t channelIndex, Adc_channelConfig *config)
{
    uint16_t regapp;
    uint8_t i;
    uint8_t shift;
    uint32_t regapp32;
    if (!dev->devInitialized)
        return ERRORS_DAC_DEVICE_NOT_INIT;

    if(channelIndex>=ADC_MAX_CHANNEL)
        return ERRORS_ADC_NUMCH_WRONG;
    i=0;
    while((i<ADC_MAX_PINS)&&(config->pin!=dev->pins[i]))
    {
        i++;
    }
    if(i==ADC_MAX_PINS)
        return ERRORS_ADC_PIN_WRONG;

    /*                      Enable pin                  */

    /* Set ALT */
    if(dev->pinsPtr[i])
        *(dev->pinsPtr[i]) = PORT_PCR_MUX(dev->pinMux[i]) | PORT_PCR_IRQC(0);

    /* Set mux */
    if(dev->channelMux[i]!=ADC_CHL_RESERVED)
    {
        regapp32=*(dev->channelmuxPtr);
        shift=0;

        if(dev->channelNumber[i]>0x08)
           shift+=8; //The channel is a B channel
        if(dev->channelNumber[i]==0x7)
           shift+=4;
        regapp32 &= ~(0x7<<shift);
        regapp32 |= (dev->channelMux[i]<<shift);
        *(dev->channelmuxPtr)=regapp32;
    }


    /* Set channel */
    regapp=*(&ADC_CLIST1_REG(dev->regMap)+(channelIndex>>2));
    regapp&=~(0xF<<((channelIndex%4)<<2));
    regapp|=(0xF&dev->channelNumber[i])<<(channelIndex%4);
    *(&ADC_CLIST1_REG(dev->regMap)+(channelIndex>>2))=regapp;

    /* Set HLIM */
    ADC_HILIM_REG(dev->regMap,channelIndex)=ADC_HILIM_HLMT(config->Hlimit);
    /* Set LLIM */
    ADC_LOLIM_REG(dev->regMap,channelIndex)=ADC_LOLIM_LLMT(config->Llimit);

    /* Set OFFSET */
    ADC_OFFST_REG(dev->regMap,channelIndex)=ADC_OFFST_OFFSET(config->offset);

    /* Set GAIN */
    regapp = *(&ADC_GC1_REG(dev->regMap)+(channelIndex>>3));
    regapp &= ~(0x3<<((channelIndex%8)<<1));
    regapp |=(config->gain<<((channelIndex%8)<<1));
    *(&ADC_GC1_REG(dev->regMap)+(channelIndex>>3))=regapp;

    /* Set ZCE */
    regapp = *(&ADC_ZXCTRL1_REG(dev->regMap)+(channelIndex>>3));
    regapp &= ~(0x3<<((channelIndex%8)<<1));
    regapp |= (config->scMode<<((channelIndex%8)<<1));
    *(&ADC_ZXCTRL1_REG(dev->regMap)+(channelIndex>>3))=regapp;

    /* Set DS */
    ADC_SDIS_REG(dev->regMap) &= ~(1<<channelIndex);

    /* Set SCHLTN */
    ADC_SCHLTEN_REG(dev->regMap)|=((0x1&config->scanIntEn)<<channelIndex);

    /* Set SCTRL */
    ADC_SCTRL_REG(dev->regMap)|=(0x1&config->sampleOnSync)<<channelIndex;

    return ERRORS_NO_ERROR;
}

System_Errors Adc_acquireStart (Adc_DeviceHandle dev)
{
    if(!dev->devInitialized)
        return ERRORS_ADC_DEVICE_NOT_INIT;
    ADC_CTRL1_REG(dev->regMap) &= ~ADC_CTRL1_STOP0_MASK;
    ADC_CTRL2_REG(dev->regMap) &= ~ADC_CTRL2_STOP1_MASK;

    return ERRORS_NO_ERROR;
}

/**
 *
 * GUIDA
 *
 *
 *
 */
/***********************************************************************************
ADC init in single ended parallel mode (2 channels per module), software triggered
***********************************************************************************/
void adc_init()
{

  SIM_SCGC5 |= SIM_SCGC5_ADC_MASK;//ADC clk enable

  /* Configuring ANA0, ANA1 for ADCA and ANB0, ANB1 for ADCB*/
  ADC_CLIST1 = ADC_CLIST1_SAMPLE0(3)|ADC_CLIST1_SAMPLE1(2);
  ADC_CLIST3 = ADC_CLIST3_SAMPLE8(14)|ADC_CLIST3_SAMPLE9(15);

  ADC_SDIS = 0xFCFC;    //enable  samples

  /* ADC_CTRL1: DMAEN0=0,STOP0=1,START0=0,SYNC0=1,EOSIE0=1,ZCIE=0,LLMTIE=0,HLMTIE=0,CHNCFG_L=0,??=0,SMODE=1 */
  ADC_CTRL1 = ADC_CTRL1_SYNC0_MASK|ADC_CTRL1_EOSIE0_MASK|ADC_CTRL1_SMODE(5);

  /* ADC_CTRL2: DMAEN1=0,STOP1=1,START1=0,SYNC1=0,EOSIE1=0,CHNCFG_H=0,SIMULT=1,DIV0=5 */
  ADC_CTRL2 = ADC_CTRL2_SIMULT_MASK|ADC_CTRL2_DIV0(5);

  /* ADC_PWR: ASB=0,??=0,??=0,??=0,PSTS1=0,PSTS0=0,PUDELAY=0x0D,APD=0,??=0,PD1=0,PD0=0 */
  ADC_PWR = ADC_PWR_PUDELAY(0x0D);

 /* ADC_ZXSTAT: ZCS=0xFFFF */
  ADC_ZXSTAT = ADC_ZXSTAT_ZCS(0xFFFF); /* Clear zero crossing status flags */

  /* ADC_LOLIMSTAT: LLS=0xFFFF */
  ADC_LOLIMSTAT = ADC_LOLIMSTAT_LLS(0xFFFF); /* Clear low limit status */

  /* ADC_HILIMSTAT: HLS=0xFFFF */
  ADC_HILIMSTAT = ADC_HILIMSTAT_HLS(0xFFFF); /* Clear high limit status */

  /* ADC_STAT: CIP0=0,CIP1=0,??=0,EOSI1=1,EOSI0=1,ZCI=0,LLMTI=0,HLMTI=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  ADC_STAT = ADC_STAT_EOSI1_MASK | ADC_STAT_EOSI0_MASK; /* Clear EOSI and EOSI1 flags */

  /* ADC_PWR2: ??=0,??=0,DIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  ADC_PWR2 = ADC_PWR2_DIV1(0x30); /* Set Power 2 */
  ADC_PWR2 |= ADC_PWR2_SPEEDA_MASK | ADC_PWR2_SPEEDB_MASK;

  SIM_ADCOPT|=0X00004000;
  /* wait for ADC to power up */
  while(ADC_PWR & (ADC_PWR_PSTS0_MASK|ADC_PWR_PSTS1_MASK)){};

  ADC_CTRL1 &= ~ADC_CTRL1_STOP0_MASK; //clearing ADC stop bit
}
