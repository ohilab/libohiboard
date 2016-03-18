/******************************************************************************
 * Copyright (C) 2014-2016 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/dma.c
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief DMA configuration and parameter.
 */

#ifdef LIBOHIBOARD_DMA


#if defined (LIBOHIBOARD_KL25Z4)

#include "dma.h"
#include "interrupt.h"

#ifdef LIBOHIBOARD_UART
   #include "uart.h"
#endif

#ifdef LIBOHIBOARD_ADC
   #include "adc.h"
#endif

#ifdef LIBOHIBOARD_DAC
   #include "dac.h"
#endif

#define MAX_DMA_CHANNEL 4

/* Device Handle definition */

typedef struct Dma_Device
{
	DMA_MemMapPtr    regMap;
	DMAMUX_MemMapPtr regMapMux;

	bool deviceInit;

	volatile uint32_t *simScgcPtrDMAMUX;
	volatile uint32_t *simScgcPtrDMA;

	uint32_t simScgcBitEnableDMAMUX;
	uint32_t simScgcBitEnableDMA;

	void (*irqCallback[MAX_DMA_CHANNEL]) (void);

}Dma_Device;


/* Initialize DMA0 */

static Dma_Device dma0={

		.regMap                  = DMA_BASE_PTR,
		.regMapMux               = DMAMUX0_BASE_PTR,

		.simScgcPtrDMAMUX        = &SIM_SCGC6,
		.simScgcPtrDMA           = &SIM_SCGC7,

		.simScgcBitEnableDMAMUX  = SIM_SCGC6_DMAMUX_MASK,
		.simScgcBitEnableDMA     = SIM_SCGC7_DMA_MASK,
		.deviceInit              = FALSE,
};


Dma_DeviceHandle OB_DMA0=&dma0;




System_Errors  dma_init(Dma_DeviceHandle dev, dma_ConfigType* config, void *callback)
{
    switch (config->requestSource)
    {
#ifdef LIBOHIBOARD_UART
       case UART3_RECEIVE:
       break;
#endif

#ifdef LIBOHIBOARD_UART
       case UART3_TRANSMIT:
        // enableDmaTrigger((Uart_DeviceHandle) config->pHandler,UART_TDRE_REQEST);
       break;
#endif

#ifdef LIBOHIBOARD_ADC
       case ADC_CONV_COMPLETE:

         enableDmaTrigger((Adc_DeviceHandle)config->pHandler);
       break;
#endif

#ifdef LIBOHIBOARD_DAC

       case DAC_UPPER_POINTER:
	   case DAC_BOTTOM_POINTER:
       case DAC_BOOTH_POINTER:
    	 enableDmaTrigger((Dac_DeviceHandle)config->pHandler, config->requestSource);
       break;
#endif

    }

    /* Enable clock for DMAMUX and DMA */
    *dev->simScgcPtrDMAMUX|=dev->simScgcBitEnableDMAMUX;
    *dev->simScgcPtrDMA|=dev->simScgcBitEnableDMA;

    /* Disable the DMA channel selected and set it */
    DMAMUX_CHCFG_REG(dev->regMapMux,config->channel)=0x00;

    /* Configure source address */
    DMA_SAR_REG(dev->regMap, config->channel)=(uint32_t)config->sourceAddress;

    /* Configure destination address */
    DMA_DAR_REG(dev->regMap, config->channel)=(uint32_t)config->destinationAddress;

    /* Configure number of byte for transfer */
    DMA_DSR_BCR_REG(dev->regMap, config->channel)=DMA_DSR_BCR_BCR(config->nByteforReq);

    DMA_DCR_REG(dev->regMap, config->channel)|= DMA_DCR_ERQ_MASK|((config->transferMode<<DMA_DCR_CS_SHIFT)&DMA_DCR_CS_MASK)|  // Enable peripheral request
    		                                    ((config->disableAfterComplete<<DMA_DCR_D_REQ_SHIFT)&DMA_DCR_D_REQ_MASK)|     // Disable request after transfer complete
    		                                    DMA_DCR_SSIZE(config->sSize)|         // Set source size
												DMA_DCR_DSIZE(config->dSize)|         // Set destination size
												DMA_DCR_DINC(config->sourceOff)|      // Set source increment
												DMA_DCR_DINC(config->destinationOff);//Set destination offset


    //Enable interrupt done generation
    if (callback)
    {


        /*Enable the generation of interrupt when the major loop is terminated*/
        DMA_DCR_REG(dev->regMap, config->channel)|= DMA_DCR_EINT_MASK;

        dev->irqCallback[config->channel]=callback;

        Interrupt_enable(INTERRUPT_DMA0+config->channel);

    }

    /*Enable dma Channel source request routing on the channel indicate by config.channel */
    DMAMUX_CHCFG_REG(dev->regMapMux,config->channel) |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(config->requestSource);

    dev->deviceInit=TRUE;

    return ERROR_DMA_OK;

}


void Dma_disableChannel(Dma_DeviceHandle dev, Dma_ChannelType channel)
{

	 DMA_DCR_REG(dev->regMap, channel)&=~DMA_DCR_ERQ_MASK;

}

void DMA0_IRQHandler(void)
{

    OB_DMA0->irqCallback[DMA_CHANNEL_0]();

    /*clear interrupt request 0*/
    DMA_DSR_BCR_REG(OB_DMA0->regMap, DMA_CHANNEL_0)|= DMA_DSR_BCR_DONE_MASK;	//Clear Done Flag
}


#endif
#endif

