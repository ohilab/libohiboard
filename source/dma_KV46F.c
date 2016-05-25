/******************************************************************************
 * Copyright (C) 2016 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/source/dma_KV46F.c
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief DMA configuration and parameter for KV46F and TRWKV46F.
 */

#ifdef LIBOHIBOARD_DMA

#if defined (LIBOHIBOARD_KV46F) || \
    defined (LIBOHIBOARD_TRWKV46F)

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

#define MAX_DMA_CHANNEL 16

typedef struct Dma_Device
{
    DMA_MemMapPtr    regMap;
    DMAMUX_MemMapPtr regMapMux;

    volatile uint32_t *simScgcPtrDMAMUX;
    volatile uint32_t *simScgcPtrDMA;

    uint32_t simScgcBitEnableDMAMUX;
    uint32_t simScgcBitEnableDMA;

    uint8_t channelSource[MAX_DMA_CHANNEL];

    Interrupt_Vector errorIsrNum;

    void (*irqCallback[MAX_DMA_CHANNEL]) (void);
    void (*irqCallbackError[MAX_DMA_CHANNEL]) (void);

} Dma_Device;


/* Initialize DMA0 */

static Dma_Device dma0 = {

        .regMap                  = DMA_BASE_PTR,
        .regMapMux               = DMAMUX_BASE_PTR,

        .simScgcPtrDMAMUX        = &SIM_SCGC6,
        .simScgcPtrDMA           = &SIM_SCGC7,

        .simScgcBitEnableDMAMUX  = SIM_SCGC6_DMAMUX_MASK,
        .simScgcBitEnableDMA     = SIM_SCGC7_DMA_MASK,
        .errorIsrNum             = INTERRUPT_DMA_ERROR,

};

Dma_DeviceHandle OB_DMA0 = &dma0;

System_Errors  Dma_init(Dma_DeviceHandle dev,
                        void* pHandler,
                        Dma_RequestSource request,
                        Dma_Channel channel,
                        void *callback,
                        void *callbackError)
{

    switch (request)
    {
#ifdef LIBOHIBOARD_UART

    case DMA_REQ_UART_RECEIVE:
//        dev->channelSource[channel] = Uart_enableDmaTrigger((Uart_DeviceHandle) pHandler, request);
    break;

    case DMA_REQ_UART_TRANSMIT:
//        dev->channelSource[channel] = Uart_enableDmaTrigger((Uart_DeviceHandle) pHandler, request);
    break;

#endif

#ifdef LIBOHIBOARD_ADC

    case DMA_REQ_ADC_CONV_COMPLETE:
        //TODO: DMA_REQ_ADC_CONV_COMPLETE
        //   channelSource=enableDmaTrigger((Adc_DeviceHandle)config->pHandler, config->requestSource);
    break;

#endif

#ifdef LIBOHIBOARD_DAC
    case DMA_REQ_DAC_BUFFER:
        dev->channelSource[channel] = Dac_enableDmaTrigger((Dac_DeviceHandle)pHandler, request);

    break;
#endif
    }

    /* Enable clock for DMAMUX and DMA */
    *dev->simScgcPtrDMAMUX |= dev->simScgcBitEnableDMAMUX;
    *dev->simScgcPtrDMA |= dev->simScgcBitEnableDMA;

    /* Reset control and status register */
    DMA_CSR_REG(dev->regMap, channel) = 0;

    /* Enable interrupt done generation */
    if (callback)
    {
        Interrupt_enable(INTERRUPT_DMA0+channel);

        /* Enable the generation of interrupt when the major loop is terminated */
        DMA_CSR_REG(dev->regMap, channel) |= DMA_CSR_INTMAJOR_MASK;

        dev->irqCallback[channel] = callback;
    }

    DMA_EEI_REG(dev->regMap) &= ~(1<<channel);

    /* Enable error interrupt ? */

    if(callbackError)
    {
        DMA_EEI_REG(dev->regMap) |= 1;
        dev->irqCallbackError[channel] = callbackError;
        Interrupt_enable(dev->errorIsrNum);
    }

    /* End interrupt error enabling */


    /* Enable dma Channel source request routing on the channel indicate by config.channel */
    DMAMUX_CHCFG_REG(dev->regMapMux, channel) = 0;
    DMAMUX_CHCFG_REG(dev->regMapMux, channel) |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(dev->channelSource[channel]);

    return ERRORS_DMA_OK;
}


void Dma_disableChannel (Dma_DeviceHandle dev, Dma_Channel channel)
{
    DMA_ERQ_REG(dev->regMap) &= (~(1<<channel))&0xFF;
}

void Dma_startChannel (Dma_DeviceHandle dev, Dma_Config* config)
{


    /* Set memory address for source and destination */
    DMA_SADDR_REG(dev->regMap,config->channel) = (uint32_t)(config->sourceAddress);
    DMA_DADDR_REG(dev->regMap,config->channel) = (uint32_t)(config->destinationAddress);

    /* Set an offset for source and destination address */
    DMA_SOFF_REG(dev->regMap,config->channel) = config->sourceOff; // Source address offset of 2 bits per transaction
    DMA_DOFF_REG(dev->regMap,config->channel) = config->destinationOff; // Destination address offset of 1 bit per transaction

    /* Set source and destination data transfer size */
    DMA_ATTR_REG(dev->regMap,config->channel) = DMA_ATTR_SSIZE(config->sSize) | DMA_ATTR_DSIZE(config->dSize);

    /* Number of bytes to be transfered in each service request of the channel */
    DMA_NBYTES_MLNO_REG(dev->regMap,config->channel) = config->nByteforReq;

    /* Current major iteration count */
    DMA_CITER_ELINKNO_REG(dev->regMap,config->channel) = DMA_BITER_ELINKNO_REG(dev->regMap,config->channel) = config->nOfCycle;

    /* Adjustment value used to restore the source and destiny address to the initial value */
    DMA_SLAST_REG(dev->regMap,config->channel) = config->lsAdjust;       // Source address adjustment
    DMA_DLAST_SGA_REG(dev->regMap,config->channel) = config->ldAdjust;   // Destination address adjustment

    /* Enable request signal for channel indicate by channel */
    DMA_ERQ_REG(dev->regMap) |= (1<<config->channel)&0xFF;
}

/* Interrupt functions */

void DMA0_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_0]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(1);
}

void DMA1_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_1]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(2);
}

void DMA3_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_3]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(4);
}

void DMA4_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_4]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(8);
}

void DMA5_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_5]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(16);
}

void DMA6_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_6]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(32);
}

void DMA7_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_7]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(64);
}

void DMA8_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_8]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(128);
}

void DMA9_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_9]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(256);
}

void DMA10_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_10]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(512);
}

void DMA11_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_11]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(1024);
}

void DMA12_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_12]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(2048);
}

void DMA13_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_13]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(4096);
}

void DMA14_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_14]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(8192);
}

void DMA15_IRQHandler(void)
{
    OB_DMA0->irqCallback[DMA_CHANNEL_15]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap) |= DMA_INT_INT0(16384);
}
void DMA_Error_IRQHandler(void)
{
    uint8_t i;

    i=0;
    while((i<MAX_DMA_CHANNEL)&&(DMA_ERR_REG(OB_DMA0->regMap)&(1<<i)))
    {
        i++;
    }
    OB_DMA0->irqCallbackError[i]();
    DMA_ERR_REG(OB_DMA0->regMap) |= 1<<i;

}

#endif /* LIBOHIBOARD_K12D5 */
#endif /* LIBOHIBOARD_DMA */
