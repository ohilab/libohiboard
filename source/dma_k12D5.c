/*
 * dma_k64F12.c
 *
 *  Created on: 20/nov/2015
 *      Author: m.civale
 */

#ifdef LIBOHIBOARD_DMA

#if defined (LIBOHIBOARD_K12D5)

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

    bool deviceInit;

    volatile uint32_t *simScgcPtrDMAMUX;
    volatile uint32_t *simScgcPtrDMA;

    uint32_t simScgcBitEnableDMAMUX;
    uint32_t simScgcBitEnableDMA;

    uint8_t channelSource[MAX_DMA_CHANNEL];

    void (*irqCallback[MAX_DMA_CHANNEL]) (void);

}Dma_Device;



/* Initialize DMA0 */

static Dma_Device dma0={

        .regMap                  = DMA_BASE_PTR,
        .regMapMux               = DMAMUX_BASE_PTR,

        .simScgcPtrDMAMUX        = &SIM_SCGC6,
        .simScgcPtrDMA           = &SIM_SCGC7,

        .simScgcBitEnableDMAMUX  = SIM_SCGC6_DMAMUX_MASK,
        .simScgcBitEnableDMA     = SIM_SCGC7_DMA_MASK,
        .deviceInit              = FALSE,
};


Dma_DeviceHandle OB_DMA0=&dma0;



System_Errors  Dma_init(Dma_DeviceHandle dev, dma_ConfigType* config, void *callback)
{

    uint8_t channelSource;

    switch (config->requestSource)
    {
       case DMA_REQ_UART_RECEIVE:
       case DMA_REQ_UART_TRANSMIT:
           dev->channelSource[config->channel]=Uart_enableDmaTrigger((Uart_DeviceHandle) config->pHandler, config->requestSource);
       break;

       case DMA_REQ_ADC_CONV_COMPLETE:
        //   channelSource=enableDmaTrigger((Adc_DeviceHandle)config->pHandler, config->requestSource);
       break;
    }

    /* Enable clock for DMAMUX and DMA */
    *dev->simScgcPtrDMAMUX|=dev->simScgcBitEnableDMAMUX;
    *dev->simScgcPtrDMA|=dev->simScgcBitEnableDMA;
/******************************************************************************/
    // Enable request signal for channel indicate by channel
    DMA_ERQ_REG(dev->regMap) = (1<<config->channel)&0xFF;

    // Set memory address for source and destination
    DMA_SADDR_REG(dev->regMap,config->channel) = (uint32_t)(config->sourceAddress);
    DMA_DADDR_REG(dev->regMap,config->channel) = (uint32_t)(config->destinationAddress);

    // Set an offset for source and destination address
    DMA_SOFF_REG(dev->regMap,config->channel) = config->sourceOff; // Source address offset of 2 bits per transaction
    DMA_DOFF_REG(dev->regMap,config->channel) = config->destinationOff; // Destination address offset of 1 bit per transaction

    // Set source and destination data transfer size
    DMA_ATTR_REG(dev->regMap,config->channel) = DMA_ATTR_SSIZE(config->sSize) | DMA_ATTR_DSIZE(config->dSize);

    // Number of bytes to be transfered in each service request of the channel
    DMA_NBYTES_MLNO_REG(dev->regMap,config->channel) =config->nByteforReq;

    // Current major iteration count (a single iteration of 5 bytes)
    DMA_CITER_ELINKNO_REG(dev->regMap,config->channel) = DMA_BITER_ELINKNO_REG(dev->regMap,config->channel) = config->nOfCycle;

    // Adjustment value used to restore the source and destiny address to the initial value
    DMA_SLAST_REG(dev->regMap,config->channel) = config->lsAdjust;       // Source address adjustment
    DMA_DLAST_SGA_REG(dev->regMap,config->channel) = config->ldAdjust;   // Destination address adjustment


    // Setup control and status register
    DMA_CSR_REG(dev->regMap,config->channel) = 0;

/***************************************************************************************/

    //Enable interrupt done generation
    if (callback)
    {
        Interrupt_enable(INTERRUPT_DMA0+config->channel);

        /*Enable the generation of interrupt when the major loop is terminated*/
        DMA_CSR_REG(dev->regMap,config->channel)|= DMA_CSR_INTMAJOR_MASK;

//        /*enable the dma generation interrupt of channel indicated by config.channel*/
//        dma->INT|=1<<config->channel;

        dev->irqCallback[config->channel]=callback;

    }

//    /*Enable dma Channel source request routing on the channel indicate by config.channel */
    DMAMUX_CHCFG_REG(dev->regMapMux,config->channel)=0;
    DMAMUX_CHCFG_REG(dev->regMapMux,config->channel) |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(dev->channelSource[config->channel]);

    return ERROR_DMA_OK;

}


void Dma_disableChannel(Dma_DeviceHandle dev, Dma_ChannelType channel)
{
    DMA_ERQ_REG(dev->regMap)&= (~(1<<channel))&0xFF;
}

void DMA0_IRQHandler(void)
{

    OB_DMA0->irqCallback[DMA_CHANNEL_0]();

    /*clear interrupt request 0*/
    DMA_INT_REG(OB_DMA0->regMap)|=DMA_INT_INT0(1);
}

void Dma_startChannel(Dma_DeviceHandle dev, Dma_ChannelType channel, uint16_t transfertNumber, dma_ConfigType* config)
{

        // Set memory address for source and destination
        DMA_DADDR_REG(dev->regMap,config->channel) = (uint32_t)(config->destinationAddress);
        /* Set number of iteration of major loop */
        DMA_CITER_ELINKNO_REG(dev->regMap,config->channel) = DMA_BITER_ELINKNO_REG(dev->regMap,config->channel) = config->nOfCycle;
        /* Enable dma Channel source request routing on the channel indicate by config.channel */
        DMAMUX_CHCFG_REG(dev->regMapMux,config->channel)=0;
        DMAMUX_CHCFG_REG(dev->regMapMux,config->channel) |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(dev->channelSource[config->channel]);
        /* Enable Channel */
        DMA_ERQ_REG(dev->regMap)|= (1<<config->channel)&0xFF;
}

#endif
#endif

