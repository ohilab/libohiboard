/* Copyright (C) 2013-2015 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *   Marco Giammarini <m.giammarini@warcomeb.it>
 *   Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 *   Nicola Orlandini <n.orlandini90@gmail.com>
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
 * @file libohiboard/source/spi_K10D10.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Niccolo' Paolinelli <nico.paolinelli@gmail.com>
 * @author Nicola Orlandini <n.orlandini90@gmail.com>
 * @brief SPI implementations for K10D10.
 */

#ifdef LIBOHIBOARD_SPI

#include "platforms.h"
#include "utility.h"
#include "spi.h"
#include "clock.h"

#if defined (LIBOHIBOARD_K10D10)

#define SPI_MAX_PINS           12

#define SPI_PIN_ENABLED        1
#define SPI_PIN_DISABLED       0

typedef struct Spi_Device {
    SPI_MemMapPtr         regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Spi_PcsPins pcsPins[SPI_MAX_PINS];
    Spi_SoutPins soutPins[SPI_MAX_PINS];
    Spi_SinPins sinPins[SPI_MAX_PINS];
    Spi_SckPins sckPins[SPI_MAX_PINS];

    volatile uint32_t* pcsPinsPtr[SPI_MAX_PINS];
    volatile uint32_t* soutPinsPtr[SPI_MAX_PINS];
    volatile uint32_t* sinPinsPtr[SPI_MAX_PINS];
    volatile uint32_t* sckPinsPtr[SPI_MAX_PINS];

    uint8_t pcsPinsMux[SPI_MAX_PINS];
    uint8_t soutPinsMux[SPI_MAX_PINS];
    uint8_t sinPinsMux[SPI_MAX_PINS];
    uint8_t sckPinsMux[SPI_MAX_PINS];

    uint8_t index;

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Spi_Device;

static Spi_Device spi0 = {
    .regMap           = SPI0_BASE_PTR,

    .simScgcPtr       = &SIM_SCGC6,
    .simScgcBitEnable = SIM_SCGC6_SPI0_MASK,

    .pcsPins          = {SPI_PINS_PTA14,
                         SPI_PINS_PTB23,
                         SPI_PINS_PTC0,
                         SPI_PINS_PTC1,
                         SPI_PINS_PTC2,
                         SPI_PINS_PTC3,
                         SPI_PINS_PTC4,
                         SPI_PINS_PTD0,
                         SPI_PINS_PTD4,
                         SPI_PINS_PTD5,
                         SPI_PINS_PTD6,
                         SPI_PINS_PTE16,
    },
    .pcsPinsPtr       = {&PORTA_PCR14,
                         &PORTB_PCR23,
                         &PORTC_PCR0,
                         &PORTC_PCR1,
                         &PORTC_PCR2,
                         &PORTC_PCR3,
                         &PORTC_PCR4,
                         &PORTD_PCR0,
                         &PORTD_PCR4,
                         &PORTD_PCR5,
                         &PORTD_PCR6,
                         &PORTE_PCR16,
    },
    .pcsPinsMux       = {2,
                         3,
                         2,
                         2,
                         2,
                         2,
                         2,
                         2,
                         2,
                         2,
                         2,
                         2,
    },

    .soutPins         = {SPI_PINS_PTA16,
                         SPI_PINS_PTC6,
                         SPI_PINS_PTD2,
                         SPI_PINS_PTE18,
    },
    .soutPinsPtr      = {&PORTA_PCR16,
                         &PORTC_PCR6,
                         &PORTD_PCR2,
                         &PORTE_PCR18
    },
    .soutPinsMux      = {2,
                         2,
                         2,
                         2,
    },

    .sinPins          = {SPI_PINS_PTA17,
                         SPI_PINS_PTC7,
                         SPI_PINS_PTD3,
                         SPI_PINS_PTE19,
    },
    .sinPinsPtr       = {&PORTA_PCR17,
                         &PORTC_PCR7,
                         &PORTD_PCR3,
                         &PORTE_PCR19,
    },
    .sinPinsMux       = {2,
                         2,
                         2,
                         2,
    },

    .sckPins          = {SPI_PINS_PTA15,
                         SPI_PINS_PTC5,
                         SPI_PINS_PTD1,
                         SPI_PINS_PTE17,
    },
    .sckPinsPtr       = {&PORTA_PCR15,
                         &PORTC_PCR5,
                         &PORTD_PCR1,
                         &PORTE_PCR17,
    },
    .sckPinsMux       = {2,
                         2,
                         2,
                         2,
    },

    .index            = 2,

    .devInitialized   = 0,
};
Spi_DeviceHandle OB_SPI0 = &spi0;

static Spi_Device spi1 = {
    .regMap           = SPI1_BASE_PTR,

    .simScgcPtr       = &SIM_SCGC6,
    .simScgcBitEnable = SIM_SCGC6_SPI1_MASK,

    .pcsPins          = {SPI_PINS_PTB9,
                         SPI_PINS_PTB10,
                         SPI_PINS_PTE0,
                         SPI_PINS_PTE4,
                         SPI_PINS_PTE5,
                         SPI_PINS_PTE6,
    },
    .pcsPinsPtr       = {&PORTB_PCR9,
                         &PORTB_PCR10,
                         &PORTE_PCR0,
                         &PORTE_PCR4,
                         &PORTE_PCR5,
                         &PORTE_PCR6,
    },
    .pcsPinsMux       = {2,
                         2,
                         2,
                         2,
                         2,
                         2,
    },

    .soutPins         = {SPI_PINS_PTB16,
                         SPI_PINS_PTE1O,
                         SPI_PINS_PTE3O,
    },
    .soutPinsPtr      = {&PORTB_PCR16,
                         &PORTE_PCR1,
                         &PORTE_PCR3,
    },
    .soutPinsMux      = {2,
                         2,
                         7,
    },

    .sinPins          = {SPI_PINS_PTB17,
                         SPI_PINS_PTE1I,
                         SPI_PINS_PTE3I,
    },
    .sinPinsPtr       = {&PORTB_PCR17,
                         &PORTE_PCR1,
                         &PORTE_PCR3,
    },
    .sinPinsMux       = {2,
                         7,
                         2,
    },

    .sckPins          = {SPI_PINS_PTB11,
                         SPI_PINS_PTE2,
    },
    .sckPinsPtr       = {&PORTB_PCR11,
                         &PORTE_PCR2,
    },
    .sckPinsMux       = {2,
                         2,
    },

    .index            = 2,

    .devInitialized   = 0,
};
Spi_DeviceHandle OB_SPI1 = &spi1;

static Spi_Device spi2 = {
    .regMap           = SPI2_BASE_PTR,

    .simScgcPtr       = &SIM_SCGC3,
    .simScgcBitEnable = SIM_SCGC3_SPI2_MASK,

    .pcsPins          = {SPI_PINS_PTB20,
                         SPI_PINS_PTD11,
                         SPI_PINS_PTD15,
    },
    .pcsPinsPtr       = {&PORTB_PCR20,
                         &PORTD_PCR11,
                         &PORTD_PCR15,
    },
    .pcsPinsMux       = {2,
                         2,
                         2,
    },

    .soutPins         = {SPI_PINS_PTB22,
                         SPI_PINS_PTD13,
    },
    .soutPinsPtr      = {&PORTB_PCR22,
                         &PORTD_PCR13,
    },
    .soutPinsMux      = {2,
                         2,
    },

    .sinPins          = {SPI_PINS_PTB23I,
                         SPI_PINS_PTD14,
    },
    .sinPinsPtr       = {&PORTB_PCR23,
                         &PORTD_PCR14,
    },
    .sinPinsMux       = {2,
                         2,
    },

    .sckPins          = {SPI_PINS_PTB21,
                         SPI_PINS_PTD12,
    },
    .sckPinsPtr       = {&PORTB_PCR21,
                         &PORTD_PCR12,
    },
    .sckPinsMux       = {2,
                         2,
    },

    .index            = 2,

    .devInitialized   = 0,

};
Spi_DeviceHandle OB_SPI2 = &spi2;

static uint8_t  Spi_pbrDiv[] = {2, 3, 5, 7};
static uint16_t Spi_brDiv[] = {
        /*00*/     2, /*01*/     4, /*02*/     6, /*03*/     8,
        /*04*/    16, /*05*/    32, /*06*/    64, /*07*/   128,
        /*08*/   256, /*09*/   512, /*10*/  1024, /*11*/  2048,
        /*12*/  4096, /*13*/  8192, /*14*/ 16384, /*15*/ 32768,
};

System_Errors Spi_setBaudrate(Spi_DeviceHandle dev, uint32_t speed)
{
    SPI_MemMapPtr regmap = dev->regMap;
    uint32_t tempReg = 0;

    uint32_t busClk;
    float spiClk;
	float PBRDIV;
	float BRDIV;
	float temporary1;
	float temporary2;
    uint32_t diff = 0xFFFFFFFF;
    uint32_t maxDiff = 0xFFFFFFFF;
    uint8_t dbr = 0;
    uint8_t pbr = 0;
    uint8_t br = 0;
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t k = 0;
    uint8_t index = 0;

    busClk = Clock_getFrequency(CLOCK_BUS);
    for(i = 0; i < 2; i++)
    {
        for(j = 0; j < 4; j++)
        {
            for(k = 0; k < 16; k++)
            {
            	spiClk = (busClk * (1U+i) / Spi_pbrDiv[j] / Spi_brDiv[k]);
                if(speed < spiClk)
                {
                    if((spiClk - speed) < diff)
                    {
                        diff = spiClk - speed;
                        dbr = i;
                        pbr = j;
                        br = k;
                    }
                }
                else
                {
                    if((speed - spiClk) < diff)
                    {
                        diff = speed - spiClk;
                        dbr = i;
                        pbr = j;
                        br = k;
                    }
                }
            }
        }
    }

    if(diff == maxDiff) return ERRORS_SPI_BAUDRATE_NOT_FOUND;

    for (index = 0; index < dev->index; index++)
    {
        tempReg = SPI_CTAR_REG(regmap, index);
        tempReg &= ~(SPI_CTAR_DBR_MASK | SPI_CTAR_PBR_MASK | SPI_CTAR_BR_MASK);
        tempReg |= ((dbr << SPI_CTAR_DBR_SHIFT) | SPI_CTAR_PBR(pbr) | SPI_CTAR_BR(br));
        SPI_CTAR_REG(regmap, index) = tempReg;
    }

    return ERRORS_NO_ERROR;
}

static System_Errors Spi_setPcsPin(Spi_DeviceHandle dev, Spi_PcsPins pcsPin)
{
    uint8_t devPinIndex;

    if (dev->devInitialized == 0)
        return ERRORS_SPI_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->pcsPins[devPinIndex] == pcsPin)
        {
            *(dev->pcsPinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->pcsPinsMux[devPinIndex]);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

static System_Errors Spi_setSoutPin(Spi_DeviceHandle dev, Spi_SoutPins soutPin)
{
    uint8_t devPinIndex;

    if (dev->devInitialized == 0)
        return ERRORS_SPI_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->soutPins[devPinIndex] == soutPin)
        {
            *(dev->soutPinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->soutPinsMux[devPinIndex]);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

static System_Errors Spi_setSinPin(Spi_DeviceHandle dev, Spi_PcsPins sinPin)
{
    uint8_t devPinIndex;

    if (dev->devInitialized == 0)
        return ERRORS_SPI_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->sinPins[devPinIndex] == sinPin)
        {
            *(dev->sinPinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->sinPinsMux[devPinIndex]);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

static System_Errors Spi_setSckPin(Spi_DeviceHandle dev, Spi_PcsPins sckPin)
{
    uint8_t devPinIndex;

    if (dev->devInitialized == 0)
        return ERRORS_SPI_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < SPI_MAX_PINS; ++devPinIndex)
    {
        if (dev->sckPins[devPinIndex] == sckPin)
        {
            *(dev->sckPinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->sckPinsMux[devPinIndex]);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_SPI_NO_PIN_FOUND;
}

System_Errors Spi_init (Spi_DeviceHandle dev, Spi_Config *config)
{
    SPI_MemMapPtr regmap = dev->regMap;
    Spi_DeviceType devType = config->devType;
    uint32_t baudrate = config->baudrate;
    uint32_t tempReg = 0;
    System_Errors error = ERRORS_NO_ERROR;

    uint8_t sckpol = 0;
    uint8_t sckphase = 0;

    uint8_t frameSize = 7;

    uint8_t index = 0;

    if (dev->devInitialized) return error = ERRORS_SPI_DEVICE_JUST_INIT;


    /* Turn on clock */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* Common CTARn parameters definition */
    if(config->sckPolarity == SPI_SCK_INACTIVE_STATE_LOW)
        sckpol = 0;
    else
        sckpol = 1;

    if(config->sckPhase == SPI_SCK_LEADING_EDGE_DATA_CAPTURED)
        sckphase = 0;
    else
        sckphase = 1;

    if (config->frameSize > 3)
        frameSize =  config->frameSize;

    /* Select device type */
    if (devType == SPI_MASTER_MODE)
    {
        if (config->continuousSck == SPI_CONTINUOUS_SCK)
        {
            SPI_MCR_REG(regmap) = (SPI_MCR_MSTR_MASK | SPI_MCR_CONT_SCKE_MASK | SPI_MCR_PCSIS(0xF)| 0);
        }
        else
        {
            SPI_MCR_REG(regmap) = (SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(0xF) | 0);
        }


        for(index = 0; index < dev->index; index++)
        {
            SPI_CTAR_REG(regmap,index) = (((frameSize - 1) << SPI_CTAR_FMSZ_SHIFT) | (sckpol << SPI_CTAR_CPOL_SHIFT) | (sckphase << SPI_CTAR_CPHA_SHIFT) | 0);
        }

        error = Spi_setBaudrate(dev, config->baudrate);
    }
    else if (devType == SPI_SLAVE_MODE)
    {
        if (config->continuousSck == SPI_CONTINUOUS_SCK)
        {
            SPI_MCR_REG(regmap) = (SPI_MCR_CONT_SCKE_MASK  | SPI_MCR_PCSIS(0xF)| 0);
        }
        else
        {
            SPI_MCR_REG(regmap) = (SPI_MCR_PCSIS(0xF) | 0);
        }

        for(index = 0; index < (dev->index-1); index++)
        {
            SPI_CTAR_SLAVE_REG(regmap,index) = (((frameSize - 1) << SPI_CTAR_FMSZ_SHIFT) | (sckpol << SPI_CTAR_CPOL_SHIFT) | (sckphase << SPI_CTAR_CPHA_SHIFT) | 0);
        }
    }
    else
    {
        return ERRORS_PARAM_VALUE;
    }

    dev->devInitialized = 1;

    /* Config the port controller */
    if (config->pcs0Pin != SPI_PINS_PCSNONE)
        Spi_setPcsPin(dev, config->pcs0Pin);
    if (config->pcs1Pin != SPI_PINS_PCSNONE)
        Spi_setPcsPin(dev, config->pcs1Pin);
    if (config->pcs2Pin != SPI_PINS_PCSNONE)
        Spi_setPcsPin(dev, config->pcs2Pin);
    if (config->pcs3Pin != SPI_PINS_PCSNONE)
        Spi_setPcsPin(dev, config->pcs3Pin);
    if (config->pcs4Pin != SPI_PINS_PCSNONE)
        Spi_setPcsPin(dev, config->pcs4Pin);

    if (config->soutPin != SPI_PINS_SOUTNONE)
        Spi_setSoutPin(dev, config->soutPin);

    if (config->sinPin != SPI_PINS_SINNONE)
        Spi_setSinPin(dev, config->sinPin);

    if (config->sckPin != SPI_PINS_SCKNONE)
        Spi_setSckPin(dev, config->sckPin);

    return error;
}


System_Errors Spi_readByte (Spi_DeviceHandle dev, uint8_t *data)
{
    SPI_MemMapPtr regmap = dev->regMap;

    // Wait till TX FIFO is Empty.
    while((SPI_SR_REG(regmap) & SPI_SR_TFFF_MASK) != SPI_SR_TFFF_MASK);

    SPI_PUSHR_REG(regmap) = 0x0000;

    SPI_MCR_REG(regmap) &= ~SPI_MCR_HALT_MASK;

    // Wait till transmit complete
    while (((SPI_SR_REG(regmap) & SPI_SR_TCF_MASK)) != SPI_SR_TCF_MASK);

    // Clear Transmit Flag.
    SPI_SR_REG(regmap) |= SPI_SR_TFFF_MASK;

    SPI_MCR_REG(regmap) &= ~SPI_MCR_HALT_MASK;

    // wait till RX_FIFO is not empty
    while((SPI_SR_REG(regmap) & SPI_SR_RFDF_MASK) != SPI_SR_RFDF_MASK);

    *data = SPI_POPR_REG(regmap) & 0x000000FF;

    // Clear the RX FIFO Drain Flag
    SPI_SR_REG(regmap) |= SPI_SR_RFDF_MASK;

    return ERRORS_NO_ERROR;
}

System_Errors Spi_writeByte (Spi_DeviceHandle dev, uint8_t data)
{
    SPI_MemMapPtr regmap = dev->regMap;

    uint16_t x = 0;

    // Wait till TX FIFO is Empty.
    while((SPI_SR_REG(regmap)  & SPI_SR_TFFF_MASK) != 0x2000000U);

    // Transmit Byte on SPI
    SPI_PUSHR_REG(regmap) = data;

    SPI_MCR_REG(regmap) &= ~SPI_MCR_HALT_MASK;

    // Wait till transmit complete
    while (((SPI_SR_REG(regmap) & SPI_SR_TCF_MASK)) != SPI_SR_TCF_MASK) ;

    // Clear Transmit Flag.
    SPI_SR_REG(regmap) |= SPI_SR_TCF_MASK;

    (void)SPI_POPR_REG(regmap);

    return ERRORS_NO_ERROR;
}

#endif /* LIBOHIBOARD_K10D10 */

#endif /* LIBOHIBOARD_SPI */
