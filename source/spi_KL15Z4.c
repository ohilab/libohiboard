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
 * @file libohiboard/source/spi_KL15Z4.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief SPI implementations for KL15Z4.
 */

#ifdef LIBOHIBOARD_SPI

#include "platforms.h"
#include "utility.h"
#include "spi.h"
#include "clock.h"

#if defined (LIBOHIBOARD_KL15Z4)

#define SPI_MAX_PINS           8

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

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Spi_Device;

static Spi_Device spi0 = {
    .regMap           = SPI0_BASE_PTR,

    .simScgcPtr       = &SIM_SCGC4,
    .simScgcBitEnable = SIM_SCGC4_SPI0_MASK,

    .pcsPins          = {SPI_PINS_PTA14,
                         SPI_PINS_PTC4,
                         SPI_PINS_PTD0,
                         SPI_PINS_PTE16,
    },
    .pcsPinsPtr       = {&PORTA_PCR14,
                         &PORTC_PCR4,
                         &PORTD_PCR0,
                         &PORTE_PCR16,
    },
    .pcsPinsMux       = {2,
                         2,
                         2,
                         2,
    },

    .soutPins         = {SPI_PINS_PTA16O,
                         SPI_PINS_PTA17O,
                         SPI_PINS_PTC6O,
                         SPI_PINS_PTC7O,
                         SPI_PINS_PTD2O,
                         SPI_PINS_PTD3O,
                         SPI_PINS_PTE18O,
                         SPI_PINS_PTE19O,
    },
    .soutPinsPtr      = {&PORTA_PCR16,
                         &PORTA_PCR17,
                         &PORTC_PCR6,
                         &PORTC_PCR7,
                         &PORTD_PCR2,
                         &PORTD_PCR3,
                         &PORTE_PCR18,
                         &PORTE_PCR19,
    },
    .soutPinsMux      = {2,
                         5,
                         2,
                         5,
                         2,
                         5,
                         2,
                         5,
    },

    .sinPins          = {SPI_PINS_PTA16I,
                         SPI_PINS_PTA17I,
                         SPI_PINS_PTC6I,
                         SPI_PINS_PTC7I,
                         SPI_PINS_PTD2I,
                         SPI_PINS_PTD3I,
                         SPI_PINS_PTE18I,
                         SPI_PINS_PTE19I,
    },
    .sinPinsPtr       = {&PORTA_PCR16,
                         &PORTA_PCR17,
                         &PORTC_PCR6,
                         &PORTC_PCR7,
                         &PORTD_PCR2,
                         &PORTD_PCR3,
                         &PORTE_PCR18,
                         &PORTE_PCR19,
    },
    .sinPinsMux       = {5,
                         2,
                         5,
                         2,
                         5,
                         2,
                         5,
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

    .devInitialized   = 0,
};
Spi_DeviceHandle SPI0 = &spi0;

static Spi_Device spi1 = {
    .regMap           = SPI1_BASE_PTR,

    .simScgcPtr       = &SIM_SCGC4,
    .simScgcBitEnable = SIM_SCGC4_SPI1_MASK,

    .pcsPins          = {SPI_PINS_PTB10,
                         SPI_PINS_PTD4,
                         SPI_PINS_PTE4,
    },
    .pcsPinsPtr       = {&PORTB_PCR10,
                         &PORTD_PCR4,
                         &PORTE_PCR4,
    },
    .pcsPinsMux       = {2,
                         2,
                         2,
    },

    .soutPins         = {SPI_PINS_PTB16O,
                         SPI_PINS_PTB17O,
                         SPI_PINS_PTD6O,
                         SPI_PINS_PTD7O,
                         SPI_PINS_PTE1O,
                         SPI_PINS_PTE3O,
    },
    .soutPinsPtr      = {&PORTB_PCR16,
                         &PORTB_PCR17,
                         &PORTD_PCR6,
                         &PORTD_PCR7,
                         &PORTE_PCR1,
                         &PORTE_PCR3,
    },
    .soutPinsMux      = {2,
                         5,
                         2,
                         5,
                         2,
                         5,
    },

    .sinPins          = {SPI_PINS_PTB16I,
                         SPI_PINS_PTB17I,
                         SPI_PINS_PTD2I,
                         SPI_PINS_PTD3I,
                         SPI_PINS_PTE18I,
                         SPI_PINS_PTE19I,
    },
    .sinPinsPtr       = {&PORTB_PCR16,
                         &PORTB_PCR17,
                         &PORTD_PCR6,
                         &PORTD_PCR7,
                         &PORTE_PCR1,
                         &PORTE_PCR3,
    },
    .sinPinsMux       = {5,
                         2,
                         5,
                         2,
                         5,
                         2,
    },

    .sckPins          = {SPI_PINS_PTB11,
                         SPI_PINS_PTD5,
                         SPI_PINS_PTE2,
    },
    .sckPinsPtr       = {&PORTB_PCR11,
                         &PORTD_PCR5,
                         &PORTE_PCR2,
    },
    .sckPinsMux       = {2,
                         2,
                         2,

    },

    .devInitialized   = 0,
};
Spi_DeviceHandle SPI1 = &spi1;

static uint16_t Spi_prDiv[]  = {
        /*00*/     2, /*01*/     4, /*02*/     8, /*03*/    16,
        /*04*/    32, /*05*/    64, /*06*/   128, /*07*/   256,
        /*08*/   512
};

System_Errors Spi_setBaudrate(Spi_DeviceHandle dev, uint32_t speed)
{
    SPI_MemMapPtr regmap = dev->regMap;
    uint32_t tempReg = 0;

    uint32_t busClk;
    float spiClk;
	float temporary1;
    uint32_t diff = 0xFFFFFFFF;
    uint32_t maxDiff = 0xFFFFFFFF;
    uint8_t ppr = 0;
    uint8_t pr = 0;

    uint8_t i = 0;
    uint8_t j = 0;


    busClk = Clock_getFrequency(CLOCK_BUS);
    for(i = 0; i < 8; i++)
    {
        for(j = 0; j < 9; j++)
        {
        	temporary1 = (float)((i+1) * Spi_prDiv[j]);
            spiClk = (float)(busClk/temporary1);
            if(speed < spiClk)
            {
                if((spiClk - speed) < diff)
                {
                    diff = spiClk - speed;
                    ppr = i;
                    pr = j;
                }
            }
            else
            {
                if((speed - spiClk) < diff)
                {
                    diff = speed - spiClk;
                    ppr = i;
                    pr = j;
                }
            }
        }
    }

    if(diff == maxDiff) return ERRORS_SPI_BAUDRATE_NOT_FOUND;

    SPI_BR_REG(regmap) = SPI_BR_SPPR(ppr) | SPI_BR_SPR(pr);

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

    /* Select device type */
    if (devType == SPI_MASTER_MODE)
    {
        SPI_C1_REG(regmap) = (sckpol << SPI_C1_CPOL_SHIFT) | (sckphase << SPI_C1_CPHA_SHIFT) | SPI_C1_MSTR_MASK | SPI_C1_SPE_MASK | 0;
    }
    else if (devType == SPI_SLAVE_MODE)
    {
        SPI_C1_REG(regmap) = (sckpol << SPI_C1_CPOL_SHIFT) | (sckphase << SPI_C1_CPHA_SHIFT) | SPI_C1_SPE_MASK | 0;
    }
    else
    {
        return ERRORS_PARAM_VALUE;
    }

    error = Spi_setBaudrate(dev, config->baudrate);

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

    /* Copy dummy data in D register */
    SPI_D_REG(regmap) = 0xFF;
    /* Wait until slave replay */
    while (!(SPI_S_REG(regmap) & SPI_S_SPRF_MASK));
    /* Save data register */
    *data = SPI_D_REG(regmap);

    return ERRORS_NO_ERROR;
}

System_Errors Spi_writeByte (Spi_DeviceHandle dev, uint8_t data)
{
    SPI_MemMapPtr regmap = dev->regMap;

    /* Wait until SPTEF bit is 1 (transmit buffer is empty) */
    while (!(SPI_S_REG(regmap) & SPI_S_SPTEF_MASK));
    (void) SPI_S_REG(regmap);
    /* Copy data in D register */
    SPI_D_REG(regmap) = data;
    /* Wait until slave replay */
    while (!(SPI_S_REG(regmap) & SPI_S_SPRF_MASK));
    /* Read data register */
    (void) SPI_D_REG(regmap);

    return ERRORS_NO_ERROR;
}

#endif /* LIBOHIBOARD_KL15Z4 */

#endif /* LIBOHIBOARD_SPI */
