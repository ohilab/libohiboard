/* Copyright (C) 2014-2015 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Alessio Paolucci <a.paolucci89@gmail.com>
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
 * @file libohiboard/source/i2c_K64F12.c
 * @author Alessio Paolucci <a.paolucci89@gmail.com>
 * @brief I2C implementations for K64F12 and FRDMK64F.
 */

#ifdef LIBOHIBOARD_IIC

#if defined (LIBOHIBOARD_K64F12)     || \
    defined (LIBOHIBOARD_FRDMK64F)

#include "platforms.h"
#include "system.h"
#include "i2c.h"
#include "clock.h"

#define IIC_MAX_PINS           9

#define IIC_PIN_ENABLED        1
#define IIC_PIN_DISABLED       0

static uint8_t Iic_firstRead = 0;

typedef struct Iic_Device {
    I2C_MemMapPtr 		  regMap;

    volatile uint32_t* simScgcPtr;    /**< SIM_SCGCx register for the device. */
    uint32_t simScgcBitEnable;       /**< SIM_SCGC enable bit for the device. */

    Iic_SclPins sclPins[IIC_MAX_PINS];
    Iic_SdaPins sdaPins[IIC_MAX_PINS];

    volatile uint32_t* sclPinsPtr[IIC_MAX_PINS];
    volatile uint32_t* sdaPinsPtr[IIC_MAX_PINS];
    uint8_t sclPinsMux[IIC_MAX_PINS];
    uint8_t sdaPinsMux[IIC_MAX_PINS];

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Iic_Device;

static Iic_Device iic0 = {
        .regMap           = I2C0_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC4,
        .simScgcBitEnable = SIM_SCGC4_I2C0_MASK,

        .sclPins          = {IIC_PINS_PTB0,
        		             IIC_PINS_PTB2,
        		             IIC_PINS_PTD2,
        		             IIC_PINS_PTD8,
        		             IIC_PINS_PTE24,
        },
        .sclPinsPtr       = {&PORTB_PCR0,
        		             &PORTB_PCR2,
        		             &PORTD_PCR2,
        		             &PORTD_PCR8,
        		             &PORTE_PCR24,
        },
        .sclPinsMux       = {2,
        		             2,
        		             7,
        		             2,
        		             5,
        },

        .sdaPins          = {IIC_PINS_PTB1,
        		             IIC_PINS_PTB3,
        		             IIC_PINS_PTD3,
        		             IIC_PINS_PTD9,
        		             IIC_PINS_PTE25,
        },
        .sdaPinsPtr       = {&PORTB_PCR1,
        		             &PORTB_PCR3,
        		             &PORTD_PCR3,
        		             &PORTD_PCR9,
        		             &PORTE_PCR25,
        },
        .sdaPinsMux       = {2,
        		             2,
        		             7,
        		             2,
        		             5,
        },

        .devInitialized = 0,
};
Iic_DeviceHandle IIC0 = &iic0;

static Iic_Device iic1 = {
        .regMap           = I2C1_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC4,
        .simScgcBitEnable = SIM_SCGC4_I2C1_MASK,

        .sclPins          = {IIC_PINS_PTC10,
        		             IIC_PINS_PTE1,
        },
        .sclPinsPtr       = {&PORTC_PCR10,
        		             &PORTE_PCR1,
        },
        .sclPinsMux       = {2,
        		             6,
        },

        .sdaPins          = {IIC_PINS_PTC11,
        		             IIC_PINS_PTE0,
        },
        .sdaPinsPtr       = {&PORTC_PCR11,
        		             &PORTE_PCR0,
        },
        .sdaPinsMux       = {2,
        		             6,
        },

        .devInitialized = 0,
};
Iic_DeviceHandle IIC1 = &iic1;

static Iic_Device iic2 = {
        .regMap           = I2C2_BASE_PTR,

        .simScgcPtr       = &SIM_SCGC1,
        .simScgcBitEnable = SIM_SCGC1_I2C2_MASK,

        .sclPins          = {IIC_PINS_PTA12,
        		             IIC_PINS_PTA14,
        },
        .sclPinsPtr       = {&PORTA_PCR12,
        		             &PORTA_PCR14,
        },
        .sclPinsMux       = {5,
        		             5,
        },

        .sdaPins          = {IIC_PINS_PTA11,
        		             IIC_PINS_PTA13,
        },
        .sdaPinsPtr       = {&PORTA_PCR11,
        		             &PORTA_PCR13,
        },
        .sdaPinsMux       = {5,
        		             5,
        },

        .devInitialized = 0,
};
Iic_DeviceHandle IIC2 = &iic2;

/* See Table 50-41 I2C Divider and Hold Values */
static uint16_t Iic_sclDivTab[] = {
    /* 00 */   20, /* 01 */   22, /* 02 */   24, /* 03 */   26,
    /* 04 */   28, /* 05 */   30, /* 06 */   34, /* 07 */   40,
    /* 08 */   28, /* 09 */   32, /* 0A */   36, /* 0B */   40,
    /* 0C */   44, /* 0D */   48, /* 0E */   56, /* 0F */   68,

    /* 10 */   48, /* 11 */   56, /* 12 */   64, /* 13 */   72,
    /* 14 */   80, /* 15 */   88, /* 16 */  104, /* 17 */  128,
    /* 18 */   80, /* 19 */   96, /* 1A */  112, /* 1B */  128,
    /* 1C */  144, /* 1D */  160, /* 1E */  192, /* 1F */  240,

    /* 20 */  160, /* 21 */  192, /* 22 */  224, /* 23 */  256,
    /* 24 */  288, /* 25 */  320, /* 26 */  384, /* 27 */  480,
    /* 28 */  320, /* 29 */  384, /* 2A */  448, /* 2B */  512,
    /* 2C */  576, /* 2D */  640, /* 2E */  768, /* 2F */  960,

    /* 30 */  640, /* 31 */  768, /* 32 */  896, /* 33 */ 1024,
    /* 34 */ 1152, /* 35 */ 1280, /* 36 */ 1536, /* 37 */ 1920,
    /* 38 */ 1280, /* 39 */ 1536, /* 3A */ 1792, /* 3B */ 2048,
    /* 3C */ 2304, /* 3D */ 2560, /* 3E */ 3072, /* 3F */ 3840,
};

static System_Errors Iic_setSclPin(Iic_DeviceHandle dev, Iic_SclPins sclPin)
{
    uint8_t devPinIndex;

    if (dev->devInitialized == 0)
        return ERRORS_IIC_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < IIC_MAX_PINS; ++devPinIndex)
    {
        if (dev->sclPins[devPinIndex] == sclPin)
        {
            *(dev->sclPinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->sclPinsMux[devPinIndex]);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_IIC_NO_PIN_FOUND;
}

static System_Errors Iic_setSdaPin(Iic_DeviceHandle dev, Iic_SclPins sdaPin)
{
    uint8_t devPinIndex;

    if (dev->devInitialized == 0)
        return ERRORS_IIC_DEVICE_NOT_INIT;

    for (devPinIndex = 0; devPinIndex < IIC_MAX_PINS; ++devPinIndex)
    {
        if (dev->sdaPins[devPinIndex] == sdaPin)
        {
            *(dev->sdaPinsPtr[devPinIndex]) =
                PORT_PCR_MUX(dev->sdaPinsMux[devPinIndex]);
            return ERRORS_NO_ERROR;
        }
    }
    return ERRORS_IIC_NO_PIN_FOUND;
}

/*
 * @brief Setting icr parameter of I2C_F register for generate the desired baudrate
 *
 * Thank for this function at https://github.com/laswick/kinetis/blob/master/phase2_embedded_c/i2c.c
 */
static System_Errors setBaudrate(Iic_DeviceHandle dev, uint32_t speed)
{
    I2C_MemMapPtr regmap = dev->regMap;
    uint32_t tempReg = 0;

    uint32_t busClk;
    uint32_t i2cClk;
    uint32_t error;
    uint32_t bestError = 0xFFFF;
    uint8_t bestIcr = 0xFF;
    uint8_t icr;
    uint8_t mul;
    uint16_t slt;

    busClk = Clock_getFrequency(CLOCK_BUS);

    for (icr = 0; icr < sizeof(Iic_sclDivTab) / sizeof(Iic_sclDivTab[0]); icr++)
    {
        i2cClk = busClk / Iic_sclDivTab[icr];
        if (i2cClk > speed)
        {
            i2cClk /= 2;
            if (i2cClk > speed)
            {
                i2cClk /= 2;
                if (i2cClk > speed)
                    continue;
            }
        }
        error = speed - i2cClk;
        if (error < bestError)
        {
            bestError = error;
            bestIcr = icr;
        }
    }

    if (bestIcr == 0xFF)
    {
        return ERRORS_IIC_WRONG_BAUDRATE;
    }

    icr = bestIcr;
    i2cClk = busClk / Iic_sclDivTab[bestIcr];
    if (i2cClk > speed)
    {
        i2cClk /= 2;
        if (i2cClk > speed)
        {
            mul = 2;
        }
        else
        {
            mul = 1;
        }
    }
    else
    {
        mul = 0;
    }

    tempReg = I2C_F_REG(regmap);
    tempReg &= ~(I2C_F_MULT_MASK | I2C_F_ICR_MASK);
    tempReg |= (I2C_F_MULT(mul) | I2C_F_ICR(icr));
    I2C_F_REG(regmap) = tempReg;

    slt = Iic_sclDivTab[icr] / 2 + 1;

    return ERRORS_NO_ERROR;
}

/**
 *
 * @param dev
 */
System_Errors Iic_init(Iic_DeviceHandle dev, Iic_Config *config)
{
	if (dev->devInitialized) return ERRORS_IIC_DEVICE_JUST_INIT;

    I2C_MemMapPtr regmap = dev->regMap;
    Iic_DeviceType devType = config->devType;
    uint32_t baudrate = config->baudRate;

    System_Errors errors;

    /* Turn on clock */
    *dev->simScgcPtr |= dev->simScgcBitEnable;

    /* Select device type */
    if (devType == IIC_MASTER_MODE)
    {
        errors = setBaudrate(dev, config->baudRate);

        if (errors != ERRORS_NO_ERROR)
            return errors;

        /* enable IIC */
        I2C_C1_REG(regmap) = I2C_C1_IICEN_MASK;
    }
    else
    {
    	/* TODO: implement slave setup */
    }

    /* Setup for manage dummy read. */
    Iic_firstRead = 1;

    dev->devInitialized = 1;

    /* Config the port controller */
    if (config->sclPin != IIC_PINS_SCLNONE)
        Iic_setSclPin(dev, config->sclPin);

    if (config->sdaPin != IIC_PINS_SDANONE)
        Iic_setSdaPin(dev, config->sdaPin);

    return ERRORS_NO_ERROR;
}

void Iic_start (Iic_DeviceHandle dev)
{
    /* If we are in communication set repeat star flag */
    if (I2C_S_REG(dev->regMap) & I2C_S_BUSY_MASK)
    {
        I2C_C1_REG(dev->regMap) |= I2C_C1_RSTA_MASK;
    }
    else
    {
        I2C_C1_REG(dev->regMap) |= I2C_C1_TX_MASK;
        I2C_C1_REG(dev->regMap) |= I2C_C1_MST_MASK;
    }
    /* Setup for manage dummy read. */
    Iic_firstRead = 1;
}

void Iic_stop (Iic_DeviceHandle dev)
{
	uint8_t i;

    I2C_C1_REG(dev->regMap) &= ~I2C_C1_MST_MASK;
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TX_MASK;

    for (i = 0; i < 100; ++i )
   {
//    	//__asm ("nop");
   }

    /* Setup for manage dummy read. */
    Iic_firstRead = 1;
}

static System_Errors Iic_waitTxTransfer (Iic_DeviceHandle dev)
{
    uint16_t i, timeoutResult = 0;

    /* Wait for interrupt flag */
    for (i = 0; i < 1000; ++i)
    {
        if (I2C_S_REG(dev->regMap) & I2C_S_IICIF_MASK)
        {
            timeoutResult = 1;
            break;
        }
    }

    if (timeoutResult == 0)
        return ERRORS_IIC_TX_TIMEOUT;

    /* Reset value */
    I2C_S_REG(dev->regMap) |= I2C_S_IICIF_MASK;
    timeoutResult = 0;

    /* Wait for transfer complete */
    for (i = 0; i < 1000; ++i)
    {
        if (I2C_S_REG(dev->regMap) & I2C_S_TCF_MASK)
        {
            timeoutResult = 1;
            break;
        }
    }

    if (timeoutResult == 0)
        return ERRORS_IIC_TX_TIMEOUT;

    /* Check ACK! */
    return (I2C_S_REG(dev->regMap) & I2C_S_RXAK_MASK) ?
            ERRORS_IIC_TX_ACK_NOT_RECEIVED : ERRORS_IIC_TX_ACK_RECEIVED;
}

static System_Errors Iic_waitRxTransfer (Iic_DeviceHandle dev)
{
    uint16_t i, timeoutResult = 0;

    /* Wait for interrupt flag */
    for (i = 0; i < 1000; ++i)
    {
        if (I2C_S_REG(dev->regMap) & I2C_S_IICIF_MASK)
        {
            timeoutResult = 1;
            break;
        }
    }

    if (timeoutResult == 0)
        return ERRORS_IIC_RX_TIMEOUT;

    /* Reset value */
    I2C_S_REG(dev->regMap) |= I2C_S_IICIF_MASK;

    return ERRORS_IIC_RX_OK;
}

static void Iic_sendNack (Iic_DeviceHandle dev)
{
    I2C_C1_REG(dev->regMap) |= I2C_C1_TXAK_MASK;
}

static void Iic_sendAck (Iic_DeviceHandle dev)
{
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TXAK_MASK;
}

System_Errors Iic_writeByte (Iic_DeviceHandle dev, uint8_t data)
{
    Iic_firstRead = 1;

    /* Set TX mode */
    I2C_C1_REG(dev->regMap) |= I2C_C1_TX_MASK;

    /* Write the data in the register */
    I2C_D_REG(dev->regMap) = data;

    /* Wait and return */
    return Iic_waitTxTransfer(dev);

}

System_Errors Iic_writeBytes (Iic_DeviceHandle dev, uint8_t address,
        const uint8_t *data, uint8_t length, Iic_StopMode stopRequest)
{
    uint8_t i;
    System_Errors error;

    Iic_start(dev);

    /* Write address */
    I2C_D_REG(dev->regMap) = address;
    error = Iic_waitTxTransfer(dev);
    if (error == ERRORS_IIC_TX_TIMEOUT)
    {
        Iic_stop(dev);
        return error;
    }

    for (i = 0; i < length; ++i)
    {
        I2C_D_REG(dev->regMap) = data[i];
        error = Iic_waitTxTransfer(dev);
        if (error == ERRORS_IIC_TX_TIMEOUT)
        {
            Iic_stop(dev);
            return error;
        }
    }

    if (stopRequest == IIC_STOP)
        Iic_stop(dev);

    return ERRORS_IIC_TX_OK;
}

System_Errors Iic_readByte (Iic_DeviceHandle dev, uint8_t *data,
        Iic_LastByteMode lastByte)
{
    /* Set RX mode */
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TX_MASK;

    /* First dummy read if necessary... */
    if (Iic_firstRead)
    {
        Iic_sendAck(dev);
        (void) I2C_D_REG(dev->regMap);
        Iic_waitRxTransfer(dev);
        Iic_firstRead = 0;
    }

    if (lastByte == IIC_LAST_BYTE)
    {
        /* Set to TX mode */
        I2C_C1_REG(dev->regMap) |= I2C_C1_TX_MASK;
        *data = (I2C_D_REG(dev->regMap) & 0xFF);
        Iic_sendNack(dev);
        return ERRORS_IIC_RX_OK;
    }
    else
    {
        Iic_sendAck(dev);
        *data = (I2C_D_REG(dev->regMap) & 0xFF);
    }

    return Iic_waitRxTransfer(dev);
}

System_Errors Iic_readBytes (Iic_DeviceHandle dev, uint8_t address,
        uint8_t *data, uint8_t length, Iic_StopMode stopRequest)
{
    uint8_t i;

    Iic_start(dev);

    /* Write address */
    I2C_D_REG(dev->regMap) = address;
    if (Iic_waitTxTransfer(dev) == ERRORS_IIC_TX_TIMEOUT)
    {
        Iic_stop(dev);
        return ERRORS_IIC_TX_ERROR;
    }

    /* Set RX mode */
    I2C_C1_REG(dev->regMap) &= ~I2C_C1_TX_MASK;

    for (i = 0; i < length; ++i)
    {
        /* Ack type */
        if (i == (length-1))
            Iic_sendNack(dev);
        else
            Iic_sendAck(dev);

        /* Delete first dummy read */
        if (i == 0)
            (void) I2C_D_REG(dev->regMap);
        else
            data[i-1] = (I2C_D_REG(dev->regMap) & 0xFF);

        if (Iic_waitRxTransfer(dev) == ERRORS_IIC_RX_TIMEOUT)
        {
            Iic_stop(dev);
            return ERRORS_IIC_RX_TIMEOUT;
        }
    }

    if (stopRequest == IIC_STOP)
        Iic_stop(dev);

    /* Last read */
    data[i-1] = (I2C_D_REG(dev->regMap) & 0xFF);

    return ERRORS_IIC_RX_OK;
}

#endif /* LIBOHIBOARD_K64F12 || LIBOHIBOARD_FRDMK64F */

#endif /* LIBOHIBOARD_IIC */
