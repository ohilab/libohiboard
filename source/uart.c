/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Edoardo Bezzeccheri <coolman3@gmail.com>
 *	Marco Giammarini <m.giammarini@warcomeb.it>
 *	Niccolo' Paolinelli <ai03@hotmail.it>
 *	
 * Project: libohiboard
 * Package: UART
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
 * @file libohiboard/uart.c
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @author Niccolo' Paolinelli <ai03@hotmail.it>
 * @brief UART module functions
 */

#include "platforms.h"
#include "uart.h"

#if defined (FRDMKL25Z)
#define PER_CLOCK_KHZ 20000
#define PER_CLOCK_MCG 40000000
#elif defined (MKL15Z4)
#define PER_CLOCK_KHZ 24000
#define PER_CLOCK_MCG 48000000
#elif defined (MK60DZ10)
#define PER_CLOCK_KHZ 50000
#elif defined (MK10DZ10)
#define PER_CLOCK_KHZ 50000 //Velocità del Bus clock 50MHz, cioè 100MHz/2
#endif
#define UART_DEF_BAUDRATE 	9600
#define UART_MIN_BAUDARATE	1200
#define UART_MAX_BAUDARATE	115200
#define UART_DEF_PARITY 	UART_PARITY_NONE
#define UART_DEF_DATABITS 	UART_EIGHT_BIT

#define UART_PIN_ENABLED    1
#define UART_PIN_DISABLED   0

static const char Uart_hexDigits[] = "0123456789ABCDEF";

/* 
 * Peripherals declaration with default values
 */

typedef struct Uart_Device
{
	UART_MemMapPtr          regMap;
#if defined(MKL15Z4) || defined(FRDMKL25Z)
	UART0_MemMapPtr         regMap0;
#endif

	uint32_t                baudRate;
	Uart_ParityMode         parityMode;
	Uart_DataBits           dataBits;
	
	uint8_t                 pinEnabled;
} Uart_Device;


#if defined(MK60DZ10)

static Uart_Device uart0 = {
		.regMap 	= UART0_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART0 = &uart0; 

static Uart_Device uart1 = {
		.regMap 	= UART1_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART1 = &uart1; 

static Uart_Device uart2 = {
		.regMap 	= UART2_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART2 = &uart2; 

static Uart_Device uart3 = {
		.regMap 	= UART3_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART3 = &uart3; 

static Uart_Device uart4 = {
        .regMap     = UART4_BASE_PTR,
        .baudRate   = UART_DEF_BAUDRATE,
        .parityMode = UART_DEF_PARITY,
        .dataBits   = UART_DEF_DATABITS,
        .pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART4 = &uart4; 

static Uart_Device uart5 = {
        .regMap     = UART5_BASE_PTR,
        .baudRate   = UART_DEF_BAUDRATE,
        .parityMode = UART_DEF_PARITY,
        .dataBits   = UART_DEF_DATABITS,
        .pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART5 = &uart5; 

#elif defined(MKL15Z4)

static Uart_Device uart0 = {
		.regMap 	= 0,
		.regMap0 	= UART0_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART0 = &uart0; 

static Uart_Device uart1 = {
		.regMap     = UART1_BASE_PTR,
		.regMap0    = 0,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART1 = &uart1; 

static Uart_Device uart2 = {
		.regMap     = UART2_BASE_PTR,
		.regMap0    = 0,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART2 = &uart2;

#elif defined(FRDMKL25Z)

static Uart_Device uart0 = {
		.regMap 	= 0,
		.regMap0 	= UART0_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART0 = &uart0; 

static Uart_Device uart1 = {
		.regMap 	= UART1_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART1 = &uart1; 

static Uart_Device uart2 = {
		.regMap 	= UART2_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART2 = &uart2;

#elif defined (MK10DZ10)

static Uart_Device uart0 = {
		.regMap 	= UART0_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART0 = &uart0; 

static Uart_Device uart1 = {
		.regMap 	= UART1_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART1 = &uart1; 

static Uart_Device uart2 = {
		.regMap 	= UART2_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART2 = &uart2;

static Uart_Device uart3 = {
		.regMap 	= UART3_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART3 = &uart3; 

static Uart_Device uart4 = {
		.regMap 	= UART4_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS,
		.pinEnabled = UART_PIN_DISABLED
};
Uart_DeviceHandle UART4 = &uart4;

#endif

/*
 *  Function definitions
 */

/**
 * @brief Set Baud Rate
 * 
 * Sets baud rate value into device structure. 
 * The settings are applied only after the device is initialized.
 * 
 * 
 * @param dev uart Device to be initialized
 * @param br Baud Rate value
 * @return Error signal
 */
System_Errors Uart_setBaudRate(Uart_DeviceHandle dev, uint32_t br)
{
	if (br >= UART_MIN_BAUDARATE && br <= UART_MAX_BAUDARATE)
	{
		dev->baudRate = br;
		return ERRORS_NO_ERROR;
	}
	else
    {
	    return ERRORS_PARAM_VALUE;
    }
}

/**
 * @brief Initialize the UART
 * 
 * Initialize the UART for 8N1 operation, interrupts disabled, and
 * no hardware flow-control
 * 
 * @note Since the UARTs are pinned out in multiple locations on most
 * Kinetis devices, this driver does not enable UART pin functions.
 * The desired pins should be enabled before calling this init function.
 * 
 * @param dev uart Device to be initialized
 * @return Error signal
 */
System_Errors Uart_init(Uart_DeviceHandle dev) 
{
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    register uint16_t sbr;
#elif defined(MK60DZ10) || defined(MK10DZ10)
    register uint16_t sbr, brfa;	//BaudRateFineAdjust
#endif
    uint8_t temp;
    
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    UART_MemMapPtr  regmap = 0;
    UART0_MemMapPtr regmap0 = 0;
    
    if (dev == UART0)
    	regmap0 = dev->regMap0;
    else
    	regmap = dev->regMap;
#elif defined(MK60DZ10) || defined(MK10DZ10)
    UART_MemMapPtr regmap = dev->regMap;
#endif
    uint32_t baudRate     = dev->baudRate;
    
    if (dev->pinEnabled == UART_PIN_DISABLED)
    	return ERRORS_HW_NOT_ENABLED;
    
	/* Enable the clock to the selected UART */    
#if defined(MK60DZ10)
	if (regmap == UART0_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
	else if (regmap == UART1_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
	else if (regmap == UART2_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
	else if (regmap == UART3_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
	else if (regmap == UART4_BASE_PTR)
		SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;
	else if (regmap == UART5_BASE_PTR)
		SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;
#elif defined(MKL15Z4) || defined(FRDMKL25Z)
	if (regmap0 == UART0_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
	else if (regmap == UART1_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
	else if (regmap == UART2_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;

#elif defined(MK10DZ10)
	if (regmap == UART0_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
	else if (regmap == UART1_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
	else if (regmap == UART2_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
	else if (regmap == UART3_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
	else if (regmap == UART4_BASE_PTR)
		SIM_SCGC1 |= SIM_SCGC1_UART4_MASK; 

#endif

	else
		return ERRORS_PARAM_VALUE;

#if defined(MKL15Z4) || defined(FRDMKL25Z)
	if (dev == UART0)
	{
		/* Make sure that the transmitter and receiver are disabled while we 
	     * change settings.
	     */
		UART0_C2_REG(regmap0) &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK );
		
	    /* Configure the UART for 8-bit mode, no parity */
	    /* We need all default settings, so entire register is cleared */
	    UART0_C1_REG(regmap0) = 0;	
	    
	    /* Calculate baud settings */
	    /* TODO: 2000 is JUST FOR TEST! */
	    sbr = (uint16_t)(PER_CLOCK_MCG/(baudRate * ((UART0_C4_REG(regmap0) & UART0_C4_OSR_MASK) + 1)));
	        
	    /* Save off the current value of the UARTx_BDH except for the SBR field */
	    temp = UART0_BDH_REG(regmap0) & ~(UART0_BDH_SBR(0x1F));
	    
	    UART0_BDH_REG(regmap0) = temp |  UART0_BDH_SBR(((sbr & 0x1F00) >> 8));
	    UART0_BDL_REG(regmap0) = (uint8_t)(sbr & UART0_BDL_SBR_MASK);
	}
	else
	{
#endif
	    /* Make sure that the transmitter and receiver are disabled while we 
	     * change settings.
	     */
		UART_C2_REG(regmap) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

	    /* Configure the UART for 8-bit mode, no parity */
	    /* We need all default settings, so entire register is cleared */
	    UART_C1_REG(regmap) = 0;	
	    
	    /* Calculate baud settings */
	    sbr = (uint16_t)((PER_CLOCK_KHZ*1000)/(baudRate * 16));
	        
	    /* Save off the current value of the UARTx_BDH except for the SBR field */
	    temp = UART_BDH_REG(regmap) & ~(UART_BDH_SBR(0x1F));
	    
	    UART_BDH_REG(regmap) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
	    UART_BDL_REG(regmap) = (uint8_t)(sbr & UART_BDL_SBR_MASK);
#if defined(MKL15Z4) || defined(FRDMKL25Z)
	}
#endif

#if defined(MK60DZ10) || defined(MK10DZ10)
    /* Determine if a fractional divider is needed to get closer to the baud rate */
    brfa = (((PER_CLOCK_KHZ*32000)/(baudRate * 16)) - (sbr * 32));
    
    /* Save off the current value of the UARTx_C4 register except for the BRFA field */
    temp = UART_C4_REG(regmap) & ~(UART_C4_BRFA(0x1F));
    
    UART_C4_REG(regmap) = temp |  UART_C4_BRFA(brfa);  
    
#endif

    return ERRORS_NO_ERROR;
}

/**
 * @brief Enable a Serial Port
 * @param dev Uart device to be enabled
 * @return Error signal
 */
System_Errors Uart_enable(Uart_DeviceHandle dev)
{
    if (dev->pinEnabled == UART_PIN_DISABLED)
    	return ERRORS_HW_NOT_ENABLED;
	
    /* Enable receiver and transmitter */
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    if (dev == UART0)
    {
    	UART0_C2_REG(dev->regMap0) |= (UART0_C2_TE_MASK | UART0_C2_RE_MASK );
    }
    else
    {
#endif
    	UART_C2_REG(dev->regMap) |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
#if defined(MKL15Z4) || defined(FRDMKL25Z)    	
    }
#endif
	return ERRORS_NO_ERROR;
}

/**
 * @brief Disable a Serial Port
 * @param dev Uart device to be disabled
 */
System_Errors Uart_disable(Uart_DeviceHandle dev)
{
	/* Disable receiver and transmitter */
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    if (dev == UART0)
    {
    	UART0_C2_REG(dev->regMap0) &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK );
    }
    else
    {
#endif
    	UART_C2_REG(dev->regMap) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
#if defined(MKL15Z4) || defined(FRDMKL25Z)    	
    }
#endif

    return ERRORS_NO_ERROR;
}

/**
 * @brief Indicate that device pin was selected.
 * @param dev Uart device.
 */
void Uart_pinEnabled (Uart_DeviceHandle dev)
{
	dev->pinEnabled = UART_PIN_ENABLED;
}
 
/**
 * @brief Wait for a character to be received on the specified uart
 * @param dev Serial port device to receive byte from
 * @param *out Buffer where to store the received character
 * @return Error signal
 */
System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out)
{
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    if (dev == UART0)
    {
		/* Wait until character has been received */
		while (!(UART0_S1_REG(dev->regMap0) & UART0_S1_RDRF_MASK));
		
		/* Save the 8-bit data from the receiver to the output param */
		*out = UART0_D_REG(dev->regMap0);
    }
    else
    {
#endif
		/* Wait until character has been received */
		while (!(UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK));
		
		/* Save the 8-bit data from the receiver to the output param */
		*out = UART_D_REG(dev->regMap);    	
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    }
#endif
    return ERRORS_NO_ERROR;
}

/**
 * @brief Wait for space in the UART Tx FIFO and then send a character
 * @param dev uart Device to send to
 * @param c Character to send
 */
void Uart_putChar (Uart_DeviceHandle dev, char c)
{
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    if (dev == UART0)
    {
    	/* Wait until space is available in the FIFO */
        while(!(UART0_S1_REG(dev->regMap0) & UART0_S1_TDRE_MASK));
        
        /* Send the character */
        UART0_D_REG(dev->regMap0) = (uint8_t)c;
    }
    else
    {
#endif
    	/* Wait until space is available in the FIFO */
        while(!(UART_S1_REG(dev->regMap) & UART_S1_TDRE_MASK));
        
        /* Send the character */
        UART_D_REG(dev->regMap) = (uint8_t)c;
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    }
#endif
}

/**
 * @brief Check to see if a character has been received
 * @param dev uart Device to check for a character
 * @return int
 * 	0 No character received
 * 	1 Character has been received
 */
int Uart_isCharPresent (Uart_DeviceHandle dev)
{
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    if (dev == UART0)
    {
        if (UART0_S1_REG(dev->regMap0) & UART0_S1_OR_MASK)
            UART0_S1_REG(dev->regMap0) |= UART0_S1_OR_MASK;
    	return (UART0_S1_REG(dev->regMap0) & UART0_S1_RDRF_MASK);
    }
    else
    {
#endif
    	return (UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK);
#if defined(MKL15Z4) || defined(FRDMKL25Z)
    }
#endif
}

void Uart_sendString (Uart_DeviceHandle dev, const char* text)
{
    if (text)
    {
        while (*text) Uart_putChar(dev, *text++);
    }
}

void Uart_sendData (Uart_DeviceHandle dev, const char* data, uint8_t length)
{
    if (data)
    {
        while (length--)
        {
            Uart_putChar(dev, *data++);
        }
    }
}

void Uart_sendHex (Uart_DeviceHandle dev, const char* data, uint8_t length)
{
    if (data)
    {
        while (length--)
        {
            uint8_t value = *data++;
            Uart_putChar(dev, Uart_hexDigits[(value >> 4) & 0x0F]);
            Uart_putChar(dev, Uart_hexDigits[(value >> 0) & 0x0F]);
        }
    }
}
