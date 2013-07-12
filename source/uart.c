/******************************************************************************
 * Copyright (C) 2012-2013 A. C. Open Hardware Ideas Lab
 * 
 * Author(s):
 *	Edoardo Bezzeccheri <coolman3@gmail.com>
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
 * @brief UART module functions
 */

#include "platforms.h"
#include "uart.h"

#define PER_CLOCK_KHZ 25000 // Temporary
#define UART_DEF_BAUDRATE 	9600
#define UART_MIN_BAUDARATE	1200
#define UART_MAX_BAUDARATE	115200
#define UART_DEF_PARITY 	UART_PARITY_NONE
#define UART_DEF_DATABITS 	UART_EIGHT_BIT

/* 
 * Peripherals declaration with default values
 */

typedef struct Uart_Device {
	UART_MemMapPtr          regMap;

	uint32_t                baudRate;
	Uart_ParityMode         parityMode;
	Uart_DataBits           dataBits;
} Uart_Device;


#if defined(MK60DZ10)
static Uart_Device uart0 = {
		.regMap 	= UART0_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS
};
Uart_DeviceHandle UART0 = &uart0; 

static Uart_Device uart1 = {
		.regMap 	= UART1_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS
};
Uart_DeviceHandle UART1 = &uart1; 

static Uart_Device uart2 = {
		.regMap 	= UART2_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS
};
Uart_DeviceHandle UART2 = &uart2; 

static Uart_Device uart3 = {
		.regMap 	= UART3_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS
};
Uart_DeviceHandle UART3 = &uart3; 

#elif defined(MKL15Z4)
static Uart_Device uart1 = {
		.regMap 	= UART1_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS
};
Uart_DeviceHandle UART1 = &uart1; 

static Uart_Device uart2 = {
		.regMap 	= UART2_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS
};
Uart_DeviceHandle UART2 = &uart2; 
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
    register uint16_t sbr, brfa;
    uint8_t temp;
    
    UART_MemMapPtr regmap = dev->regMap;
    uint32_t baudRate     = dev->baudRate;
    
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
#elif defined(MKL15Z4)
	/* WARNING: UART device doesn't contain UART0 peripheral */
	if (regmap == UART1_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
	else if (regmap == UART2_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
#endif

	else
		return ERRORS_PARAM_VALUE;
                                
    /* Make sure that the transmitter and receiver are disabled while we 
     * change settings.
     */
    UART_C2_REG(regmap) &= ~(UART_C2_TE_MASK
				| UART_C2_RE_MASK );

    /* Configure the UART for 8-bit mode, no parity */
    /* We need all default settings, so entire register is cleared */
    UART_C1_REG(regmap) = 0;	
    
    /* Calculate baud settings */
    sbr = (uint16)((PER_CLOCK_KHZ*1000)/(baudRate * 16));
        
    /* Save off the current value of the UARTx_BDH except for the SBR field */
    temp = UART_BDH_REG(regmap) & ~(UART_BDH_SBR(0x1F));
    
    UART_BDH_REG(regmap) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
    UART_BDL_REG(regmap) = (uint8)(sbr & UART_BDL_SBR_MASK);
    
    /* Determine if a fractional divider is needed to get closer to the baud rate */
    brfa = (((PER_CLOCK_KHZ*32000)/(baudRate * 16)) - (sbr * 32));
    
    /* Save off the current value of the UARTx_C4 register except for the BRFA field */
    /* FIXME: In KL15 this register not match with register of K60 */
    temp = UART_C4_REG(regmap) & ~(UART_C4_BRFA(0x1F));
    
    UART_C4_REG(regmap) = temp |  UART_C4_BRFA(brfa);    
    
    return ERRORS_NO_ERROR;
}

/**
 * @brief Enable a Serial Port
 * @param dev uart Device to be enabled
 * @return Error signal
 */
System_Errors Uart_enable(Uart_DeviceHandle dev)
{
    /* Enable receiver and transmitter */
	UART_C2_REG(dev->regMap) |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
	
	return ERRORS_NO_ERROR;
}

/**
 * @brief Disable a Serial Port
 * @param dev uart Device to be disabled
 */
System_Errors Uart_disable(Uart_DeviceHandle dev)
{
	/* Disable receiver and transmitter */
    UART_C2_REG(dev->regMap) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

    return ERRORS_NO_ERROR;
}

/**
 * @brief Wait for a character to be received on the specified uart
 * @param dev Serial port Device to receive byte from
 * @param *out Buffer where to store the received character
 * @return Error signal
 */
System_Errors Uart_getChar (Uart_DeviceHandle dev, char *out)
{
    /* Wait until character has been received */
    while (!(UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK));
    
    /* Save the 8-bit data from the receiver to the output param */
    *out = UART_D_REG(dev->regMap);
    
    return ERRORS_NO_ERROR;
}

/**
 * @brief Wait for space in the UART Tx FIFO and then send a character
 * @param dev uart Device to send to
 * @param c Character to send
 */
void Uart_putChar (Uart_DeviceHandle dev, char c)
{
	/* Wait until space is available in the FIFO */
    while(!(UART_S1_REG(dev->regMap) & UART_S1_TDRE_MASK));
    
    /* Send the character */
    UART_D_REG(dev->regMap) = (uint8_t)c;
}

/**
 * @brief Check to see if a character has been received
 * @param dev uart Device to check for a character
 * @return int
 * 	0 No character received
 * 	1 Character has been received
 */
int Uart_getCharPresent (Uart_DeviceHandle dev)
{
	return (UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK);
}

