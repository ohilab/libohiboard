/**
 * @file libohiboard/uart.c
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @brief UART module functions
 */

#include "uart.h"

#define PER_CLOCK_KHZ 25000 // Temporary
#define UART_DEF_BAUDRATE 	9600
#define UART_DEF_PARITY 	None
#define UART_DEF_DATABITS 	bit8

/* 
 * Peripherals declaration with default values
 */
typedef struct uart_dev_ {
	UART_MemMapPtr 		regMap;
	
	uint32				baudRate;
	ParityMode_t 		parityMode;
	DataBits_t			dataBits;
	uint8				unsaved;
} uart_dev_;

static uart_dev_ uart2 = {
		.regMap 	= UART2_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS
};
uart_dev UART2 = &uart2; 

static uart_dev_ uart3 = {
		.regMap 	= UART2_BASE_PTR,
		.baudRate 	= UART_DEF_BAUDRATE,
		.parityMode = UART_DEF_PARITY,
		.dataBits 	= UART_DEF_DATABITS
};
uart_dev UART3 = &uart3; 


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
error_t uart_setBaudRate(uart_dev dev, uint32 br)
{
	if (br >= 9600 && br <= 115200)
	{
		dev->baudRate = br;
		dev->unsaved = 1;
		return ERR_OK;
	}
	else ERR_PARAM_VALUE;
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
error_t uart_init(uart_dev dev) 
{
    register uint16 sbr, brfa;
    uint8 temp;
    
    UART_MemMapPtr regmap = dev->regMap;
    uint32 baudRate = 		dev->baudRate;
    
	/* Enable the clock to the selected UART */    
    if (regmap == UART0_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
	else if (regmap == UART1_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
	else if (regmap == UART2_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
#ifdef K60
	else if (regmap == UART3_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
	else if (regmap == UART4_BASE_PTR)
		SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;
	else if (regmap == UART5_BASE_PTR)
		SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;
#endif
	else
		return ERR_PARAM_MASK;
		
                                
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
    temp = UART_C4_REG(regmap) & ~(UART_C4_BRFA(0x1F));
    
    UART_C4_REG(regmap) = temp |  UART_C4_BRFA(brfa);    
    
    dev->unsaved = 0;
    
    return ERR_OK;
}

/**
 * @brief Enable a Serial Port
 * @param dev uart Device to be enabled
 * @return Error signal
 */
error_t uart_enable(uart_dev dev)
{
    /* Enable receiver and transmitter */
	UART_C2_REG(dev->regMap) |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
	
	return ERR_OK;
}

/**
 * @brief Disable a Serial Port
 * @param dev uart Device to be disabled
 */
error_t uart_disable(uart_dev dev)
{
	/* Disable receiver and transmitter */
    UART_C2_REG(dev->regMap) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

    return ERR_OK;
}

/**
 * @brief Wait for a character to be received on the specified uart
 * @param dev Serial port Device to receive byte from
 * @param *out Buffer where to store the received character
 * @return Error signal
 */
error_t uart_getChar (uart_dev dev, char *out)
{
    /* Wait until character has been received */
    while (!(UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK));
    
    /* Save the 8-bit data from the receiver to the output param */
    *out = UART_D_REG(dev->regMap);
    
    return ERR_OK;
}

/**
 * @brief Wait for space in the UART Tx FIFO and then send a character
 * @param dev uart Device to send to
 * @param c Character to send
 */
void uart_putChar (uart_dev dev, char c)
{
	/* Wait until space is available in the FIFO */
    while(!(UART_S1_REG(dev->regMap) & UART_S1_TDRE_MASK));
    
    /* Send the character */
    UART_D_REG(dev->regMap) = (uint8)c;
}

/**
 * @brief Check to see if a character has been received
 * @param dev uart Device to check for a character
 * @return int
 * 	0 No character received
 * 	1 Character has been received
 */
int uart_getCharPresent (uart_dev dev)
{
	return (UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK);
}

