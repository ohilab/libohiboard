/**
 * @file libohiboard/uart.c
 * @author Edoardo Bezzeccheri <coolman3@gmail.com>
 * @brief UART module functions
 */

#include "uart.h"

static uart_dev uart2 = {
		.regMap = UART2_BASE_PTR
};
uart_dev *UART2 = &uart2;


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
 * @param sysclk UART Device Clock in kHz (used to calculate baud)
 * @param baud uart Device baud rate
 * @return Error signal integer
 */
int uart_init(uart_dev *dev, int sysclk, int baud) 
{
    register uint16_t sbr, brfa;
    uint8 temp;
    UART_MemMapPtr regmap;
    regmap = dev->regMap;

	/* Enable the clock to the selected UART */    
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
    sbr = (uint16)((sysclk*1000)/(baud * 16));
        
    /* Save off the current value of the UARTx_BDH except for the SBR field */
    temp = UART_BDH_REG(regmap) & ~(UART_BDH_SBR(0x1F));
    
    UART_BDH_REG(regmap) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
    UART_BDL_REG(regmap) = (uint8)(sbr & UART_BDL_SBR_MASK);
    
    /* Determine if a fractional divider is needed to get closer to the baud rate */
    brfa = (((sysclk*32000)/(baud * 16)) - (sbr * 32));
    
    /* Save off the current value of the UARTx_C4 register except for the BRFA field */
    temp = UART_C4_REG(regmap) & ~(UART_C4_BRFA(0x1F));
    
    UART_C4_REG(regmap) = temp |  UART_C4_BRFA(brfa);    
    
    /* Copy settings into dev struct */
    dev->baudRate = (uint32) baud;
    dev->parityMode = None;
    dev->dataBits = bit8;
    
    return ERR_OK;
}

/**
 * @brief Enable a Serial Port
 * @param dev uart Device to be enabled
 */
void uart_enable(uart_dev *dev)
{
    /* Enable receiver and transmitter */
	UART_C2_REG(dev->regMap) |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
}

/**
 * @brief Disable a Serial Port
 * @param dev uart Device to be disabled
 */
void uart_disable(uart_dev *dev)
{
	/* Disable receiver and transmitter */
    UART_C2_REG(dev->regMap) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
}

/**
 * @brief Wait for a character to be received on the specified uart
 * @param dev Serial port Device to receive byte from
 * @return The received character
 */
char uart_getChar (uart_dev *dev)
{
    /* Wait until character has been received */
    while (!(UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK));
    
    /* Return the 8-bit data from the receiver */
    return UART_D_REG(dev->regMap);
}

/**
 * @brief Wait for space in the UART Tx FIFO and then send a character
 * @param dev uart Device to send to
 * @param c Character to send
 */
void uart_putChar (uart_dev *dev, char c)
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
int uart_getCharPresent (uart_dev *dev)
{
	return (UART_S1_REG(dev->regMap) & UART_S1_RDRF_MASK);
}

