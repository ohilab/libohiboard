/* Copyright (C) 2014-2015 A. C. Open Hardware Ideas Lab
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
 * @file libohiboard/include/MOD_BUS.h
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief modbus  definition for KL25Z4 and FRDM-KL25Z.
 */
#ifndef MOD_BUS_H_
#define MOD_BUS_H_

//-------------------------------------------------------------------
//void (*function_vector[10])(int u);
//-------------------------------------------------------------------



//include library
	#include "errors.h"
	#include "types.h"
    #include "uart.h"
	#include "ftm.h"

//define costant
#define RX_BUFFER_LEN 30

static unsigned short modbus_16_tbl[];

//define reciver buffer type

typedef union //RX_buffer
{
    uint8_t Raw[RX_BUFFER_LEN];

    struct {
    	uint8_t Address;
    	uint8_t Function;
    	uint8_t Data[RX_BUFFER_LEN-2];
    }get_Field;

} RX_buffer_Type;

//define logical error type
	typedef enum
	{
		NO_ERROR			=0,
		ILLEGAL_FUNCTION	=1,
		ILLEGAL_DATA_ADDRESS=2,
		ILLEGAL_DATA_VALUE	=3,
		NEG_ACK				=7,
	} L_Error_Type;


//define trasmission mode type
	typedef enum
	{
		RTU_T_Mode,
		ASCII_T_TMode,
	} Mod_Bus_TM_Type;

//define Mod Bus state type
	typedef enum
	{

		IDLE,
		NEW_MESSAGE,
		IN_RECEPTION,
		PARITY_ERROR,
		SYNTAX_ERROR,
		CRCL_ERROR

	}State_type;

//define device type
	typedef enum
	{
		MASTER_Device,
		SLAVE_Device,
	} Device_Type;

//define serial configuration
	typedef enum
		{
		 S1_D8_Odd_Stop1,  //1 bit start,8 bit data, Odd parity, 1 stop bit

		} Serial_Config_Type;
//define
	typedef struct _Modbus_Config_Type
	{
		Device_Type D_Type;

		Mod_Bus_TM_Type T_Mode;

		uint32_t Baudrate;

		Serial_Config_Type Serial_Config;

		Uart_DeviceHandle COM;

		Ftm_DeviceHandle Counter;



		Uart_RxPins RX;

		Uart_TxPins	TX;
		uint8_t ID;


	} Modbus_Config_Type;



	typedef struct _Mod_Bus_handler
	{
		Modbus_Config_Type *config;
		State_type state;
		uint8_t interface_is_set;
		uint8_t timeout_flag;
		uint8_t error_parity_flag;
		Ftm_DeviceHandle ftm_Handler;
		Uart_DeviceHandle uart_handler;

		RX_buffer_Type buffer;
		uint8_t pos;
		uint8_t length;
		uint8_t ID;
		L_Error_Type Log_erroro;


	}Mod_Bus_handler;

	extern Mod_Bus_handler Mod_Bus_Interface;

	System_Errors Mod_Bus_Inizialize(Modbus_Config_Type *Bus_config);
	void Mod_Bus_Listener(void);
	uint16_t CRC16_Check(uint8_t *head,uint8_t len);
	void Analize_Frame(void);
	void Send_Logical_Error(L_Error_Type error);




#endif /* MOD_BUS_H_ */
