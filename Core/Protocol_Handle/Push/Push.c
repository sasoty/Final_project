/*
 * Push.c
 *
 *  Created on: Aug 4, 2023
 *      Author: Minh Nhat
 */

#include "Push.h"

extern UART_HandleTypeDef huart2;

bool Tx_State = false;

void Push_Cmd(Cmd Cmd_Type)
{
	while(Tx_State)
	{
		;
	}
	Tx_RingBufferInit();
	uint8_t Cmd_buffer[3];
	Cmd_buffer[0] = STX;
	Cmd_buffer[1] = TYPE1;
	Cmd_buffer[2] = Cmd_Type;
	uint8_t TxData;
//	Std_Return Tx_Avail = Tx_RingData_Avail();
	for(int Count = 0;Count < 3;Count++)
	{
		Tx_BufStuff(Cmd_buffer[Count]);
	}
	Tx_State = true;
	TxData = Tx_ReadBuf();
	HAL_UART_Transmit_IT(&huart2,(uint8_t *)&TxData,1);
}


void Push_Data(float Val)
{
	while(Tx_State)
	{
		;
	}
	Tx_RingBufferInit();
	Crypto_Init_Out(Val);
	uint8_t TxData;
	uint8_t Cmd_buffer[2];
	Cmd_buffer[0] = STX;
	Cmd_buffer[1] = TYPE2;
	for(int Count=0; Count < 2;Count++)
	{
		Tx_BufStuff(Cmd_buffer[Count]);
	}
	uint8_t Length = Crypto_Extract();
	Tx_BufStuff(Length);
	for(int Count = 0; Count < Length; Count++)
	{
		Tx_BufStuff(Crypto_BufferStuff());
	}

	//Call back frame from Crypto

	TxData = Tx_ReadBuf();
	HAL_UART_Transmit_IT(&huart2,(uint8_t *)&TxData,1);
}
 void Push_Avail(void)
 {
	 Tx_State = false;
 }


