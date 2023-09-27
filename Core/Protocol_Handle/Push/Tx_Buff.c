/*
 * Tx_Buff.c
 *
 *  Created on: Jun 7, 2023
 *      Author: Minh Nhat
 */





#include "Tx_Buff.h"



/**
 * For Interrupt
 */
RingContext_Type Tx_Buffer;
RingContext_Type *Tx_RingBuffer;






void Tx_RingBufferInit(void)
{
	Tx_RingBuffer = &Tx_Buffer;
	Tx_RingBuffer->Head = 0;
	Tx_RingBuffer->Tail = 0;

}
void Tx_BufStuff(uint8_t Data)
{
	Tx_RingBuffer->Buffer[Tx_RingBuffer->Head] = Data;
	Tx_RingBuffer->Head++ ;
	if(Tx_RingBuffer->Head == MAX_BUFFER_SIZE)
	{
		Tx_RingBuffer->Head = 0;
	}
}

Std_Return Tx_RingData_Avail(void)
{
	Std_Return Retval = E_NOT_OK;
	if(Tx_RingBuffer->Head == Tx_RingBuffer->Tail)
	{
		return Retval;
	}
	Retval = E_OK;
	return Retval;
}

//void UartRing_WriteBuf(uint8_t data)
//{
//	_RingContext->buffer[_RingContext->head] = data;
//	RingContext.head++;
//}

uint8_t Tx_ReadBuf(void)
{
	uint8_t data;
	data = Tx_RingBuffer->Buffer[Tx_RingBuffer->Tail];
	Tx_RingBuffer->Tail++;
	if(Tx_RingBuffer->Tail == MAX_BUFFER_SIZE)
	{
		Tx_RingBuffer->Tail = 0;
	}

	return data;
}

void Tx_ResetBuf(void)
{
	Tx_RingBuffer->Head = 0;
	Tx_RingBuffer->Tail = 0;
}


