/*
 * Rx_Buff.c
 *
 *  Created on: Jun 7, 2023
 *      Author: Minh Nhat
 */


#include "Rx_Buff.h"



/**
 * For Interrupt
 */
RingContext_Type Rx_Buffer;
RingContext_Type *Rx_RingBuffer;






void Rx_RingBufferInit(void)
{
	Rx_RingBuffer = &Rx_Buffer;
	Rx_RingBuffer->Head = 0;
	Rx_RingBuffer->Tail = 0;

}
void Rx_BufStuff(uint8_t Data)
{
	Rx_RingBuffer->Buffer[Rx_RingBuffer->Head] = Data;
	Rx_RingBuffer->Head++ ;
	if(Rx_RingBuffer->Head >= MAX_BUFFER_SIZE)
	{
		Rx_RingBuffer->Head = 0;
	}
}

Std_Return Rx_RingData_Avail(void)
{
	Std_Return Retval = E_NOT_OK;
	if(Rx_RingBuffer->Head == Rx_RingBuffer->Tail)
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

uint8_t Rx_ReadBuf(void)
{
	uint8_t data;
	data = Rx_RingBuffer->Buffer[Rx_RingBuffer->Tail];
	Rx_RingBuffer->Tail++;
	if(Rx_RingBuffer->Tail >= MAX_BUFFER_SIZE)
	{
		Rx_RingBuffer->Tail = 0;
	}

	return data;
}

void Rx_ResetBuf(void)
{
	Rx_RingBuffer->Head = 0;
	Rx_RingBuffer->Tail = 0;
}



