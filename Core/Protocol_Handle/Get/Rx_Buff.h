/*
 * Rx_Buff.h
 *
 *  Created on: Jun 7, 2023
 *      Author: Minh Nhat
 */

#ifndef INC_RX_BUFF_H_
#define INC_RX_BUFF_H_

#include "Std_Types.h"
#include "main.h"
#include "stdbool.h"


//#define MAX_BUFFER_SIZE 2048U
//typedef struct _RingContext_Type_
//{
//	volatile uint8_t Head;
//	volatile uint8_t Tail;
//	uint8_t  Buffer[MAX_BUFFER_SIZE];
//}RingContext_Type;
void Rx_RingBufferInit(void);

void Rx_BufStuff(uint8_t Data);

Std_Return Rx_RingData_Avail(void);

uint8_t Rx_ReadBuf(void);

void Rx_ResetBuf(void);


#endif /* INC_RX_BUFF_H_ */
