/*
 * Tx_Buff.h
 *
 *  Created on: Jun 7, 2023
 *      Author: Minh Nhat
 */

#ifndef INC_TX_BUFF_H_
#define INC_TX_BUFF_H_

#include "Std_Types.h"
#include "main.h"
#include "stdbool.h"

void Tx_RingBufferInit(void);

void Tx_BufStuff(uint8_t Data);

Std_Return Tx_RingData_Avail(void);

uint8_t Tx_ReadBuf(void);

void Tx_ResetBuf(void);

//#define MAX_BUFFER_SIZE 2048U
//typedef struct _RingContext_Type_
//{
//	volatile uint8_t Head;
//	volatile uint8_t Tail;
//	uint8_t  Buffer[MAX_BUFFER_SIZE];
//}RingContext_Type;


#endif /* INC_TX_BUFF_H_ */
