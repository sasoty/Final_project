/*
 * Std_Types.h
 *
 *  Created on: Sep 4, 2022
 *      Author: ngocn
 *
 */

#ifndef APPLICATION_USER_BOOTFUNCTION_INC_STD_TYPES_H_
#define APPLICATION_USER_BOOTFUNCTION_INC_STD_TYPES_H_

#include "stm32f1xx_hal.h"
#include "stdbool.h"

#define MAX_BUFFER_SIZE 32
#define FLG 0x07
typedef enum
{
	E_OK = 1,
	E_NOT_OK = 0
}Std_Return;

typedef struct{
	GPIO_PinState spin1;
	GPIO_PinState spin2;

}gpin_state;

typedef enum
{
	REQ_ = 0x05,
	ACK_= 0x06,
	VAL_ = 0x07,
	ERR_ = 0x27
}Cmd;


enum
{
    STX = 0x01,
    ETX = 0X02,
    TYPE1 = 0X03,
    TYPE2 = 0X04,
};

typedef union exchange
{
    float Fl;
    uint8_t Segment[4];
}Convert;


typedef struct _RingContext_Type_
{
	volatile uint8_t Head;
	volatile uint8_t Tail;
	uint8_t  Buffer[MAX_BUFFER_SIZE];
}RingContext_Type;

typedef struct {
    float value;
    bool connected;
    bool side;
} ValueResult;


//const gpin_state pinsta[] =
//{
//		{GPIO_PIN_RESET,GPIO_PIN_RESET},
//		{GPIO_PIN_RESET,GPIO_PIN_SET},
//		{GPIO_PIN_SET,GPIO_PIN_RESET},
//		{GPIO_PIN_SET,GPIO_PIN_SET}
//};

#endif /* APPLICATION_USER_BOOTFUNCTION_INC_STD_TYPES_H_ */
