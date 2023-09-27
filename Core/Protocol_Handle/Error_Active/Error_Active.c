/*
 * Error_Active.c
 *
 *  Created on: Aug 4, 2023
 *      Author: Minh Nhat
 */


#include "Error_Active.h"

static uint8_t Err_counter;
bool Error_FLG = false;
int TO_Counter = 0;

void Error_Trigger(void)
{
	Err_counter ++;
//	Error_Handler();
}

void Error_Clear(void)
{
	Err_counter = 0;
	Error_FLG = false;
}

void Error_Set(void)
{
	Error_FLG = true;
}

Std_Return Error_Counter(void)
{
	Std_Return Error_Exist;
	if(Err_counter >= 3)
	{
		Error_Exist = E_NOT_OK;
	}
	else
	{
		Error_Exist = E_OK;
	}
	return Error_Exist;
}

void Error_ModHandle(void)
{
	if(Error_FLG)
	{
		if(TO_Counter >=300)
		{
			TO_Counter = 0;
			Get_Error();
		}
		else
		{
			TO_Counter++;
		}

	}
	else
	{
		TO_Counter = 0;
		Err_counter = 0;
	}
}
