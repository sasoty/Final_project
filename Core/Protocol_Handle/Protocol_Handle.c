/*
 * Protocol_Handle.c
 *
 *  Created on: Aug 6, 2023
 *      Author: Minh Nhat
 */

#include "Protocol_Handle.h"
ValueResult *Scale_t;

void Protocol_Req(void)
{
	if(Scale_t->side == true)
	{
		for(int count = 0; count<=3;count++)
	{
			Push_Cmd(REQ_);
	}
		HAL_Delay(10);
		if(Get_Avail())
		{
			Cmd Cmd_Temp = Get_Cmd();
			if(Cmd_Temp == ACK_)
			{
				Scale_t->connected = true;
			}
			else
			{
				Error_Trigger();
			}
		}
		else
		{
			Error_Trigger();
		}
	}
	else
	{
		if(Get_Avail())
		{

			Cmd Cmd_Temp = Get_Cmd();
			if(Cmd_Temp == REQ_)
			{
				Scale_t->connected = true;
				for(int count = 0; count<=3;count++)
			{
					Push_Cmd(ACK_);
			}
			}
			else
			{
				Error_Trigger();
			}
		}
		else
		{
			Error_Trigger();
		}

	}
}

Std_Return Protocol_Data(void)
{
	Std_Return Ret_Avail =  E_NOT_OK;
	if(Scale_t->side == true)
	{
			Push_Cmd(VAL_);
			HAL_Delay(50);
		if(Get_Avail())
		{
// Note:Scale_t->value = 12;
			Std_Return Val_Avail = Get_Val_Init();
			if(Val_Avail == E_OK)
			{
				Scale_t->value = Get_Val();
				Ret_Avail = E_OK;
			}
			else
			{
				Error_Trigger();
			}
		}
		else
		{
			Error_Trigger();
		}
	}
	else
		{
			if(Get_Avail())
			{

				Cmd Cmd_Temp = Get_Cmd();
							if(Cmd_Temp == VAL_)
							{
								float Val = Scale_t->value;

								Push_Data(Val);

								Ret_Avail = E_OK;
							}
							else
							{
								Error_Trigger();
							}
			}
			else
			{
				Error_Trigger();
			}

		}
	return Ret_Avail;
}

void Protocol_Connect(ValueResult *Scale_temp)
{
	Scale_t = Scale_temp;
	Error_Set();
    while((!Scale_t->connected)&&(Error_Counter()))
    {
    	Protocol_Req();
    }

    Error_Clear();

}

void Protocol_Value(ValueResult *Scale_temp)
{
	Std_Return Ret_Avail =  E_NOT_OK;
	Scale_t = Scale_temp;
	Error_Set();
    while((!Ret_Avail)&&(Error_Counter()))
    {
    	Ret_Avail = Protocol_Data();
    }
	Error_Clear();
//	return Ret_Avail;
}


