/*
 * Protocol_Handle.h
 *
 *  Created on: Aug 6, 2023
 *      Author: Minh Nhat
 */

#ifndef PROTOCOL_HANDLE_PROTOCOL_HANDLE_H_
#define PROTOCOL_HANDLE_PROTOCOL_HANDLE_H_

#include "Std_Types.h"
#include "Push/Push.h"
#include "Get/Get.h"
#include "Error_Active/Error_Active.h"



void Protocol_Req(void);

Std_Return Protocol_Data(void);

void Protocol_Connect(ValueResult *Scale_temp);

void Protocol_Value(ValueResult *Scale_temp);


#endif /* PROTOCOL_HANDLE_PROTOCOL_HANDLE_H_ */
