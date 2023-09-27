/*
 * Error_Active.h
 *
 *  Created on: Aug 4, 2023
 *      Author: Minh Nhat
 */

#ifndef ERROR_ACTIVE_ERROR_ACTIVE_H_
#define ERROR_ACTIVE_ERROR_ACTIVE_H_

#include "Std_Types.h"
#include "../Get/Get.h"



void Error_Trigger(void);

void Error_Clear(void);

void Error_Set(void);

Std_Return Error_Counter(void);

void Error_ModHandle(void);


#endif /* ERROR_ACTIVE_ERROR_ACTIVE_H_ */
