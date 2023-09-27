/*
 * Get.h
 *
 *  Created on: Aug 5, 2023
 *      Author: Minh Nhat
 */

#ifndef GET_GET_H_
#define GET_GET_H_

#include "Std_Types.h"
#include "../Crypt/Crypto.h"
#include "../Error_Active/Error_Active.h"
#include "Rx_Buff.h"


Std_Return Get_Avail(void);


Cmd Get_Cmd(void);

Std_Return Get_Val_Init(void);

float Get_Val(void);

void Get_Error(void);


#endif /* GET_GET_H_ */
