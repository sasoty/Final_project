/*
 * Push.h
 *
 *  Created on: Aug 4, 2023
 *      Author: Minh Nhat
 */

#ifndef PUSH_PUSH_H_
#define PUSH_PUSH_H_

#include "Std_Types.h"
#include "Tx_Buff.h"
//#include "Rx_Buff.h"
#include "../Crypt/Crypto.h"

void Push_Cmd(Cmd Cmd_Type);

void Push_Data(float Val);

void Push_Avail(void);


#endif /* PUSH_PUSH_H_ */
