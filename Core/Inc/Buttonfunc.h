/*
 * Buttonfunc.h
 *
 *  Created on: Feb 10, 2023
 *      Author: Minh Nhat
 */

#ifndef INC_BUTTONFUNC_H_
#define INC_BUTTONFUNC_H_

#include "stdio.h"
typedef enum
{
    NON_ = 0,
    NUM_,
    CALIB_,
    MAX_,
	SLP_,

}BTON;
void bton_reset(void);

BTON bton_buff(void);

int16_t bton_change(int16_t first, int16_t pre);
#endif /* INC_BUTTONFUNC_H_ */
