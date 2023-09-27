/*
 * Buttonfunc.c
 *
 *  Created on: Feb 10, 2023
 *      Author: Minh Nhat
 */

#include "Buttonfunc.h"

BTON bton_state = NON_;
int16_t changed = 0;
int16_t ret_bton = 0;;
void bton_reset(void)
{
	bton_state = NON_;
}

BTON bton_buff(void)
{
	if(bton_state == 3)
	{bton_state=0;}
	else
		{bton_state ++;}
	return bton_state;

}



int16_t bton_change(int16_t firstt, int16_t pre)
{

    switch (bton_state)
    {
    case NON_:
        {

        break;
        }
    
    case NUM_:
    {
    	ret_bton = abs(pre - firstt);
    	if(ret_bton >= 200)
    	{
    		ret_bton = 200;
    	}
    	else if(ret_bton == 0)
    	{
    		ret_bton = 1;
    	}
        break;
    }
    case CALIB_:
    {
    	changed = pre - firstt;
    	ret_bton = (changed/8)*10;
    	if(ret_bton>=100)
    	{
    		ret_bton = 100;
    	}
        break;
    }
    case MAX_:
    {
    	changed = abs(pre - firstt);
    	ret_bton = (changed/8)*30;
    	if(ret_bton >= 150)
    	{
    		ret_bton = 100;
    	}
        break;
    }

}
    return ret_bton;
}
