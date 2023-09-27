/*
 * MQTT_Buffer.c
 *
 *  Created on: Sep 17, 2023
 *      Author: Minh Nhat
 */


#include "MQTT_Buffer.h"
#include "Std_Types.h"

uint8_t Head_count,Tail_count;
uint8_t Wifi_Buff[24];


void MQTT_Init(void)
{
	Head_count = 0;
	Tail_count =0;
}

void MQTT_BuffStuff(uint8_t Wifi_temp)
{
	Head_count++;
	if(Head_count == 24)
	{
		Head_count = 0;
	}
	Wifi_Buff[Head_count] = Wifi_temp;

}

uint8_t MQTT_BuffDrain(void)
{
	uint8_t Wifi_temp;
	Tail_count++;
	if(Tail_count == 24)
	{
		Tail_count = 0;
	}
	Wifi_temp = Wifi_Buff[Tail_count];
	return Wifi_temp;
}

Std_Return MQTT_Avail(void)
{
	Std_Return Avail_Data = E_OK;
	if(Head_count == Tail_count)
	{
		Avail_Data =E_NOT_OK;
	}
	return Avail_Data;
}

