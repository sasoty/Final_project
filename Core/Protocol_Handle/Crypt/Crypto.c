/*
 * Crypto.c
 *
 *  Created on: Aug 4, 2023
 *      Author: Minh Nhat
 */

#include "Crypto.h"

Convert Exchange_Out;
Convert *Exchange_ptr;
uint8_t Glo_Count;
uint8_t Length;
uint8_t Frame[8];

void Crypto_Init_Out(float input)
{
	Exchange_Out.Fl = input;
	Exchange_ptr = &Exchange_Out;
}
void Crypto_ClearLen(void)
{
	Length= 0;
}
void Crypto_In(uint8_t Chunk)
{
		Frame[Length] = Chunk;
		Length++;
}

float Crypto_Compress(void)
{
	Convert Exchange_In;
	Glo_Count = 0;
	for(int count = 0; count<4; count++)
	{
		Exchange_In.Segment[count]=Crypto_Decrypt();
		Glo_Count++;
	}
	Glo_Count = 0;
	return Exchange_In.Fl;
}

uint8_t Crypto_Extract(void)
{
	Length = 0;
	for(int count = 0; count < 4; count++)
	{
		Crypto_Encrypt(Exchange_ptr->Segment[count]);
	}
	Glo_Count = 0;
	return Length;
}

uint8_t Crypto_Decrypt(void)
{
		while(Frame[Glo_Count]==FLG)
		{
			Glo_Count++;
			Frame[Glo_Count]=Frame[Glo_Count]^FLG;
		}
		return Frame[Glo_Count];
}
//
void Crypto_Encrypt(uint8_t input)
{
			if((input==STX)||(input==FLG))
	        {
				Frame[Length]=FLG;
				Length++;
	            Frame[Length]=input^FLG;
	        }
	        else
	        {
	        	Frame[Length]=input;
	        }
				Length++;
}

//uint8_t Crypto_Checksum(void)
//{
//
//}

uint8_t Crypto_BufferStuff(void)
{
	uint8_t Crypto_Out = Frame[Glo_Count];
	Glo_Count++;
	return Crypto_Out;
}



