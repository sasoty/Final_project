/*
 * Crypto.h
 *
 *  Created on: Aug 4, 2023
 *      Author: Minh Nhat
 */

#ifndef CRYPT_CRYPTO_H_
#define CRYPT_CRYPTO_H_

#include "Std_Types.h"



void Crypto_Init_Out(float input);

//void Crypto_In(uint8_t Chunk[],uint8_t input_len);

void Crypto_Init_In(uint8_t Input_Len);

void Crypto_ClearLen(void);

void Crypto_In(uint8_t Chunk);

float Crypto_Compress(void);

uint8_t Crypto_Extract(void);

uint8_t Crypto_Decrypt(void);

void Crypto_Encrypt(uint8_t input);

uint8_t Crypto_BufferStuff(void);


#endif /* CRYPT_CRYPTO_H_ */
