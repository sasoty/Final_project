/*
 * MQTT_Buffer.h
 *
 *  Created on: Sep 17, 2023
 *      Author: Minh Nhat
 */

#ifndef INC_MQTT_BUFFER_H_
#define INC_MQTT_BUFFER_H_

#include "Std_Types.h"

void MQTT_Init(void);

void MQTT_BuffStuff(uint8_t Wifi_temp);

uint8_t MQTT_BuffDrain(void);

Std_Return MQTT_Avail(void);

#endif /* INC_MQTT_BUFFER_H_ */
