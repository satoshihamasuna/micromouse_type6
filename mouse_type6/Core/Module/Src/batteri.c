/*
 * batteri.c
 *
 *  Created on: 2023/06/18
 *      Author: sato1
 */


#include "index.h"

#define BATTRY_REFERENCE	(3.25f)
#define BATTERY_LIMIT		(3.6f)


float Battery_GetVoltage(){
	return (BATTRY_REFERENCE * (47.0f+10.0f)/(10.0f) * (float)Sensor_GetBatteryValue())/4096.f;
}
