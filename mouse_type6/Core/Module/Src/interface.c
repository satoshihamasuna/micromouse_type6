/*
 * interface.c
 *
 *  Created on: Jun 7, 2023
 *      Author: sato1
 */

#include "../Include/index.h"

const uint16_t  MAX_MODE_NUM =  0x0f;
uint16_t mouse_mode;
uint16_t is_mode_enable;

void Mode_Change_ENC()
{

	if(Encoder_GetProperty_Right().wheel_speed > 0.2){
		if(is_mode_enable == ENABLE_MODE3) is_mode_enable = DISENABLE_MODE;
		else mouse_mode = (mouse_mode == 0x0f) ? 0 : mouse_mode + 1 ;
		HAL_Delay(100);
	}
	else if(Encoder_GetProperty_Right().wheel_speed  < -0.2){
		if(is_mode_enable == ENABLE_MODE3) is_mode_enable = DISENABLE_MODE;
		else mouse_mode = (mouse_mode == DISENABLE_MODE) ? MAX_MODE_NUM : mouse_mode - 1 ;
		HAL_Delay(100);
	}

	if(Encoder_GetProperty_Left().wheel_speed  > 0.2){
		if(is_mode_enable == DISENABLE_MODE) is_mode_enable = ENABLE_MODE3;
		HAL_Delay(100);
	}

	Indicate_LED((is_mode_enable)| mouse_mode);

}

void Mode_Init()
{
	  mouse_mode = 0x00;
	  is_mode_enable = 0;
}

uint8_t Mode_State()
{
	return is_mode_enable|mouse_mode;
}

void Mode_Disable()
{
	is_mode_enable = DISABLE;
}

