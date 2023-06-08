/*
 * interface.c
 *
 *  Created on: Jun 7, 2023
 *      Author: sato1
 */
#include "index.h"
#include "glob_var.h"

#define MAX_MODE_NUM 0x0f

void Mode_Change_ENC()
{
	if(enc_R.wheel_speed > 0.2){
		if(is_mode_enable == ENABLE_MODE3) is_mode_enable = DISENABLE_MODE;
		else mouse_mode = (mouse_mode == 0x0f) ? 0 : mouse_mode + 1 ;
		HAL_Delay(100);
	}
	else if(enc_R.wheel_speed < -0.2){
		if(is_mode_enable == ENABLE_MODE3) is_mode_enable = DISENABLE_MODE;
		else mouse_mode = (mouse_mode == DISENABLE_MODE) ? MAX_MODE_NUM : mouse_mode - 1 ;
		HAL_Delay(100);
	}

	if(enc_L.wheel_speed > 0.2){
		if(is_mode_enable == DISENABLE_MODE) is_mode_enable = ENABLE_MODE3;
		HAL_Delay(100);
	}

	Indicate_LED((is_mode_enable)| mouse_mode);
}
