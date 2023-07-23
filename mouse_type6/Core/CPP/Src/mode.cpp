/*
 * mode.cpp
 *
 *  Created on: 2023/07/22
 *      Author: sato1
 */

#include "mode.h"
#include "interrupt.h"
#include "../../Module/Include/index.h"
#include "../../Module/Include/typedef.h"

#define ENABLE (0x01 << 4)

namespace Mode
{

	uint8_t Select(uint8_t _param,uint8_t max,t_encoder enc)
	{
		uint8_t param =  _param;
		if(enc.wheel_speed > 0.2){
			param = (param == max) ? 0 : param + 1 ;
			HAL_Delay(100);
		}
		else if(enc.wheel_speed  < -0.2){
			param = (param == 0) ? max : param - 1 ;
			HAL_Delay(100);
		}
		return param;
	}


	void Demo()
	{
		t_bool demo_end = False;
		uint8_t mode = Return_LED_Status() & 0x30;
		uint8_t param = 0x00;
		uint8_t enable = 0x00;
		while(demo_end == False)
		{
			enable = Mode::Select(enable,0x01,Encoder_GetProperty_Left());
			param = (enable == 0x00) ? Mode::Select(param,0x0f,Encoder_GetProperty_Right()) : param;
			Indicate_LED(mode|param);
			switch((enable<<4)|param)
			{
				case ENABLE|0x00:
					break;
				case ENABLE|0x01:
					break;
				case ENABLE|0x02:
					break;
				case ENABLE|0x03:
					break;
				case ENABLE|0x04:
					break;
				case ENABLE|0x05:
					break;
				case ENABLE|0x06:
					break;
				case ENABLE|0x07:
					break;
				case ENABLE|0x08:
					break;
				case ENABLE|0x09:
					break;
				case ENABLE|0x0A:
					break;
				case ENABLE|0x0B:
					break;
				case ENABLE|0x0C:
					break;
				case ENABLE|0x0D:
					break;
				case ENABLE|0x0E:
					break;
				case ENABLE|0x0F:
					demo_end = True;
					break;
				default:
					break;
			}
		}
	}

	void Debug()
	{
		t_bool debug_end = False;
		uint8_t mode = Return_LED_Status() & 0x30;
		uint8_t param = 0x00;
		uint8_t Indicate_param = param;
		uint8_t enable = 0x00;
		uint32_t observe_time = 0;
		while(debug_end == False)
		{
			enable = Mode::Select(enable,0x01,Encoder_GetProperty_Left());
			param = (enable == 0x00) ? Mode::Select(param,0x0f,Encoder_GetProperty_Right()) : param;
			Indicate_LED(mode|param);
			switch((enable<<4)|param)
			{
				case ENABLE|0x00:
					break;
				case ENABLE|0x01:
					break;
				case ENABLE|0x02:
					break;
				case ENABLE|0x03:
					break;
				case ENABLE|0x04:
					break;
				case ENABLE|0x05:
					break;
				case ENABLE|0x06:
					break;
				case ENABLE|0x07:
					break;
				case ENABLE|0x08:
					break;
				case ENABLE|0x09:
					break;
				case ENABLE|0x0A:
					break;
				case ENABLE|0x0B:
					break;
				case ENABLE|0x0C:
					break;
				case ENABLE|0x0D:
					break;
				case ENABLE|0x0E:
					break;
				case ENABLE|0x0F:
					debug_end = True;
					break;
				default:
					break;
			}
		}

	}

	void Select_Mode()
	{
		uint8_t mode = 0;
		uint8_t enable = 0;

		while(1)
		{
			enable 	= Mode::Select(enable,0x01,Encoder_GetProperty_Left() );
			mode = (enable == 0x00) ? Mode::Select(mode  ,0x03,Encoder_GetProperty_Right()):mode;

			Indicate_LED(mode << 4);
			switch((enable << 4)|mode)
			{
				case ENABLE|0x01:
					Mode::Demo();
					enable = 0;
					break;
				case ENABLE|0x02:
					Mode::Debug();
					enable = 0;
					break;
				case ENABLE|0x03:
					break;
				default:
					break;
			}
		}
	}
}

