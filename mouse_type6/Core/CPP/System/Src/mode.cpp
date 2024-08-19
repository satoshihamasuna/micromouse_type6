/*
 * mode.cpp
 *
 *  Created on: 2023/07/22
 *      Author: sato1
 */

#include "../../Pheripheral/Include/index.h"
#include "../../Pheripheral/Include/macro.h"
#include "../../Pheripheral/Include/typedef.h"

//#include "../../Pheripheral/Include/interrupt.h"

#include "../../Subsys/Inc/search_class.h"
#include "../../Subsys/Inc/make_map_class.h"
#include "../../Subsys/Inc/wall_class.h"
#include "../../Subsys/Inc/make_path.h"
#include "../Inc/myshell.h"

#include "../../Component/Inc/controller.h"
#include "../../Component/Inc/Kalman_filter.h"
#include "../../Component/Inc/queue_class.h"
#include "../../Component/Inc/priority_queue.h"
#include "../../Component/Inc/path_follow.h"

#include "../../Task/Inc/sensing_task.h"
#include "../../Task/Inc/ctrl_task.h"

#include "../../Module/Inc/interrupt.h"
#include "../../Module/Inc/log_data.h"
#include "../../Module/Inc/flash.h"
#include "../../Module/Inc/communicate.h"

#include "../../Params/run_param.h"

#include "../Inc/mode.h"

#define ENABLE (0x01 << 4)

namespace Mode
{

	void indicate_error()
	{
		while(button_status() != True)
		{
			Indicate_LED(0xff);
			HAL_Delay(80);
			Indicate_LED(0x00);
			HAL_Delay(80);
		}
	}

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


	void Select_Mode()
	{
		uint8_t mode = 0;
		uint8_t enable = 0;

		while(1)
		{
			enable 	= Mode::Select(enable,0x01,Encoder_GetProperty_Left() );
			mode = (enable == 0x00) ? Mode::Select(mode  ,0x03,Encoder_GetProperty_Right()):mode;
			Battery_LimiterVoltage();
			Indicate_LED(mode << 4);
			switch((enable << 4)|mode)
			{
				case ENABLE|0x00:
					Mode::Interface_Check();
					enable = 0;
					break;
				case ENABLE|0x01:
					Mode::Demo();
					Mode::Demo2();
					enable = 0;
					break;
				case ENABLE|0x02:
					Mode::Debug(&st_param_650_acc7,mode_650,0);
					Mode::Debug(&st_param_800_acc7,mode_800,0);
					Mode::Debug(&st_param_1200,mode_1200,900);
				    Mode::Debug(&st_param_1400,mode_1400,900);

					enable = 0;
					break;
				case ENABLE|0x03:
					Myshell_Initialize();
					Myshell_Execute();
					enable = 0;
					break;
				default:
					break;
			}
		}
	}
}

