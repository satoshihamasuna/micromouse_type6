/*
 * interface_check.cpp
 *
 *  Created on: 2023/12/03
 *      Author: sato1
 */



#include "../../Module/Include/index.h"
#include "../../Module/Include/macro.h"
#include "../../Module/Include/typedef.h"

#include "stdio.h"
#include "../Inc/sensing_task.h"
#include "../Inc/motion.h"
#include "../Inc/interrupt.h"
#include "../Inc/controll.h"
#include "../Inc/log_data.h"
#include "../Inc/Kalman_filter.h"
#include "../Inc/run_param.h"
#include "../Inc/search_class.h"
#include "../Inc/make_map_class.h"
#include "../Inc/wall_class.h"
#include "../Inc/queue_class.h"
#include "../Inc/priority_queue.h"
#include "../Inc/wall_class.h"
#include "../Inc/flash.h"
#include "../Inc/make_path.h"
#include "../Inc/mode.h"

#define ENABLE (0x01 << 4)

namespace Mode
{
	void Interface_Check()
	{
		t_bool debug_end = False;
		uint8_t mode = Return_LED_Status() & 0x30;
		uint8_t param = 0x00;
		uint8_t enable = 0x00;

		ring_queue<1024,t_MapNode> maze_q;
		motion_plan mp(&motion_task::getInstance());
		//Search solve_maze;
		wall_class wall_data(&SensingTask::getInstance());
		wall_data.init_maze();
		make_map map_data(&wall_data,&maze_q);
		Dijkstra run_path(&wall_data);
		uint32_t time = Interrupt::getInstance().return_time_count();
		 float fr,fl,sr,sl;
		 int16_t int_fr,int_fl,int_sr,int_sl;
		while(debug_end == False)
		{
			enable = Mode::Select(enable,0x01,Encoder_GetProperty_Left());
			param = (enable == 0x00) ? Mode::Select(param,0x0f,Encoder_GetProperty_Right()) : param;

			Battery_LimiterVoltage();
			if(enable == 0x01)
			{
				if((Interrupt::getInstance().return_time_count() - time) > 400)
				{
					time = Interrupt::getInstance().return_time_count();
					Indicate_LED((Return_LED_Status() != (mode|param)) ?  mode|param : 0x00);
				}
			}
			else
			{
				Indicate_LED(mode|param);
			}
			switch((enable<<4)|param)
			{
				case ENABLE|0x00:
					 fr = SensingTask::getInstance().sen_fr.distance;	fl = SensingTask::getInstance().sen_fl.distance;
					 sr = SensingTask::getInstance().sen_r.avg_distance;	sl = SensingTask::getInstance().sen_l.avg_distance;
					 int_fr = SensingTask::getInstance().sen_r.value_sum;	int_fl = SensingTask::getInstance().sen_l.value_sum;
					 int_sr = SensingTask::getInstance().sen_r.value;	int_sl = SensingTask::getInstance().sen_l.value;
					 printf("fr:%f,fl:%f,sr:%f,sl:%f\n",fr,fl,sr,sl);
					 printf("fr:%4d,fl:%4d,sr:%4d,sl:%4d\n",int_fr,int_fl,int_sr,int_sl);
					 break;
				case ENABLE|0x01:
					 fr = SensingTask::getInstance().sen_fr.distance;	fl = SensingTask::getInstance().sen_fl.distance;
					 sr = SensingTask::getInstance().sen_r.distance;	sl = SensingTask::getInstance().sen_l.distance;
					 int_fr = SensingTask::getInstance().sen_fr.value;	int_fl = SensingTask::getInstance().sen_fl.value;
					 int_sr = SensingTask::getInstance().sen_r.value;	int_sl = SensingTask::getInstance().sen_l.value;
					 printf("fr:%f,fl:%f,sr:%f,sl:%f\n",fr,fl,sr,sl);
					 printf("fr:%4d,fl:%4d,sr:%4d,sl:%4d\n",int_fr,int_fl,int_sr,int_sl);
					 break;
				case ENABLE|0x02:
					printf("gyro:%lf\n",(-1.0)*read_gyro_z_axis()*PI/180);
					HAL_Delay(10);
					break;
				case ENABLE|0x03:
					while(1)
					{
						printf("length:%lf\n",motion_task::getInstance().mouse.length);
						HAL_Delay(10);
						//if(Button() == 1) break;
					}
						break;
				case ENABLE|0x04:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  mp.free_rotation();
						  while(motion_task::getInstance().run_task !=No_run){
								printf("gyro:%ld,%ld\n",Encoder_GetProperty_Right().sp_pulse,Encoder_GetProperty_Left().sp_pulse);
								HAL_Delay(10);
						  }
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x05:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){

								  for(int i = 0;i < 11;i++)
								  {
									  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
									  HAL_Delay(50);
								  }
								  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
								  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.01, 0.0);
								  KalmanFilter::getInstance().filter_init();
								  mp.motion_start();
								  LogData::getInstance().data_count = 0;
								  LogData::getInstance().log_enable = True;
								  mp.pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
								  while(motion_task::getInstance().run_task !=No_run){}
								  LogData::getInstance().log_enable = False;
								  enable = 0x00;
								  HAL_Delay(500);

					}
					break;
				case ENABLE|0x06:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
							  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
							  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.01, 0.0);
							  KalmanFilter::getInstance().filter_init();
							  mp.motion_start();
							  LogData::getInstance().data_count = 0;
							  LogData::getInstance().log_enable = True;
								mp.fix_wall(3000);
								while(motion_task::getInstance().run_task !=No_run){}
							  LogData::getInstance().log_enable = False;
							  enable = 0x00;
							  HAL_Delay(500);
					}
					break;
				case ENABLE|0x07:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
							for(int i = 10; i <= 800; i = i + 10)
							{
								FAN_Motor_SetDuty(i);;
								HAL_Delay(3);
							}
							HAL_Delay(10000);
							FAN_Motor_SetDuty(0);
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x08:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x09:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0A:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0B:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0C:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0D:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0E:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						for(int i = 0;i < 11;i++)
						{
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
						}
						LogData::getInstance().indicate_data();
						enable = 0x00;
					}
					break;
				case ENABLE|0x0F:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						for(int i = 0;i < 11;i++)
						{
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
						}
						debug_end = True;
					}
					break;
				default:
					break;
			}
		}

	}

}






