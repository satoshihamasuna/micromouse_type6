/*
 * mode.cpp
 *
 *  Created on: 2023/07/22
 *      Author: sato1
 */

#include "../../Module/Include/index.h"
#include "../../Module/Include/macro.h"

#include "stdio.h"
#include "sensing_task.h"
#include "motion.h"
#include "interrupt.h"
#include "controll.h"
#include "log_data.h"
#include "Kalman_filter.h"
#include "run_param.h"
#include "search_class.h"
#include "make_map_class.h"
#include "wall_class.h"
#include "queue_class.h"
#include "priority_queue.h"
#include "wall_class.h"
#include "flash.h"
#include "make_path.h"
#include "test_wall.h"
#include "mode.h"

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
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						for(int i = 0;i < 11;i++)
						{
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
						}
						demo_end = True;
					}
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
		//uint8_t Indicate_param = param;
		uint8_t enable = 0x00;
		//uint32_t observe_time = 0;

		ring_queue<1024,t_MapNode> maze_q;
		motion_plan mp(&motion_task::getInstance());
		//Search solve_maze;
		wall_class wall_data(&SensingTask::getInstance());
		wall_data.init_maze();
		make_map map_data(&wall_data,&maze_q);
		Dijkstra run_path(&wall_data);

		while(debug_end == False)
		{
			enable = Mode::Select(enable,0x01,Encoder_GetProperty_Left());
			param = (enable == 0x00) ? Mode::Select(param,0x0f,Encoder_GetProperty_Right()) : param;
			Indicate_LED(mode|param);
			switch((enable<<4)|param)
			{
				case ENABLE|0x00:
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
						  mp.straight( 45.0,6.0,0.3,0.3);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.searchSlalom( &param_L90_search);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight(45.0,6.0,0.3,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
			  	    }
			  	    break;
				case ENABLE|0x01:
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
						  mp.straight( 45.0,6.0,0.3,0.3);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.searchSlalom( &param_R90_search);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight(45.0,6.0,0.3,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
			  	    }
					break;
				case ENABLE|0x02:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
						  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.05, 0.0);
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= 500; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.search_straight(SECTION,9.0,1.0,1.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.long_turn( &param_R180_1000,Long_turnR180);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.search_straight( SECTION,9.0,1.0,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
				  		  enable = 0x00;
				  		  HAL_Delay(500);
					}
					break;
				case ENABLE|0x03:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
						  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.05, 0.0);
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= 500; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,9.0,1.0,1.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.long_turn( &param_R90_1000,Long_turnR90);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight( SECTION,9.0,1.0,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x04:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
						  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.05, 0.0);
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= 500; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,9.0,1.0,1.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.long_turn( &param_inR45_1000,Turn_in_R45);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal( DIAG_SECTION,9.0,1.0,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
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
						  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.05, 0.0);
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= 500; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,9.0,1.0,1.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.long_turn( &param_inR135_1000,Turn_in_R135);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal( DIAG_SECTION,9.0,1.0,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
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
						  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.05, 0.0);
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= 500; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal(DIAG_SECTION,9.0,1.0,1.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.long_turn( &param_outR45_1000,Turn_out_R45);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight( SECTION,9.0,1.0,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
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
						  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
						  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.05, 0.0);
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= 500; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal(DIAG_SECTION,9.0,1.0,1.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.long_turn( &param_outR135_1000,Turn_out_R135);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight( SECTION,9.0,1.0,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
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
						  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
						  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.05, 0.0);
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= 500; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal(DIAG_SECTION,9.0,1.0,1.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.long_turn( &param_RV90_1000,Turn_RV90);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal( SECTION,10.0,1.0,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
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

