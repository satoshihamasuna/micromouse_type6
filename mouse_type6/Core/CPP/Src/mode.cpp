/*
 * mode.cpp
 *
 *  Created on: 2023/07/22
 *      Author: sato1
 */

#include "../../Module/Include/index.h"
#include "../../Module/Include/macro.h"
#include "../../Module/Include/typedef.h"

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
		uint32_t time = Interrupt::getInstance().return_time_count();

		ring_queue<1024,t_MapNode> maze_q;
		motion_plan mp(&motion_task::getInstance());
		Search solve_maze;
		wall_class wall_data(&SensingTask::getInstance());
		wall_data.init_maze();
		make_map map_data(&wall_data,&maze_q);
		Dijkstra run_path(&wall_data);

		t_position start,goal;
		start.x = start.y = 0;start.dir = North;
		goal.x = MAZE_GOAL_X, goal.y = MAZE_GOAL_Y;
		uint8_t goal_size = MAZE_GOAL_SIZE;

		while(demo_end == False)
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
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}

						Indicate_LED(mode|param);
						motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
						motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.005, 0.0);
						KalmanFilter::getInstance().filter_init();
						t_position return_pos = solve_maze.search_adachi_1(start, goal, goal_size, &wall_data, &map_data,&mp);
						write_save_data(&wall_data);
						solve_maze.search_adachi_2(return_pos, start, 1, &wall_data, &map_data,&mp);
						write_save_data(&wall_data);
						enable = 0x00;
					}
					break;
				case ENABLE|0x01:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}

						Indicate_LED(mode|param);
						motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
						motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.005, 0.0);
						KalmanFilter::getInstance().filter_init();
						t_position return_pos = solve_maze.search_adachi_1(start, goal, goal_size, &wall_data, &map_data,&mp);
						write_save_data(&wall_data);
						solve_maze.search_adachi_1(return_pos, start, 1, &wall_data, &map_data,&mp);
						write_save_data(&wall_data);
						enable = 0x00;
					}
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
				   if(SensingTask::getInstance().IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}
				  		motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
				  		motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.005, 0.0);
				  		KalmanFilter::getInstance().filter_init();
				  		run_path.turn_time_set(mode_1000);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, 2,900,
															st_mode_1000_v1, (int)(sizeof(st_mode_1000_v1)/sizeof(t_straight_param *const)),
															di_mode_1000_v1, (int)(sizeof(di_mode_1000_v1)/sizeof(t_straight_param *const)), mode_1000,&mp);

						enable = 0x00;
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
		uint8_t enable = 0x00;

		ring_queue<1024,t_MapNode> maze_q;
		motion_plan mp(&motion_task::getInstance());
		//Search solve_maze;
		wall_class wall_data(&SensingTask::getInstance());
		wall_data.init_maze();
		make_map map_data(&wall_data,&maze_q);
		Dijkstra run_path(&wall_data);
		const t_param *const *turn_mode;
		turn_mode = mode_1000;
		float acc  = 10.0;
		float velo = 1.0;
		uint32_t time = Interrupt::getInstance().return_time_count();
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
						  mp.straight( 90.0*3.0,6.0,0.3,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  /*
						  mp.searchSlalom( &param_L90_search);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight(45.0,6.0,0.3,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  */
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
						  mp.search_straight(SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.long_turn(turn_mode[Long_turnR180],Long_turnR180);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.search_straight( SECTION,acc,velo,0.0);
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
						  mp.straight(SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.long_turn(turn_mode[Long_turnR90],Long_turnR90);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight( SECTION,acc,velo,0.0);
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
						  mp.straight(SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_in(turn_mode[Turn_in_R45],Turn_in_R45);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal( DIAG_SECTION,acc,velo,0.0);
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
						  mp.straight(SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_in(turn_mode[Turn_in_R135],Turn_in_R135);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal( DIAG_SECTION,acc,velo,0.0);
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
						  mp.diagonal(DIAG_SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_out(turn_mode[Turn_out_R45],Turn_out_R45);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight( SECTION,acc,velo,0.0);
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
						  mp.diagonal(DIAG_SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_out(turn_mode[Turn_out_R135],Turn_out_R135);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight( SECTION,acc,velo,0.0);
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
						  mp.diagonal(DIAG_SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_v90(turn_mode[Turn_RV90],Turn_RV90);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal( SECTION,acc,velo,0.0);
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
						  mp.search_straight(SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.long_turn(turn_mode[Long_turnL180],Long_turnL180);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.search_straight( SECTION,acc,velo,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
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
						  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
						  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.005, 0.0);
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= 500; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_in(turn_mode[Turn_in_L45],Turn_in_L45);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal( DIAG_SECTION,acc,velo,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
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
						  mp.diagonal(DIAG_SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_out(turn_mode[Turn_out_L45],Turn_out_L45);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight( SECTION,acc,velo,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
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
						  mp.diagonal(DIAG_SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_out(turn_mode[Turn_out_L135],Turn_out_L135);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.straight( SECTION,acc,velo,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
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
						  mp.diagonal(DIAG_SECTION,acc,velo,velo);
						  while(motion_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_v90(turn_mode[Turn_LV90],Turn_LV90);
						  while(motion_task::getInstance().run_task !=No_run){}
						  mp.diagonal( SECTION,acc,velo,0.0);
						  while(motion_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
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
