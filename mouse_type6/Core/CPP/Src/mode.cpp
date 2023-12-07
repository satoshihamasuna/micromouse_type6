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
#include "../Inc/test_wall.h"
#include "../Inc/mode.h"

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
						motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.05, 0.0);
						KalmanFilter::getInstance().filter_init();
						t_position return_pos = solve_maze.search_adachi_1_acc(start, goal, goal_size, &wall_data, &map_data,&mp);
						write_save_data(&wall_data);
						solve_maze.search_adachi_2_acc(return_pos, start, 1, &wall_data, &map_data,&mp);
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
						motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.05, 0.0);
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
					if(SensingTask::getInstance().IrSensor_Avg() > 2500)
					{
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}
						read_save_data(&wall_data);
						enable = 0x00;
					}
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
				  		motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.01, 0.0);
				  		KalmanFilter::getInstance().filter_init();
				  		run_path.turn_time_set(mode_1000);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal,MAZE_GOAL_SIZE,950,
															st_mode_1000_v0, (int)(sizeof(st_mode_1000_v0)/sizeof(t_straight_param *const)),
															di_mode_1000_v0, (int)(sizeof(di_mode_1000_v0)/sizeof(t_straight_param *const)), mode_1000,&mp);

						enable = 0x00;
					}
					break;
				case ENABLE|0x09:
				   if(SensingTask::getInstance().IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}
				  		motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
				  		motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.01, 0.0);
				  		KalmanFilter::getInstance().filter_init();
				  		run_path.turn_time_set(mode_1000);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,950,
															st_mode_1000_v1, (int)(sizeof(st_mode_1000_v1)/sizeof(t_straight_param *const)),
															di_mode_1000_v1, (int)(sizeof(di_mode_1000_v1)/sizeof(t_straight_param *const)), mode_1000,&mp);

						enable = 0x00;
					}
					break;
				case ENABLE|0x0A:
				   if(SensingTask::getInstance().IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}
				  		motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
				  		motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.01, 0.0);
				  		KalmanFilter::getInstance().filter_init();
				  		run_path.turn_time_set(mode_1200);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,950,
															st_mode_1200_v0, (int)(sizeof(st_mode_1200_v0)/sizeof(t_straight_param *const)),
															di_mode_1200_v0, (int)(sizeof(di_mode_1200_v0)/sizeof(t_straight_param *const)), mode_1200,&mp);

						enable = 0x00;
					}
					break;
				case ENABLE|0x0B:
				   if(SensingTask::getInstance().IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}
				  		motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
				  		motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.01, 0.0);
				  		KalmanFilter::getInstance().filter_init();
				  		run_path.turn_time_set(mode_1400);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,950,
															st_mode_1400_v0, (int)(sizeof(st_mode_1400_v0)/sizeof(t_straight_param *const)),
															di_mode_1400_v0, (int)(sizeof(di_mode_1400_v0)/sizeof(t_straight_param *const)), mode_1400,&mp);

						enable = 0x00;
					}
					break;
				case ENABLE|0x0C:
				   if(SensingTask::getInstance().IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}
				  		motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
				  		motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.01, 0.0);
				  		KalmanFilter::getInstance().filter_init();
				  		run_path.turn_time_set(mode_1400);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,950,
															st_mode_1200_v1, (int)(sizeof(st_mode_1200_v1)/sizeof(t_straight_param *const)),
															di_mode_1200_v1, (int)(sizeof(di_mode_1200_v1)/sizeof(t_straight_param *const)), mode_1200,&mp);

						enable = 0x00;
					}
					break;
				case ENABLE|0x0D:
				   if(SensingTask::getInstance().IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}
				  		motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
				  		motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.01, 0.0);
				  		KalmanFilter::getInstance().filter_init();
				  		run_path.turn_time_set(mode_1400);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,950,
															st_mode_1400_v1, (int)(sizeof(st_mode_1400_v1)/sizeof(t_straight_param *const)),
															di_mode_1400_v1, (int)(sizeof(di_mode_1400_v1)/sizeof(t_straight_param *const)), mode_1400,&mp);

						enable = 0x00;
					}
					break;
				case ENABLE|0x0E:
				   if(SensingTask::getInstance().IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(Mode_State()):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}
						t_position start,goal;
				  		start.x = start.y = 0;start.dir = North;
				  		goal.x = MAZE_GOAL_X, goal.y = MAZE_GOAL_Y;
				  		run_path.di_param_set(di_mode_1000_v0, 1);
				  		run_path.di_param_set(di_mode_1000_v0, 1);
				  		run_path.turn_time_set(mode_1000);
						run_path.check_run_Dijkstra(start, Dir_None, goal, 2);

						Mode_Disable();
					}
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
					enable = 0;
					break;
				case ENABLE|0x02:
					Mode::Debug(&st_param_1400,mode_1400);
					enable = 0;
					break;
				case ENABLE|0x03:
					Mode::Debug2(&st_param_1400_acc16,mode_1400);
					enable = 0;
					break;
				default:
					break;
			}
		}
	}
}

