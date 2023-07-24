/*
 * main.cpp
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */

#include <iostream>

#include "../../Module/Include/index.h"
#include "stdio.h"
#include "sensing_task.h"
#include "motion.h"
#include "interrupt.h"
#include "controll.h"
#include "../../Module/Include/macro.h"
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
#include "mode.h"



void Module_Initialize()
{
	  IMU_initialize();
	  Sensor_Initialize();
	  Motor_Initialize();
	  FAN_Motor_Initialize();
	  Encoder_Initialize();
	  Interrupt_Initialize();
	  IMU_read_DMA_Start();
}

void CPP_Main()
{
	  Module_Initialize();
	  Mode::Select_Mode();


	  /*
	  while (1)
	  {

		  	  Battery_LimiterVoltage();
			  Mode_Change_ENC();
			  HAL_Delay(5);
			  switch(Mode_State()){
			  	  case (ENABLE_MODE3|0x00):
			  			  float fr,fl,sr,sl;
			  	  	  	  int16_t int_fr,int_fl,int_sr,int_sl;
			  	  	  	  fr = SensingTask::getInstance().sen_fr.distance;	fl = SensingTask::getInstance().sen_fl.distance;
			  	  	  	  sr = SensingTask::getInstance().sen_r.distance;	sl = SensingTask::getInstance().sen_l.distance;
			  	  	  	  int_fr = SensingTask::getInstance().sen_fr.value;	int_fl = SensingTask::getInstance().sen_fl.value;
			  	  	  	  int_sr = SensingTask::getInstance().sen_r.value;	int_sl = SensingTask::getInstance().sen_l.value;
			  	  	  	  printf("fr:%f,fl:%f,sr:%f,sl:%f\n",fr,fl,sr,sl);
			  	  	  	  printf("fr:%4d,fl:%4d,sr:%4d,sl:%4d\n",int_fr,int_fl,int_sr,int_sl);
			  			  break;
			  	  case (ENABLE_MODE3|0x01):
			  			  if(SensingTask::getInstance().IrSensor_Avg() > 2500){
			  				  HAL_Delay(1000);
			  				  FAN_Motor_SetDuty(700);;
			  				  HAL_Delay(6000);
			  				  FAN_Motor_SetDuty(0);;
			  				  HAL_Delay(100);
			  				  Mode_Disable();
			  			  }
			  			  break;
			  	  case (ENABLE_MODE3|0x02):
					  	  if(SensingTask::getInstance().IrSensor_Avg() > 2500){
							  for(int i = 0;i < 11;i++)
							  {
								  (i%2 == 0) ? Indicate_LED(Mode_State()):Indicate_LED(0x00|0x00);
								  HAL_Delay(50);
							  }
					  		  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
					  		  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.01, 0.0);
					  		  //mp.fix_wall( 400);
			  				  //HAL_Delay(100);
			  				  //FAN_Motor_SetDuty(700);;
			  				  //while(motion_task::getInstance().run_task !=No_run){}
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
			  				  //HAL_Delay(100);
			  				  //FAN_Motor_SetDuty(0);;
			  				  HAL_Delay(500);
					  		  Mode_Disable();
					  	   }
			  			   break;
			  	  case (ENABLE_MODE3|0x03):
						if(SensingTask::getInstance().IrSensor_Avg() > 2500){
							  for(int i = 0;i < 11;i++)
							  {
								  (i%2 == 0) ? Indicate_LED(Mode_State()):Indicate_LED(0x00|0x00);
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
					  		  mp.search_straight(SECTION,9.0,1.0,1.0);
					  		  while(motion_task::getInstance().run_task !=No_run){}
					  		  LogData::getInstance().data_count = 0;
					  		  LogData::getInstance().log_enable = True;
					  		  for(int i = 0; i < 8;i++)
					  		  {
								  mp.long_turn( &param_R180_1000,Long_turnR180);
								  while(motion_task::getInstance().run_task !=No_run){}
					  		  }
					  		  mp.search_straight( SECTION,9.0,1.0,0.0);
					  		  while(motion_task::getInstance().run_task !=No_run){}
					  		  HAL_Delay(200);
					  		  FAN_Motor_SetDuty(0);;
					  		  HAL_Delay(200);
					  		  LogData::getInstance().log_enable = False;
					  		  Mode_Disable();
						 }
			  			 break;
			  	  case (ENABLE_MODE3|0x04):
						if(SensingTask::getInstance().IrSensor_Avg() > 2500){
							LogData::getInstance().indicate_data();
							Mode_Disable();
				   		}
			  			  break;
			  	  case (ENABLE_MODE3|0x05):
						if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(Mode_State()):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
				  		  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
				  		  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.005, 0.0);
				 		  KalmanFilter::getInstance().filter_init();
				 		  LogData::getInstance().data_count = 0;
				 		  LogData::getInstance().log_enable = True;
				  		  mp.motion_start();
				 		  mp.search_straight( 45.0,4.0,0.3,0.30);
				 		  while(motion_task::getInstance().run_task !=No_run){}
				 		  for(int i = 0;i < 8;i++){
							  mp.searchSlalom( &param_L90_search);
							  while(motion_task::getInstance().run_task !=No_run){}
				 		  }
						  mp.search_straight( 45.0,4.0,0.3,0.0);
				 		  while(motion_task::getInstance().run_task !=No_run){}
				 		  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  Mode_Disable();
						}
			  			  break;
			  	  case (ENABLE_MODE3|0x06):
						if(SensingTask::getInstance().IrSensor_Avg() > 2500)
						{
							for(int i = 0;i < 11;i++)
							{
							  (i%2 == 0) ? Indicate_LED(Mode_State()):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
							}
							Indicate_LED(ENABLE_MODE3|0x06);
					  		motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
					  		motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.005, 0.0);
					  		KalmanFilter::getInstance().filter_init();
					  		t_position start,goal;
					  		start.x = start.y = 0;start.dir = North;
					  		goal.x = MAZE_GOAL_X, goal.y = MAZE_GOAL_Y;
					  		t_position return_pos = solve_maze.search_adachi_1(start, goal, 2, &wall_data, &map_data,&mp);
					  		write_save_data(&wall_data);
					  		solve_maze.search_adachi_2(return_pos, start, 1, &wall_data, &map_data,&mp);
					  		write_save_data(&wall_data);
					  		Mode_Disable();
						}
						break;
			  	  case (ENABLE_MODE3|0x07):
						if(SensingTask::getInstance().IrSensor_Avg() > 2500)
						{
							for(int i = 0;i < 11;i++)
							{
							  (i%2 == 0) ? Indicate_LED(Mode_State()):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
							}
							//test_wall_set(&wall_data);
							t_position start,goal;
					  		start.x = start.y = 0;start.dir = North;
						  	goal.x = MAZE_GOAL_X, goal.y = MAZE_GOAL_Y;
							map_data.init_map(goal.x, goal.y, 2);
							map_data.make_map_queue(goal.x, goal.y, start, 2, 0x01);
							map_data.Display();
							Mode_Disable();

						}
			  			  break;
			  	  case (ENABLE_MODE3|0x08):
						if(SensingTask::getInstance().IrSensor_Avg() > 2500)
						{
							for(int i = 0;i < 11;i++)
							{
								(i%2 == 0) ? Indicate_LED(Mode_State()):Indicate_LED(0x00|0x00);
								HAL_Delay(50);
							}
							read_save_data(&wall_data);
							Mode_Disable();
						}
			  			  break;
			  	  case (ENABLE_MODE3|0x09):
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
					  		motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
					  		motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.4, 0.005, 0.0);
					  		KalmanFilter::getInstance().filter_init();
					  		run_path.turn_time_set(mode_1000);
							run_path.run_Dijkstra_suction(		start, Dir_None, goal, 2,900,
																st_mode_1000_v1, (int)(sizeof(st_mode_1000_v1)/sizeof(t_straight_param *const)),
																di_mode_1000_v1, (int)(sizeof(di_mode_1000_v1)/sizeof(t_straight_param *const)), mode_1000,&mp);

							Mode_Disable();
						}
			  			  break;
			  	  case (ENABLE_MODE3|0x0A):
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
			  	  case (ENABLE_MODE3|0x0B):
			  			  break;
			  	  case (ENABLE_MODE3|0x0C):
			  			  break;
			  	  case (ENABLE_MODE3|0x0D):
			  			  break;
			  	  case (ENABLE_MODE3|0x0E):
			  			  break;
			  	  case (ENABLE_MODE3|0x0F):
			  			  break;
			  }
		  }
		  */
}
