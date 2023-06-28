/*
 * main.cpp
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */


#include "index.h"
#include "stdio.h"
#include "sensing_task.h"
#include "motion.h"
#include "interrupt.h"
#include "controll.h"
#include "macro.h"
#include "log_data.h"
#include "Kalman_filter.h"
#include "run_param.h"
#include "search_class.h"
#include "make_map_class.h"
#include "wall_class.h"
#include "queue_class.h"
#include "priority_queue.h"


void CPP_Main()
{
	  Priority_queue<10,int> q;
	  ring_queue<1024,t_MapNode> maze_q;
	  imu_initialize();
	  Sensor_Initialize();
	  Motor_Initialize();
	  FAN_Motor_Initialize();
	  Encoder_Initialize();
	  Interrupt_Initialize();
	  IMU_read_DMA_Start();
	  Mode_Init();
	  motion_plan mp;
	  Search solve_maze;
	  wall_class wall_data(&SensingTask::getInstance());
	  wall_data.init_maze();
	  make_map map_data(&wall_data,&maze_q);

	  q.push(0);
	  while (1)
	  {

		  	  //Battery_LimiterVoltage();
			  Mode_Change_ENC();
			  HAL_Delay(5);
			  switch(Mode_State()){
			  	  case (ENABLE_MODE3|0x00):
			  			  printf("fr:%f,fl:%f,sr:%f,sl:%f\n",
			  					  SensingTask::getInstance().sen_fr.distance,SensingTask::getInstance().sen_fl.distance
								 ,SensingTask::getInstance().sen_r.distance,SensingTask::getInstance().sen_l.distance);
			  	  	  	  //printf("length:%lf\n",Battery_GetVoltage());
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
					  		  motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
					  		  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.2, 0.001, 0.0);
					  		  KalmanFilter::getInstance().filter_init();

					  		  mp.motion_start(&motion_task::getInstance());
					  		  //mp.search_straight(&motion_task::getInstance(),90.0,4.0,0.3,0.30);
					  		  //while(motion_task::getInstance().run_task !=No_run){}
					  		  LogData::getInstance().data_count = 0;
					  		  LogData::getInstance().log_enable = True;
					  		  //mp.search_slalom(&motion_task::getInstance(), &param_L90_search);
					  		  //while(motion_task::getInstance().run_task !=No_run){}
					  		  mp.search_straight(&motion_task::getInstance(),180.0,4.0,0.3,0.0);
					  		  while(motion_task::getInstance().run_task !=No_run){}
					  		  LogData::getInstance().log_enable = False;
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
					  		  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.2, 0.001, 0.0);
					  		  KalmanFilter::getInstance().filter_init();
					  		  LogData::getInstance().data_count = 0;
					  		  LogData::getInstance().log_enable = True;
					  		  mp.motion_start(&motion_task::getInstance());
					  		  mp.search_straight(&motion_task::getInstance(),45.0,4.0,0.3,0.30);
					  		  while(motion_task::getInstance().run_task !=No_run){}
					  		  mp.searchSlalom(&motion_task::getInstance(),&param_R90_search);
					  		  while(motion_task::getInstance().run_task !=No_run){}
					  		  mp.search_straight(&motion_task::getInstance(),45.0,4.0,0.3,0.0);
					  		  while(motion_task::getInstance().run_task !=No_run){}
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
				    		  motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.2, 0.001, 0.0);
					 		  KalmanFilter::getInstance().filter_init();
					 		  LogData::getInstance().data_count = 0;
					 		  LogData::getInstance().log_enable = True;
					  		  mp.motion_start(&motion_task::getInstance());
					 		  mp.search_straight(&motion_task::getInstance(),45.0,4.0,0.3,0.30);
					 		  while(motion_task::getInstance().run_task !=No_run){}
					 		  mp.searchSlalom(&motion_task::getInstance(),&param_L90_search);
					 		  while(motion_task::getInstance().run_task !=No_run){}
							  mp.search_straight(&motion_task::getInstance(),45.0,4.0,0.3,0.0);
					 		  while(motion_task::getInstance().run_task !=No_run){}
					 		  HAL_Delay(200);
							  LogData::getInstance().log_enable = False;
							  Mode_Disable();
						}
			  			  break;
			  	  case (ENABLE_MODE3|0x06):
						if(SensingTask::getInstance().IrSensor_Avg() > 2500)
						{
							for(int i = 0;i < 10;i++)
							{
								(i%2 == 0) ? Indicate_LED(ENABLE_MODE3|0x06):Indicate_LED(0x00|0x00);
								HAL_Delay(50);
							}
							Indicate_LED(ENABLE_MODE3|0x06);
							motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
							motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.2, 0.005, 0.0);
					  		KalmanFilter::getInstance().filter_init();
					  		t_position start,goal;
					  		start.x = start.y = 0;start.dir = North;
					  		goal.x =0, goal.y = 3;
					  		solve_maze.search_adachi_1(start, goal, 1, &wall_data, &map_data,&mp);
					  		Mode_Disable();
						}
						break;
			  	  case (ENABLE_MODE3|0x07):
						if(SensingTask::getInstance().IrSensor_Avg() > 2500)
						{
							for(int i = 0;i < 10;i++)
							{
								(i%2 == 0) ? Indicate_LED(ENABLE_MODE3|0x06):Indicate_LED(0x00|0x00);
								HAL_Delay(50);
							}
							t_position start,goal;
					  		start.x = start.y = 0;start.dir = North;
					  		goal.x =0, goal.y = 3;
							map_data.init_map(goal.x, goal.y, 1);
							map_data.make_map_queue(goal.x, goal.y, start, 1, 0x01);
							map_data.Display();
							Mode_Disable();

						}
			  			  break;
			  	  case (ENABLE_MODE3|0x08):
			  			  break;
			  	  case (ENABLE_MODE3|0x09):
			  			  break;
			  	  case (ENABLE_MODE3|0x0A):
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
}
