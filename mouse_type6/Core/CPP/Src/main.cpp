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

void CPP_Main()
{
	  imu_initialize();
	  Sensor_Initialize();
	  Motor_Initialize();
	  FAN_Motor_Initialize();
	  Encoder_Initialize();
	  Interrupt_Initialize();
	  IMU_read_DMA_Start();
	  Mode_Init();
	  motion_plan mp;
	  while (1)
	  {
			  Mode_Change_ENC();
			  HAL_Delay(50);
			  switch(Mode_State()){
			  	  case (ENABLE_MODE3|0x00):
			  			  printf("acc:%lf\n",read_accel_y_axis());
			  	  	  	  printf("length:%lf\n",Battery_GetVoltage());
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

					  		  	 motion_task::getInstance().ct.speed_ctrl.Gain_Set(4.0, 0.05, 0.0);
					  		  	 motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.2, 0.00, 0.0);
					  		  	 KalmanFilter::getInstance().filter_init();
					  		  	 mp.search_straight(&motion_task::getInstance(),90.0,4.0,0.3,0.0);
					  		  	 LogData::getInstance().data_count = 0;
					  		  	 LogData::getInstance().log_enable = True;
					  		  	 while(motion_task::getInstance().run_task !=No_run)
					  		  	 {
					  		  		//printf("Kp:%lf,velo:%lf\n",motion_task::getInstance().mouse.velo,motion_task::getInstance().target.velo);
					  		  		//HAL_Delay(1);
					  		  	 }
					  		  	LogData::getInstance().log_enable = False;
					  		    Mode_Disable();
					  		}
			  			  break;
			  	  case (ENABLE_MODE3|0x03):
						if(SensingTask::getInstance().IrSensor_Avg() > 2500){

					  		  	 motion_task::getInstance().ct.speed_ctrl.Gain_Set(4.0, 0.05, 0.0);
					  		     motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.2, 0.0, 0.0);
					  		   KalmanFilter::getInstance().filter_init();
					  		  	 mp.pivot_turn(&motion_task::getInstance(),DEG2RAD(360.0f),40.0f*PI,3.0f * PI);
					  		  	 //HAL_Delay(100);
					  		  	 while(motion_task::getInstance().run_task !=No_run)
							  	 {
							   		   //printf("length:%lf\n",motion_task::getInstance().mouse.radian);
							    		//HAL_Delay(1);
							  	 }
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
			  			  break;
			  	  case (ENABLE_MODE3|0x06):
			  			  break;
			  	  case (ENABLE_MODE3|0x07):
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
