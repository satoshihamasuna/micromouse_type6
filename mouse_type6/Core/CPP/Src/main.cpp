/*
 * main.cpp
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */


#include "index.h"
#include "stdio.h"
#include "sensing_task.h"

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
	  while (1)
	  {
			  Mode_Change_ENC();
			  HAL_Delay(50);
			  switch(Mode_State()){
			  	  case (ENABLE_MODE3|0x00):
			  			  printf("gyro:%lf\n",read_gyro_z_axis());
			  			  break;
			  	  case (ENABLE_MODE3|0x01):
			  			  if(SensingTask::getInstance().IrSensor_Avg() > 2500){
			  				  HAL_Delay(1000);
			  				  FAN_Motor_SetDuty(500);;
			  				  HAL_Delay(1000);
			  				  FAN_Motor_SetDuty(0);;
			  				  HAL_Delay(100);
			  				  Mode_Disable();
			  			  }
			  			  break;
			  	  case (ENABLE_MODE3|0x02):
			  			  break;
			  	  case (ENABLE_MODE3|0x03):
			  			  break;
			  	  case (ENABLE_MODE3|0x04):
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
