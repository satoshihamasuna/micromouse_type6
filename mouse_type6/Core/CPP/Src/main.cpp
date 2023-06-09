/*
 * main.cpp
 *
 *  Created on: 2023/06/10
 *      Author: sato1
 */
/*
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
*/

//#include "typedef_node"
#include <stdio.h>
#include <iostream>
#include <index.h>
#include <typedef.h>
#include "../Inc/typedef_node.h"
#include "../Inc/queue.h"


void CPP_main()
{
	  imu_initialize();
	  Sensor_Initialize();
	  Motor_Initialize();
	  FAN_Motor_Initialize();
	  Encoder_Initialize();
	  Interrupt_Initialize();
	  IMU_read_DMA_Start();

	  mouse_mode = 0x00;
	  is_mode_enable = 0;

	  Priority_queue<t_MapNode> queue();


	 while (1)
	  {
		  Mode_Change_ENC();
		  HAL_Delay(50);
		  switch(is_mode_enable|mouse_mode){
		  	  case (ENABLE_MODE3|0x00):
		  			  printf("gyro:%lf\n",read_gyro_z_axis());
		  			  break;
		  	  case (ENABLE_MODE3|0x01):
		  			  FAN_Motor_SetDuty(500);
		  	  	  	  HAL_Delay(1500);
		  	  	      FAN_Motor_SetDuty(0);
		  	  	      HAL_Delay(100);
		  	  	  	  is_mode_enable = DISABLE;
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
