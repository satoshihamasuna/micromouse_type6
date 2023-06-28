/*
 * interrupt.c
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */


#include <motion.h>
#include "index.h"
#include "interrupt.h"
#include "sensing_task.h"
#include "controll.h"
#include "macro.h"
#include "log_data.h"
#include "Kalman_filter.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim5){
    	Interrupt::getInstance().preprocess();
    	Interrupt::getInstance().main();
    	Interrupt::getInstance().postprocess();
    }
}

void Interrupt_Initialize(){
	HAL_TIM_Base_Start_IT(&htim5);
}

void Interrupt::preprocess(){

	SensingTask::getInstance().IrSensorSet();

	Encoder_SetSpeed_Left();
	Encoder_SetSpeed_Right();
	t_encoder Renc = Encoder_GetProperty_Right();
	t_encoder Lenc = Encoder_GetProperty_Left();
	float sp = KalmanFilter::getInstance().calc_speed_filter(read_accel_y_axis(), (Renc.wheel_speed - Lenc.wheel_speed)/2.0);
	motion_task::getInstance().mouse.velo 	  = sp;//(Renc.wheel_speed - Lenc.wheel_speed)/2.0;
	motion_task::getInstance().mouse.length  += (Renc.wheel_speed - Lenc.wheel_speed)/2.0;
	motion_task::getInstance().mouse.rad_velo = (-1.0)*read_gyro_z_axis()*PI/180;
	motion_task::getInstance().mouse.radian  += motion_task::getInstance().mouse.rad_velo/1000.0;

}

void Interrupt::main()
{
	motion_task::getInstance().motion_inInterrupt();
	motion_task::getInstance().motionControll();
}

void Interrupt::postprocess()
{

	if(LogData::getInstance().log_enable == True)
	{
		LogData::getInstance().data[0][LogData::getInstance().data_count%1000] = motion_task::getInstance().mouse.velo;
		LogData::getInstance().data[1][LogData::getInstance().data_count%1000] = motion_task::getInstance().target.velo;
		LogData::getInstance().data[2][LogData::getInstance().data_count%1000] = motion_task::getInstance().mouse.rad_velo;
		LogData::getInstance().data[3][LogData::getInstance().data_count%1000] = motion_task::getInstance().target.rad_velo;
		LogData::getInstance().data_count++;
		if(LogData::getInstance().data_count++ >= 1000) LogData::getInstance().data_count = 0;
	}
	motion_task::getInstance().motionPostControll();
	IMU_read_DMA_Start();
}

