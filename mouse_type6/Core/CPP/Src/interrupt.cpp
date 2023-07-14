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
	static int cnt;
	static float z_acc;
	z_acc = 0.8*z_acc + 0.2*read_accel_z_axis();
	t_encoder Renc = Encoder_GetProperty_Right();
	t_encoder Lenc = Encoder_GetProperty_Left();
	if(motion_task::getInstance().is_controll_enable() == True && motion_task::getInstance().run_task != Fix_wall)
	{
		if(motion_task::getInstance().target.velo != 0.0f)
		{
			if(ABS((motion_task::getInstance().mouse.velo - motion_task::getInstance().target.velo)/motion_task::getInstance().target.velo) >= 0.2)
			{
				cnt = cnt + 10;
			}
			else if(ABS(z_acc) >= 30.0)
			{
				cnt = cnt + 200;
			}
			else if(ABS(Renc.wheel_speed) > 8.0 ||  ABS(Lenc.wheel_speed) > 8.0 )
			{
				cnt = cnt + 500;
			}
			else
			{
				cnt = cnt - 2;
				if(cnt < 0) cnt = 0;
			}
		}
		else
		{
			cnt = 0;
		}
		if(cnt >= 1000)
		{
			Motor_Stop();FAN_Motor_Stop();
			NVIC_SystemReset();

		}
	}
	else
	{
		cnt = 0;
	}

	if(motion_task::getInstance().is_controll_enable() == True)
	{
		if(SensingTask::getInstance().sen_l.is_wall == True)
		{
			Indicate_LED(Return_LED_Status()|(0x01 << 5));
		}
		else
		{
			Indicate_LED(Return_LED_Status()&(0xff - (0x01 << 5)));
		}

		if(SensingTask::getInstance().sen_r.is_wall == True)
		{
			Indicate_LED(Return_LED_Status()|(0x01 << 6));
		}
		else
		{
			Indicate_LED(Return_LED_Status()&(0xff - (0x01 << 6)));
		}
	}

	if(LogData::getInstance().log_enable == True)
	{
		LogData::getInstance().data[0][LogData::getInstance().data_count%1000] = motion_task::getInstance().mouse.velo;
		LogData::getInstance().data[1][LogData::getInstance().data_count%1000] = motion_task::getInstance().target.velo;
		LogData::getInstance().data[2][LogData::getInstance().data_count%1000] = motion_task::getInstance().mouse.rad_velo;
		LogData::getInstance().data[3][LogData::getInstance().data_count%1000] = motion_task::getInstance().target.rad_velo;
		LogData::getInstance().data_count++;
		if(LogData::getInstance().data_count >= 1000) LogData::getInstance().data_count = 999;
	}
	motion_task::getInstance().motionPostControll();

	IMU_read_DMA_Start();
}

