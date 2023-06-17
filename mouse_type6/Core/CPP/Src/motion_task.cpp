/*
 * motion_task.cpp
 *
 *  Created on: 2023/06/17
 *      Author: sato1
 */


#include "motion.h"
#include "run_task.h"
#include "index.h"
#include "typedef.h"

void motion_task::motion_inInterrupt(){
	switch(run_task)
	{
		case Search_st_section:
			break;
		case Search_st_half:
			break;
		case Long_turnR90:
			break;
		case motor_free:
				rT.MotionFree(&run_time);
				if(run_time_limit < run_time)
				{
					run_task = No_run;
				}
			break;
		default:
				Motor_SetDuty_Left(0);
				Motor_SetDuty_Right(0);
			break;
	}
}


void motion_task::motionControll()
{
	float Vr,Vl,motor_out_r,motor_out_l;
	Vr			= Vl			= 0.0;
	motor_out_r = motor_out_l	= 0.0;

	if(is_controll_enable() )
	{

	}

}

t_bool motion_task::is_controll_enable()
{
	t_bool is_controll_enable = False;
	switch(run_task)
	{
		case Straight 			:
		case Diagonal			:
		case Long_turnR90		:
		case Long_turnL90		:
		case Long_turnR180		:
		case Long_turnL180		:
		case Turn_in_R45		:
		case Turn_in_L45		:
		case Turn_out_R45		:
		case Turn_out_L45		:
		case Turn_in_R135		:
		case Turn_in_L135		:
		case Turn_out_R135		:
		case Turn_out_L135		:
		case Turn_RV90			:
		case Turn_LV90			:
		case Diagonal_R			:
		case Diagonal_L			:
		case Search_st_section	:
		case Search_st_half		:
		case Pivot_turn_R		:
		case Pivot_turn_L		:
		case run_brake			:
		case motor_free		:
			is_controll_enable = True;
			break;
		default:
			is_controll_enable = False;
			break;
	}
	return is_controll_enable;
}
