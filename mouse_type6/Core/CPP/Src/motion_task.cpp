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
			rT.search_straight(st_set, &target, &mouse, 1.0);
			break;
		case Search_st_half:
			break;
		case Long_turnR90:
			break;
		case motor_free:
				rT.MotionFree(&run_time,run_time_limit);
			break;
		default:
				Motor_SetDuty_Left(0);
				Motor_SetDuty_Right(0);
			break;
	}
}


void motion_task::motionControll()
{
	float V_r,V_l;
	int motor_out_r,motor_out_l;
	V_r			= V_l			= 0.0;
	motor_out_r = motor_out_l	= 0;

	if(is_controll_enable() == True && run_task != motor_free)
	{
		float sp_fb_controll = ct.speed_ctrl.Controll(target.velo, mouse.velo, 1.0);
		float om_fb_controll = ct.omega_ctrl.Controll(target.rad_velo, mouse.rad_velo,  1.0);
		printf("initerrupt%lf\n",sp_fb_controll);
		V_r += sp_fb_controll;
		V_l -= sp_fb_controll;

		//V_r += om_fb_controll;
		//V_l += om_fb_controll;



		float duty_r = V_r/4.0;
		float duty_l = V_l/4.0;

		if(ABS(duty_r) > 1.0){
			motor_out_r = (int)(SIGN(duty_r) * 4.0f * 250.0f);
		}else{
			motor_out_r = (int)(duty_r * 1000.0f);
		}
		Motor_SetDuty_Right(motor_out_r);

		if(ABS(duty_l) > 1.0){
			motor_out_l = (int)(SIGN(duty_l) * 4.0f * 250.0f);
		}else{
			motor_out_l = (int)(duty_l * 1000.0f);
		}

		Motor_SetDuty_Left(motor_out_l);
	}
	else if(run_task == motor_free)
	{

	}
	else
	{
		Motor_SetDuty_Left(0);
		Motor_SetDuty_Right(0);
	}

}

void motion_task::motionPostControll()
{
	if(rT.is_exe_runTask() == False)
	{
		run_task = No_run;
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
