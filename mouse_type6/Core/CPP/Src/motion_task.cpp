/*
 * motion_task.cpp
 *
 *  Created on: 2023/06/17
 *      Author: sato1
 */


#include "motion.h"
#include "run_task.h"
#include "../../Module/Include/index.h"
#include "../../Module/Include/typedef.h"
#include "sensing_task.h"

void motion_task::motion_inInterrupt(){
	switch(run_task)
	{
		case Search_st_section:
			rT.search_straight(mt_set, &target, &mouse, 1.0);
			//SensingTask::getInstance().SetWallControll_RadVelo(&target,1.0);
			break;
		case Straight:
			rT.straight(mt_set, &target, &mouse, 1.0);
			//SensingTask::getInstance().SetWallControll_RadVelo(&target,1.0);
			break;
		case Diagonal:
			rT.diagonal(mt_set, &target, &mouse, 1.0);
			break;
		case Search_st_half:
			break;
		case Pivot_turn_L:
		case Pivot_turn_R:
			rT.pivotturn(mt_set,&target, &mouse, 1.0);
			break;
		case Search_slalom_L:
		case Search_slalom_R:
			rT.search_slalom(&mt_set,_turn_param,&target, &mouse, 1.0);
			break;
		case Long_turnR180:
		case Long_turnL180:
		case Long_turnR90:
		case Long_turnL90:
			rT.long_turn(&mt_set,_turn_param,&target, &mouse, 1.0);
			break;
		case Turn_in_R45:
		case Turn_in_L45:
		case Turn_in_R135:
		case Turn_in_L135:
			rT.turn_in(&mt_set,_turn_param,&target, &mouse, 1.0);
			break;
		case Turn_out_R45:
		case Turn_out_L45:
		case Turn_out_R135:
		case Turn_out_L135:
			rT.turn_out(&mt_set,_turn_param,&target, &mouse, 1.0);
			break;
		case Turn_LV90:
		case Turn_RV90:
			rT.turn_v90(&mt_set,_turn_param,&target, &mouse, 1.0);
			break;
		case motor_free:
				rT.MotionFree(&run_time,run_time_limit);
			break;
		case Fix_wall:
				rT.fix_wall(&target, &run_time, run_time_limit, 1.0);
				break;
		default:
				target.accel = 0.0;
				target.velo  = 0.0;
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
		float motor_r_rpm = (1.0f)*RAD_2_RPM*GEAR_N*(target.velo*1000/TIRE_RADIUS + 1.0f*TREAD_WIDTH*target.rad_velo/(2*TIRE_RADIUS));
		float motor_l_rpm = (1.0f)*RAD_2_RPM*GEAR_N*(target.velo*1000/TIRE_RADIUS - 1.0f*TREAD_WIDTH*target.rad_velo/(2*TIRE_RADIUS));
		float motor_r_ampere = 1/(MOTOR_K_TR*GEAR_N)*(WEIGHT*target.accel/1000*TIRE_RADIUS/2) + MOTOR_BR*motor_r_rpm/MOTOR_K_TR;
		float motor_l_ampere = 1/(MOTOR_K_TR*GEAR_N)*(WEIGHT*target.accel/1000*TIRE_RADIUS/2) + MOTOR_BR*motor_l_rpm/MOTOR_K_TR;
		float sp_FF_controll_r =  MOTOR_R*motor_r_ampere + MOTOR_K_ER*motor_r_rpm/1000;
		float sp_FF_controll_l =  MOTOR_R*motor_l_ampere + MOTOR_K_ER*motor_l_rpm/1000;

		float sp_fb_controll = ct.speed_ctrl.Controll(target.velo, mouse.velo, 1.0);
		float om_fb_controll = ct.omega_ctrl.Controll(target.rad_velo, mouse.rad_velo,  1.0);

		//printf("initerrupt%lf\n",sp_fb_controll);
		V_r += sp_FF_controll_r;
		V_l -= sp_FF_controll_l;

		V_r += sp_fb_controll;
		V_l -= sp_fb_controll;

		V_r += om_fb_controll;
		V_l += om_fb_controll;


		float battery = Battery_GetVoltage();
		if(battery < 3.30f) battery = 3.30f;

		float duty_r = V_r/battery;
		float duty_l = V_l/battery;

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

	z_acc = 0.8*z_acc + 0.2*read_accel_z_axis();

	if(is_controll_enable() == True && run_task != Fix_wall)
	{
		if(target.velo != 0.0f)
		{
			if(ABS((mouse.velo - target.velo)/target.velo) >= 0.2)
			{
				//error_cnt = error_cnt + 10;
			}
			else if(ABS(z_acc) >= 30.0)
			{
				//error_cnt = error_cnt +	100;
			}
			else
			{
				error_cnt = error_cnt - 2;
				if(error_cnt < 0) error_cnt = 0;
			}
		}
		else
		{
			error_cnt = 0;
		}
		if(error_cnt >= 1000)
		{
			//Motor_Stop();FAN_Motor_Stop();
			//NVIC_SystemReset();
		}
	}
	else
	{
		error_cnt = 0;
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
		case Search_slalom_L	:
		case Search_slalom_R	:
		case run_brake			:
		case motor_free			:
		case Fix_wall			:
			is_controll_enable = True;
			break;
		default:
			is_controll_enable = False;
			break;
	}
	return is_controll_enable;
}
