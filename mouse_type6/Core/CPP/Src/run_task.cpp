/*
 * run_task.cpp
 *
 *  Created on: 2023/06/16
 *      Author: sato1
 */

#include "index.h"
#include "typedef.h"
#include "run_task.h"
#include "macro.h"

#define BRAKE_TIME_LIMIT (500)

void RunTask::MotionFree(float *run_time,float run_time_limit)
{
	is_runTask = True;
	Motor_SetDuty_Right(500);
	Motor_SetDuty_Left(500);
	*run_time = *run_time + 1.0f;
	if(*run_time > run_time_limit)
	{
		is_runTask = False;
	}
}
void RunTask::search_straight(t_motion_param mt_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{

	is_runTask = True;
	float deccel_length = 1000*(mt_param.max_velo*mt_param.max_velo
								-mt_param.end_velo*mt_param.end_velo)
								/(2.0*ABS(mt_param.deccel));
	if(deccel_length < ( mt_param.length - machine_->length ))
	{
		target_->accel = mt_param.accel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;
		if(target_->velo > mt_param.max_velo)
		{
			target_->velo = mt_param.max_velo;
		}

	}
	else if(mt_param.length > machine_->length)
	{
		target_->accel = mt_param.deccel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;
		if(target_->velo < mt_param.end_velo)
		{
			target_->velo = mt_param.end_velo;

		}

	}
	else
	{
		if(mt_param.end_velo == 0.0f)target_->velo = 0.0f;
		else						is_runTask = False;
	}

	if(target_->velo == 0.0f)
	{
		is_runTask = True;
		brake_time++;
		if(brake_time > BRAKE_TIME_LIMIT)
		{
			is_runTask = False;
			brake_time = 0;
		}
	}

}

void RunTask::pivotturn(t_motion_param mt_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{

	is_runTask = True;
	target_->velo  = 0.0;
	target_ ->accel = 0.0;
	float deccel_radian = (mt_param.rad_max_velo*mt_param.rad_max_velo)/(2.0*ABS(mt_param.rad_accel));
	if(ABS(deccel_radian) < ( ABS(mt_param.radian) - ABS(machine_->radian)))
	{
		target_->rad_accel = mt_param.rad_accel;
		target_->rad_velo  = target_->rad_velo + target_->rad_accel*delta_t_ms/1000.0;
		if(ABS(target_->rad_velo) > ABS(mt_param.rad_max_velo))
		{
			target_->rad_velo = mt_param.rad_max_velo;
		}
	}

	else if(ABS(mt_param.radian) > ABS(machine_->radian))
	{
		target_->rad_accel = mt_param.rad_deccel;
		target_->rad_velo  = target_->rad_velo + target_->rad_accel*delta_t_ms/1000.0;
		if(target_->rad_velo <= 0.0 && mt_param.radian > 0.0)
		{
			target_->rad_velo = 0.0 ;
		}
		else if(target_->rad_velo >= 0.0 && mt_param.radian < 0.0)
		{
			target_->rad_velo = 0.0 ;
		}

	}
	else
	{
		target_->rad_velo = 0.0 ;
		//is_runTask = False;
	}



	if(target_->rad_velo == 0.0f)
	{
		is_runTask = True;
		brake_time++;
		if(brake_time > BRAKE_TIME_LIMIT)
		{
			is_runTask = False;
			brake_time = 0;
		}
	}

}



t_bool RunTask::is_exe_runTask()
{
	return is_runTask;
}
