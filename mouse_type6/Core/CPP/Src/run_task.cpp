/*
 * run_task.cpp
 *
 *  Created on: 2023/06/16
 *      Author: sato1
 */

#include "index.h"
#include "typedef.h"
#include "run_task.h"

#define BRAKE_TIME_LIMIT (300)

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
void RunTask::search_straight(t_straight_param st_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{

	is_runTask = True;
	float deccel_length = 1000*(st_param.max_velo*st_param.max_velo-st_param.end_velo*st_param.end_velo)/(2.0*ABS(st_param.deccel));
	if(deccel_length < ( st_param.length - machine_->length ))
	{
		target_->accel = st_param.accel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;
		if(target_->velo > st_param.max_velo)
		{
			target_->velo = st_param.max_velo;
		}

	}
	else if(st_param.length < machine_->length)
	{
		target_->accel = st_param.deccel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;
		if(target_->velo < st_param.end_velo)
		{
			target_->velo = st_param.end_velo;
		}

	}
	else
	{
		is_runTask = False;
	}

	if(target_->velo == 0.0f)
	{
		is_runTask = True;
		brake_time++;
		if(brake_time < BRAKE_TIME_LIMIT)
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
