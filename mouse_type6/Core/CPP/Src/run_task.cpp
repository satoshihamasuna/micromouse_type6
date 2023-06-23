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
#include "turn_table.h"

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
			target_->accel = 0.0;
		}


	}
	else if(mt_param.length > machine_->length)
	{
		target_->accel = mt_param.deccel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;
		if(target_->velo < mt_param.end_velo)
		{
			target_->velo = mt_param.end_velo;
			target_->accel = 0.0;

		}

	}
	else
	{
		if(mt_param.end_velo == 0.0f)
		{
			target_->velo = 0.0f;
			target_->accel = 0.0;
		}
		else
		{
			is_runTask = False;
			target_->accel = 0.0;
		}
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
	target_->radian = target_->radian + target_->rad_velo*delta_t_ms/1000.0f;


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

void RunTask::search_slalom(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{
	is_runTask = True;
	target_->velo = turn_param->param->velo;
	if(mt_param->radian ==  0.0 && mt_param->turn_d == Turn_None)
	{
		run_turn_table_time = 0.0f;
		if(machine_->length < turn_param->param->Lstart)
		{
			target_->velo = turn_param->param->velo;
		}
		else
		{
			mt_param->turn_d =  turn_param->param->turn_dir;
		}
	}

	if(mt_param->radian ==  DEG2RAD(turn_param->param->degree) && mt_param->turn_d == Turn_None)
	{
		run_turn_table_time = 0.0f;
		if(machine_->length < turn_param->param->Lend)
		{
			target_->velo = turn_param->param->velo;
		}
		else
		{
			is_runTask = False;
		}
	}

	if( mt_param->turn_d ==  turn_param->param->turn_dir)
	{
		float turn_time_limit = DEG2RAD(turn_param->param->degree)/(accel_Integral*turn_param->param->velo);
		machine_->length = 0.0;
		if(run_turn_table_time <= turn_time_limit)
		{
			int std_a = (int)(run_turn_table_time/turn_time_limit);
			int std_b = std_a + 1;
			float m = run_turn_table_time/turn_time_limit - (float)(std_a);
			float n = (float)(std_b) - run_turn_table_time/turn_time_limit;
			float set_rad_velo =  mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);
			target_->rad_accel = (set_rad_velo - target_->rad_velo)*1000.0f/delta_t_ms;
			target_->rad_velo = set_rad_velo;
			run_turn_table_time = run_turn_table_time + delta_t_ms;
		}
		if(run_turn_table_time > turn_time_limit)
		{
			mt_param->radian =  DEG2RAD(turn_param->param->degree);
			mt_param->turn_d = Turn_None;
			machine_->length = 0.0;
			target_->rad_velo = 0.0f;
			target_->rad_accel = 0.0f;

		}
	}



}

t_bool RunTask::is_exe_runTask()
{
	return is_runTask;
}
