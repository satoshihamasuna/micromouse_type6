/*
 * motion.cpp
 *
 *  Created on: 2023/06/15
 *      Author: sato1
 */


#include "motion.h"
#include "index.h"
#include "controll.h"

void motion_plan::free_rotation(motion_task *move_task)
{
	move_task->run_time_limit = 1000.0;
	move_task->run_time = 0.0;
	move_task->run_task = motor_free;
}

void motion_plan::search_straight(motion_task *move_task,float len_target,float acc,float max_sp,float end_sp)
{
	t_straight_param st_set_;
	st_set_.accel 		=  acc;
	st_set_.deccel 		= -acc;
	st_set_.max_velo 	=  max_sp;
	st_set_.length      =  len_target;
	st_set_.rad_accel   =  0.0f;
	st_set_.rad_velo    =  0.0f;
	st_set_.radian      =  0.0f;
	move_task->st_set 	= st_set_;
	move_task->run_task = Search_st_section;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->target.length = 0.0;
}

