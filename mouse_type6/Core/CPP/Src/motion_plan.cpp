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
	t_motion_param mt_set_;
	mt_set_.accel 			=  acc;
	mt_set_.deccel 			= -acc;
	mt_set_.max_velo 		=  max_sp;
	mt_set_.length      	=  len_target;
	mt_set_.rad_accel   	=  0.0f;
	mt_set_.rad_deccel  	=  0.0f;
	mt_set_.rad_max_velo    =  0.0f;
	mt_set_.radian      	=  0.0f;
	mt_set_.turn_d          =  Turn_None;
	move_task->mt_set 		= mt_set_;
	move_task->run_task 	= Search_st_section;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->target.length = 0.0;
}

void motion_plan::pivot_turn(motion_task *move_task,float rad_target,float rad_acc,float rad_velo)
{
	t_motion_param mt_set_;
	mt_set_.accel 			=  0.0f;
	mt_set_.deccel 			=  0.0f;
	mt_set_.max_velo 		=  0.0f;
	mt_set_.length      	=  0.0f;
	mt_set_.rad_accel   	=  rad_acc;
	mt_set_.rad_deccel  	=  -rad_acc;
	mt_set_.rad_max_velo    =  rad_velo;
	mt_set_.radian      	=  rad_target;
	mt_set_.turn_d          =  (rad_target >= 0) ? Turn_L:Turn_R;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = (rad_target >= 0) ? Pivot_turn_L:Pivot_turn_R;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
}
