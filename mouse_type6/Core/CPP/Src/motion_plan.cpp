/*
 * motion.cpp
 *
 *  Created on: 2023/06/15
 *      Author: sato1
 */


#include "motion.h"
#include "../../Module/Include/index.h"
#include "controll.h"
#include "sensing_task.h"

void motion_plan::motion_start()
{
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;

	move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	move_task->target.rad_velo = 0.0;
	move_task->target.rad_accel = 0.0;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.reset_brake_time();
}

void motion_plan::free_rotation()
{
	move_task->run_time_limit = 1000.0;
	move_task->run_time = 0.0;
	move_task->run_task = motor_free;
}

void motion_plan::search_straight(float len_target,float acc,float max_sp,float end_sp)
{
	t_motion_param mt_set_;
	mt_set_.accel 			=  acc;
	mt_set_.deccel 			= -acc;
	mt_set_.max_velo 		=  max_sp;
	mt_set_.end_velo        =  end_sp;
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
	move_task->mouse.radian  = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.reset_brake_time();
	move_task->_turn_param = nullptr;
	move_task->rT.is_wallControl_Enable = Non_controll;
	SensingTask::getInstance().Division_Wall_Correction_Reset();
}

void motion_plan::straight(float len_target,float acc,float max_sp,float end_sp)
{
	t_motion_param mt_set_;
	mt_set_.accel 			=  acc;
	mt_set_.deccel 			= -acc;
	mt_set_.max_velo 		=  max_sp;
	mt_set_.end_velo        =  end_sp;
	mt_set_.length      	=  len_target;
	mt_set_.rad_accel   	=  0.0f;
	mt_set_.rad_deccel  	=  0.0f;
	mt_set_.rad_max_velo    =  0.0f;
	mt_set_.radian      	=  0.0f;

	mt_set_.turn_d          =  Turn_None;
	move_task->mt_set 		= mt_set_;
	move_task->run_task 	= Straight;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.reset_brake_time();
	move_task->_turn_param = nullptr;
	move_task->rT.is_wallControl_Enable = Non_controll;
	SensingTask::getInstance().Division_Wall_Correction_Reset();
}


void motion_plan::diagonal(float len_target,float acc,float max_sp,float end_sp)
{
	t_motion_param mt_set_;
	mt_set_.accel 			=  acc;
	mt_set_.deccel 			= -acc;
	mt_set_.max_velo 		=  max_sp;
	mt_set_.end_velo        =  end_sp;
	mt_set_.length      	=  len_target;
	mt_set_.rad_accel   	=  0.0f;
	mt_set_.rad_deccel  	=  0.0f;
	mt_set_.rad_max_velo    =  0.0f;
	mt_set_.radian      	=  0.0f;

	mt_set_.turn_d          =  Turn_None;
	move_task->mt_set 		= mt_set_;
	move_task->run_task 	= Diagonal;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.reset_brake_time();
	move_task->_turn_param = nullptr;
	move_task->rT.is_wallControl_Enable = Non_controll;
	SensingTask::getInstance().Division_Wall_Correction_Reset();
}

void motion_plan::pivot_turn(float rad_target,float rad_acc,float rad_velo)
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
	move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	move_task->target.rad_velo = 0.0;
	move_task->target.rad_accel = 0.0;
	move_task->_turn_param = nullptr;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.is_wallControl_Enable = Non_controll;
	SensingTask::getInstance().Division_Wall_Correction_Reset();

}

void motion_plan::searchSlalom(const t_param *turn_param)
{
	t_motion_param mt_set_;
	mt_set_.accel 			=  0.0f;
	mt_set_.deccel 			=  0.0f;
	mt_set_.max_velo 		=  turn_param->param->velo;
	mt_set_.rad_accel   	=  0.0;
	mt_set_.rad_deccel  	=  0.0;
	mt_set_.rad_max_velo    =  turn_param->param->velo/turn_param->param->r_min*1000.0f;
	mt_set_.radian      	=  0.0;//DEG2RAD(turn_param->param->degree);
	mt_set_.turn_d          =  Turn_None;//(turn_param->param->turn_dir == Turn_L) ? Turn_L:Turn_R;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = (turn_param->param->turn_dir == Turn_L) ? Search_slalom_L:Search_slalom_R;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();

	move_task->ct.omega_ctrl.Kp = turn_param->om_gain->Kp;
	move_task->ct.omega_ctrl.Ki = turn_param->om_gain->Ki;
	move_task->ct.omega_ctrl.Kd = turn_param->om_gain->Kd;

	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->run_time		 = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->_turn_param = turn_param;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.is_wallControl_Enable = Non_controll;
	SensingTask::getInstance().Division_Wall_Correction_Reset();
}

void motion_plan::turn_in(const t_param *turn_param,t_run_pattern run_pt)
{
	t_motion_param mt_set_;
	mt_set_.accel 			=  0.0f;
	mt_set_.deccel 			=  0.0f;
	mt_set_.max_velo 		=  turn_param->param->velo;
	mt_set_.rad_accel   	=  0.0;
	mt_set_.rad_deccel  	=  0.0;
	mt_set_.rad_max_velo    =  turn_param->param->velo/turn_param->param->r_min*1000.0f;
	mt_set_.radian      	=  0.0;//DEG2RAD(turn_param->param->degree);
	mt_set_.turn_d          =  Turn_None;//(turn_param->param->turn_dir == Turn_L) ? Turn_L:Turn_R;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = run_pt;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->run_time		 = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->_turn_param = turn_param;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.is_wallControl_Enable = Non_controll;
	SensingTask::getInstance().Division_Wall_Correction_Reset();
}
void motion_plan::turn_out(const t_param *turn_param,t_run_pattern run_pt)
{
	t_motion_param mt_set_;
	mt_set_.accel 			=  0.0f;
	mt_set_.deccel 			=  0.0f;
	mt_set_.max_velo 		=  turn_param->param->velo;
	mt_set_.rad_accel   	=  0.0;
	mt_set_.rad_deccel  	=  0.0;
	mt_set_.rad_max_velo    =  turn_param->param->velo/turn_param->param->r_min*1000.0f;
	mt_set_.radian      	=  0.0;//DEG2RAD(turn_param->param->degree);
	mt_set_.turn_d          =  Turn_None;//(turn_param->param->turn_dir == Turn_L) ? Turn_L:Turn_R;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = run_pt;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->run_time		 = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->_turn_param = turn_param;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.is_wallControl_Enable = Non_controll;
	SensingTask::getInstance().Division_Wall_Correction_Reset();
}
void motion_plan::long_turn(const t_param *turn_param,t_run_pattern run_pt)
{
	t_motion_param mt_set_;
	mt_set_.accel 			=  0.0f;
	mt_set_.deccel 			=  0.0f;
	mt_set_.max_velo 		=  turn_param->param->velo;
	mt_set_.rad_accel   	=  0.0;
	mt_set_.rad_deccel  	=  0.0;
	mt_set_.rad_max_velo    =  turn_param->param->velo/turn_param->param->r_min*1000.0f;
	mt_set_.radian      	=  0.0;//DEG2RAD(turn_param->param->degree);
	mt_set_.turn_d          =  Turn_None;//(turn_param->param->turn_dir == Turn_L) ? Turn_L:Turn_R;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = run_pt;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->run_time		 = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->_turn_param = turn_param;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.is_wallControl_Enable = Non_controll;
	SensingTask::getInstance().Division_Wall_Correction_Reset();
}
void motion_plan::turn_v90(const t_param *turn_param,t_run_pattern run_pt)
{
	t_motion_param mt_set_;
	mt_set_.accel 			=  0.0f;
	mt_set_.deccel 			=  0.0f;
	mt_set_.max_velo 		=  turn_param->param->velo;
	mt_set_.rad_accel   	=  0.0;
	mt_set_.rad_deccel  	=  0.0;
	mt_set_.rad_max_velo    =  turn_param->param->velo/turn_param->param->r_min*1000.0f;
	mt_set_.radian      	=  0.0;//DEG2RAD(turn_param->param->degree);
	mt_set_.turn_d          =  Turn_None;//(turn_param->param->turn_dir == Turn_L) ? Turn_L:Turn_R;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = run_pt;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->run_time		 = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->_turn_param = turn_param;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.is_wallControl_Enable = Non_controll;
	SensingTask::getInstance().Division_Wall_Correction_Reset();
}

void motion_plan::fix_wall(float set_time)
{
	t_motion_param mt_set_;
	mt_set_.accel 			=  0.0f;
	mt_set_.deccel 			=  0.0f;
	mt_set_.max_velo 		=  0.0f;
	mt_set_.rad_accel   	=  0.0;
	mt_set_.rad_deccel  	=  0.0;
	mt_set_.rad_max_velo    =  0.0f;
	mt_set_.radian      	=  0.0;
	mt_set_.turn_d          =  Turn_None;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = Fix_wall;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->run_time		 = 0.0;
	move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	move_task->target.rad_velo = 0.0;
	move_task->target.rad_accel = 0.0;
	move_task->_turn_param = nullptr;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->run_time_limit = set_time;
	move_task->run_time = 0.0;
	move_task->rT.is_wallControl_Enable = Non_controll;
	SensingTask::getInstance().Division_Wall_Correction_Reset();

}
