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
#include "run_task.h"
#include "turn_table.h"

void motion_plan::motion_start()
{
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->mouse.x_point = 0.0;

	move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	move_task->target.rad_velo = 0.0;
	move_task->target.rad_accel = 0.0;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.reset_brake_time();
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
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
	move_task->mouse.radian  = 0.0;move_task->mouse.x_point = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	move_task->mouse.x_point = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.reset_brake_time();
	move_task->_turn_param = nullptr;
	move_task->rT.is_wallControl_Enable = Non_controll;
	move_task->straight_gain_set.set_sp_gain(6.0, 0.05, 0.0);
	move_task->straight_gain_set.set_om_gain(0.2, 0.01, 0.00);
	move_task->turn_gain_set.set_sp_gain(6.0, 0.05, 0.0);
	move_task->turn_gain_set.set_om_gain(0.4, 0.05, 0.00);
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
	move_task->mouse.x_point = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.reset_brake_time();
	move_task->_turn_param = nullptr;
	move_task->rT.is_wallControl_Enable = Non_controll;
	move_task->straight_gain_set.set_sp_gain(6.0, 0.05, 0.0);
	move_task->straight_gain_set.set_om_gain(0.2, 0.01, 0.00);
	move_task->turn_gain_set.set_sp_gain(6.0, 0.05, 0.0);
	move_task->turn_gain_set.set_om_gain(0.4, 0.05, 0.00);
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
	move_task->mouse.x_point = 0.0;
	mt_set_.turn_d          =  Turn_None;
	move_task->mt_set 		= mt_set_;
	move_task->run_task 	= Diagonal;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->mouse.x_point = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.reset_brake_time();
	move_task->_turn_param = nullptr;
	move_task->rT.is_wallControl_Enable = Non_controll;
	move_task->straight_gain_set.set_sp_gain(6.0, 0.05, 0.0);
	move_task->straight_gain_set.set_om_gain(0.2, 0.01, 0.00);
	move_task->turn_gain_set.set_sp_gain(6.0, 0.05, 0.0);
	move_task->turn_gain_set.set_om_gain(0.4, 0.05, 0.00);
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
	move_task->mouse.x_point = 0.0;
	move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	move_task->target.rad_velo = 0.0;
	move_task->target.rad_accel = 0.0;
	move_task->_turn_param = nullptr;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.is_wallControl_Enable = Non_controll;
	move_task->straight_gain_set.set_sp_gain( 6.0, 0.05, 0.00);
	move_task->straight_gain_set.set_om_gain(0.4, 0.05, 0.00);
	move_task->turn_gain_set.set_sp_gain(6.0, 0.05, 0.00);
	move_task->turn_gain_set.set_om_gain(0.4, 0.05, 0.00);move_task->mouse.x_point = 0.0;
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
	mt_set_.turn_d          =  Prev_Turn;//(turn_param->param->turn_dir == Turn_L) ? Turn_L:Turn_R;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = (turn_param->param->turn_dir == Turn_L) ? Search_slalom_L:Search_slalom_R;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.x_point = 0.0;
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
	if(turn_param->param->turn_dir == Turn_L)
	{
		if(SensingTask::getInstance().sen_r.is_wall == True)
			move_task->rT. post_run_fix = 45.0 -SensingTask::getInstance().sen_r.distance;
		else
			move_task->rT.post_run_fix = 0.0;
	}
	else if(turn_param->param->turn_dir == Turn_R)
	{
		if(SensingTask::getInstance().sen_l.is_wall == True)
			move_task->rT.post_run_fix = 45.0 -SensingTask::getInstance().sen_l.distance;
		else
			move_task->rT.post_run_fix = 0.0;
	}
	move_task->straight_gain_set.set_sp_gain(6.0,  0.01, 0.0001);
	move_task->straight_gain_set.set_om_gain(0.2, 0.01, 0.00);
	move_task->turn_gain_set.set_sp_gain(6.0, 0.05, 0.0);
	move_task->turn_gain_set.set_om_gain(turn_param->om_gain->Kp, turn_param->om_gain->Ki, turn_param->om_gain->Kd);
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
	//mt_set_.turn_d          =  Turn_None;//(turn_param->param->turn_dir == Turn_L) ? Turn_L:Turn_R;
	mt_set_.turn_d          =  Prev_Turn;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = run_pt;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->mouse.x_point = 0.0;
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
	move_task->straight_gain_set.set_sp_gain(turn_param->sp_gain->Kp, turn_param->sp_gain->Ki, turn_param->sp_gain->Kd);
	//move_task->straight_gain_set.set_om_gain(0.05, 0.01, 0.00);
	move_task->straight_gain_set.set_om_gain(0.2, 0.01, 0.00);
	move_task->turn_gain_set.set_sp_gain(turn_param->sp_gain->Kp, turn_param->sp_gain->Ki, turn_param->sp_gain->Kd);
	move_task->turn_gain_set.set_om_gain(turn_param->om_gain->Kp, turn_param->om_gain->Ki, turn_param->om_gain->Kd);
	move_task->rT.turn_rmin_fix = 0.0f;
	move_task->rT.post_run_fix  = 0.0f;
	move_task->rT.prev_run_fix  = 0.0f;

	t_bool r_fix,l_fix;
	r_fix = l_fix = False;
	if(SensingTask::getInstance().sen_r.is_wall == True && ABS(45.0 - SensingTask::getInstance().sen_r.distance) <= 10.0)
	{
		r_fix = True;
	}
	if(SensingTask::getInstance().sen_l.is_wall == True && ABS(45.0 - SensingTask::getInstance().sen_l.distance) <= 10.0)
	{
		l_fix = True;
	}
	float diff = 0.0;

	if(ABS(turn_param->param->degree) == 45.0f)
	{
		if(turn_param->param->turn_dir == Turn_R)
		{
			if(l_fix == True)
			{
				diff =((-1.0)*(45.0 - SensingTask::getInstance().sen_l.distance));
			}
			else if(r_fix == True)
			{
				diff =((45.0 - SensingTask::getInstance().sen_r.distance));
			}
			move_task->rT.prev_run_fix = (1.0)*diff;
			move_task->rT.post_run_fix = (-1.0)*SQRT2*diff;
		}
		else if(turn_param->param->turn_dir == Turn_L)
		{
			if(r_fix == True)
			{
				diff =((45.0 - SensingTask::getInstance().sen_r.distance));
			}
			else if(l_fix == True)
			{
				diff =((-1.0)*(45.0 - SensingTask::getInstance().sen_l.distance));
			}
			move_task->rT.prev_run_fix = (-1.0)*diff;
			move_task->rT.post_run_fix = (1.0)*SQRT2*diff;
		}
	}

	if(ABS(turn_param->param->degree) == 135.0f)
	{
		if(turn_param->param->turn_dir == Turn_R)
		{
			if(l_fix == True)
			{
				diff =((-1.0)*(45.0 - SensingTask::getInstance().sen_l.distance));
			}
			else if(r_fix == True)
			{
				diff =((45.0 - SensingTask::getInstance().sen_r.distance));
			}
			move_task->rT.prev_run_fix = (-1.0)*diff;
			move_task->rT.post_run_fix = (-1.0)*SQRT2*diff;
		}
		else if(turn_param->param->turn_dir == Turn_L)
		{
			if(r_fix == True)
			{
				diff =((45.0 - SensingTask::getInstance().sen_r.distance));
			}
			else if(l_fix == True)
			{
				diff =((-1.0)*(45.0 - SensingTask::getInstance().sen_l.distance));
			}
			move_task->rT.prev_run_fix = (1.0)*diff;
			move_task->rT.post_run_fix = (1.0)*SQRT2*diff;
		}
	}

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
	//mt_set_.turn_d          =  Turn_None;//(turn_param->param->turn_dir == Turn_L) ? Turn_L:Turn_R;
	mt_set_.turn_d          =  Prev_Turn;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = run_pt;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->mouse.x_point = 0.0;
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
	move_task->straight_gain_set.set_sp_gain(turn_param->sp_gain->Kp, turn_param->sp_gain->Ki, turn_param->sp_gain->Kd);
	//move_task->straight_gain_set.set_om_gain(0.05, 0.01, 0.00);
	move_task->straight_gain_set.set_om_gain(0.2, 0.01, 0.00);
	move_task->turn_gain_set.set_sp_gain(turn_param->sp_gain->Kp, turn_param->sp_gain->Ki, turn_param->sp_gain->Kd);
	move_task->turn_gain_set.set_om_gain(turn_param->om_gain->Kp, turn_param->om_gain->Ki, turn_param->om_gain->Kd);
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
	//mt_set_.turn_d          =  Turn_None;//(turn_param->param->turn_dir == Turn_L) ? Turn_L:Turn_R;
	mt_set_.turn_d          =  Prev_Turn;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = run_pt;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->mouse.x_point = 0.0;
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
	move_task->straight_gain_set.set_sp_gain(turn_param->sp_gain->Kp, turn_param->sp_gain->Ki, turn_param->sp_gain->Kd);
	//move_task->straight_gain_set.set_om_gain(0.05, 0.01, 0.00);
	move_task->straight_gain_set.set_om_gain(0.2, 0.01, 0.00);
	move_task->turn_gain_set.set_sp_gain(turn_param->sp_gain->Kp, turn_param->sp_gain->Ki, turn_param->sp_gain->Kd);
	move_task->turn_gain_set.set_om_gain(turn_param->om_gain->Kp, turn_param->om_gain->Ki, turn_param->om_gain->Kd);
	move_task->rT.turn_rmin_fix = 0.0f;
	move_task->rT.post_run_fix  = 0.0f;
	move_task->rT.prev_run_fix  = 0.0f;


	if(ABS(turn_param->param->degree) == 90.0f)
	{
		t_bool r_fix,l_fix;
		r_fix = l_fix = False;
		if(SensingTask::getInstance().sen_r.is_wall == True && ABS(45.0 - SensingTask::getInstance().sen_r.distance) <= 10.0)
		{
			r_fix = True;
		}
		if(SensingTask::getInstance().sen_l.is_wall == True && ABS(45.0 - SensingTask::getInstance().sen_l.distance) <= 10.0)
		{
			l_fix = True;
		}
		float diff = 0.0;

		if(turn_param->param->turn_dir == Turn_R)
		{
			if(l_fix == True)
			{
				diff =((-1.0)*(45.0 - SensingTask::getInstance().sen_l.distance));
			}
			else if(r_fix == True)
			{
				diff =((45.0 - SensingTask::getInstance().sen_r.distance));
			}
			move_task->rT.post_run_fix = (-1.0)*diff;
		}
		else if(turn_param->param->turn_dir == Turn_L)
		{
			if(r_fix == True)
			{
				diff =((45.0 - SensingTask::getInstance().sen_r.distance));
			}
			else if(l_fix == True)
			{
				diff =((-1.0)*(45.0 - SensingTask::getInstance().sen_l.distance));
			}
			move_task->rT.post_run_fix = (1.0)*diff;
		}
	}

	if(ABS(turn_param->param->degree) == 180.0f)
	{
		float diff = 0.0;
		float fix_th = 10.0;
		if(turn_param->param->Lstart < 10.0) fix_th = turn_param->param->Lstart;
		if(turn_param->param->turn_dir == Turn_L)
		{
			if(SensingTask::getInstance().sen_l.is_wall == True && ABS(45.0 - SensingTask::getInstance().sen_l.distance) < fix_th)
			{
				move_task->rT.turn_rmin_fix = (-1.0)*PI/accel_Integral/10.0f*(45.0 - SensingTask::getInstance().sen_l.distance);
				mt_set_.rad_max_velo    =  turn_param->param->velo/(turn_param->param->r_min+move_task->rT.turn_rmin_fix)*1000.0f;
				diff = (45.0 - SensingTask::getInstance().sen_l.distance);
			}
			else if(SensingTask::getInstance().sen_r.is_wall == True && ABS(45.0 - SensingTask::getInstance().sen_r.distance) < fix_th)
			{
				move_task->rT.turn_rmin_fix = (1.0)*PI/accel_Integral/10.0f*(45.0 - SensingTask::getInstance().sen_r.distance);
				mt_set_.rad_max_velo    =  turn_param->param->velo/(turn_param->param->r_min+move_task->rT.turn_rmin_fix)*1000.0f;
				diff = (45.0 - SensingTask::getInstance().sen_r.distance);
			}
			else
			{
				move_task->rT.post_run_fix = 0.0;
			}
		}
		else if(turn_param->param->turn_dir == Turn_R)
		{
			if(SensingTask::getInstance().sen_r.is_wall == True && ABS(45.0 - SensingTask::getInstance().sen_r.distance) < fix_th)
			{
				move_task->rT.turn_rmin_fix = (-1.0)*PI/accel_Integral/10.0f*(45.0 - SensingTask::getInstance().sen_r.distance);
				mt_set_.rad_max_velo    =  turn_param->param->velo/(turn_param->param->r_min+move_task->rT.turn_rmin_fix)*1000.0f;
				diff = (45.0 - SensingTask::getInstance().sen_r.distance);
			}
			else if(SensingTask::getInstance().sen_l.is_wall == True && ABS(45.0 - SensingTask::getInstance().sen_l.distance) < fix_th)
			{
				move_task->rT.turn_rmin_fix = (1.0)*PI/accel_Integral/10.0f*(45.0 - SensingTask::getInstance().sen_l.distance);
				mt_set_.rad_max_velo    =  turn_param->param->velo/(turn_param->param->r_min+move_task->rT.turn_rmin_fix)*1000.0f;
				diff = (45.0 - SensingTask::getInstance().sen_l.distance);
			}
			else
			{
				move_task->rT.post_run_fix = 0.0;
			}
		}

		if(move_task->rT.turn_rmin_fix > 0.0)
		{
			move_task->rT.post_run_fix  = (-1.0f)*ABS(diff);
			move_task->rT.prev_run_fix  = (-1.0f)*ABS(diff);
		}
		else if(move_task->rT.turn_rmin_fix < 0.0)
		{
			move_task->rT.post_run_fix  = (1.0f)*ABS(diff);
			move_task->rT.prev_run_fix  = (1.0f)*ABS(diff);
		}
	}

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
	//mt_set_.turn_d          =  Turn_None;//(turn_param->param->turn_dir == Turn_L) ? Turn_L:Turn_R;
	mt_set_.turn_d          =  Prev_Turn;
	move_task->mt_set 		=  mt_set_;
	move_task->run_task = run_pt;
	move_task->ct.speed_ctrl.I_param_reset();
	move_task->ct.omega_ctrl.I_param_reset();
	move_task->mouse.length  = 0.0;
	move_task->mouse.radian  = 0.0;
	move_task->mouse.x_point = 0.0;
	move_task->run_time		 = 0.0;
	//move_task->target.velo = 0.0;
	move_task->target.accel = 0.0;
	//move_task->target.rad_velo = 0.0;
	//move_task->target.rad_accel = 0.0;
	move_task->_turn_param = turn_param;
	move_task->target.length = 0.0;
	move_task->target.radian = 0.0;
	move_task->rT.is_wallControl_Enable = Non_controll;
	//move_task->ct.omega_ctrl.Kp = turn_param->om_gain->Kp;
	//move_task->ct.omega_ctrl.Ki = turn_param->om_gain->Ki;
	//move_task->ct.omega_ctrl.Kd = turn_param->om_gain->Kd;
	SensingTask::getInstance().Division_Wall_Correction_Reset();
	move_task->straight_gain_set.set_sp_gain(turn_param->sp_gain->Kp, turn_param->sp_gain->Ki, turn_param->sp_gain->Kd);
	//move_task->straight_gain_set.set_om_gain(0.05, 0.01, 0.00);
	move_task->straight_gain_set.set_om_gain(0.2, 0.01, 0.00);
	move_task->turn_gain_set.set_sp_gain(turn_param->sp_gain->Kp, turn_param->sp_gain->Ki, turn_param->sp_gain->Kd);
	move_task->turn_gain_set.set_om_gain(turn_param->om_gain->Kp, turn_param->om_gain->Ki, turn_param->om_gain->Kd);
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
	move_task->mouse.x_point = 0.0;
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
	move_task->straight_gain_set.set_sp_gain(6.0, 0.01, 0.0);
	move_task->straight_gain_set.set_om_gain(0.05, 0.01, 0.00);
	move_task->turn_gain_set.set_sp_gain(6.0, 0.05, 0.0);
	move_task->turn_gain_set.set_om_gain(0.4, 0.05, 0.00);
}
