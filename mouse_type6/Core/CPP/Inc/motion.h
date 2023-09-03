/*
 * motion.h
 *
 *  Created on: Jun 12, 2023
 *      Author: sato1
 */

#ifndef CPP_INC_MOTION_H_
#define CPP_INC_MOTION_H_

#include <stdio.h>
#include "../../Module/Include/typedef.h"
#include "singleton.h"
#include "run_task.h"
#include "controll.h"


class motion_task:public Singleton<motion_task>
{
	private:
		int error_cnt;
		float z_acc;
	public:
		float V_r,V_l;
		int motor_out_r,motor_out_l;
		float ff_turn = 0.0f;
		float ff_st = 0.0f;
		float delta_t;
		float run_time;
		float run_time_limit;
		t_run_pattern	run_task = No_run;
		RunTask rT;
		PID_Setting straight_gain_set;
		PID_Setting turn_gain_set;
		mouse_Controll ct;
		t_machine_param target,mouse;
		t_motion_param mt_set;
		const t_param *_turn_param = nullptr;
		void motion_inInterrupt();
		void motionControll();
		void motionPostControll();
		t_bool is_controll_enable();

};

class motion_plan
{
	private:
		//float motion_time;
	public:
		//void straight(motion_task *move_task);
		motion_task *move_task;
		motion_plan(motion_task *_move_task)
		{
			move_task = _move_task;
		}
		void motion_start();
		void free_rotation();
		void search_straight(float len_target,float acc,float max_sp,float end_sp);
		void straight(float len_target,float acc,float max_sp,float end_sp);
		void diagonal(float len_target,float acc,float max_sp,float end_sp);
		void pivot_turn(float rad_target,float rad_acc,float rad_velo);
		void searchSlalom(const t_param *turn_param);
		void turn_in(const t_param *turn_param,t_run_pattern run_pt);
		void turn_out(const t_param *turn_param,t_run_pattern run_pt);
		void long_turn(const t_param *turn_param,t_run_pattern run_pt);
		void turn_v90(const t_param *turn_param,t_run_pattern run_pt);
		void fix_wall(float set_time);
		void stop_brake();
};


#endif /* CPP_INC_MOTION_H_ */
