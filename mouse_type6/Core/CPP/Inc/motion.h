/*
 * motion.h
 *
 *  Created on: Jun 12, 2023
 *      Author: sato1
 */

#ifndef CPP_INC_MOTION_H_
#define CPP_INC_MOTION_H_

#include <stdio.h>
#include "typedef.h"
#include "singleton.h"
#include "run_task.h"
#include "controll.h"


class motion_task:public Singleton<motion_task>
{
	public:
		float delta_t;
		float run_time;
		float run_time_limit;
		t_run_pattern	run_task = No_run;
		RunTask rT;
		mouse_Controll ct;
		t_machine_param target,max_set,mouse;
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
		void free_rotation(motion_task *move_task);
		void search_straight(motion_task *move_task,float len_target,float acc,float max_sp,float end_sp);
		void pivot_turn(motion_task *move_task,float rad_target,float rad_acc,float rad_velo);
		void search_slalom(motion_task *move_task);
		void stop_brake();
};


#endif /* CPP_INC_MOTION_H_ */
