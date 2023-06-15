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



typedef enum{
	No_run				= 0,
	Straight 			= 1,
	Diagonal			= 2,
	Long_turnR90		= 3,
	Long_turnL90		= 4,
	Long_turnR180		= 5,
	Long_turnL180		= 6,
	Turn_in_R45			= 7,
	Turn_in_L45			= 8,
	Turn_out_R45		= 9,
	Turn_out_L45		= 10,
	Turn_in_R135		= 11,
	Turn_in_L135		= 12,
	Turn_out_R135		= 13,
	Turn_out_L135		= 14,
	Turn_RV90			= 15,
	Turn_LV90			= 16,
	Diagonal_R			= 17,
	Diagonal_L			= 18,
	Search_st_section	= 19,
	Search_st_half		= 20,
	Pivot_turn_R		= 21,
	Pivot_turn_L		= 22,
	run_pt_none			= 23,
	motor_free			= 24,
}t_run_pattern;

class motion_task:public Singleton<motion_task>
{
	private:
		void MotionFree();
	public:
		float delta_t;
		float run_time;
		float run_time_limit;
		t_run_pattern	run_task = No_run;
		t_machine_param target,max_set,mouse;
		t_straight_param st_set;
		void motion_inInterrupt();
};

class motion_plan
{
	private:
		float motion_time;
	public:
		//void straight(motion_task *move_task);
		void free_rotation(motion_task *move_task);
		void search_straight(motion_task *move_task);
		void pivot_turn(motion_task *move_task);
		void search_slalom(motion_task *move_task);
		void stop_brake();
};


#endif /* CPP_INC_MOTION_H_ */
