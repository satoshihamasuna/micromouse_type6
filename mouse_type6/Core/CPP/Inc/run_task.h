/*
 * run_task.h
 *
 *  Created on: 2023/06/16
 *      Author: sato1
 */

#ifndef CPP_INC_RUN_TASK_H_
#define CPP_INC_RUN_TASK_H_

#include "typedef.h"

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
	run_brake			= 23,
	motor_free			= 24,
}t_run_pattern;

class RunTask
{
	private:
		t_bool is_runTask = False;
		int	   brake_time = 0;
	public:
		t_bool is_exe_runTask();
		void MotionFree(float *run_time,float run_time_limit);
		void search_straight(t_straight_param st_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms);
		void search_slalom();
		void search_pivotturn();
		void straight();
		void long_turn();
		void turn_v90();
		void turn_in();
		void turn_out();
		void brake();
};



#endif /* CPP_INC_RUN_TASK_H_ */
