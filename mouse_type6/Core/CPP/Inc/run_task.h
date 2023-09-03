/*
 * run_task.h
 *
 *  Created on: 2023/06/16
 *      Author: sato1
 */

#ifndef CPP_INC_RUN_TASK_H_
#define CPP_INC_RUN_TASK_H_

#include "../../Module/Include/typedef.h"

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
	Search_slalom_R		= 23,
	Search_slalom_L		= 24,
	run_brake			= 25,
	motor_free			= 26,
	Fix_wall			= 27,
}t_run_pattern;

typedef enum{
	NOP_MODE 		= 0,
	STRAIGHT_MODE 	= 1,
	DIAGONAL_MODE 	= 2,
	TURN_MODE 		= 3,
	SPIN_TURN_MODE  = 4,
}t_run_mode;

typedef struct{
	float velo;
	float r_min;
	float Lstart;
	float Lend;
	float degree;
	t_turn_dir turn_dir;
}t_turn_param_table;

typedef struct{
	t_turn_param_table const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_param;

typedef struct{
	//float base_velo;
	float max_velo;
	float acc;
}t_velo_param;

typedef struct{
	t_velo_param const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_straight_param;

typedef enum
{
	Non_controll = 0,
	Enable_st = 1,
	Enable_di = 2,
}t_wall_controll;
class PID_Setting
{
	private:
		t_pid_gain sp_gain;
		t_pid_gain om_gain;
	public:
		void set_sp_gain(float _kp,float _ki,float _kd)
		{
			sp_gain.Kp = _kp;
			sp_gain.Ki = _ki;
			sp_gain.Kd = _kd;
		}
		void set_om_gain(float _kp,float _ki,float _kd)
		{
			om_gain.Kp = _kp;
			om_gain.Ki = _ki;
			om_gain.Kd = _kd;
		}

		t_pid_gain get_sp_gain()
		{
			return sp_gain;
		}

		t_pid_gain get_om_gain()
		{
			return om_gain;
		}
};
class RunTask
{
	private:
		t_bool is_runTask = False;
		int	   brake_time = 0;
		float  run_turn_table_time = 0.0f;
		t_run_mode run_mode_state;
	public:
		t_wall_controll is_wallControl_Enable = Non_controll;
		float post_run_fix = 0.0;
		float prev_run_fix = 0.0;
		float turn_rmin_fix =0.0;
		t_bool is_exe_runTask();
		void set_stragith_sp_gain(float _kp,float _ki,float _kd);
		void MotionFree(float *run_time,float run_time_limit);
		void search_straight(t_motion_param mt_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms);
		void search_slalom(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms);
		void pivotturn(t_motion_param mt_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms);
		void fix_wall(t_machine_param *target_,float *run_time,float run_time_limit,float delta_t_ms);
		void straight(t_motion_param mt_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms);
		void diagonal(t_motion_param mt_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms);
		void long_turn(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms);
		void turn_v90(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms);
		void turn_in(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms);
		void turn_out(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms);
		void brake();

		void reset_brake_time()
		{
			brake_time = 0;
		}

		void set_run_mode_state(t_run_mode _run_mode_state)
		{
			run_mode_state = _run_mode_state;
		}

		t_run_mode get_run_mode_state()
		{
			return run_mode_state;
		}
};



#endif /* CPP_INC_RUN_TASK_H_ */
