/*
 * motion.cpp
 *
 *  Created on: 2023/06/15
 *      Author: sato1
 */


#include "motion.h"
#include "index.h"

void motion_task::motion_inInterrupt(){
	switch(run_task)
	{
		case Search_st_section:
			break;
		case Search_st_half:
			break;
		case Long_turnR90:
			break;
		case motor_free:
				rT.MotionFree(&run_time);
				if(run_time_limit < run_time)
				{
					run_task = run_pt_none;
				}
			break;
		default:
				Motor_SetDuty_Left(0);
				Motor_SetDuty_Right(0);
			break;
	}
}

/*

*/
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

}

