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
				MotionFree();
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


void motion_task::MotionFree()
{
	Motor_SetDuty_Right(500);
	Motor_SetDuty_Left(500);
	run_time = run_time + 1.0;
}

void motion_plan::free_rotation(motion_task *move_task)
{
	move_task->run_time_limit = 1000.0;
	move_task->run_time = 0.0;
	move_task->run_task = motor_free;
}

