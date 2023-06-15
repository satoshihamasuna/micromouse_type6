/*
 * run_task.cpp
 *
 *  Created on: 2023/06/16
 *      Author: sato1
 */

#include "index.h"
#include "run_task.h"

void RunTask::MotionFree(float *run_time)
{
	Motor_SetDuty_Right(100);
	Motor_SetDuty_Left(100);
	*run_time = *run_time + 1.0f;
}
