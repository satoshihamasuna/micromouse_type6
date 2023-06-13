/*
 * motion.h
 *
 *  Created on: Jun 12, 2023
 *      Author: sato1
 */

#ifndef CPP_INC_MOTION_PLAN_H_
#define CPP_INC_MOTION_PLAN_H_

#include <stdio.h>
#include "typedef.h"
#include "singleton.h"



class motion_plan
{
	public:
		void straight();
		void pivot_turn();
		void slalom();
		void stop_brake();
};

class motion_task:public Singleton<motion_task>
{
	public:
};
#endif /* CPP_INC_MOTION_PLAN_H_ */
