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

class motion_plan
{
	public:
	    t_motion_task run_mode;
		void straight();
		void pivot_turn();
		void slalom();
		void stop_brake();
};



#endif /* CPP_INC_MOTION_PLAN_H_ */
