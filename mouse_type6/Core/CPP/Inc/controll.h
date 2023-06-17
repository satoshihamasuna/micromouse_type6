/*
 * controll.h
 *
 *  Created on: 2023/06/14
 *      Author: sato1
 */

#ifndef CPP_INC_CONTROLL_H_
#define CPP_INC_CONTROLL_H_

#include "singleton.h"

class PID_Controller
{
	private:
		float Kp,Ki,Kd;
	public:
		float target,I_target,prev_target;
		float output,I_output,prev_output;
		PID_Controller()
		{
			Kp = 0.0;Ki = 0.0; Kd = 0.0;
		}
		void Gain_Set(float _Kp,float _Ki,float _Kd)
		{
			Kp = _Kp;
			Ki = _Ki;
			Kd = _Kd;
		}
		void I_param_reset()
		{
			I_target = I_output = 0.0;
		}

		float Controll(float _target,float _output,float dt)
		{
			float operation = 0.0;
			target = _target;
			output = _output;
			operation = Kp*(target - output) + Ki*(I_target - I_output) + Kd*(prev_target - prev_output)/dt;
			prev_target = 	target;	prev_output 	= 	output;
			I_target 	+= 	target;	I_output		+= 	output;
			return operation;
		}
};


class Controll
{
	public:
		//template <class T> T *runtask;
		PID_Controller omega_ctrl();
		PID_Controller speed_ctrl();
		PID_Controller Ir_ctrl();
};


#endif /* CPP_INC_CONTROLL_H_ */
