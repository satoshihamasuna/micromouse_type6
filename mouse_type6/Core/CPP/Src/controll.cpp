/*
 * controll.cpp
 *
 *  Created on: 2023/06/18
 *      Author: sato1
 */
#include "controll.h"
#include "../../Module/Include/typedef.h"
#include "../../Module/Include/macro.h"

void PID_Controller::Gain_Set(float _Kp,float _Ki,float _Kd)
{
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
}

void PID_Controller::I_param_reset()
{
	I_target = I_output = 0.0;
}

float PID_Controller::Controll(float _target,float _output,float dt)
{
	float operation = 0.0;
	target = _target;
	output = _output;
	operation = Kp*(target - output) + Ki*(I_target - I_output) + Kd*(prev_target - prev_output)/dt;
	prev_target = 	target;	prev_output 	= 	output;
	I_target 	+= 	target;	I_output		+= 	output;

	return operation;
}

float PID_Controller::Anti_windup_1(float operation,float limit)
{
	if(ABS(operation) > limit)
	{
		I_target = I_target - target;
		I_output = I_output - output;
		return SIGN(operation) * limit;
	}
	return operation;
}

float PID_Controller::Anti_windup_2(float operation,float limit)
{
	if(ABS(operation) > limit)
	{
		float diff = operation - SIGN(operation) * limit;
		if(Kp != 0.0f)
		{
			I_target = I_target - 1/Kp*diff/2.0;
			I_output = I_output + 1/Kp*diff/2.0;
		}
		return SIGN(operation) * limit;
	}
	return operation;
}


