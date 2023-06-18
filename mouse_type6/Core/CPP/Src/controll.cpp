/*
 * controll.cpp
 *
 *  Created on: 2023/06/18
 *      Author: sato1
 */
#include "controll.h"
#include "typedef.h"
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



