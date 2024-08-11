/*
 * path_follow.cpp
 *
 *  Created on: 2024/02/27
 *      Author: sato1
 */

#include "../Inc/path_follow.h"
#include <math.h>


float path_follow_class::calc_control_yaw_rate(float ideal_velo,float vehicle_velo,float ideal_yaw_rate,float vehicle_yaw_rate)
{
	float turn_rho = ideal_yaw_rate/ideal_velo;
	controll_yaw = 0.0f;
	virtual_velo = ideal_velo*cos(yaw_theta_error)/(1-position_error*turn_rho/1000.0f);
	virtual_yaw_rate = turn_rho*virtual_velo;
	yaw_theta_error += (vehicle_yaw_rate - virtual_yaw_rate)*m_dt/1000.0f;
	float tmp_sin_yaw_err = sin(yaw_theta_error);
	position_error  += ideal_velo*tmp_sin_yaw_err*m_dt;
	controll_yaw = virtual_yaw_rate - k_pos*position_error*vehicle_velo - k_yaw*yaw_theta_error;//tmp_sin_yaw_err;
	return controll_yaw;
}
