/*
 * kalman_filter.cpp
 *
 *  Created on: 2023/06/21
 *      Author: sato1
 */


#include "Kalman_filter.h"

void KalmanFilter::KalmanFilter::filter_init(float mdt)
{
	P[0][0] = 0.0;
	P[0][1] = 0.0;
	P[1][0] = 0.0;
	P[1][1] = 0.0;

	K[0] = 0.0f;
	K[1] = 0.0f;

	y = 0.0f;
	dt = mdt/1000.0f;
}

float KalmanFilter::calc_speed_filter(float acc,float velo){
	k_v = k_v - k_w*dt + acc*dt;
	//k_w = k_w;

	P[0][0] += dt*(P[1][1]*dt - P[0][1] - P[1][0] + Q_acc);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	//P[1][1]  = P[1][1];

	y = velo - k_v;

	float S = P[0][0] + Q_vel;

	K[0] = P[0][0]/S;
	K[1] = P[1][0]/S;

	k_v = k_v + K[0]*y;
	k_w = k_w + K[1]*y;

	float P00_temp = P[0][0];
	float P01_temp = P[0][1];
	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;

	return k_v;
}
