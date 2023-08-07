/*
 * Kalman_filter.h
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */

#ifndef CPP_INC_KALMAN_FILTER_H_
#define CPP_INC_KALMAN_FILTER_H_

#include "../../Module/Include/index.h"
#include "singleton.h"

class KalmanFilter:public Singleton<KalmanFilter>
{
	private:
		float Q_acc = 0.01;
		float Q_vel = 0.001;//0.0008;
		float k_w;
		float k_v;
		float P[2][2];
		float K[2];
		float y;
		float dt;

	public:
		float calc_speed_filter(float acc,float velo);
		void filter_init(float mdt = 1.0f);
};



#endif /* CPP_INC_KALMAN_FILTER_H_ */
