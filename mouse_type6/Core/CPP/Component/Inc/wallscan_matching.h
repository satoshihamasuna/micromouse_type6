/*
 * wallscan_matching.h
 *
 *  Created on: 2023/11/13
 *      Author: sato1
 */

#ifndef CPP_INC_WALLSCAN_MATCHING_H_
#define CPP_INC_WALLSCAN_MATCHING_H_

class wallscan_matching
{
	public:
		float x_corr,theta_corr;
		float predict_sensX,sensX;
		float predict_sens2x,predict_sens2theta;
		float x_odd,theta_odd;
		float correction_x,correction_theta;
		float k1 = 1.0;
		float k2 = 20.0;
		float alpha,beta;

		void set_oddmetry(float x,float theta)
		{
			x_odd 		= x;
			theta_odd	= theta;
		}
		void set_predict_sens()
		{
			predict_sensX = x_odd + k2*theta_odd;
		}
		void set_sens(float sensing_value)
		{
			sensX = sensing_value;
		}
		void matching()
		{
			x_corr = predict_sensX - sensX;
			theta_corr = (predict_sensX - sensX)*k2;
		}

		float return_correction_x()
		{
			return (x_odd - alpha*x_corr);
		}

		float return_correcton_theta()
		{
			return (theta_odd - beta*theta_corr);
		}
};



#endif /* CPP_INC_WALLSCAN_MATCHING_H_ */
