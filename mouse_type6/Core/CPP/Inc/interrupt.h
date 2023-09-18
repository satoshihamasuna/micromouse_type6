/*
 * interrupt.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_INTERRUPT_H_
#define CPP_INC_INTERRUPT_H_

#include "singleton.h"

#define ACC_BUFF_SIZE (30)

class Interrupt:public Singleton<Interrupt>
{
	private:
		uint32_t time_count;
		uint32_t acc_time_cnt;
		float acc_buff[ACC_BUFF_SIZE];
		float acc_sum;
		float velo_buff[ACC_BUFF_SIZE];
		float velo_sum;
		float Rvelo_buff[ACC_BUFF_SIZE];
		float Rvelo_sum;
		float Lvelo_buff[ACC_BUFF_SIZE];
		float Lvelo_sum;


	public:
		void setTask();
		void preprocess();
		void main();
		void postprocess();
		uint32_t return_time_count()
		{
			return time_count;
		}
};


#endif /* CPP_INC_INTERRUPT_H_ */
