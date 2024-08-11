/*
 * interrupt.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_INTERRUPT_H_
#define CPP_INC_INTERRUPT_H_

#include "../../Component/Inc/singleton.h"
#include "../../Pheripheral/Include/macro.h"
#include "../../Pheripheral/Include/index.h"
//#include "../Include/macro.h"

class Interrupt:public Singleton<Interrupt>
{
	private:
		uint32_t time_count;
		uint32_t acc_time_cnt;
		float acc_buff[ACC_BUFF_SIZE];
		float acc_sum;

		float z_acc_buff[ACC_BUFF_SIZE];
		float z_acc_sum;

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
