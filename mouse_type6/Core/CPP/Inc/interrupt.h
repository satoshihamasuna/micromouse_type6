/*
 * interrupt.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_INTERRUPT_H_
#define CPP_INC_INTERRUPT_H_

#include "singleton.h"

class Interrupt:public Singleton<Interrupt>
{
	private:
		uint32_t time_count;
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
