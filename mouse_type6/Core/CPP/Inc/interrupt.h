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
	public:
		void setTask();
		void preprocess();
		void main();
		void postprocess();
};


#endif /* CPP_INC_INTERRUPT_H_ */
