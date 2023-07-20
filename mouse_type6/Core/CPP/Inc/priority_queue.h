/*
 * priority_queue.h
 *
 *  Created on: 2023/06/27
 *      Author: sato1
 */

#ifndef CPP_INC_PRIORITY_QUEUE_H_
#define CPP_INC_PRIORITY_QUEUE_H_

#include <iostream>
#include "../../Module/Include/typedef.h"
#include "../../Module/Include/macro.h"
#include "wall_class.h"

template<std::size_t SIZE,typename T>
class Priority_queue{
	private:
		uint16_t queue_length();
	    void min_heapify(uint16_t parent_pos);
	    void swap(T *a,T *b);
	public:
	    Priority_queue();
	    void push(T push_data);
		T heap_pop();
		void heap_push(T push_data);
		void build_heap();
		void queue_init();
		bool is_Empty_queue();
		//Priority_queue();
		int16_t tail;
	    T buff[SIZE];
};




#endif /* CPP_INC_PRIORITY_QUEUE_H_ */
