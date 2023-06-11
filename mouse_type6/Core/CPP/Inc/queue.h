/*
 * queue.hpp
 *
 *  Created on: 2023/06/10
 *      Author: sato1
 */

#ifndef CPP_INC_QUEUE_H_
#define CPP_INC_QUEUE_H_

#include <iostream>


template<std::size_t SIZE,typename T> class Priority_queue{
	private:
		uint16_t queue_length();
	    void min_heapify(uint16_t parent_pos);
	    void swap(T *a,T *b);
	public:
	    void push(T push_data);
		T heap_pop();
		void heap_push(T push_data);
		void build_heap();
		void queue_init();
		bool is_Empty_queue();
		Priority_queue();
		int16_t tail;
	    T buff[SIZE];
};

template<std::size_t SIZE,typename T> class ring_queue{
	private:
		T buff[SIZE];
		int16_t tail;
		int16_t head;
		int16_t length;
		const uint16_t cap = SIZE;
	public:
		//ring_queue();
		//~ring_queue();
		T pop();
		void push(T push_data);
		int queue_length();
		void queue_reset();
};


#endif /* CPP_INC_QUEUE_H_ */
