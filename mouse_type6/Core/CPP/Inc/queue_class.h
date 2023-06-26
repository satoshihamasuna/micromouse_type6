/*
 * queue.hpp
 *
 *  Created on: 2023/06/10
 *      Author: sato1
 */

#ifndef CPP_INC_QUEUE_CLASS_H_
#define CPP_INC_QUEUE_CLASS_H_

#include <iostream>
#include "typedef.h"
template<std::size_t SIZE,typename T>
class ring_queue{
	private:
		T buff[SIZE];
		int16_t tail;
		int16_t head;
		int16_t length;
		const uint16_t cap = SIZE;
	public:
		ring_queue();
		T pop();
		void push(T push_data);
		int queue_length();
		void queue_reset();
};

/*
class ring_queue{
	private:
		t_MapNode buff[1024];
		int16_t tail;
		int16_t head;
		int16_t length;
		const uint16_t cap = 1024;
	public:
		ring_queue();
		t_MapNode pop();
		void push(t_MapNode push_data);
		int queue_length();
		void queue_reset();
};
*/
//template class ring_queue<1024u,t_MapNode>;


#endif /* CPP_INC_QUEUE_CLASS_H_ */
