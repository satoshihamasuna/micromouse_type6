/*
 * ring_queue.cpp
 *
 *  Created on: Jun 10, 2023
 *      Author: sato1
 */


#include <queue.h>
#include "iostream"

/*
template<std::size_t SIZE,typename T> ring_queue<SIZE,T>::ring_queue()
{
	tail = -1;
	head = 0;
	length = 0;
	cap  = SIZE;
}
*/
template<std::size_t SIZE,typename T> T ring_queue<SIZE,T>::pop()
{
	if(length > 0)
	{
		T pop_data = buff[head];
		head = (head + 1)%cap;
		length = length - 1;
		return pop_data;
	}
	else
	{
		return NULL;
	}
}

template<std::size_t SIZE,typename T> void ring_queue<SIZE,T>::push(T push_data)
{
	buff[(tail + 1)%cap] = push_data;
	tail = (tail + 1)%cap;
	length = length + 1;
}

template<std::size_t SIZE,typename T> void ring_queue<SIZE,T>::queue_reset()
{
	tail = -1;
	head = (tail + 1)%cap;
	length = 0;
}

template<std::size_t SIZE,typename T> int ring_queue<SIZE,T>::queue_length()
{
	return length;
}
