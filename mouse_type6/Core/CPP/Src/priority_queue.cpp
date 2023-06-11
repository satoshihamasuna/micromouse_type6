/*
 * priority_queue.cpp
 *
 *  Created on: 2023/06/10
 *      Author: sato1
 */
#include <../Include/queue_class.h>
#include "iostream"
#include "../Include/typedef.h"

template<typename T> bool less_than(T a,T b)
{
     return ((a < b) ? true : false);
}

template<> bool less_than(t_MapNode a,t_MapNode b)
{
     return ((a.cost < b.cost) ? true : false);
}

template<std::size_t SIZE,typename T> Priority_queue<SIZE,T>::Priority_queue()
{

    tail = -1;
}

template<std::size_t SIZE,typename T> void Priority_queue<SIZE,T>::queue_init()
{
	tail = -1;
}

template<std::size_t SIZE,typename T> uint16_t Priority_queue<SIZE,T>::queue_length(){
	return tail + 1;
}


template<std::size_t SIZE,typename T> bool Priority_queue<SIZE,T>::is_Empty_queue()
{
	if(queue_length() == 0)
		return true;
	else
		return false;
}

template<std::size_t SIZE,typename T>  T Priority_queue<SIZE,T>::heap_pop()
{
	//T pop_data;
	T pop_data = buff[0];
	buff[0] = buff[tail];
	tail = tail - 1;
	if(is_Empty_queue() == false)
		min_heapify(0);
	return pop_data;
}

template<std::size_t SIZE,typename T> void Priority_queue<SIZE,T>::push(T push_data)
{
	buff[tail + 1] = push_data;
	tail = tail + 1;
}

template<std::size_t SIZE,typename T> void Priority_queue<SIZE,T>::heap_push(T push_data)
{
	buff[tail + 1] = push_data;
	tail = tail + 1;
	if(queue_length() > 1)
	    min_heapify((tail-1)/2);
}

template<std::size_t SIZE,typename T> void Priority_queue<SIZE,T>::swap(T *a,T *b)
{
	T temp;
	temp = *b;
	*b = *a;
	*a = temp;
}



template<std::size_t SIZE,typename T> void Priority_queue<SIZE,T>::min_heapify(uint16_t parent_pos)
{
	uint16_t left_ch  = 2*parent_pos + 1;
	uint16_t right_ch = 2*parent_pos + 2;
	uint16_t smallest = parent_pos;

	while(1)
	{

		//if(left_ch <= tail && ((buff[smallest]) > (buff[left_ch])))
		if(left_ch <= tail && less_than(buff[left_ch],buff[smallest]))
			smallest = left_ch;
		//if(right_ch <= tail && ((buff[smallest]) > (buff[right_ch])))
		if(right_ch <= tail && less_than(buff[right_ch],buff[smallest]))
			smallest = right_ch;

		if(smallest != parent_pos)
		{
			swap(&buff[parent_pos],&buff[smallest]);
			left_ch = 2*smallest + 1;
			right_ch = 2*smallest + 2;
			parent_pos = smallest;
		}
		else
		{
			break;
		}
	}
}

template<std::size_t SIZE,typename T> void  Priority_queue<SIZE,T>::build_heap()
{
    if( queue_length() > 1)
    {
    	for(int i = (tail-1)/2; i >= 0; i--)
    	{
    		min_heapify(i);
    	}
    }
}
