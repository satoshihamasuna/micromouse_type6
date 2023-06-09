/*
 * priority_queue.cpp
 *
 *  Created on: 2023/06/10
 *      Author: sato1
 */
#include <queue.h>
#include "iostream"
#include <typedef_node.h>


//#include "typedef_node.hpp"
/*
void list_init(t_queue *queue){
	queue->tail = -1;
}

t_MapNode pop(t_queue *queue){
	t_MapNode pop_data;
	pop_data = queue->node[queue->tail];
	queue->tail = queue->tail - 1;
	return pop_data;
}

void push(t_queue *queue,t_MapNode input)
{
	queue->node[queue->tail + 1] = input;
	queue->tail = queue->tail + 1;
}

void swap(t_MapNode* a,t_MapNode* b){
	t_MapNode temp;
	temp = *b;
	*b = *a;
	*a = temp;
}


void min_heapify(t_queue *queue,int i){	//iは親ノードの位置
	int left_ch = 2*i + 1;
	int right_ch = 2*i + 2;
	int smallest = i;

	while(1){
		if(left_ch <= queue->tail && ((queue->node[smallest].cost+queue->node[smallest].cost_h) > (queue->node[left_ch].cost+queue->node[left_ch].cost_h)))
			smallest = left_ch;
		if(right_ch <= queue->tail && ((queue->node[smallest].cost+queue->node[smallest].cost_h) > (queue->node[right_ch].cost+queue->node[right_ch].cost_h)))
			smallest = right_ch;

		if(smallest != i){
			swap(&queue->node[i],&queue->node[smallest]);
			left_ch = 2*smallest + 1;
			right_ch = 2*smallest + 2;
			i = smallest;
		}
		else
		{
			break;
		}
	}
}


void build_heap(t_queue *queue){
	if(queue->tail - 1 > 0)
	{
		for(int i = (queue->tail - 1)/2; i >= 0;i--){
			min_heapify(queue,i);
		}
	}
}

t_MapNode heap_pop(t_queue *queue){
	t_MapNode n;
	n = queue->node[0];
	queue->node[0] = queue->node[queue->tail];
	queue->tail = queue->tail - 1;
	if(queue->tail != -1)
		min_heapify(queue,0);
	return n;
}

void heap_push(t_queue *queue,t_MapNode element){
	push(queue,element);
	if(queue->tail - 1 > 0)
	{
		min_heapify(queue,queue->tail-1/2);
	}
}
*/

template<typename T> Priority_queue<T>::Priority_queue(int size)
{
    buff = new T[size];
    tail = -1;
}

template<typename T> void Priority_queue<T>::queue_init()
{
	tail = -1;
}

template<typename T> uint16_t Priority_queue<T>::queue_length(){
	return tail + 1;
}


template<typename T> bool Priority_queue<T>::is_Empty_queue()
{
	if(queue_length() == 0)
		return true;
	else
		return false;
}

template<typename T>  T Priority_queue<T>::heap_pop()
{
	//T pop_data;
	T pop_data = buff[0];
	buff[0] = buff[tail];
	tail = tail - 1;
	if(is_Empty_queue() == false)
		min_heapify(0);
	return pop_data;
}

template<typename T> void Priority_queue<T>::push(T push_data)
{
	buff[tail + 1] = push_data;
	tail = tail + 1;
}

template<typename T> void Priority_queue<T>::heap_push(T push_data)
{
	buff[tail + 1] = push_data;
	tail = tail + 1;
	if(queue_length() > 1)
	    min_heapify((tail-1)/2);
}

template<typename T> void Priority_queue<T>::swap(T *a,T *b)
{
	T temp;
	temp = *b;
	*b = *a;
	*a = temp;
}

template<> void Priority_queue<t_MapNode>::min_heapify(uint16_t parent_pos)
{
	uint16_t left_ch  = 2*parent_pos + 1;
	uint16_t right_ch = 2*parent_pos + 2;
	uint16_t smallest = parent_pos;

	while(1)
	{
		if(left_ch <= tail && ((buff[smallest].cost) > (buff[left_ch].cost)))
			smallest = left_ch;
		if(right_ch <= tail && ((buff[smallest].cost) > (buff[right_ch].cost)))
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

template<typename T> void Priority_queue<T>::min_heapify(uint16_t parent_pos)
{
	uint16_t left_ch  = 2*parent_pos + 1;
	uint16_t right_ch = 2*parent_pos + 2;
	uint16_t smallest = parent_pos;

	while(1)
	{
		if(left_ch <= tail && ((buff[smallest]) > (buff[left_ch])))
			smallest = left_ch;
		if(right_ch <= tail && ((buff[smallest]) > (buff[right_ch])))
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

template<typename T> void  Priority_queue<T>::build_heap()
{
    if( queue_length() > 1)
    {
    	for(int i = (tail-1)/2; i >= 0; i--)
    	{
    		min_heapify(i);
    	}
    }
}
