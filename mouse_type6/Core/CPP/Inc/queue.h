/*
 * queue.hpp
 *
 *  Created on: 2023/06/10
 *      Author: sato1
 */

#ifndef CPP_INC_QUEUE_H_
#define CPP_INC_QUEUE_H_

#include <iostream>

template<typename T> class Priority_queue{
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
		Priority_queue(int size = 20);
		int16_t tail;
	    T *buff;
};

class ring_queue{
	private:
		int buff[200];
	public:
			   int i;
};
/*
typedef struct QUEUE{
	int tail;
	t_MapNode node[1024];
}t_queue;


void list_init(t_queue *queue);
t_MapNode pop(t_queue *queue);
void push(t_queue *queue,t_MapNode input);
void swap(t_MapNode* a,t_MapNode* b);
void min_heapify(t_queue *queue,int i);
void build_heap(t_queue *queue);
t_MapNode heap_pop(t_queue *queue);
void heap_push(t_queue *queue,t_MapNode element);
t_MapNode node_set(int16_t st_x,int16_t st_y,int16_t cost,int16_t cost_h);
*/

#endif /* CPP_INC_QUEUE_H_ */
