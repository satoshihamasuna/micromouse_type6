/*
 * priority_queue.c
 *
 *  Created on: 2023/06/10
 *      Author: sato1
 */

#include "typedef.h"
#include "priority_queue.h"

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

t_MapNode node_set(int16_t st_x,int16_t st_y,int16_t cost,int16_t cost_h){
	t_MapNode n;
	n.st_x = st_x;		n.st_y = st_y;
	n.cost = cost;		n.cost_h = cost_h;
	return n;
}
