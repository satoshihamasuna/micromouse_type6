/*
 * makeMap_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_MAKE_MAP_CLASS_H_
#define CPP_INC_MAKE_MAP_CLASS_H_

#include "../../Module/Include/typedef.h"
#include "../../Module/Include/macro.h"
#include "queue_class.h"
#include "wall_class.h"

class make_map{
	public:
		make_map(wall_class *wall_property_,ring_queue<1024,t_MapNode> *maze_q_);
		wall_class *wall_property;
		ring_queue<1024,t_MapNode> *maze_q;
		uint16_t map[MAZE_SIZE_X][MAZE_SIZE_Y];
		void init_map(int x, int y,int goal_size);
		void expand(t_MapNode n,int mask);
		void make_map_queue(int x, int y,t_position expand_end,int size,int mask);
		void make_map_queue_zenmen(int x, int y,t_position expand_end,int size,int mask);
		void Display();
};



#endif /* CPP_INC_MAKE_MAP_CLASS_H_ */
