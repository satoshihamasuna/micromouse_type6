/*
 * adachi.h
 *
 *  Created on: Jun 10, 2023
 *      Author: sato1
 */

#ifndef CPP_INC_ADACHI_H_
#define CPP_INC_ADACHI_H_

#include "maze_def.h"
#include "queue.h"

void init_maze();
void init_map(int *x,int *y,int goal_size);
void expand(t_MapNode n,int mask);
void make_map_queue(int *x, int *y,int size,int mask);

void goal_set_vwall(int *gx,int *gy,int goal_size);
void goal_clear_vwall(int *gx,int *gy,int goal_size);
bool i_am_goal(t_position my_pos,int *gx,int *gy,int goal_size);


#endif /* CPP_INC_ADACHI_H_ */
