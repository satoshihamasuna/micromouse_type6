/*
 * adachi.h
 *
 *  Created on: Jun 10, 2023
 *      Author: sato1
 */

#ifndef CPP_INC_ADACHI_H_
#define CPP_INC_ADACHI_H_

#include "maze_def.h"

void init_maze();
void init_map(int *x,int *y,int goal_size);
void goal_set_vwall(int *gx,int *gy,int goal_size);
void goal_clear_vwall(int *gx,int *gy,int goal_size);
bool i_am_goal(t_position my_pos,int *gx,int *gy,int goal_size);


#endif /* CPP_INC_ADACHI_H_ */
