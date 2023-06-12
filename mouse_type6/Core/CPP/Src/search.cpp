/*
 * search.cpp
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */


#include "queue_class.h"
#include "make_map_class.h"
#include "typedef.h"
#include "index.h"
//#include "glob_var_machine.h"
t_bool i_am_goal(int x,int y,int *gx,int *gy,int goal_size){
	t_bool flag = False;
	for (int i = 0; i < goal_size;i++){
		for(int j = 0; j < goal_size;j++){
			if(x == gx[i] && y == gy[j]) flag = True;
		}
	}
	return flag;
}

t_wall   wall[MAZE_SIZE_X][MAZE_SIZE_Y];

void goal_set_vwall(int *gx,int *gy,int goal_size){
	if(goal_size == 3)
	{
		wall[gx[1]][gy[1]].north = wall[gx[1]][gy[1]].east = wall[gx[1]][gy[1]].south = wall[gx[1]][gy[1]].west = VWALL;
		wall[gx[1]][gy[2]].south = wall[gx[2]][gy[1]].west = wall[gx[1]][gy[0]].north = wall[gx[0]][gy[1]].east = VWALL;
	}

}

void goal_clear_vwall(int *gx,int *gy,int goal_size){
	if(goal_size == 3)
	{
		wall[gx[1]][gy[1]].north = wall[gx[1]][gy[1]].east = wall[gx[1]][gy[1]].south = wall[gx[1]][gy[1]].west = NOWALL;
		wall[gx[1]][gy[2]].south = wall[gx[2]][gy[1]].west = wall[gx[1]][gy[0]].north = wall[gx[0]][gy[1]].east = NOWALL;
	}
}

