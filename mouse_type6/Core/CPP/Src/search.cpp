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
#include "search_class.h"
#include "adachi_class.h"
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


t_position Search::search_adachi_1(t_position start_pos,wall_class *_wall,make_map *_map,motion_plan *_motion)
{
	t_position tmp_my_pos = start_pos;
	t_position my_position = tmp_my_pos;

	adachi search_algolithm(_wall, _map);

	int direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);
	switch(direction)
	{
		case front:
			break;
		case right:
			break;
		case left:
			break;
		case rear:
			break;

	}

}
