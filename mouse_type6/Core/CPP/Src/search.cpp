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
t_bool Search::i_am_goal(int x,int y,int gx,int gy,int goal_size){
	t_bool flag = False;
	if(gx <= x && x < (gx + goal_size))
	{
		if(gy <= y && y < (gy + goal_size))
		{
			flag = True;
		}
	}
	return flag;
}

t_bool Search::i_am_goal(t_position pos,t_position g_pos,int goal_size)
{
	int x = pos.x,		y = pos.y;
	int gx = g_pos.x,	gy = g_pos.y;

	t_bool flag = False;
	if(gx <= x && x < (gx + goal_size))
	{
		if(gy <= y && y < (gy + goal_size))
		{
			flag = True;
		}
	}
	return flag;
}

t_position Search::search_adachi_1(	t_position start_pos,t_position goal_pos,int goal_size,
									wall_class *_wall,make_map *_map,motion_plan *_motion)
{
	t_position tmp_my_pos = start_pos;
	t_position my_position = tmp_my_pos;

	adachi search_algolithm(_wall, _map);

	int direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);
	my_position = tmp_my_pos;
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

	while(i_am_goal(my_position, goal_pos, goal_size) != True)
	{
		direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);
		if(_wall->is_unknown(my_position.x, my_position.y))
		{
				_wall->set_wall(my_position);
		}
		my_position = tmp_my_pos;
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
	return my_position;
}
