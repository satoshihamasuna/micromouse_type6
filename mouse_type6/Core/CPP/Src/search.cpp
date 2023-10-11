/*
 * search.cpp
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */


#include "queue_class.h"
#include "make_map_class.h"
#include "../../Module/Include/typedef.h"
#include "../../Module/Include/index.h"
#include "Kalman_filter.h"
#include "search_class.h"
#include "adachi_class.h"
#include "run_param.h"
#include "motion.h"
//#include "glob_var_machine.h"

#define ALLOW_SIDE_DIFF 8.0

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
									wall_class *_wall,make_map *_map,motion_plan *motion_plan)
{
	t_position tmp_my_pos = start_pos;
	t_position my_position = tmp_my_pos;

	adachi search_algolithm(&(*_wall),&(*_map));

	_map->init_map(goal_pos.x, goal_pos.y, goal_size);
	_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
	motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
	motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.2, 0.001, 0.0);
	KalmanFilter::getInstance().filter_init();

	int direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);
	switch(direction)
	{
		case Front:
			if(my_position.x == 0 && my_position.x == 0 )
			{
				motion_plan->straight(45.0+15.0, 4.0, 0.30f, 0.30f);
			}
			else
			{
				motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
			}
			_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;
		case Right:
			motion_plan->pivot_turn(DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;
		case Left:
			motion_plan->pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}

			break;
		case Rear:
			motion_plan->pivot_turn(DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;

	}
	my_position = tmp_my_pos;

	while(i_am_goal(my_position, goal_pos, goal_size) != True)
	{

		//if(_wall->is_unknown(my_position.x, my_position.y) == True)
		//{
				_wall->set_wall(my_position);
		//}
		direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);
		switch(direction)
		{
			case Front:
				motion_plan->search_straight(90.0, 4.0, 0.30f, 0.30f);
				_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
				while(motion_task::getInstance().run_task !=No_run){}
				break;
			case Right:
				if(SensingTask::getInstance().sen_l.is_wall == True && ABS(SensingTask::getInstance().sen_l.distance - 45.0) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight(45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall(500);
						while(motion_task::getInstance().run_task !=No_run){}
					}

					motion_plan->pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->fix_wall(500);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->pivot_turn(DEG2RAD(-180.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(my_position) == WALL && ABS(SensingTask::getInstance().sen_fr.distance - SensingTask::getInstance().sen_fl.distance) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight(45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall(500);
						while(motion_task::getInstance().run_task !=No_run){}
					}
					motion_plan->pivot_turn(DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else
				{
					motion_plan->searchSlalom(&param_R90_search);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
		  	    	while(motion_task::getInstance().run_task !=No_run){}
				}
		  	    break;
			case Left:
				if(SensingTask::getInstance().sen_r.is_wall == True && ABS(SensingTask::getInstance().sen_r.distance - 45.0) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight(45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall(500);
						while(motion_task::getInstance().run_task !=No_run){}
					}

					motion_plan->pivot_turn(DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->fix_wall(500);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(my_position) == WALL && ABS(SensingTask::getInstance().sen_fr.distance - SensingTask::getInstance().sen_fl.distance) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall(500);
						while(motion_task::getInstance().run_task !=No_run){}
					}
					motion_plan->pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else
				{
					motion_plan->searchSlalom(&param_L90_search);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				break;
			case Rear:
				motion_plan->straight(45.0, 4.0, 0.30f, 0.00f);
				_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);

				while(motion_task::getInstance().run_task !=No_run){}
				if(_wall->get_WallState(my_position) == WALL)
				{
					motion_plan->fix_wall( 500);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				t_position r_pos = my_position;
				r_pos.dir = (t_direction)(((int)(r_pos.dir) + 1) % 4);
				t_position l_pos = my_position;
				l_pos.dir = (t_direction)(((int)(l_pos.dir) + 3) % 4);
				if(_wall->get_WallState(r_pos) == WALL)
				{
					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->fix_wall( 300);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(l_pos) == WALL)
				{
					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->fix_wall( 300);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else{
					motion_plan->pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}

				motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
				while(motion_task::getInstance().run_task !=No_run){}
				break;
		}
		my_position = tmp_my_pos;
	}
	_wall->set_wall(my_position);
	motion_plan->straight( 45.0, 4.0, 0.30f, 0.0f);
	while(motion_task::getInstance().run_task !=No_run){}
	HAL_Delay(100);
	return my_position;
}

t_position Search::search_adachi_1_acc(	t_position start_pos,t_position goal_pos,int goal_size,wall_class *_wall,make_map *_map,motion_plan *motion_plan)
{
	t_position tmp_my_pos = start_pos;
	t_position my_position = tmp_my_pos;

	adachi search_algolithm(&(*_wall),&(*_map));

	_map->init_map(goal_pos.x, goal_pos.y, goal_size);
	_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
	motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
	motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.2, 0.001, 0.0);
	KalmanFilter::getInstance().filter_init();

	int direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);
	switch(direction)
	{
		case Front:
			if(my_position.x == 0 && my_position.x == 0 )
			{
				motion_plan->straight(45.0+15.0, 4.0, 0.30f, 0.30f);
			}
			else
			{
				motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
			}
			_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;
		case Right:
			motion_plan->pivot_turn(DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;
		case Left:
			motion_plan->pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}

			break;
		case Rear:
			motion_plan->pivot_turn(DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;

	}
	my_position = tmp_my_pos;

	while(i_am_goal(my_position, goal_pos, goal_size) != True)
	{

		if(_wall->is_unknown(my_position.x, my_position.y) == True)
		{
				_wall->set_wall(my_position);
		}
		direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);

		uint8_t next_acc_flag = 0x00;
		t_position next_acc_pos = tmp_my_pos;
		int		next_acc_dir  = direction;
		if(_wall->is_unknown(tmp_my_pos.x, tmp_my_pos.y) == False)
		{
			if(i_am_goal(tmp_my_pos,goal_pos,goal_size) == False){
				next_acc_flag = 0x80;
				next_acc_dir  = search_algolithm.get_next_dir(tmp_my_pos, 0x01, &next_acc_pos);
			}
		}

		switch(direction | next_acc_flag)
		{
			case Front:
				motion_plan->search_straight(90.0, 4.0, motion_task::getInstance().target.velo, 0.30f);
				_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
				while(motion_task::getInstance().run_task !=No_run){}
				break;
			case Front|0x80:
				if(next_acc_dir == Front)
					motion_plan->search_straight(90.0, 4.0, 0.60, 0.60f);
				else
					motion_plan->search_straight(90.0, 4.0, 0.60, 0.30f);
				_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
				while(motion_task::getInstance().run_task !=No_run){}
				break;
			case Right:
			case (Right|0x80):
				if(SensingTask::getInstance().sen_l.is_wall == True && ABS(SensingTask::getInstance().sen_l.distance - 45.0) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight(45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall(500);
						while(motion_task::getInstance().run_task !=No_run){}
					}

					motion_plan->pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->fix_wall(500);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->pivot_turn(DEG2RAD(-180.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(my_position) == WALL && ABS(SensingTask::getInstance().sen_fr.distance - SensingTask::getInstance().sen_fl.distance) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight(45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall(500);
						while(motion_task::getInstance().run_task !=No_run){}
					}
					motion_plan->pivot_turn(DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else
				{
					motion_plan->searchSlalom(&param_R90_search);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
		  	    	while(motion_task::getInstance().run_task !=No_run){}
				}
		  	    break;
			case Left:
			case (Left|0x80):
				if(SensingTask::getInstance().sen_r.is_wall == True && ABS(SensingTask::getInstance().sen_r.distance - 45.0) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight(45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall(500);
						while(motion_task::getInstance().run_task !=No_run){}
					}

					motion_plan->pivot_turn(DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->fix_wall(500);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(my_position) == WALL && ABS(SensingTask::getInstance().sen_fr.distance - SensingTask::getInstance().sen_fl.distance) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall(500);
						while(motion_task::getInstance().run_task !=No_run){}
					}
					motion_plan->pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight(45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else
				{
					motion_plan->searchSlalom(&param_L90_search);
					_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				break;
			case Rear:
			case (Rear|0x80):
				motion_plan->straight(45.0, 4.0, 0.30f, 0.00f);
				_map->make_map_queue(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);

				while(motion_task::getInstance().run_task !=No_run){}
				if(_wall->get_WallState(my_position) == WALL)
				{
					motion_plan->fix_wall( 500);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				t_position r_pos = my_position;
				r_pos.dir = (t_direction)(((int)(r_pos.dir) + 1) % 4);
				t_position l_pos = my_position;
				l_pos.dir = (t_direction)(((int)(l_pos.dir) + 3) % 4);
				if(_wall->get_WallState(r_pos) == WALL)
				{
					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->fix_wall( 300);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(l_pos) == WALL)
				{
					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->fix_wall( 300);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else{
					motion_plan->pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}

				motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
				while(motion_task::getInstance().run_task !=No_run){}
				break;
		}
		my_position = tmp_my_pos;
	}
	_wall->set_wall(my_position);
	motion_plan->straight( 45.0, 4.0, 0.30f, 0.0f);
	while(motion_task::getInstance().run_task !=No_run){}
	HAL_Delay(100);
	return my_position;
}

t_position Search::search_adachi_2(	t_position start_pos,t_position goal_pos,int goal_size,
									wall_class *_wall,make_map *_map,motion_plan *motion_plan)
{
	t_position tmp_my_pos = start_pos;
	t_position my_position = tmp_my_pos;

	adachi search_algolithm(&(*_wall),&(*_map));

	_map->init_map(goal_pos.x, goal_pos.y, goal_size);
	_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
	motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
	motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.2, 0.001, 0.0);
	KalmanFilter::getInstance().filter_init();

	int direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);
	switch(direction)
	{
		case Front:
			if(my_position.x == 0 && my_position.x == 0 )
			{
				motion_plan->straight( 45.0+15.0, 4.0, 0.30f, 0.30f);
			}
			else
			{
				motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
			}
			_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;
		case Right:
			motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;
		case Left:
			motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}

			break;
		case Rear:
			motion_plan->pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;

	}
	my_position = tmp_my_pos;

	while(i_am_goal(my_position, goal_pos, goal_size) != True)
	{

		//if(_wall->is_unknown(my_position.x, my_position.y) == True)
		//{
				_wall->set_wall(my_position);
		//}
		direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);
		switch(direction)
		{
			case Front:
				motion_plan->search_straight( 90.0, 4.0, 0.30f, 0.30f);
				_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
				while(motion_task::getInstance().run_task !=No_run){}
				break;
			case Right:
				if(SensingTask::getInstance().sen_l.is_wall == True && ABS(SensingTask::getInstance().sen_l.distance - 45.0) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall( 500);
						while(motion_task::getInstance().run_task !=No_run){}
					}

					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->fix_wall( 500);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->pivot_turn( DEG2RAD(-180.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(my_position) == WALL && ABS(SensingTask::getInstance().sen_fr.distance - SensingTask::getInstance().sen_fl.distance) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall( 500);
						while(motion_task::getInstance().run_task !=No_run){}
					}
					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else
				{
					motion_plan->searchSlalom(&param_R90_search);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
		  	    	while(motion_task::getInstance().run_task !=No_run){}
				}
		  	    break;
			case Left:
				if(SensingTask::getInstance().sen_r.is_wall == True && ABS(SensingTask::getInstance().sen_r.distance - 45.0) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall( 500);
						while(motion_task::getInstance().run_task !=No_run){}
					}

					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->fix_wall( 500);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
										while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(my_position) == WALL && ABS(SensingTask::getInstance().sen_fr.distance - SensingTask::getInstance().sen_fl.distance) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall( 500);
						while(motion_task::getInstance().run_task !=No_run){}
					}
					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else
				{
					motion_plan->searchSlalom(&param_L90_search);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				break;
			case Rear:
				motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
				_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
				while(motion_task::getInstance().run_task !=No_run){}
				if(_wall->get_WallState(my_position) == WALL)
				{
					motion_plan->fix_wall( 500);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				if(_wall->get_WallState(my_position) == WALL)
				{
					motion_plan->fix_wall( 500);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				t_position r_pos = my_position;
				r_pos.dir = (t_direction)(((int)(r_pos.dir) + 1) % 4);
				t_position l_pos = my_position;
				l_pos.dir = (t_direction)(((int)(l_pos.dir) + 3) % 4);
				if(_wall->get_WallState(r_pos) == WALL)
				{
					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->fix_wall( 300);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(l_pos) == WALL)
				{
					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->fix_wall( 300);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else{
					motion_plan->pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
				while(motion_task::getInstance().run_task !=No_run){}
				break;
		}
		my_position = tmp_my_pos;
	}
	_wall->set_wall(my_position);
	motion_plan->straight( 45.0, 4.0, 0.30f, 0.0f);
	while(motion_task::getInstance().run_task !=No_run){}
	HAL_Delay(100);
	return my_position;
}

t_position Search::search_adachi_2_acc(	t_position start_pos,t_position goal_pos,int goal_size,
									wall_class *_wall,make_map *_map,motion_plan *motion_plan)
{
	t_position tmp_my_pos = start_pos;
	t_position my_position = tmp_my_pos;

	adachi search_algolithm(&(*_wall),&(*_map));

	_map->init_map(goal_pos.x, goal_pos.y, goal_size);
	_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
	motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.05, 0.0);
	motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.2, 0.001, 0.0);
	KalmanFilter::getInstance().filter_init();

	int direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);
	switch(direction)
	{
		case Front:
			if(my_position.x == 0 && my_position.x == 0 )
			{
				motion_plan->straight( 45.0+15.0, 4.0, 0.30f, 0.30f);
			}
			else
			{
				motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
			}
			_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;
		case Right:
			motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;
		case Left:
			motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}

			break;
		case Rear:
			motion_plan->pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
			while(motion_task::getInstance().run_task !=No_run){}

			motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
			_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
			while(motion_task::getInstance().run_task !=No_run){}
			break;

	}
	my_position = tmp_my_pos;

	while(i_am_goal(my_position, goal_pos, goal_size) != True)
	{

		if(_wall->is_unknown(my_position.x, my_position.y) == True)
		{
				_wall->set_wall(my_position);
		}
		direction = search_algolithm.get_next_dir(my_position, 0x01, &tmp_my_pos);

		uint8_t next_acc_flag = 0x00;
		t_position next_acc_pos = tmp_my_pos;
		int		next_acc_dir  = direction;
		if(_wall->is_unknown(tmp_my_pos.x, tmp_my_pos.y) == False)
		{
			if(i_am_goal(tmp_my_pos,goal_pos,goal_size) == False){
				next_acc_flag = 0x80;
				next_acc_dir  = search_algolithm.get_next_dir(tmp_my_pos, 0x01, &next_acc_pos);
			}
		}
		switch(direction | next_acc_flag)
		{
			case Front:
				motion_plan->search_straight( 90.0, 4.0, 0.30f, 0.30f);
				_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
				while(motion_task::getInstance().run_task !=No_run){}
				break;
			case Front|0x80:
				if(next_acc_dir == Front)
					motion_plan->search_straight(90.0, 4.0, 0.60, 0.60f);
				else
					motion_plan->search_straight(90.0, 4.0, 0.60, 0.30f);
				_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
				while(motion_task::getInstance().run_task !=No_run){}
				break;
			case Right:
			case Right|0x80:
				if(SensingTask::getInstance().sen_l.is_wall == True && ABS(SensingTask::getInstance().sen_l.distance - 45.0) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall( 500);
						while(motion_task::getInstance().run_task !=No_run){}
					}

					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->fix_wall( 500);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->pivot_turn( DEG2RAD(-180.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(my_position) == WALL && ABS(SensingTask::getInstance().sen_fr.distance - SensingTask::getInstance().sen_fl.distance) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall( 500);
						while(motion_task::getInstance().run_task !=No_run){}
					}
					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else
				{
					motion_plan->searchSlalom(&param_R90_search);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
		  	    	while(motion_task::getInstance().run_task !=No_run){}
				}
		  	    break;
			case Left:
			case Left|0x80:
				if(SensingTask::getInstance().sen_r.is_wall == True && ABS(SensingTask::getInstance().sen_r.distance - 45.0) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall( 500);
						while(motion_task::getInstance().run_task !=No_run){}
					}

					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->fix_wall( 500);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
										while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(my_position) == WALL && ABS(SensingTask::getInstance().sen_fr.distance - SensingTask::getInstance().sen_fl.distance) >= ALLOW_SIDE_DIFF)
				{
					motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}

					if(_wall->get_WallState(my_position) == WALL)
					{
						motion_plan->fix_wall( 500);
						while(motion_task::getInstance().run_task !=No_run){}
					}
					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}

					motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else
				{
					motion_plan->searchSlalom(&param_L90_search);
					_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				break;
			case Rear:
			case Rear|0x80:
				motion_plan->straight( 45.0, 4.0, 0.30f, 0.00f);
				_map->make_map_queue_zenmen(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01);
				while(motion_task::getInstance().run_task !=No_run){}
				if(_wall->get_WallState(my_position) == WALL)
				{
					motion_plan->fix_wall( 500);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				if(_wall->get_WallState(my_position) == WALL)
				{
					motion_plan->fix_wall( 500);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				t_position r_pos = my_position;
				r_pos.dir = (t_direction)(((int)(r_pos.dir) + 1) % 4);
				t_position l_pos = my_position;
				l_pos.dir = (t_direction)(((int)(l_pos.dir) + 3) % 4);
				if(_wall->get_WallState(r_pos) == WALL)
				{
					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->fix_wall( 300);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else if(_wall->get_WallState(l_pos) == WALL)
				{
					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->fix_wall( 300);
					while(motion_task::getInstance().run_task !=No_run){}
					motion_plan->pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				else{
					motion_plan->pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
					while(motion_task::getInstance().run_task !=No_run){}
				}
				motion_plan->straight( 45.0, 4.0, 0.30f, 0.30f);
				while(motion_task::getInstance().run_task !=No_run){}
				break;
		}
		my_position = tmp_my_pos;
	}
	_wall->set_wall(my_position);
	motion_plan->straight( 45.0, 4.0, 0.30f, 0.0f);
	while(motion_task::getInstance().run_task !=No_run){}
	HAL_Delay(100);
	return my_position;
}
