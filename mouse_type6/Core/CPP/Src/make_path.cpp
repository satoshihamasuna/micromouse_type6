/*
 * make_path.cpp
 *
 *  Created on: 2023/06/26
 *      Author: sato1
 */

#include "../../Module/Include/index.h"
#include "../../Module/Include/typedef.h"
#include "../../Module/Include/macro.h"
#include "make_path.h"
#include "run_task.h"
#include "motion.h"
#include "log_data.h"
#define DIJKSTRA_MAX_TIME (65535-1)

t_posDijkstra Dijkstra::conv_t_pos2t_posDijkstra(t_position pos,t_direction wall_pos)
{
	t_posDijkstra position;
	switch(wall_pos)
	{
		case North:
			position = SetNodePos(pos.x,pos.y,N_pos);
			break;
		case East:
			position = SetNodePos(pos.x,pos.y,E_pos);
			break;
		case South:
			position = SetNodePos(pos.x,pos.y-1,N_pos);
			break;
		case West :
			position = SetNodePos(pos.x-1,pos.y,E_pos);
			break;
		case Dir_None:
		default:
			position = SetNodePos(pos.x,pos.y,C_pos);
			break;
	}
	return position;
}


t_posDijkstra Dijkstra::conv_t_pos2t_posDijkstra(int _x,int _y,t_direction wall_pos)
{
	t_posDijkstra position;
	switch(wall_pos)
	{
		case North:
			position = SetNodePos(_x,_y,N_pos);
			break;
		case East:
			position = SetNodePos(_x,_y,E_pos);
			break;
		case South:
			position = SetNodePos(_x,_y-1,N_pos);
			break;
		case West :
			position = SetNodePos(_x-1,_y,E_pos);
			break;
		case Dir_None:
		default:
			position = SetNodePos(_x,_y,C_pos);
			break;
	}
	return position;
}

t_posDijkstra Dijkstra::SetNodePos(uint8_t _x,uint8_t _y,t_DijkstraWallPos _dpos)
{
	t_posDijkstra pos;
	pos.x = _x;
	pos.y = _y;
	pos.NodePos = _dpos;
	return pos;
}

t_element Dijkstra::SetNode(t_posDijkstra _parent,	uint16_t _time,		t_direction _dir
				 ,t_run_pattern _run_pt,		t_bool _determine)
{
	t_element node;
	node.parent_pos = _parent;
	node.time		= _time;
	node.dir		= _dir;
	node.run_pt		= _run_pt;
	node.determine  = _determine;
	return node;
}

void Dijkstra::init_dijkstra_map()
{
	for(int i = 0;i < MAZE_SIZE_X;i++)
	{
		for(int j = 0;j < MAZE_SIZE_Y;j++)
		{
			for(int d = 0; d < 3;d++)
			{
				switch(d)
				{
					case C_pos:
						closure[i][j].Center = SetNode(SetNodePos(i,j,C_pos), DIJKSTRA_MAX_TIME, Dir_None, No_run, False);
						break;
					case N_pos:
						closure[i][j].North = SetNode(SetNodePos(i,j,N_pos), DIJKSTRA_MAX_TIME, Dir_None, No_run, False);
						break;
					case E_pos:
						closure[i][j].East = SetNode(SetNodePos(i,j,E_pos), DIJKSTRA_MAX_TIME, Dir_None, No_run, False);
						break;
				}
			}
		}
	}
}

void Dijkstra::start_node_setUp(t_posDijkstra start_pos,t_direction dir)
{
	switch(start_pos.NodePos)
	{
		case C_pos:
			closure[start_pos.x][start_pos.y].Center = SetNode(SetNodePos(start_pos.x,start_pos.y,C_pos), 0, dir, No_run, False);
			break;
		case N_pos:
			closure[start_pos.x][start_pos.y].North  = SetNode(SetNodePos(start_pos.x,start_pos.y,N_pos), 0, dir, No_run, False);
			break;
		case E_pos:
			closure[start_pos.x][start_pos.y].East  = SetNode(SetNodePos(start_pos.x,start_pos.y,E_pos), 0, dir, No_run, False);
			break;
	}
}

t_bool Dijkstra::is_goal_Dijkstra(t_posDijkstra check_pos,t_position goal_pos,uint8_t goal_size)
{
	if(goal_pos.x <= check_pos.x  && check_pos.x < (goal_pos.x + goal_size))
	{
		if(goal_pos.y <= check_pos.y  && check_pos.y < (goal_pos.y + goal_size))
		{
			return True;
		}
	}

	if(check_pos.x == (goal_pos.x - 1) && check_pos.NodePos == E_pos)
	{
		if(goal_pos.y <= check_pos.y  && check_pos.y < (goal_pos.y + goal_size))
		{
			return True;
		}
	}


	if(check_pos.y == (goal_pos.y - 1) && check_pos.NodePos == N_pos)
	{
		if(goal_pos.x <= check_pos.x  && check_pos.x < (goal_pos.x + goal_size))
		{
			return True;
		}
	}
	return False;
}

void Dijkstra::set_determine(t_posDijkstra set_pos)
{
	switch(set_pos.NodePos)
	{
		case N_pos:
			closure[set_pos.x][set_pos.y].North.determine = True;
			break;
		case C_pos:
			closure[set_pos.x][set_pos.y].Center.determine = True;
			break;
		case E_pos:
			closure[set_pos.x][set_pos.y].East.determine = True;
			break;
	}
}

t_posDijkstra Dijkstra::min_search()
{
	t_posDijkstra min_pos = SetNodePos(0, 0, C_pos);
	uint16_t min_time = DIJKSTRA_MAX_TIME;
	for(int i = 0;i < MAZE_SIZE_X;i++)
	{
		for(int j = 0;j < MAZE_SIZE_Y;j++)
		{
			if(closure[i][j].Center.time < min_time && closure[i][j].Center.determine == False)
			{
				min_pos = SetNodePos(i, j, C_pos);
				min_time = closure[i][j].Center.time;
			}
			if(closure[i][j].North.time < min_time && closure[i][j].North.determine == False)
			{
				min_pos = SetNodePos(i, j, N_pos);
				min_time = closure[i][j].North.time;
			}
			if(closure[i][j].East.time < min_time && closure[i][j].East.determine == False)
			{
				min_pos = SetNodePos(i, j, E_pos);
				min_time = closure[i][j].East.time;
			}
		}
	}
	return min_pos;
}

t_posDijkstra Dijkstra::make_path_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size)
{
	t_posDijkstra min_pos;
	init_dijkstra_map();
	start_node_setUp(conv_t_pos2t_posDijkstra(start_pos, start_wallPos), start_pos.dir);
	for(int i = 0; i < 1000;i++)
	{
		min_pos = min_search();
		//set_determine
		#ifdef DEBUG_MODE
		printf("minimum->%d,%d,%d\n",min_pos.x,min_pos.y,min_pos.NodePos);
		#endif
		set_determine(min_pos);

		if(is_goal_Dijkstra(min_pos, goal_pos, goal_size))
		{
			//last_expand(min_pos,goal_pos,(int)goal_size);
			t_direction pos_dir = (*get_closure_inf(min_pos)).dir;
			min_pos = last_expand(min_pos,pos_dir ,goal_pos, goal_size);
			break;
		}
		expand(min_pos);
	}
	return min_pos;
}

void Dijkstra::expand(t_posDijkstra pos)
{
	t_direction pos_dir = (*get_closure_inf(pos)).dir;
	switch(pos.NodePos)
	{
		case N_pos:
		case E_pos:
			diagonal_expand(pos,pos_dir);
			turn_outR45_expand(pos,pos_dir);
			turn_outL45_expand(pos,pos_dir);
			turn_outR135_expand(pos,pos_dir);
			turn_outL135_expand(pos,pos_dir);
			turn_vR90_expand(pos,pos_dir);
			turn_vL90_expand(pos,pos_dir);
			break;
		case C_pos:
			straight_expand(pos, pos_dir);
			turn_inR135_expand(pos, pos_dir);
			turn_inL135_expand(pos, pos_dir);
			turn_inR45_expand(pos, pos_dir);
			turn_inL45_expand(pos, pos_dir);
			turn_inR45_expand(pos, pos_dir);
			turn_inL45_expand(pos, pos_dir);
			longturn_R90_expand(pos, pos_dir);
			longturn_L90_expand(pos, pos_dir);
			longturn_R180_expand(pos, pos_dir);
			longturn_L180_expand(pos, pos_dir);
			break;
	}
}

t_posDijkstra Dijkstra::last_expand(t_posDijkstra pos,t_direction m_dir,t_position goal_pos,uint8_t goal_size)
{
	t_posDijkstra  last_pos = pos;
	t_direction next_dir = m_dir;
	t_posDijkstra pos1 = pos;
	t_posDijkstra pos2 = pos;
	t_posDijkstra next_pos = pos;
	switch(last_pos.NodePos)
	{
		case N_pos:
		case E_pos:
			pos1 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Front);
			next_pos= pos1;
			pos2 = LocalPosDir2GlobWallPos_WPos(next_pos, next_dir, Front);
			for(int i = 1;; i++)
			{
				int time = (*get_closure_inf(pos)) .time + diagonal_time_set(DIAG_SECTION*i);
				if(get_wall_inf(pos1) == NOWALL && get_wall_inf(pos2) == NOWALL && is_goal_Dijkstra(next_pos, goal_pos, goal_size) == True)
				{
					if((*get_closure_inf(next_pos)) .determine == False )//&& (*get_closure_inf(next_pos)) .time >= time)
					{
						(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Diagonal, False);
						#ifdef DEBUG_MODE
						printf("Diagonal_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
						HAL_Delay(10);
						#endif
					}
				}
				else
				{
					break;
				}
				last_pos = next_pos;
				pos1 = pos2;
				next_pos = pos1;
				pos2 = LocalPosDir2GlobWallPos_WPos(next_pos, next_dir, Front);
			}
			break;
		case C_pos:
			pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
			next_pos = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, None);
			for(int i = 1;; i++)
			{
				int time  = (*get_closure_inf(pos)) .time + straight_time_set(SECTION*i);
				if(get_wall_inf(pos1) == NOWALL && is_goal_Dijkstra(next_pos, goal_pos, goal_size) == True)
				{
						if((*get_closure_inf(next_pos)) .determine == False )
						{
							/*
							if((*get_closure_inf(pos)).run_pt == Straight)
							{
								parent = (*get_closure_inf(pos)).parent_pos;
							}
							*/
							(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Straight, False);
							#ifdef DEBUG_MODE
							printf("Straight_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
							HAL_Delay(10);
							#endif
						}
				}
				else
				{
					break;
				}
				last_pos = next_pos;
				pos1 = LocalPosDir2GlobWallPos_Center(next_pos, m_dir, Front, Rear);
				next_pos = LocalPosDir2GlobWallPos_Center(next_pos, m_dir, Front, None);
			}
			break;
	}
	return last_pos;
}

uint16_t Dijkstra::straight_section_num(t_posDijkstra s_pos,t_posDijkstra e_pos,t_direction dir)
{
	switch(dir)
	{
		case North:
		case South:
			return ABS(s_pos.y - e_pos.y);
		case East:
		case West:
			return ABS(s_pos.x - e_pos.x);
		default:
			return 0;
	}
	return 0;
}

uint16_t Dijkstra::diagonal_section_num(t_posDijkstra s_pos,t_posDijkstra e_pos,t_direction dir)
{
	uint16_t count = 0;
	t_posDijkstra pos = s_pos;
	switch(dir)
	{
		case NorthWest:
		case NorthEast:
		case SouthEast:
		case SouthWest:
			for(count = 1;;count++)
			{
				pos = LocalPosDir2GlobWallPos_WPos(pos, dir, Front);
				if(pos.x == e_pos.x && pos.y == e_pos.y && pos.NodePos == e_pos.NodePos)
				{
					break;
				}
			}
			return count;
		default:
			return 0;
	}
	return count;
}

void Dijkstra::check_run_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size)
{
	t_posDijkstra last_pos = make_path_Dijkstra(start_pos, start_wallPos, goal_pos, goal_size);
	t_posDijkstra tmp_pos = last_pos;
	t_posDijkstra start = conv_t_pos2t_posDijkstra(start_pos, start_wallPos);
	int tail = 0;;
	for(int i = 0;;i++)
	{
		#ifdef DEBUG_MODE
			printf("x:%2d,y:%2d,d:%2d->",tmp_pos.x,tmp_pos.y,tmp_pos.NodePos);
		#endif
		switch((*get_closure_inf(tmp_pos)).run_pt)
		{
			#ifdef DEBUG_MODE
			case No_run: 			printf("No_run\n"); 			break;
			case Straight:	 		printf("Straight\n"); 			break;
			case Diagonal: 			printf("Diagonal\n"); 			break;
			case Long_turnR90: 		printf("Long_turnR90\n"); 		break;
			case Long_turnL90: 		printf("Long_turnL90\n"); 		break;
			case Long_turnR180: 	printf("Long_turnR180\n"); 		break;
			case Long_turnL180: 	printf("Long_turnL180\n"); 		break;
			case Turn_in_R45: 		printf("Turn_in_R45\n"); 		break;
			case Turn_in_L45: 		printf("Turn_in_L45\n"); 		break;
			case Turn_out_R45: 		printf("Turn_out_R45\n"); 		break;
			case Turn_out_L45: 		printf("Turn_out_L45\n"); 		break;
			case Turn_in_R135: 		printf("Turn_in_R135\n"); 		break;
			case Turn_in_L135: 		printf("Turn_in_L135\n"); 		break;
			case Turn_out_R135: 	printf("Turn_out_R135\n"); 		break;
			case Turn_out_L135: 	printf("Turn_out_L135\n"); 		break;
			case Turn_RV90: 		printf("Turn_RV90\n"); 			break;
			case Turn_LV90: 		printf("Turn_LV90\n"); 			break;
			case Diagonal_R: 		printf("Diagonal_R\n"); 		break;
			case Diagonal_L: 		printf("Diagonal_L\n"); 		break;
			case Search_st_section: printf("Search_st_section\n"); 	break;
			case Search_st_half: 	printf("Search_st_half\n"); 	break;
			case Pivot_turn_R: 		printf("Pivot_turn_R\n"); 		break;
			case Pivot_turn_L: 		printf("Pivot_turn_L\n"); 		break;
			case Search_slalom_R: 	printf("Search_slalom_R\n"); 	break;
			case Search_slalom_L: 	printf("Search_slalom_L\n"); 	break;
			case run_brake: 		printf("run_brake\n"); 			break;
			case motor_free: 		printf("motor_free\n"); 		break;
			case Fix_wall: 			printf("Fix_wall\n"); 			break;
			#endif
			default :
				break;
		}
		run_pos_buff[i] = tmp_pos;
		tmp_pos = (*get_closure_inf(tmp_pos)).parent_pos;
		if(tmp_pos.x == start.x && tmp_pos.y == start.y && tmp_pos.NodePos == start.NodePos)
		{
			tail = i;
			break;
		}
	}

	#ifdef DEBUG_MODE
		printf("\nstart\n");
	#endif

	for(int i = tail ; i >= 0;i--)
	{
		//#ifdef DEBUG_MODE
			printf("x:%2d,y:%2d,d:%2d,time:%d->",run_pos_buff[i].x,run_pos_buff[i].y,run_pos_buff[i].NodePos,(*get_closure_inf(run_pos_buff[i])).time);
		//#endif
		switch((*get_closure_inf(run_pos_buff[i])).run_pt)
		{
			//#ifdef DEBUG_MODE
			case No_run: 			printf("No_run\n"); 			break;
			case Straight:
				printf("count->%2d",straight_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir));
				printf("Straight\n"); 			break;
			case Diagonal:
				printf("count->%2d",diagonal_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir));
				printf("Diagonal\n"); 			break;
			case Long_turnR90: 		printf("Long_turnR90\n"); 		break;
			case Long_turnL90: 		printf("Long_turnL90\n"); 		break;
			case Long_turnR180: 	printf("Long_turnR180\n"); 		break;
			case Long_turnL180: 	printf("Long_turnL180\n"); 		break;
			case Turn_in_R45: 		printf("Turn_in_R45\n"); 		break;
			case Turn_in_L45: 		printf("Turn_in_L45\n"); 		break;
			case Turn_out_R45: 		printf("Turn_out_R45\n"); 		break;
			case Turn_out_L45: 		printf("Turn_out_L45\n"); 		break;
			case Turn_in_R135: 		printf("Turn_in_R135\n"); 		break;
			case Turn_in_L135: 		printf("Turn_in_L135\n"); 		break;
			case Turn_out_R135: 	printf("Turn_out_R135\n"); 		break;
			case Turn_out_L135: 	printf("Turn_out_L135\n"); 		break;
			case Turn_RV90: 		printf("Turn_RV90\n"); 			break;
			case Turn_LV90: 		printf("Turn_LV90\n"); 			break;
			case Diagonal_R: 		printf("Diagonal_R\n"); 		break;
			case Diagonal_L: 		printf("Diagonal_L\n"); 		break;
			case Search_st_section: printf("Search_st_section\n"); 	break;
			case Search_st_half: 	printf("Search_st_half\n"); 	break;
			case Pivot_turn_R: 		printf("Pivot_turn_R\n"); 		break;
			case Pivot_turn_L: 		printf("Pivot_turn_L\n"); 		break;
			case Search_slalom_R: 	printf("Search_slalom_R\n"); 	break;
			case Search_slalom_L: 	printf("Search_slalom_L\n"); 	break;
			case run_brake: 		printf("run_brake\n"); 			break;
			case motor_free: 		printf("motor_free\n"); 		break;
			case Fix_wall: 			printf("Fix_wall\n"); 			break;
			//#endif
			default :
				break;
		}
	}
}

void Dijkstra::run_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size,
				  const t_straight_param *const *st_mode,uint16_t size_st_mode,
				  const t_straight_param *const *di_mode,uint16_t size_di_mode,
				  const t_param *const *turn_mode , motion_plan *motionPlan)
{
	turn_time_set(turn_mode);
	st_param_set(st_mode, size_st_mode);
	di_param_set(di_mode, size_di_mode);

	t_posDijkstra last_pos = make_path_Dijkstra(start_pos, start_wallPos, goal_pos, goal_size);
	t_posDijkstra tmp_pos = last_pos;
	t_posDijkstra start = conv_t_pos2t_posDijkstra(start_pos, start_wallPos);
	t_straight_param st_parameter ;

	int tail = 0;
	for(int i = 0;;i++)
	{
		run_pos_buff[i] = tmp_pos;
		tmp_pos = (*get_closure_inf(tmp_pos)).parent_pos;
		if(tmp_pos.x == start.x && tmp_pos.y == start.y && tmp_pos.NodePos == start.NodePos)
		{
			tail = i;
			break;
		}
	}

	motionPlan->search_straight(15.0, straight_base_velo().param->acc, straight_base_velo().param->max_velo, straight_base_velo().param->max_velo);
	while(motion_task::getInstance().run_task !=No_run);
	uint16_t section_count = 0;
	for(int i = tail ; i >= 0;i--)
	{
		switch((*get_closure_inf(run_pos_buff[i])).run_pt)
		{
			//#ifdef DEBUG_MODE
			case No_run: 	break;
			case Straight:
				section_count = straight_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir);
				st_parameter =  calc_end_straight_max_velo(SECTION * section_count);
				if(i == 0)
					motionPlan->straight(  SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, 0.0f);
				else
					motionPlan->straight(  SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, straight_base_velo().param->max_velo);
				break;
			case Diagonal:
				section_count = diagonal_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir);
				st_parameter =  calc_end_diagonal_max_velo(DIAG_SECTION * section_count);
				if(i == 0)
					motionPlan->diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, 0.0f);
				else
					motionPlan->diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, diagonal_base_velo().param->max_velo);
				break;
			case Long_turnR90:
				motionPlan->long_turn(  turn_mode[Long_turnR90],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Long_turnL90:
				motionPlan->long_turn(  turn_mode[Long_turnL90],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Long_turnR180:
				motionPlan->long_turn(  turn_mode[Long_turnR180],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Long_turnL180:
				motionPlan->long_turn(  turn_mode[Long_turnL180],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_in_R45:
				motionPlan->turn_in(  turn_mode[Turn_in_R45],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_in_L45:
				motionPlan->turn_in(  turn_mode[Turn_in_L45],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_out_R45:
				motionPlan->turn_out(  turn_mode[Turn_out_R45],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_out_L45:
				motionPlan->turn_out(  turn_mode[Turn_out_L45],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_in_R135:
				motionPlan->turn_in(  turn_mode[Turn_in_R135],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_in_L135:
				motionPlan->turn_in(  turn_mode[Turn_in_L135],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_out_R135:
				motionPlan->turn_out(  turn_mode[Turn_out_R135],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_out_L135:
				motionPlan->turn_out(  turn_mode[Turn_out_L135],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_RV90:
				motionPlan->turn_v90(  turn_mode[Turn_RV90],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_LV90:
				motionPlan->turn_v90(  turn_mode[Turn_LV90],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Diagonal_R: 		break;
			case Diagonal_L: 		break;
			case Search_st_section: break;
			case Search_st_half: 	break;
			case Pivot_turn_R: 		break;
			case Pivot_turn_L: 		break;
			case Search_slalom_R: 	break;
			case Search_slalom_L: 	break;
			case run_brake: 		break;
			case motor_free: 		break;
			case Fix_wall: 			break;
			//#endif
			default :
				break;
		}
		while(motion_task::getInstance().run_task !=No_run);
	}
}

void Dijkstra::run_Dijkstra_suction(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size,int suction,
									  const t_straight_param *const *st_mode,uint16_t size_st_mode,
									  const t_straight_param *const *di_mode,uint16_t size_di_mode,
									  const t_param *const *turn_mode , motion_plan *motionPlan)
{
	turn_time_set(turn_mode);
	st_param_set(st_mode, size_st_mode);
	di_param_set(di_mode, size_di_mode);

	t_posDijkstra last_pos = make_path_Dijkstra(start_pos, start_wallPos, goal_pos, goal_size);
	t_posDijkstra tmp_pos = last_pos;
	t_posDijkstra start = conv_t_pos2t_posDijkstra(start_pos, start_wallPos);
	t_straight_param st_parameter ;

	int tail = 0;
	for(int i = 0;;i++)
	{
		run_pos_buff[i] = tmp_pos;
		tmp_pos = (*get_closure_inf(tmp_pos)).parent_pos;
		if(tmp_pos.x == start.x && tmp_pos.y == start.y && tmp_pos.NodePos == start.NodePos)
		{
			tail = i;
			break;
		}
	}

	motionPlan->motion_start();
	//motionPlan->fix_wall( suction+200);
	for(int i = 10; i <= suction; i = i + 10)
	{
		FAN_Motor_SetDuty(i);;
		HAL_Delay(10);
	}
	//while(motion_task::getInstance().run_task !=No_run){}
	motionPlan->motion_start( );
	  LogData::getInstance().data_count = 0;
	  LogData::getInstance().log_enable = True;
	motionPlan->straight(  16.10-3.0, straight_base_velo().param->acc, straight_base_velo().param->max_velo, straight_base_velo().param->max_velo);
	while(motion_task::getInstance().run_task !=No_run);
	uint16_t section_count = 0;
	for(int i = tail ; i >= 0;i--)
	{
		switch((*get_closure_inf(run_pos_buff[i])).run_pt)
		{
			//#ifdef DEBUG_MODE
			case No_run: 	break;
			case Straight:
				section_count = straight_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir);
				st_parameter =  calc_end_straight_max_velo(SECTION * section_count);
				if(i == 0)
					motionPlan->straight(  SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, 0.0f);
				else
					motionPlan->straight(  SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, straight_base_velo().param->max_velo);
				break;
			case Diagonal:
				section_count = diagonal_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir);
				st_parameter =  calc_end_diagonal_max_velo(DIAG_SECTION * section_count);
				if(i == 0)
					motionPlan->diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, 0.0f);
				else
					motionPlan->diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, diagonal_base_velo().param->max_velo);
				break;
			case Long_turnR90:
				motionPlan->long_turn(  turn_mode[Long_turnR90],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Long_turnL90:
				motionPlan->long_turn(  turn_mode[Long_turnL90],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Long_turnR180:
				motionPlan->long_turn(  turn_mode[Long_turnR180],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Long_turnL180:
				motionPlan->long_turn(  turn_mode[Long_turnL180],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_in_R45:
				motionPlan->turn_in(  turn_mode[Turn_in_R45],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_in_L45:
				motionPlan->turn_in(  turn_mode[Turn_in_L45],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_out_R45:
				motionPlan->turn_out(  turn_mode[Turn_out_R45],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_out_L45:
				motionPlan->turn_out(  turn_mode[Turn_out_L45],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_in_R135:
				motionPlan->turn_in(  turn_mode[Turn_in_R135],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_in_L135:
				motionPlan->turn_in(  turn_mode[Turn_in_L135],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_out_R135:
				motionPlan->turn_out(  turn_mode[Turn_out_R135],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_out_L135:
				motionPlan->turn_out(  turn_mode[Turn_out_L135],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_RV90:
				motionPlan->turn_v90(  turn_mode[Turn_RV90],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Turn_LV90:
				motionPlan->turn_v90(  turn_mode[Turn_LV90],(*get_closure_inf(run_pos_buff[i])).run_pt);
				break;
			case Diagonal_R: 		break;
			case Diagonal_L: 		break;
			case Search_st_section: break;
			case Search_st_half: 	break;
			case Pivot_turn_R: 		break;
			case Pivot_turn_L: 		break;
			case Search_slalom_R: 	break;
			case Search_slalom_L: 	break;
			case run_brake: 		break;
			case motor_free: 		break;
			case Fix_wall: 			break;
			//#endif
			default :
				break;
		}
		while(motion_task::getInstance().run_task !=No_run);

	}
	  HAL_Delay(200);
	  FAN_Motor_SetDuty(0);;
	  HAL_Delay(200);
}
