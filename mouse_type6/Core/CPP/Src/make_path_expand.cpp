/*
 * make_path_expand.cpp
 *
 *  Created on: 2023/06/30
 *      Author: sato1
 */

#include "queue_class.h"
#include "make_map_class.h"
#include "make_path.h"
#include "typedef.h"
#include "index.h"

#define DIR_TURN_NEWS_R90(x) ((x + 1 + 4) % 4)
#define DIR_TURN_NEWS_L90(x) ((x - 1 + 4) % 4)
#define DIR_TURN_NEWS_R45(x) ((x + 4 + 8) % 8)
#define DIR_TURN_NEWS_L45(x) (((x + 3) % 4 + 4) % 8)
#define DIR_TURN_NEWS_R135(x) (DIR_TURN_NEWS_R45(DIR_TURN_NEWS_R90(x)))
#define DIR_TURN_NEWS_L135(x) (DIR_TURN_NEWS_L45(DIR_TURN_NEWS_L90(x)))
#define DIR_TURN_NEWS_R180(x) (DIR_TURN_NEWS_R90(DIR_TURN_NEWS_R90(x)))
#define DIR_TURN_NEWS_L180(x) (DIR_TURN_NEWS_L90(DIR_TURN_NEWS_L90(x)))

#define DIR_TURN_DIAG_R45(x) ((x - 3 + 4)%4)
#define DIR_TURN_DIAG_L45(x) ((x - 4 + 4)%4)
#define DIR_TURN_DIAG_R90(x) ((x + 1 + 8) % 8)
#define DIR_TURN_DIAG_L90(x) ((x - 1 + 8) % 8)
#define DIR_TURN_DIAG_R135(x) (DIR_TURN_NEWS_R90((DIR_TURN_DIAG_R45(x))))
#define DIR_TURN_DIAG_L135(x) (DIR_TURN_NEWS_L90((DIR_TURN_DIAG_L45(x))))
#define DIR_TURN_DIAG_R180(x) (DIR_TURN_DIAG_R90((DIR_TURN_DIAG_R90(x))))
#define DIR_TURN_DIAG_L180(x) (DIR_TURN_DIAG_L90((DIR_TURN_DIAG_L90(x))))


t_element* Dijkstra::get_closure_inf(t_posDijkstra position)
{
	switch(position.NodePos)
	{
		case N_pos:
			return &(closure[position.x][position.y].North);
		case C_pos:
			return &(closure[position.x][position.y].Center);
		case E_pos:
			return &(closure[position.x][position.y].East);
	}
	return nullptr;
}

void Dijkstra::straight_expand(t_posDijkstra pos,t_direction m_dir)
{

}

void Dijkstra::diagonal_expand(t_posDijkstra pos,t_direction m_dir)
{

}

void Dijkstra::turn_inR45_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_R45((int)m_dir);

}

void Dijkstra::turn_inL45_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_L45((int)m_dir);

}

void Dijkstra::turn_outR45_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_R45((int)m_dir);
}

void Dijkstra::turn_outL45_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_L45((int)m_dir);
}

void Dijkstra::turn_inR135_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_R135((int)m_dir);
}

void Dijkstra::turn_inL135_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_L135((int)m_dir);
}

void Dijkstra::turn_outR135_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_R135((int)m_dir);
}

void Dijkstra::turn_outL135_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_L135((int)m_dir);
}

void Dijkstra::longturn_R90_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_R90((int)m_dir);
}

void Dijkstra::longturn_L90_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_L90((int)m_dir);
}

void Dijkstra::longturn_R180_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_R180((int)m_dir);
}

void Dijkstra::longturn_L180_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_L180((int)m_dir);
}

void Dijkstra::turn_vR90_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_R90((int)m_dir);
}

void Dijkstra::turn_vL90_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_L90((int)m_dir);
}