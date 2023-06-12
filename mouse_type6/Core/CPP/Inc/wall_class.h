/*
 * wall_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_WALL_CLASS_H_
#define CPP_INC_WALL_CLASS_H_

#include "typedef.h"
#include "macro.h"




class wall_class
{
	public:
		t_wall wall[MAZE_SIZE_X][MAZE_SIZE_Y];
		void set_wall(t_position pos);
};



#endif /* CPP_INC_WALL_CLASS_H_ */
