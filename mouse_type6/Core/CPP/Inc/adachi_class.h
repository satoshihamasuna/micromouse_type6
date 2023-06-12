/*
 * adachi_class.h
 *
 *  Created on: Jun 12, 2023
 *      Author: sato1
 */

#ifndef CPP_INC_ADACHI_CLASS_H_
#define CPP_INC_ADACHI_CLASS_H_

#include "make_map_class.h"
#include "wall_class.h"
#include <stdio.h>
#include <macro.h>

class adachi{
	public:
		int32_t priority();
		int get_next_dir(t_wall wall_property[MAZE_SIZE_X][MAZE_SIZE_Y],uint16_t map_property[MAZE_SIZE_X][MAZE_SIZE_Y]);
};


#endif /* CPP_INC_ADACHI_CLASS_H_ */
