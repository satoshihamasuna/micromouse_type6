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
#include <../../Module/Include/macro.h>


class adachi{
	private:
		wall_class *wall_property;
		make_map   *map_property;
	public:
		adachi(wall_class *_wall_class,make_map *_make_map_class)
		{
			wall_property 	= _wall_class;
			map_property 	= _make_map_class;
		}
		int get_priority(t_position mypos,t_position next_pos);
		int get_next_dir(t_position mypos,int mask,t_position *glob_next_pos);

};

#endif /* CPP_INC_ADACHI_CLASS_H_ */
