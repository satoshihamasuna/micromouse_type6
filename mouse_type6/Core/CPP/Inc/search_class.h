/*
 * search_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_SEARCH_CLASS_H_
#define CPP_INC_SEARCH_CLASS_H_


#include "wall_class.h"
#include "make_map_class.h"
#include "motion.h"
#include "adachi_class.h"

class Search
{
	private:
		wall_class *wall_property;
		make_map   *map_property;
		motion_plan *motion_planning;
	public:
		void search_adachi_1(wall_class *_wall,make_map *_map,motion_plan *_motion);

};


#endif /* CPP_INC_SEARCH_CLASS_H_ */
