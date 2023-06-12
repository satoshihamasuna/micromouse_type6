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

class adachi{
	public:
		void priority();
		void get_next_dir(wall_class *wall_property,make_map *map_property);
};


#endif /* CPP_INC_ADACHI_CLASS_H_ */
