/*
 * glob_var.h
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */

#ifndef MODULE_INC_GLOB_VAR_H_
#define MODULE_INC_GLOB_VAR_H_

#include "typedef.h"
#include "macro.h"

#define GLOBAL
#else
#define GLOBAL extern
#endif


GLOBAL uint16_t map[MAZE_SIZE_X][MAZE_SIZE_Y];
GLOBAL t_wall   wall[MAZE_SIZE_X][MAZE_SIZE_Y];



//#endif /* MODULE_INC_GLOB_VAR_H_ */
