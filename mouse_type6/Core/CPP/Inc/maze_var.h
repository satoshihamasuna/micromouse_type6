/*
 * maze_var.h
 *
 *  Created on: Jun 10, 2023
 *      Author: sato1
 */

#ifndef CPP_INC_MAZE_VAR_H_
#define CPP_INC_MAZE_VAR_H_

#include "maze_def.h"
#include "queue.h"

extern t_wall 		wall[MAZE_SIZE_X][MAZE_SIZE_Y];
extern uint16_t 	map[MAZE_SIZE_X][MAZE_SIZE_Y];
extern t_position   mypos;

#endif /* CPP_INC_MAZE_VAR_H_ */
