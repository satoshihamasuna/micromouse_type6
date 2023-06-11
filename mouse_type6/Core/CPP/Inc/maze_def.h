/*
 * search.h
 *
 *  Created on: Jun 10, 2023
 *      Author: sato1
 */

#ifndef CPP_INC_MAZE_DEF_H_
#define CPP_INC_MAZE_DEF_H_

#include "iostream"

#define RIGHT	0
#define LEFT	1
#define FRONT	2
#define REAR	3

#define MASK_SEARCH	0x01
#define MASK_SECOND	0x03

#define CONV_SEN2WALL(w)	((w)?WALL:NOWALL)


const uint16_t MAZE_SIZE_X = 32;
const uint16_t MAZE_SIZE_Y = 32;
const uint16_t MAZE_SIZE = (MAZE_SIZE_X*MAZE_SIZE_Y);

const uint16_t MAZE_GOAL_X = 1;
const uint16_t MAZE_GOAL_Y = 1;
const uint16_t MAZE_GOAL_SIZE = 3;

const uint16_t MAP_MAX_VALUE = MAZE_SIZE;

typedef enum{
	UNKNOWN	= 2,
	NOWALL	= 0,
	WALL	= 1,
	VWALL   = 3,
}t_wall_state;

typedef enum{
	front = 0,
	right = 1,
	rear  = 2,
	left  = 3,
}t_local_dir;

typedef enum{
	wall_n = 0,
	wall_e = 1,
	wall_s = 2,
	wall_w = 3,
}t_wall_pos_dir;

typedef enum{
	North 		= 0,
	NorthEast 	= 1,
	East		= 2,
	SouthEast	= 3,
	South		= 4,
	SouthWest   = 5,
	West 		= 6,
	NorthWest 	= 7,
	Dir_None	= 8,
}t_direction;

typedef struct{
	uint8_t x;
	uint8_t y;
	t_direction dir;
}t_position;

typedef struct{
	unsigned char north : 2;
	unsigned char east  : 2;
	unsigned char south : 2;
	unsigned char west  : 2;
}t_wall;

typedef struct{
	int16_t st_x;
	int16_t st_y;
	int16_t cost;
	int16_t cost_h;
}t_MapNode;


#endif /* CPP_INC_MAZE_DEF_H_ */
