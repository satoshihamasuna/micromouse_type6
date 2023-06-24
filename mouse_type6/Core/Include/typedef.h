/*
 * typedef.h
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */

#ifndef MODULE_INC_TYPEDEF_H_
#define MODULE_INC_TYPEDEF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

typedef enum{
	True   = 1,
	False = 0,
}t_bool;

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
	Turn_None 	= 0,
	Turn_R 		= 1,
	Turn_L		= 2,
}t_turn_dir;

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


typedef struct{
	float velo;
	float accel;
	float length;
	float rad_accel;
	float rad_velo;
	float radian;
	float x_point;
	float y_point;
}t_machine_param;

typedef struct{
	float velo;
	float max_velo;
	float end_velo;
	float accel;
	float deccel;
	float length;
	float rad_accel;
	float rad_deccel;
	float rad_max_velo;
	float radian;
	t_turn_dir turn_d;
}t_motion_param;

typedef struct{
	float Kp;
	float Ki;
	float Kd;
}t_pid_gain;

typedef struct{
	int32_t sp_pulse;
	int32_t prev_sp_pulse;
	float 	wheel_speed;
	float	prev_wheel_speed;
}t_encoder;

typedef struct{
	int16_t value;
	t_bool is_wall;
	float distance;
}t_sensor;


typedef enum{
	sensor_fl = 3,
	sensor_fr = 4,
	sensor_sl = 1,
	sensor_sr = 2,
}t_sensor_dir;

typedef enum{
	x_axis = 0,
	y_axis = 1,
	z_axis = 2,
}t_axis;



#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_TYPEDEF_H_ */

