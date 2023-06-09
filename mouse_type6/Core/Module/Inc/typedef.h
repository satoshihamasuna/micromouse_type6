/*
 * typedef.h
 *
 *  Created on: Jun 6, 2023
 *      Author: sato1
 */

#include "index.h"
#include "macro.h"


#ifndef MODULE_INC_TYPEDEF_H_
#define MODULE_INC_TYPEDEF_H_

typedef enum{
	 false = 0,
	 true  = 1,
}t_bool;

typedef struct{
	int32_t sp_pulse;
	int32_t prev_sp_pulse;
	float 	wheel_speed;
	float	prev_wheel_speed;
}t_encoder;

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

typedef struct{
	float velo;
	float prev_velo;
	float I_velo;
	float accel;
	float prev_accel;
	float rad_velo;
	float prev_rad_velo;
	float I_rad_velo;
	float rad_accel;
	float prev_rad_accel;
	float length;
	float radian;
}t_sp_param;

typedef struct{
	int16_t st_x;
	int16_t st_y;
	int16_t cost;
	int16_t cost_h;
}t_MapNode;

#endif /* MODULE_INC_TYPEDEF_H_ */
