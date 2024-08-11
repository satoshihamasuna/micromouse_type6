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
#include "macro.h"
#include "ir_sensor.h"
//#include "maze_typedef.h"
#include "../../Component/Inc/maze_typedef.h"

typedef enum{
	True   = 1,
	False = 0,
}t_bool;

typedef struct{
	int32_t sp_pulse;
	int32_t prev_sp_pulse;
	float 	wheel_speed;
	float	prev_wheel_speed;
	int32_t buff[ACC_BUFF_SIZE];
	int32_t sum;
	int cnt;
}t_encoder;



typedef enum{
	x_axis = 0,
	y_axis = 1,
	z_axis = 2,
}t_axis;



#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_TYPEDEF_H_ */

