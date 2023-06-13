/*
 * sensing_task.cpp
 *
 *  Created on: 2023/06/14
 *      Author: sato1
 */



#include "sensing_task.h"
#include "typedef.h"
#include "index.h"

t_wall_state SensingTask::conv_Sensin2Wall(t_sensor_dir sens_dir)
{
	switch(sens_dir){
		case sensor_fl:
			return ((sen_fl.is_wall)?WALL:NOWALL);
		case sensor_fr:
			return ((sen_fr.is_wall)?WALL:NOWALL);
		case sensor_sl:
			return ((sen_l.is_wall)?WALL:NOWALL);
		case sensor_sr:
			return ((sen_r.is_wall)?WALL:NOWALL);
		default :
			return NOWALL;
	}
}

void SensingTask::IrSensorSet()
{
	sen_fl.value =  Sensor_GetValue(sensor_fl);
	sen_fr.value =  Sensor_GetValue(sensor_fr);
	sen_l.value  =  Sensor_GetValue(sensor_sl);
	sen_r.value  =  Sensor_GetValue(sensor_sr);
}

int16_t SensingTask::IrSensor_Avg()
{
	return (sen_fl.value + sen_fr.value + sen_l.value + sen_r.value)/4 ;
}
