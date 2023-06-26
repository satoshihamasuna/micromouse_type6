/*
 * sensing_task.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_SENSING_TASK_H_
#define CPP_INC_SENSING_TASK_H_

#include "singleton.h"
#include "typedef.h"

#define SIDE_THRESHOLD		(65.0)
#define FRONT_THRESHOLD		(120.0)

class SensingTask:public Singleton<SensingTask>
{
	private:
		float Sensor_CalcDistance(t_sensor_dir dir,int16_t value);
	public:
		t_sensor sen_fr,sen_fl,sen_r,sen_l;
		t_wall_state conv_Sensin2Wall(t_sensor_dir sens_dir);
		void IrSensorSet();
		void IrSensorDistanceSet();
		int16_t IrSensor_Avg();
		void IrSensorWallSet();
		void SetWallControll_RadVelo(t_machine_param *target_,float delta_tms);
};


#endif /* CPP_INC_SENSING_TASK_H_ */
