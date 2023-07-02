/*
 * sensing_task.cpp
 *
 *  Created on: 2023/06/14
 *      Author: sato1
 */



#include "sensing_task.h"
#include "sens_table.h"
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
	IrSensorDistanceSet();
	IrSensorWallSet();
}

float SensingTask::Sensor_CalcDistance(t_sensor_dir dir,int16_t value)
{
	float distance = 0.0f;
	int array_length = 0;
	int count = 0;
	float m,n;
	switch(dir)
	{
		case sensor_fr:
			array_length = sizeof(sens_front_length_table) / sizeof(uint16_t);
			if(value >= sens_fr_table[0]) distance = (float)sens_front_length_table[0];
			else if (value <= sens_fr_table[array_length-1]) distance = (float)sens_front_length_table[array_length-1];
			else{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_fr_table[count] && value > sens_fr_table[count+1]) break;
				}
				m = (float)(sens_fr_table[count] - value);
				n = (float)(value - sens_fr_table[count+1]);
				distance = (n*(float)sens_front_length_table[count] + m*(float)sens_front_length_table[count+1])/(m+n);
			}
			break;
		case sensor_fl:
			array_length = sizeof(sens_front_length_table) / sizeof(uint16_t);
			if(value >= sens_fl_table[0]) distance = (float)sens_front_length_table[0];
			else if (value <= sens_fl_table[array_length-1]) distance = (float)sens_front_length_table[array_length-1];
			else
			{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_fl_table[count] && value > sens_fl_table[count+1]) break;
				}
				m = (float)(sens_fl_table[count] - value);
				n = (float)(value - sens_fl_table[count+1]);
				distance = (n*(float)sens_front_length_table[count] + m*(float)sens_front_length_table[count+1])/(m+n);
			}
			break;
		case sensor_sr:
			array_length = sizeof(sens_side_length_table) / sizeof(uint16_t);
			if(value >= sens_sr_table[0]) distance = (float)sens_side_length_table[0];
			else if (value <= sens_sr_table[array_length-1]) distance = (float)sens_side_length_table[array_length-1];
			else
			{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_sr_table[count] && value > sens_sr_table[count+1]) break;
				}
				m = (float)(sens_sr_table[count] - value);
				n = (float)(value - sens_sr_table[count+1]);
				distance = (n*(float)sens_side_length_table[count] + m*(float)sens_side_length_table[count+1])/(m+n);
			}
			break;
		case sensor_sl:
			array_length = sizeof(sens_side_length_table) / sizeof(uint16_t);
			if(value >= sens_sl_table[0]) distance = (float)sens_side_length_table[0];
			else if (value <= sens_sl_table[array_length-1]) distance = (float)sens_side_length_table[array_length-1];
			else
			{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_sl_table[count] && value > sens_sl_table[count+1]) break;
				}
				m = (float)(sens_sl_table[count] - value);
				n = (float)(value - sens_sl_table[count+1]);
				distance = (n*(float)sens_side_length_table[count] + m*(float)sens_side_length_table[count+1])/(m+n);
			}
			break;
	}
	return distance;
}


void SensingTask::IrSensorDistanceSet()
{
	sen_fl.distance = Sensor_CalcDistance(sensor_fl,sen_fl.value);
	sen_fr.distance = Sensor_CalcDistance(sensor_fr,sen_fr.value);
	sen_l.distance = Sensor_CalcDistance(sensor_sl,sen_l.value);
	sen_r.distance = Sensor_CalcDistance(sensor_sr,sen_r.value);
}

void SensingTask::IrSensorWallSet()
{
	sen_fr.is_wall 	= (sen_fr.distance <= FRONT_THRESHOLD)? True:False;
	sen_fl.is_wall 	= (sen_fl.distance <= FRONT_THRESHOLD)? True:False;
	sen_r.is_wall 	= (sen_r.distance <= SIDE_THRESHOLD)? True:False;
	sen_l.is_wall 	= (sen_l.distance <= SIDE_THRESHOLD)? True:False;

	sen_fr.controll_cnt = (sen_fr.is_wall == True) ? sen_fr.controll_cnt + 1 : 0;
	sen_fl.controll_cnt = (sen_fl.is_wall == True) ? sen_fl.controll_cnt + 1 : 0;
	sen_r.controll_cnt = (sen_r.is_wall == True) ? sen_r.controll_cnt + 1 : 0;
	sen_l.controll_cnt = (sen_l.is_wall == True) ? sen_l.controll_cnt + 1 : 0;

	sen_fr.controll_th = (sen_fr.controll_cnt > 10) ? FRONT_THRESHOLD : 90.0;
	sen_fl.controll_th = (sen_fl.controll_cnt > 10) ? FRONT_THRESHOLD : 90.0;
	sen_r.controll_th = (sen_r.controll_cnt > 10) ? SIDE_THRESHOLD: 45.0;
	sen_l.controll_th = (sen_l.controll_cnt > 10) ? SIDE_THRESHOLD: 45.0;

	sen_r.is_controll 	= (sen_r.is_wall == True && sen_r.distance <= sen_r.controll_th)? True:False;
	sen_l.is_controll 	= (sen_l.is_wall == True && sen_l.distance <= sen_l.controll_th)? True:False;


	sen_r.is_controll 	= (sen_fr.distance <= 80.0)? False:sen_r.is_controll;
	sen_l.is_controll 	= (sen_fl.distance <= 80.0)? False:sen_l.is_controll;

	sen_r.error	= (sen_r.is_controll == True) ? sen_r.distance - 45.0 : 0.0;
	sen_l.error	= (sen_l.is_controll == True) ? sen_l.distance - 45.0 : 0.0;

}

void SensingTask::SetWallControll_RadVelo(t_machine_param *target_,float delta_tms)
{
	float ir_rad_acc_controll = 0.0;
	if(sen_r.is_controll == True && sen_l.is_controll == True)
	{
		ir_rad_acc_controll = (sen_l.error - sen_r.error)/2.0;
	}
	else
	{
		ir_rad_acc_controll = (sen_l.error - sen_r.error);
	}

	target_->rad_accel = (3.0)*ir_rad_acc_controll-(target_->rad_velo*30.0);
	target_->rad_velo = target_->rad_velo + target_->rad_accel*delta_tms/1000.0f;
}

int16_t SensingTask::IrSensor_Avg()
{
	return (sen_fl.value + sen_fr.value + sen_l.value + sen_r.value)/4 ;
}
