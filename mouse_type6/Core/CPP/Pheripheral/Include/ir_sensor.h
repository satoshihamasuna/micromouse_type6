/*
 * ir_sensor.h
 *
 *  Created on: 2024/08/10
 *      Author: sato1
 */

#ifndef CPP_PHERIPHERAL_INCLUDE_IR_SENSOR_H_
#define CPP_PHERIPHERAL_INCLUDE_IR_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef  enum {
	LED_FL_ON 	= 2,
	LED_FL_OFF 	= 3,
	LED_SL_ON 	= 6,
	LED_SL_OFF 	= 7,
	LED_SR_ON 	= 4,
	LED_SR_OFF 	= 5,
	LED_FR_ON 	= 0,
	LED_FR_OFF 	= 1,
}t_sensor_mode;

typedef enum{
	sensor_fl = 3,
	sensor_fr = 4,
	sensor_sl = 1,
	sensor_sr = 2,
}t_sensor_dir;

#ifdef __cplusplus
}
#endif

#endif /* CPP_PHERIPHERAL_INCLUDE_IR_SENSOR_H_ */
