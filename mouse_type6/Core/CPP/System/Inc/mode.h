/*
 * mode_class.h
 *
 *  Created on: 2023/07/21
 *      Author: sato1
 */

#ifndef CPP_INC_MODE_H_
#define CPP_INC_MODE_H_

#include "../../Pheripheral/Include/index.h"
#include "../../Pheripheral/Include/typedef.h"


namespace Mode
{
	void Demo();
	void Demo2();
	void Debug(const  t_straight_param *st_param,const t_param *const *turn_mode,int suction);
	void Debug2(const  t_straight_param *st_param,const t_param *const *turn_mode,int suction);
	void Debug3();
	void Interface_Check();
	uint8_t Select(uint8_t _param,uint8_t max,t_encoder enc);
	void Select_Mode();
	void LED_Toggle(uint8_t led_num,uint32_t period_time);
	void indicate_error();
}


#endif /* CPP_INC_MODE_H_ */
