/*
 * indicator.c
 *
 *  Created on: 2023/06/03
 *      Author: sato1
 */

#include "index.h"

void Indicate_LED(uint8_t led_num)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (t_bool)(led_num >> 4)&0x01);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (t_bool)(led_num >> 5)&0x01);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (t_bool)(led_num >> 0)&0x01);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (t_bool)(led_num >> 1)&0x01);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, (t_bool)(led_num >> 2)&0x01);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, (t_bool)(led_num >> 3)&0x01);
}
