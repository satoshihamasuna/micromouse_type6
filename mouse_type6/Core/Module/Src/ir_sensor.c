/*
 * ir_sensor.c
 *
 *  Created on: Jun 4, 2023
 *      Author: sato1
 */

#include "index.h"
#include "glob_var_machine.h"
#include "tim.h"
#include "dma.h"
#include "stm32f4xx_hal.h"

#define NUM_ADC				(10)
#define GET_ADC_DATA(x)		adc_value[x-1]

#define	SENSOR_ALL_PATTERN		((photo_sr_Pin|photo_sl_Pin|photo_fr_Pin|photo_fl_Pin))
//#define	SENSOR_ALL_PATTERN		((LED1_Pin|LED4_Pin))

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

static uint32_t		led_on_pattern[NUM_ADC]  	= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};				// LED点灯コマンド

//static uint16_t		led_off_pattern[NUM_ADC] 	= {0x0000, 0x0000, 0x0000, 0x0000, 0x0000,0x0000, 0x0000, 0x0000, 0x0000, 0x0000};	// LED消灯コマンド


static uint32_t		led_off_pattern[] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
										 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};

static uint16_t		adc_value[NUM_ADC];		// AD変換値

void Sensor_TurnOffLED()
{
	for( int8_t i = 0; i < NUM_ADC; i++ ) {
		led_on_pattern[i] = 0x00000000;
		led_off_pattern[i] = (uint32_t)SENSOR_ALL_PATTERN << 16;
	}
}

void Sensor_TurnOnLED()
{
	Sensor_TurnOffLED();
	led_on_pattern[LED_SL_ON] = photo_sl_Pin;
	led_on_pattern[LED_SR_ON] = photo_sr_Pin;
	led_on_pattern[LED_FL_ON] = photo_fl_Pin;
	led_on_pattern[LED_FR_ON] = photo_fr_Pin;
	//led_on_pattern[8] = (uint32_t)LED1_Pin;
	//led_on_pattern[9] = (uint32_t)LED4_Pin;
}

void Sensor_Initialize()
{
	htim1.Instance->DIER |= TIM_DIER_CC2DE | TIM_DIER_CC4DE;
	//__HAL_TIM_MOE_ENABLE(&htim1);
	//__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	Sensor_TurnOnLED();
	HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC2], (uint32_t)led_on_pattern,  (uint32_t)(&(GPIOA->BSRR)), NUM_ADC);//ODR
	HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC4], (uint32_t)led_off_pattern, (uint32_t)(&(GPIOA->BSRR)), NUM_ADC);//OR

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_value, NUM_ADC);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}


void Sensor_StopADC()
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	HAL_GPIO_WritePin(GPIOA, SENSOR_ALL_PATTERN, GPIO_PIN_RESET);
}

int16_t ADC_get_value(int num)
{
	return adc_value[num];
}

int16_t Sensor_GetValue(t_sensor_dir dir)
{
	switch(dir)
	{
		case sensor_fl:
			return ((int16_t)adc_value[LED_FL_ON] - (int16_t)adc_value[LED_FL_OFF]);
			break;
		case sensor_fr:
			return ((int16_t)adc_value[LED_FR_ON] - (int16_t)adc_value[LED_FR_OFF]);
			break;
		case sensor_sr:
			return ((int16_t)adc_value[LED_SR_ON] - (int16_t)adc_value[LED_SR_OFF]);
			break;
		case sensor_sl:
			return ((int16_t)adc_value[LED_SL_ON] - (int16_t)adc_value[LED_SL_OFF]);
			break;
	}
	return 0;
}

t_bool Sensor_is_wall(t_sensor_dir dir)
{
	switch(dir)
	{
		case sensor_fl:
			return sen_fl.is_wall;
			break;
		case sensor_fr:
			return sen_fr.is_wall;
			break;
		case sensor_sr:
			return sen_r.is_wall;
			break;
		case sensor_sl:
			return sen_l.is_wall;
			break;
	}
	return 0;
}

