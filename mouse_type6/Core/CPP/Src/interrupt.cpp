/*
 * interrupt.c
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */


#include "index.h"
#include "interrupt.h"
#include "sensing_task.h"
#include "controll.h"
#include "motion_plan.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim5){
    	Interrupt::getInstance().preprocess();
    	Interrupt::getInstance().main();
    	Interrupt::getInstance().postprocess();
    }
}

void Interrupt_Initialize(){
	HAL_TIM_Base_Start_IT(&htim5);
}

void Interrupt::preprocess(){
	Encoder_SetSpeed_Left();	Encoder_SetSpeed_Right();
	SensingTask::getInstance().IrSensorSet();
	//Interrupt_Set_Target_Speed();
}

void Interrupt::main()
{
	//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void Interrupt::postprocess()
{
	IMU_read_DMA_Start();
}

