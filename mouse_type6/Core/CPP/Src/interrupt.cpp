/*
 * interrupt.c
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */


#include "index.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == &htim5){
        Interrupt_PreProcess();
        Interrupt_Main();
        Interrupt_PostProcess();
    }
}

void Interrupt_Initialize(){
	HAL_TIM_Base_Start_IT(&htim5);
}

void Interrupt_PreProcess(){
	//get & calc encoder pulse
	//Interrupt_Get_Irsensor();
	Interrupt_Get_Speed();
	//Interrupt_Set_Target_Speed();
}

void Interrupt_Main()
{
	//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void Interrupt_PostProcess()
{
	IMU_read_DMA_Start();
}

void Interrupt_Get_Irsensor()
{

}

void Interrupt_Get_Speed()
{

	Encoder_SetSpeed_Left();
	Encoder_SetSpeed_Right();

}

void Interrupt_Set_Target_Speed()
{

}

