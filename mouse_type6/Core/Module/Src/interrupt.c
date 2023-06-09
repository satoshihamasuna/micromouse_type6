/*
 * interrupt.c
 *
 *  Created on: Jun 7, 2023
 *      Author: sato1
 */


#include "index.h"

const int   m_dt = 1;
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
	enc_R.prev_sp_pulse = enc_R.sp_pulse;
	enc_L.prev_sp_pulse = enc_L.sp_pulse;

	enc_R.sp_pulse = Encoder_GetPosition_Right();		Encoder_ResetPosition_Right();
	enc_L.sp_pulse = Encoder_GetPosition_Left();		Encoder_ResetPosition_Left();

	enc_R.prev_wheel_speed = enc_R.wheel_speed;
	enc_L.prev_wheel_speed = enc_L.wheel_speed;

	enc_R.wheel_speed =  (float)enc_R.sp_pulse * MMPP * m_dt; //計測はmm mm/ms-> m/s
	enc_L.wheel_speed =  (float)enc_L.sp_pulse * MMPP * m_dt;
}

void Interrupt_Set_Target_Speed()
{

}
