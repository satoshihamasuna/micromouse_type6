/*
 * encoder.c
 *
 *  Created on: Jun 4, 2023
 *      Author: sato1
 */


#include "../Include/index.h"
#include "../Include/typedef.h"
#include "../Include/macro.h"

#define ENC_CNT_L 		(TIM2 -> CNT)
#define ENC_CNT_R 		(TIM3 -> CNT)

#define ENC_ZERO		(20000)

float   m_dt = 1.0;

t_encoder enc_R,enc_L;

void set_ms_dt(float _m_dt)
{
	m_dt = _m_dt;
}

void Encoder_Initialize(){
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	Encoder_ResetPosition_Left();
	Encoder_ResetPosition_Right();
}

void Encoder_ResetPosition_Left(){
	ENC_CNT_L = ENC_ZERO;
}

void Encoder_ResetPosition_Right(){
	ENC_CNT_R = ENC_ZERO;
}

uint32_t Encoder_Counts_Left(){
	return (uint32_t)ENC_CNT_L;
}

uint32_t Encoder_Counts_Right(){
	return (uint32_t)ENC_CNT_R;
}

int32_t Encoder_GetPosition_Right(){
	return -((int32_t)ENC_CNT_R - (int32_t)ENC_ZERO);
}

int32_t Encoder_GetPosition_Left(){
	return -((int32_t)ENC_CNT_L - (int32_t)ENC_ZERO);
}

void Encoder_SetSpeed_Right(){
	enc_R.prev_sp_pulse = enc_R.sp_pulse;

	enc_R.sp_pulse = Encoder_GetPosition_Right();		Encoder_ResetPosition_Right();


	enc_R.prev_wheel_speed = enc_R.wheel_speed;

	enc_R.wheel_speed =  (float)enc_R.sp_pulse * MMPP * m_dt; //計測はmm mm/ms-> m/s

}

void Encoder_SetSpeed_Left(){
	enc_L.prev_sp_pulse = enc_L.sp_pulse;

	enc_L.sp_pulse = Encoder_GetPosition_Left();		Encoder_ResetPosition_Left();

	enc_L.prev_wheel_speed = enc_L.wheel_speed;

	enc_L.wheel_speed =  (float)enc_L.sp_pulse * MMPP * m_dt;
}

t_encoder Encoder_GetProperty_Right(){
	return enc_R;
}

t_encoder Encoder_GetProperty_Left(){
	return enc_L;
}
