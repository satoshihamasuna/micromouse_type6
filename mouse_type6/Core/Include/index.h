/*
 * index.h
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */

#ifndef CPP_INC_INDEX_H_
#define CPP_INC_INDEX_H_



#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "typedef.h"
#include "lsm6dsr_reg.h"
#include "macro.h"
//#include "../CPP/machine_var.h"

void CPP_Main();

void Indicate_LED(uint8_t led_num);
//interface
void Mode_Change_ENC();
void Mode_Init();
void Mode_Disable();
uint8_t Mode_State();

//imu
uint8_t read_byte(uint8_t reg);
void write_byte(uint8_t reg, uint8_t data);
void imu_initialize();
void IMU_read_DMA_Start();
float read_gyro_x_axis();
float read_gyro_y_axis();
float read_gyro_z_axis();
float read_accel_x_axis();
float read_accel_y_axis();
float read_accel_z_axis();


//encoder
void Encoder_Initialize();
void Encoder_ResetPosition_Left();
void Encoder_ResetPosition_Right();
uint32_t Encoder_Counts_Left();
uint32_t Encoder_Counts_Right();
int32_t Encoder_GetPosition_Right();
int32_t Encoder_GetPosition_Left();
void Encoder_SetSpeed_Right();
void Encoder_SetSpeed_Left();
t_encoder Encoder_GetProperty_Right();
t_encoder Encoder_GetProperty_Left();


//ir_sensor
void Sensor_Initialize();
void Sensor_TurnOnLED();
void Sensor_TurnOffLED();
int16_t ADC_get_value(int num);
int16_t Sensor_GetValue(t_sensor_dir dir);

void Sensor_StopADC();
//motor
void Motor_Initialize();
void Motor_Stop();
void Motor_SetDuty_Left( int16_t duty_l );
void Motor_SetDuty_Right( int16_t duty_r );
void Motor_SetDuty_Left( int16_t duty_l );
void FAN_Motor_Initialize();
void FAN_Motor_SetDuty(int16_t duty_f);
//interrupt
void Interrupt_Initialize();

void goal_set_vwall(int *gx,int *gy,int goal_size);
void goal_clear_vwall(int *gx,int *gy,int goal_size);
t_bool i_am_goal(t_position my_pos,int *gx,int *gy,int goal_size);

#ifdef __cplusplus
}
#endif


#endif /* CPP_INC_INDEX_H_ */
