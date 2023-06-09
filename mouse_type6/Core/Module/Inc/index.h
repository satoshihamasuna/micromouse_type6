/*
 * index.h
 *
 *  Created on: 2023/06/03
 *      Author: sato1
 */

#ifndef MODULE_INC_INDEX_H_
#define MODULE_INC_INDEX_H_

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
#include "lsm6dsr_reg.h"
#include "typedef.h"
#include "macro.h"


void CPP_main();
t_encoder enc_R,enc_L;
uint8_t mouse_mode;
uint8_t is_mode_enable;

uint8_t run_mode;
t_sp_param machine;
t_sp_param target;
t_sp_param max_set;

void Indicate_LED(uint8_t led_num);
//interface
void Mode_Change_ENC();

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
void Interrupt_PreProcess();
void Interrupt_Main();
void Interrupt_PostProcess();
void Interrupt_Get_Irsensor();
void Interrupt_Get_Speed();
void Interrupt_Set_Target_Speed();
#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_INDEX_H_ */
