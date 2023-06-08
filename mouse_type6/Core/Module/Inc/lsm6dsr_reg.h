/*
 * lsm6dsr_reg.h
 *
 *  Created on: Jun 3, 2023
 *      Author: sato1
 */

#ifndef MODULE_INC_LSM6DSR_REG_H_
#define MODULE_INC_LSM6DSR_REG_H_

#define WHO_AM_I 0x0f	//RETURN Value is 0x6B

//gyro output register
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

//accel output register
#define OUTX_L_A 0x28
#define OUTX_H_A 0x29
#define OUTY_L_A 0x2A
#define OUTY_H_A 0x2B
#define OUTZ_L_A 0x2C
#define OUTZ_H_A 0x2D

//accelerometer control register
#define CTRL1_XL 0x10
//gyroscope control register
#define CTRL2_G  0x11

#define GYRO_ODR_SET 0x80
#define GYRO_2000_DPS 0x0C
#define GYRO_4000_DPS 0x01

#endif /* MODULE_INC_LSM6DSR_REG_H_ */
