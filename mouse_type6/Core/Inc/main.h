/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Button_Pin GPIO_PIN_13
#define Button_GPIO_Port GPIOC
#define battery_Pin GPIO_PIN_0
#define battery_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOA
#define sens_fr_Pin GPIO_PIN_2
#define sens_fr_GPIO_Port GPIOA
#define photo_fr_Pin GPIO_PIN_4
#define photo_fr_GPIO_Port GPIOA
#define photo_sr_Pin GPIO_PIN_5
#define photo_sr_GPIO_Port GPIOA
#define photo_sl_Pin GPIO_PIN_6
#define photo_sl_GPIO_Port GPIOA
#define photo_fl_Pin GPIO_PIN_7
#define photo_fl_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOB
#define NSS_Pin GPIO_PIN_12
#define NSS_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_11
#define LED6_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_12
#define LED5_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_13
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
