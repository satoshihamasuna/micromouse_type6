/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "index.h"
#include "glob_var.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  imu_initialize();
  Sensor_Initialize();
  Motor_Initialize();
  FAN_Motor_Initialize();
  Encoder_Initialize();
  Interrupt_Initialize();
  IMU_read_DMA_Start();
  mouse_mode = 0x00;
  is_mode_enable = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  uint8_t who_am_i = 0;
	  who_am_i = read_byte(WHO_AM_I);
	  //printf("who_am_I:%d,Renc:%ld,Lenc:%ld\n",who_am_i,Encoder_Counts_Right(),Encoder_Counts_Left());
	  printf("%d,%d,%d,%d,",ADC_get_value(0),ADC_get_value(1),ADC_get_value(2),ADC_get_value(3));
	  printf("%d,%d,%d,%d\n",ADC_get_value(4),ADC_get_value(5),ADC_get_value(6),ADC_get_value(7));
	  HAL_Delay(10);
	  int16_t test = 0;
		*/

	  Mode_Change_ENC();
	  HAL_Delay(50);
	  switch(is_mode_enable|mouse_mode){
	  	  case (ENABLE_MODE3|0x00):
	  			  printf("gyro:%lf\n",read_gyro_z_axis());
	  			  break;
	  	  case (ENABLE_MODE3|0x01):
	  			  FAN_Motor_SetDuty(500);
	  	  	  	  HAL_Delay(1500);
	  	  	      FAN_Motor_SetDuty(0);
	  	  	      HAL_Delay(100);
	  	  	  	  is_mode_enable = DISABLE;
	  			  break;
	  	  case (ENABLE_MODE3|0x02):
	  			  break;
	  	  case (ENABLE_MODE3|0x03):
	  			  break;
	  	  case (ENABLE_MODE3|0x04):
	  			  break;
	  	  case (ENABLE_MODE3|0x05):
	  			  break;
	  	  case (ENABLE_MODE3|0x06):
	  			  break;
	  	  case (ENABLE_MODE3|0x07):
	  			  break;
	  	  case (ENABLE_MODE3|0x08):
	  			  break;
	  	  case (ENABLE_MODE3|0x09):
	  			  break;
	  	  case (ENABLE_MODE3|0x0A):
	  			  break;
	  	  case (ENABLE_MODE3|0x0B):
	  			  break;
	  	  case (ENABLE_MODE3|0x0C):
	  			  break;
	  	  case (ENABLE_MODE3|0x0D):
	  			  break;
	  	  case (ENABLE_MODE3|0x0E):
	  			  break;
	  	  case (ENABLE_MODE3|0x0F):
	  			  break;
	  }

//	  if(mouse_mode == 0x00 && is_mode_enable == true) Sensor_StopADC();

	  //if(i == 2)write_byte(CTRL2_G, GYRO_ODR_SET|GYRO_2000_DPS);
	  /*read_byte(WHO_AM_I);
	   * */

	  //printf("%d\n",read_byte(CTRL2_G));
	  //HAL_Delay(200);*/
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,10);
  return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
