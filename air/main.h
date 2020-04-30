/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern uint32_t g_ADCValue[1];
extern int adcInt;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PA0_Pin GPIO_PIN_0
#define PA0_GPIO_Port GPIOA
#define led_g1_Pin GPIO_PIN_2
#define led_g1_GPIO_Port GPIOA
#define led_g2_Pin GPIO_PIN_4
#define led_g2_GPIO_Port GPIOA
#define led_g3_Pin GPIO_PIN_6
#define led_g3_GPIO_Port GPIOA
#define led_g4_Pin GPIO_PIN_4
#define led_g4_GPIO_Port GPIOC
#define led_y1_Pin GPIO_PIN_2
#define led_y1_GPIO_Port GPIOB
#define led_y2_Pin GPIO_PIN_8
#define led_y2_GPIO_Port GPIOE
#define led_y3_Pin GPIO_PIN_10
#define led_y3_GPIO_Port GPIOE
#define led_y4_Pin GPIO_PIN_12
#define led_y4_GPIO_Port GPIOE
#define led_r1_Pin GPIO_PIN_12
#define led_r1_GPIO_Port GPIOB
#define led_r2_Pin GPIO_PIN_14
#define led_r2_GPIO_Port GPIOB
#define led_r3_Pin GPIO_PIN_8
#define led_r3_GPIO_Port GPIOD
#define led_r4_Pin GPIO_PIN_10
#define led_r4_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
