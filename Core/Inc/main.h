/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Analog_IN_Pin GPIO_PIN_0
#define Analog_IN_GPIO_Port GPIOA
#define Direction_IN_Pin GPIO_PIN_1
#define Direction_IN_GPIO_Port GPIOA
#define Direction_IN_EXTI_IRQn EXTI1_IRQn
#define A_OUT1_Pin GPIO_PIN_2
#define A_OUT1_GPIO_Port GPIOA
#define A_OUT2_Pin GPIO_PIN_3
#define A_OUT2_GPIO_Port GPIOA
#define B_OUT1_Pin GPIO_PIN_4
#define B_OUT1_GPIO_Port GPIOA
#define B_OUT2_Pin GPIO_PIN_5
#define B_OUT2_GPIO_Port GPIOA
#define Enable_IN_Pin GPIO_PIN_0
#define Enable_IN_GPIO_Port GPIOB
#define Enable_IN_EXTI_IRQn EXTI0_IRQn
#define PWM_A_Pin GPIO_PIN_8
#define PWM_A_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_9
#define PWM_B_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */


typedef enum
{
	VFD_OFF,
	VFD_ON
}VFDState;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
