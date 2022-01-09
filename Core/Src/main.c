/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "H_Bridge.h"
#include "LookUpTable.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Since the sine wave is shifted by the Hbridge by half the real starting point of the output sine wave out of the H bridge is also shifted.

sample 		0 - V_MOTOR-
sample (90-1) - V_MOTOR+

sample (45-1) - GND2

Signals V_MOTOR-, V_MOTOR+, GND2 are shown in schematic.png in main folder.

*/
#define PHASE_U_START_POINT (45 - 1)

#define PHASE_V_START_120_DEG (45 + 120 - 1)	//Second sine wave start point shiffted by 120 samples of the LUT which equals 120 [deg] phase shift, between 1 and 2 channel.
#define PHASE_V_START_240_DEG (105 - 1)			//Second sine wave start point shiffted by -120 samples (45-120= -75 => 180-75= 105) of the LUT which equals -120 [deg] phase shift, between 1 and 2 channel.

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t uPhaseStep = 0;
uint8_t vPhaseStep = 0;
Direction channelAState = CHANNEL_A_INIT_DIR;
Direction channelBState = CHANNEL_B_INIT_DIR;

uint32_t adcDMABuffer = 0;
float computedFrequency = 0.0f;
uint16_t computedARRValue = 0;

Direction motorDir = DIR_CCW;
VFDState vfdStatus = VFD_OFF;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{

			if (vfdStatus == VFD_ON)
			{

				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,SineLookUpTable[uPhaseStep]);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,SineLookUpTable[vPhaseStep]);

					if (uPhaseStep == (N_SINE_POINTS - 1))
						uPhaseStep = 0;

					if (vPhaseStep == (N_SINE_POINTS - 1))
						vPhaseStep = 0;

					uPhaseStep++;
					vPhaseStep++;
			}
	}
	if (htim->Instance == TIM3)
	{

			if (vfdStatus == VFD_ON)
			{

				computedFrequency = adcDMABuffer * ADC_SCALE_RATIO;		// Scaling Measured Voltage 0-3.3[V] to 2-167[Hz].

					if ( computedFrequency  >= 2.0f)	// Checking if frequency is greater than 2[Hz] to avoid carrier timer ARR value being greater than 65535.
					{
						computedARRValue = (uint16_t)roundf((18000000/(computedFrequency* F_OUT_TO_F_CARRIER)) - 1);	//Computing the Carrier frequency for a given output frequency F_OUT_TO_F_CARRIER being number of LUT samples.
						__HAL_TIM_SET_AUTORELOAD(&htim2,computedARRValue);
					}

			}
	}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

		if (GPIO_Pin == Direction_IN_Pin)
		{

				if (HAL_GPIO_ReadPin(Direction_IN_GPIO_Port, Direction_IN_Pin) == GPIO_PIN_SET)
				{

					uPhaseStep = PHASE_U_START_POINT;
					vPhaseStep = PHASE_V_START_240_DEG;

					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,SineLookUpTable[uPhaseStep]);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,SineLookUpTable[vPhaseStep]);

				}
				else
				if (HAL_GPIO_ReadPin(Direction_IN_GPIO_Port, Direction_IN_Pin) == GPIO_PIN_RESET)
				{

					uPhaseStep = PHASE_U_START_POINT;
					vPhaseStep = PHASE_V_START_120_DEG;

					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,SineLookUpTable[uPhaseStep]);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,SineLookUpTable[vPhaseStep]);

				}

		}
		if (GPIO_Pin == Enable_IN_Pin)
		{

				if (HAL_GPIO_ReadPin(Enable_IN_GPIO_Port, Enable_IN_Pin) == GPIO_PIN_SET)
				{
					vfdStatus = VFD_ON;
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				}
				else
				if (HAL_GPIO_ReadPin(Enable_IN_GPIO_Port, Enable_IN_Pin) == GPIO_PIN_RESET)
				{
					vfdStatus = VFD_OFF;
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				}

		}

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_TIM_Base_MspInit(&htim1);
  HAL_TIM_Base_MspInit(&htim2);
  HAL_TIM_Base_MspInit(&htim3);
  HAL_ADC_MspInit(&hadc1);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Stop(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, &adcDMABuffer, 1);

  H_Bridge_SetDirection(CHANNEL_A_INIT_DIR, H_BRIDGE_DIR_PORT_A, &channelAState);
  H_Bridge_SetDirection(CHANNEL_B_INIT_DIR, H_BRIDGE_DIR_PORT_B, &channelBState);


  uPhaseStep = PHASE_U_START_POINT;
  vPhaseStep = PHASE_V_START_120_DEG;

  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,SineLookUpTable[uPhaseStep]);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,SineLookUpTable[vPhaseStep]);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
