/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOGIC_CIRCUIT_PIN_1   GPIO_PIN_6
#define LOGIC_CIRCUIT_PORT_1  GPIOA

#define LOGIC_CIRCUIT_PIN_2   GPIO_PIN_8
#define LOGIC_CIRCUIT_PORT_2  GPIOA

#define LOGIC_CIRCUIT_PIN_3   GPIO_PIN_10
#define LOGIC_CIRCUIT_PORT_3  GPIOA

#define RED_LED_PIN_1         GPIO_PIN_7
#define RED_LED_PORT_1        GPIOA

#define YELLOW_LED_PIN        GPIO_PIN_9
#define YELLOW_LED_PORT       GPIOA

#define GREEN_LED_PIN         GPIO_PIN_11
#define GREEN_LED_PORT        GPIOA

#define MOTOR_PIN            GPIO_PIN_13
#define MOTOR_PORT           GPIOB

#define BUTTON_PIN            GPIO_PIN_7
#define BUTTON_PORT           GPIOB



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
uint8_t hardwareInterruptStatus = 0;
uint8_t logicCircuitsStatus = 0;
TIM_HandleTypeDef htim2;
uint8_t buttonPressedPB7 = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_15)
  {
    hardwareInterruptStatus = 1;
  }
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void EXTI9_5_IRQHandler(void);

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  MX_GPIO_Init();
  MX_TIM2_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (buttonPressedPB7)
	     {

	        for (int i = 0; i < 3; ++i)
	        {
	        	for (int j = 0; j < 3; ++j)
	        	{
	        		HAL_Delay(25);

	        		HAL_GPIO_WritePin(RED_LED_PORT_1, RED_LED_PIN_1, GPIO_PIN_RESET);
	        		HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_RESET);
	        		HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
	        		HAL_Delay(50);

	        		HAL_GPIO_WritePin(RED_LED_PORT_1, RED_LED_PIN_1, GPIO_PIN_SET);
	        		HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_SET);
	        		HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
	        		if (i < 2)
	        		{
	          	       HAL_Delay(25);
	          	    }

	        	}
	        	if (i < 2)
	        	{
	                HAL_Delay(200);
	            }
	        }
	       buttonPressedPB7 = 0;
	    }
	  if (hardwareInterruptStatus == 1)
	  {
	  	      HAL_GPIO_WritePin(RED_LED_PORT_1, RED_LED_PIN_1, GPIO_PIN_RESET);
	  	      HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_RESET);
	  	      HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);

	  	      HAL_Delay(600);

	  	      HAL_GPIO_WritePin(RED_LED_PORT_1, RED_LED_PIN_1, GPIO_PIN_SET);
	  	      HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_SET);
	  	      HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);

	  	      hardwareInterruptStatus = 0;
	  }
	      if (HAL_GPIO_ReadPin(LOGIC_CIRCUIT_PORT_1, LOGIC_CIRCUIT_PIN_1) == GPIO_PIN_SET &&
	          HAL_GPIO_ReadPin(LOGIC_CIRCUIT_PORT_2, LOGIC_CIRCUIT_PIN_2) == GPIO_PIN_SET &&
	          HAL_GPIO_ReadPin(LOGIC_CIRCUIT_PORT_3, LOGIC_CIRCUIT_PIN_3) == GPIO_PIN_SET)
	      {
	        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN, GPIO_PIN_SET);
	        logicCircuitsStatus = 1;
	      }
	      else
	      {
	        if (logicCircuitsStatus == 0)
	        {
	          HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN, GPIO_PIN_RESET);
	        }
	        logicCircuitsStatus = 0;
	      }

	      if (HAL_GPIO_ReadPin(LOGIC_CIRCUIT_PORT_1, LOGIC_CIRCUIT_PIN_1) == GPIO_PIN_SET)
	      {
	        HAL_GPIO_WritePin(RED_LED_PORT_1, RED_LED_PIN_1, GPIO_PIN_RESET);
	      }
	      else
	      {
	        HAL_GPIO_WritePin(RED_LED_PORT_1, RED_LED_PIN_1, GPIO_PIN_SET);
	      }

	      if (HAL_GPIO_ReadPin(LOGIC_CIRCUIT_PORT_2, LOGIC_CIRCUIT_PIN_2) == GPIO_PIN_SET)
	      {
	        HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_RESET);
	      }
	      else
	      {
	        HAL_GPIO_WritePin(YELLOW_LED_PORT, YELLOW_LED_PIN, GPIO_PIN_SET);
	      }

	      if (HAL_GPIO_ReadPin(LOGIC_CIRCUIT_PORT_3, LOGIC_CIRCUIT_PIN_3) == GPIO_PIN_SET)
	      {
	        HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
	      }
	      else
	      {
	        HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
	      }


	      HAL_Delay(0);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */

/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = LOGIC_CIRCUIT_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LOGIC_CIRCUIT_PORT_1, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RED_LED_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_PORT_1, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LOGIC_CIRCUIT_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LOGIC_CIRCUIT_PORT_2, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GREEN_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LOGIC_CIRCUIT_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LOGIC_CIRCUIT_PORT_3, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = YELLOW_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(YELLOW_LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MOTOR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI9_5_IRQHandler(void)
{
  if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET)
  {
    buttonPressedPB7 = 1;
  }
  HAL_GPIO_EXTI_IRQHandler(BUTTON_PIN);
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
  __disable_irq();
  while (1)
  {
    // Handle error
  }
  /* USER CODE END Error_Handler_Debug */
}
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
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
