/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t rx_byte;
char rx_buffer[32];

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN1_2_Pin|IN2_2_Pin|IN3_2_Pin|IN4_2_Pin
                          |IN4_1_Pin|IN3_1_Pin|IN2_1_Pin|IN1_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_2_Pin IN2_2_Pin IN3_2_Pin IN4_2_Pin
                           IN4_1_Pin IN3_1_Pin IN2_1_Pin IN1_1_Pin */
  GPIO_InitStruct.Pin = IN1_2_Pin|IN2_2_Pin|IN3_2_Pin|IN4_2_Pin
                          |IN4_1_Pin|IN3_1_Pin|IN2_1_Pin|IN1_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin PA5 */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart->Instance == UART4){
	  HAL_UART_Receive_IT(&huart4, &rx_byte, sizeof(rx_byte));
	  if (rx_byte == 0x01){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 20);
      }
	  else if (rx_byte == 0x02){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 40);
	    	}
	  else if (rx_byte == 0x03){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 60);
	  	    	}
	  else if (rx_byte == 0x04){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 80);
	  	  	    }
	  else if (rx_byte == 0x05){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
	  	    	}
	  else if (rx_byte == 0x06){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 20);
	  	    	}
	  else if (rx_byte == 0x07){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 40);
	  	    	}
	  else if (rx_byte == 0x08){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 60);
	  	    	}
	  else if (rx_byte == 0x09){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 80);
	  	    	}
	  else if (rx_byte == 0x0A){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
	  	    	}
	  else if (rx_byte == 0x0B){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 20);
	  	    	}
	  else if (rx_byte == 0x0C){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 40);
	  	    	}
	  else if (rx_byte == 0x0D){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60);
	  	    	}
	  else if (rx_byte == 0x0E){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 80);
	  	    	}
	  else if (rx_byte == 0x0F){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);
	  	    	}
	  else if (rx_byte == 0x10){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 20);
	  	    	}
	  else if (rx_byte == 0x11){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 40);
	  	    	}
	  else if (rx_byte == 0x12){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60);
	  	    	}
	  else if (rx_byte == 0x13){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 80);
	  	    	}
	  else if (rx_byte == 0x14){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);
	  	    	}
	  else if (rx_byte == 0x15){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 20);
	  	    	}
	  else if (rx_byte == 0x16){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 40);
	  	    	}
	  else if (rx_byte == 0x17){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60);
	  	    	}
	  else if (rx_byte == 0x18){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 80);
	  	    	}
	  else if (rx_byte == 0x19){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);
	  	    	}
	  else if (rx_byte == 0x1A){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 20);
	  	    	}
	  else if (rx_byte == 0x1B){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 40);
	  	    	}
	  else if (rx_byte == 0x1C){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60);
	  	    	}
	  else if (rx_byte == 0x1D){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 80);
	  	    	}
	  else if (rx_byte == 0x1E){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);
	  	    	}
	  else if (rx_byte == 0x1F){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 20);
	  	    	}
	  else if (rx_byte == 0x20){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 40);
	  	    	}
	  else if (rx_byte == 0x21){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 60);
	  	    	}
	  else if (rx_byte == 0x22){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 80);
	  	    	}
	  else if (rx_byte == 0x23){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 100);
	  	    	}
	  else if (rx_byte == 0x24){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 20);
	  	    	}
	  else if (rx_byte == 0x25){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 40);
	  	    	}
	  else if (rx_byte == 0x26){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 60);
	  	    	}
	  else if (rx_byte == 0x27){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 80);
	  	    	}
	  else if (rx_byte == 0x28){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 100);
	  	    	}
	  else if (rx_byte == 0x29){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 20);
	  	    	}
	  else if (rx_byte == 0x2A){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 40);
	  	    	}
	  else if (rx_byte == 0x2B){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60);
	  	    	}
	  else if (rx_byte == 0x2C){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 80);
	  	    	}
	  else if (rx_byte == 0x2D){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);
	  	    	}
	  else if (rx_byte == 0x2E){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 20);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 20);
	  	    	}
	  else if (rx_byte == 0x2F){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 40);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 40);
	  	    	}
	  else if (rx_byte == 0x30){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 60);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60);
	  	    	}
	  else if (rx_byte == 0x31){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 80);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 80);
	  	    	}
	  else if (rx_byte == 0x32){
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 100);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);
	  }
	  else if(rx_byte == 0x33){
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10);
	  }
	  else if(rx_byte == 0x34){
	  	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 20);
	  }
	  else if(rx_byte == 0x35){
	  	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 30);
	  }
	  else if(rx_byte == 0x36){
	  	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 40);
	  }
	  else if(rx_byte == 0x37){
	  	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
	  }
	  else if(rx_byte == 0x38){
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 60);
	  }
	  else if(rx_byte == 0x39){
	  	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 70);
	  }
	  else if(rx_byte == 0x3A){
	  	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 80);
	  }
	  else if(rx_byte == 0x3B){
	  	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 90);
	  }
	  else if(rx_byte == 0x3C){
	  	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
	  }
      else {
    	  HAL_GPIO_WritePin(IN1_1_GPIO_Port, IN1_1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(IN2_1_GPIO_Port, IN2_1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(IN3_1_GPIO_Port, IN3_1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(IN4_1_GPIO_Port, IN4_1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(IN1_2_GPIO_Port, IN1_2_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(IN2_2_GPIO_Port, IN2_2_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(IN3_2_GPIO_Port, IN3_2_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(IN4_2_GPIO_Port, IN4_2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
      }
    }
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
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
