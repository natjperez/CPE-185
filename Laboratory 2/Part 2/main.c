/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum //Creating the data type eSystemState
{
	Transition_NS_State,
	NS_Pass_EW_Stop_State,
	All_Stop_EW_State,
	NS_Stop_EW_Pass_State,
	Transition_EW_State,
	All_Stop_NS_State

} eSystemState;

/* Prototype Event Handlers */
eSystemState NorthSouthPassHandler(void)
{
	HAL_GPIO_WritePin(GPIOB, NS_GREEN_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, NS_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, NS_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_RED_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(10*1000); //10 seconds
	return Transition_NS_State;

}

eSystemState TransitionNorthSouthHandler(void)
{
	HAL_GPIO_WritePin(GPIOB, NS_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, NS_YELLOW_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, NS_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_RED_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(5*1000); //5 seconds
	return All_Stop_EW_State;
}

eSystemState AllStopEastWestHandler(void)
{
	HAL_GPIO_WritePin(GPIOB, NS_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, NS_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, NS_RED_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, EW_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_RED_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(5*1000); //5 seconds
	return NS_Stop_EW_Pass_State;
}
/* Add if statement that checks the state of the push button and if the button
 * is pressed, set the EW state to the next state */
eSystemState EastWestPassHandler(void) //NW Stop, EW Pass state
{
	HAL_GPIO_WritePin(GPIOB, NS_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, NS_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, NS_RED_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, EW_GREEN_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, EW_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_RED_LED_Pin, GPIO_PIN_RESET);
	//HAL_Delay(10*1000); //10 seconds

	//If a car arrives at the NS road, transition the EW light
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)) //Using PB1 = D6
	{
		return Transition_EW_State;
	}

	return NS_Stop_EW_Pass_State; //Repeat this state
	//return Transition_EW_State;
}

eSystemState TransitionEastWestHandler(void) //Transition EW State
{
	HAL_GPIO_WritePin(GPIOB, NS_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, NS_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, NS_RED_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, EW_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_YELLOW_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, EW_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_Delay(5*1000); //5 seconds
	return All_Stop_NS_State;
}

eSystemState AllStopNorthSouthHandler(void) //All Stop NS State
{
	HAL_GPIO_WritePin(GPIOB, NS_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, NS_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, NS_RED_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, EW_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, EW_RED_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(5*1000); //5 seconds
	return NS_Pass_EW_Stop_State;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /*Declare eNextState and initialize it to All Stop North South */
  eSystemState eNextState = Transition_NS_State; //Change to NS Stop EW Pass
  //eSystemState eNextState = NS_Stop_EW_Pass_State;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch(eNextState)
	  {
		  case Transition_NS_State:
		  eNextState = TransitionNorthSouthHandler();
		  break;

		  case NS_Pass_EW_Stop_State:
		  eNextState = NorthSouthPassHandler();
		  break;

		  case All_Stop_EW_State:
		  eNextState = AllStopEastWestHandler();
		  break;

		  case NS_Stop_EW_Pass_State:
		  eNextState = EastWestPassHandler(); //Added an if statement
		  break;

		  case Transition_EW_State:
		  eNextState = TransitionEastWestHandler();
		  break;

		  case All_Stop_NS_State:
		  eNextState = AllStopNorthSouthHandler();
		  break;

		  default:
		  eNextState = AllStopNorthSouthHandler();
		  break;
	  }

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
  huart2.Init.BaudRate = 38400;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EW_GREEN_LED_Pin|EW_YELLOW_LED_Pin|EW_RED_LED_Pin|NS_RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NS_GREEN_LED_Pin|NS_YELLOW_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EW_GREEN_LED_Pin EW_YELLOW_LED_Pin EW_RED_LED_Pin NS_RED_LED_Pin */
  GPIO_InitStruct.Pin = EW_GREEN_LED_Pin|EW_YELLOW_LED_Pin|EW_RED_LED_Pin|NS_RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PUSH_PUTTON_Pin */
  GPIO_InitStruct.Pin = PUSH_PUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PUSH_PUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NS_GREEN_LED_Pin NS_YELLOW_LED_Pin */
  GPIO_InitStruct.Pin = NS_GREEN_LED_Pin|NS_YELLOW_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

