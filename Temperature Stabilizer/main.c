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
//#include "liquidcrystal_i2c.h"

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
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum
{
	DetermineState,
	HotState,
	RoomState,
	ColdState,
	AirConditionerState,
	IdleState,
	HeaterState
}TempState;

/* Depending on the temperature read, the next state will be directed to the appropriate state. */
TempState DetermineCategoryHandler(float tempF)
{
/* If the temperature is read to be between 67 and 78 degrees Fahrenheit, the next state will
* be the "Room" state, corresponding to the RoomTemperatureHandler. */
	if((67.0 < tempF) & (tempF <= 78.0))
	{
		return RoomState;
	}
	else if((78.0 < tempF) & (tempF < 91.0))
	{

		return HotState;
	}
	else if (tempF <= 67.0)
	{
		return ColdState;
	}
	else
	{
		return DetermineState;
	}
}

/* If the temperature is determined to be between 78 and 90 degrees, the
 * LED resembling hot temperature measurements will turn on. This LED is red.*/
TempState HotTemperatureHandler(float tempF)
{
	HAL_GPIO_WritePin(GPIOA, HOT_RED_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, ROOM_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COLD_GREEN_LED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, AC_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, HEATER_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IDLE_YELLOW_LED_Pin, GPIO_PIN_RESET);

	HAL_Delay(2000);

	return AirConditionerState;
}

/* If the temperature is determined to be between 67 and 78 degrees, the
 * LED resembling room temperature measurements will turn on. This LED is yellow.*/
TempState RoomTemperatureHandler(float tempF)
{
	HAL_GPIO_WritePin(GPIOA, HOT_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ROOM_YELLOW_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, COLD_GREEN_LED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, AC_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, HEATER_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IDLE_YELLOW_LED_Pin, GPIO_PIN_RESET);

	HAL_Delay(2000);

	return IdleState;
}

/* If the temperature is determined to be less than or equal to 67 degrees, the
 * LED resembling cold temperature measurements will turn on. This LED is green.*/
TempState ColdTemperatureHandler(float tempF)
{
	HAL_GPIO_WritePin(GPIOA, HOT_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ROOM_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COLD_GREEN_LED_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, AC_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, HEATER_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IDLE_YELLOW_LED_Pin, GPIO_PIN_RESET);

	HAL_Delay(2000);

	return HeaterState;
}

/* If the temperature is determined to be hot, the LEDs resembling the air conditioner (AC)
 * and hot temperature will turn on, indicating the environment must be cooled down to
 * room temperature. The AC LED is green. */
TempState AirConditionerHandler(float tempF)
{
	HAL_GPIO_WritePin(GPIOA, HOT_RED_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, ROOM_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COLD_GREEN_LED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, AC_GREEN_LED_Pin,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, HEATER_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IDLE_YELLOW_LED_Pin, GPIO_PIN_RESET);

	HAL_Delay(2000);

	return DetermineState;
}

/* If the temperature is determined to be room temperature, the LEDs resembling the idle state
 * and room temperature will turn on, indicating the environment is at the appropriate temperature.
 * The idle LED is yellow. */
TempState IdleHandler(float tempF)
{
	HAL_GPIO_WritePin(GPIOA, HOT_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ROOM_YELLOW_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, COLD_GREEN_LED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, AC_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, HEATER_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, IDLE_YELLOW_LED_Pin, GPIO_PIN_SET);

	HAL_Delay(2000);

	return DetermineState;
}

/* If the temperature is determined to be cold, the LEDs resembling the heater and cold temperature
 * will turn on, indicating the environment must be heated to room temperature. The heater LED is red.*/
TempState HeaterHandler(float tempF)
{
	HAL_GPIO_WritePin(GPIOA, HOT_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ROOM_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, COLD_GREEN_LED_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, AC_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, HEATER_RED_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, IDLE_YELLOW_LED_Pin, GPIO_PIN_RESET);

	HAL_Delay(2000);

	return DetermineState;
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  char msg[20];
  uint16_t rawValue;
  float voltage = 0.0;
  float tempCel = 0.0;
  float tempF = 0.0;

  TempState NextState = DetermineCategoryHandler(tempF);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/* For Dallas TMP Sensors, use rawValue/1024 */
	HAL_Delay(2000);
	
	/* The following lines are used to compute the voltage to temperature units.*/
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	rawValue = HAL_ADC_GetValue(&hadc1);
	voltage =((float)rawValue)/4095 * 3.3; // voltage = (rawVoltage/(2^bit resolution-1)*Vref)
	tempCel = (voltage - 0.5) * 100;
	tempF = 1.8*(tempCel) + 32.0;

	/* The following lines will be displayed on the serial COM3 terminal. */
	sprintf(msg, "RawValue: %hu\r\n", rawValue);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	sprintf(msg, "Voltage: %f\r\n", voltage);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	sprintf(msg, "Celsius: %f\r\n", tempCel);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	sprintf(msg, "Fahrenheit: %f\r\n\n", tempF);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(5000);

	/* Switch statements controlling the states. */
	switch(NextState)
	{

		case DetermineState:
		sprintf(msg, "DETERMINING... \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		NextState = DetermineCategoryHandler(tempF);
		break;

		case HotState:
		sprintf(msg, "HOT \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		NextState = HotTemperatureHandler(tempF);
		break;

		case RoomState:
		sprintf(msg, "ROOM \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		NextState = RoomTemperatureHandler(tempF);
		break;

		case ColdState:
		sprintf(msg, "COLD \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		NextState = ColdTemperatureHandler(tempF);
		break;

		case AirConditionerState:
		sprintf(msg, "HOT - AC ON \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		NextState = AirConditionerHandler(tempF);
		break;

		case IdleState:
		sprintf(msg, "ROOM - IDLE \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		NextState = IdleHandler(tempF);
		break;

		case HeaterState:
		sprintf(msg, "COLD - HEATER ON \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		NextState = HeaterHandler(tempF);
		break;

		default:
		NextState = DetermineCategoryHandler(tempF);
		break;

	}
	//HAL_Delay(5000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, AC_GREEN_LED_Pin|HOT_RED_LED_Pin|ROOM_YELLOW_LED_Pin|IDLE_YELLOW_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HEATER_RED_LED_Pin|COLD_GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AC_GREEN_LED_Pin HOT_RED_LED_Pin ROOM_YELLOW_LED_Pin IDLE_YELLOW_LED_Pin */
  GPIO_InitStruct.Pin = AC_GREEN_LED_Pin|HOT_RED_LED_Pin|ROOM_YELLOW_LED_Pin|IDLE_YELLOW_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HEATER_RED_LED_Pin COLD_GREEN_LED_Pin */
  GPIO_InitStruct.Pin = HEATER_RED_LED_Pin|COLD_GREEN_LED_Pin;
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
