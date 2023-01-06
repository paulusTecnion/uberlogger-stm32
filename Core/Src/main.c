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
#include "esp32_interface.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BUFFERSIZE 3*sizeof(uint16_t) // 3 channels * uint16
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t MainState = MAIN_IDLE, NextState = MAIN_IDLE;
uint8_t logging_en = 0;
uint8_t msgRx = 0;
uint8_t RxBuffer[1];


#define ADC_BUFFERSIZE 1024

union _adc_result{
	uint16_t t16[ADC_BUFFERSIZE];
	uint8_t t8[ADC_BUFFERSIZE*2];
};

uint16_t  counter;


static union _adc_result adc_result;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
	// When a non-circular DMA transfer is finished, the DMA transfer has to be disabled
	HAL_SPI_DMAStop(&hspi1);
	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	// Successfully transmitted values. Set data ready pin to low a
//	MainState = MAIN_ADC_START;
//	spiDone = 1;
//	HAL_SPI_DMAStop(&hspi1);
//	msgRx = 1;
//	HAL_SPI_DMAStop(&hspi1);
	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);
}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
//{
//	msgRx = 1;
//	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);
//}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_DMAStop(&hspi1);
	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);
  //Error_Handler();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 /* Prevent unused argument(s) compilation warning */
 UNUSED(htim);

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_StatusTypeDef ret;

 	ret= HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)adc_result.t8,   ADC_BUFFERSIZE);
 	if (ret == HAL_OK)
 		HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_SET);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_StatusTypeDef ret;

//	HAL_SPI_Transmit_DMA(&hspi1,  (uint8_t*) t.t8,   TESTBUFFERSIZE*2);
	ret = HAL_SPI_Transmit_DMA(&hspi1,  (uint8_t*) (&adc_result.t8[ADC_BUFFERSIZE]),   ADC_BUFFERSIZE);
	if (ret == HAL_OK)
		HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_SET);

}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	adc_result.t8[0] = 255;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == STM_ADC_EN_Pin)
	{
			logging_en = 1;
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == STM_ADC_EN_Pin)
	{
		//HAL_SPI_DMAStop(&hspi1);

		logging_en = 0;

	}
}


uint8_t HAL_SPI_Send_cmd(spi_cmd_resp_t cmd, spi_cmd_esp_t cmd_esp)
{

	uint8_t t8[2];
	t8[0] = (uint8_t)cmd;
	t8[1] = cmd_esp;
	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_SET);
	HAL_StatusTypeDef errorcode = HAL_SPI_TransmitReceive(&hspi1, t8, RxBuffer, 2, 2000);

	if (errorcode == HAL_OK)
	{
		// Indicate to esp32 that data is ready
		HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);
		// wait until transfer is done. Note that on a hardware level this may not have been sent yet!
//		while(hspi1.State == HAL_SPI_STATE_BUSY_TX_RX);
	}
	return errorcode;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Always receive
//	  if (!logging_en && hspi1.State == HAL_SPI_STATE_READY && msgRx == 0)
//	  {
//		  HAL_SPI_Receive_DMA(&hspi1, RxBuffer, 2);
//	  }
	  switch(MainState)
	  {
		  case MAIN_IDLE:
			  if (logging_en)
			  {
				  HAL_TIM_Base_Start_IT(&htim3);
				  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_result.t16, ADC_BUFFERSIZE);
				  counter =0;
				  NextState = MAIN_LOGGING;
			  } else {
				  Idle_Handler();
			  }
			  break;

		  case MAIN_CONFIG:
			  Config_Handler();
			  break;

		  case MAIN_LOGGING:
			  // Doing nothing
			  if (!logging_en)
			  {
//				  currentTick = HAL_GetTick();
//				  // Wait for SPI to finish any remaining transmissions.
//				  while(hspi1.State == HAL_SPI_STATE_BUSY_TX)
//				  {
//					  differenceTick = currentTick - HAL_GetTick();
//					  if (differenceTick> 750)
//					  {
//
//						 HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);
//						 break;
//					  }
//
//				  }
//				  HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);
				  HAL_TIM_Base_Stop_IT(&htim3);
				  HAL_ADC_Stop_DMA(&hadc1);
				  HAL_SPI_DMAStop(&hspi1);
				  NextState = MAIN_IDLE;
			  }
			  break;

	  }

	  if (NextState != MainState)
	  {
		  MainState = NextState;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 64000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIGITAL_IN_0_Pin DIGITAL_IN_1_Pin DIGITAL_IN_2_Pin DIGITAL_IN_3_Pin
                           DIGITAL_IN_4_Pin DIGITAL_IN_5_Pin */
  GPIO_InitStruct.Pin = DIGITAL_IN_0_Pin|DIGITAL_IN_1_Pin|DIGITAL_IN_2_Pin|DIGITAL_IN_3_Pin
                          |DIGITAL_IN_4_Pin|DIGITAL_IN_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN_RANGE_SELECT_CLK_Pin AIN_RANGE_SELECT_CLR_Pin AIN_PULLUP_SELECT_CLR_Pin */
  GPIO_InitStruct.Pin = AIN_RANGE_SELECT_CLK_Pin|AIN_RANGE_SELECT_CLR_Pin|AIN_PULLUP_SELECT_CLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : STM_DATA_RDY_Pin */
  GPIO_InitStruct.Pin = STM_DATA_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STM_DATA_RDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STM_ADC_EN_Pin */
  GPIO_InitStruct.Pin = STM_ADC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STM_ADC_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

static uint8_t Config_Set_Sample_freq(uint8_t sampleFreq)
{
	// Make sure timer3 has stopped
	HAL_TIM_Base_Stop_IT(&htim3);


	 htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	 htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	 htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	 switch(sampleFreq)
	 {
	 case ADC_SAMPLE_RATE_1Hz:
		 // Reconfig the timer
		 htim3.Init.Prescaler = 1000;
		 htim3.Init.Period = 64000;
		 break;


	 case ADC_SAMPLE_RATE_10Hz:
		 htim3.Init.Prescaler = 100;
		 htim3.Init.Period = 64000;
		 break;

	 case ADC_SAMPLE_RATE_25Hz:
		 htim3.Init.Prescaler = 100;
		 htim3.Init.Period = 25600;
		 break;

	 case ADC_SAMPLE_RATE_50Hz:
		 htim3.Init.Prescaler = 100;
		 htim3.Init.Period = 12800;
		 break;
	 case 	ADC_SAMPLE_RATE_100Hz:
		htim3.Init.Prescaler = 10;
		htim3.Init.Period = 64000;
		 break;

	 case 	ADC_SAMPLE_RATE_200Hz:
		 htim3.Init.Prescaler = 10;
		 htim3.Init.Period = 32000;

		 break;

	 case ADC_SAMPLE_RATE_400Hz:
		 htim3.Init.Prescaler = 10;
		 htim3.Init.Period = 16000;

		 break;

	 case ADC_SAMPLE_RATE_500Hz:
		 htim3.Init.Prescaler = 10;
		 htim3.Init.Period = 12800;

		 break;

	 case ADC_SAMPLE_RATE_1000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 64000;

	 break;

	 case ADC_SAMPLE_RATE_2000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 64000;

		 break;

	 case ADC_SAMPLE_RATE_4000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 16000;

		 break;

	 case ADC_SAMPLE_RATE_5000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 12800;

		 break;

	 case ADC_SAMPLE_RATE_8000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 8000;

		 break;
	 case ADC_SAMPLE_RATE_10000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 6400;

		 break;
	 case ADC_SAMPLE_RATE_20000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 3200;

		 break;

	 case ADC_SAMPLE_RATE_40000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 1600;

		 break;

	 case ADC_SAMPLE_RATE_50000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 1280;

		 break;

	 case ADC_SAMPLE_RATE_100000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 640;

		 break;

	 case ADC_SAMPLE_RATE_250000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 256;

		 break;

	 case ADC_SAMPLE_RATE_500000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 128;

		 break;

	 case ADC_SAMPLE_RATE_1000000Hz:
		 htim3.Init.Prescaler = 1;
		 htim3.Init.Period = 64;

		 break;
		 // Unknown rate
	 default:
	 	return 1;

	 }

	 // Reinit timer
	 if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	  {
	    return 1;
	  }


	 return 0;
}


static uint8_t Config_Enable_Adc_channel (uint8_t channel, uint8_t rank)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	switch (channel)
	{

		case 0:
			sConfig.Channel = ADC_CHANNEL_0;
			break;

		case 1:
			sConfig.Channel = ADC_CHANNEL_1;
		break;

		case 2:
			sConfig.Channel = ADC_CHANNEL_2;
				break;

		case 3:
			sConfig.Channel = ADC_CHANNEL_3;
				break;

		case 4:
			sConfig.Channel = ADC_CHANNEL_4;
				break;

		case 5:
			sConfig.Channel = ADC_CHANNEL_5;
				break;

		case 6:
			sConfig.Channel = ADC_CHANNEL_6;
				break;

		case 7:
			sConfig.Channel = ADC_CHANNEL_7;
				break;

		default:
			return 1;
	}


	switch (rank)
		{

			case 1:
				sConfig.Rank = ADC_REGULAR_RANK_1;
			break;

			case 2:
				sConfig.Rank = ADC_REGULAR_RANK_2;
					break;

			case 3:
				sConfig.Rank = ADC_REGULAR_RANK_3;
					break;

			case 4:
				sConfig.Rank = ADC_REGULAR_RANK_4;
					break;

			case 5:
				sConfig.Rank = ADC_REGULAR_RANK_5;
					break;

			case 6:
				sConfig.Rank = ADC_REGULAR_RANK_6;
					break;

			case 7:
				sConfig.Rank = ADC_REGULAR_RANK_7;
					break;

			case 8:
				sConfig.Rank = ADC_REGULAR_RANK_8;
				break;

			default:
				return 1;

		}




	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	return 0;



}

static uint8_t Config_Set_Adc_channels(uint8_t channels)
{

	uint8_t channel_num=1,j, rank =1;

	// first find number of channels
	for (j=1; j<128; j = j << 1)
	{
		if (channels & j)
			channel_num++;
	}
	// Initiate number of channels
	 hadc1.Init.NbrOfConversion = channel_num;

	 if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
		Error_Handler();
		return 1;
	  }

	 channel_num = 0;
	// Mask channels data and see which ones are enabled (again)
	for (j = 1; j<128; j = j << 1)
	{
		// If channel is enabled...
		if (channels & j)
		{
			// Activate it in the first available rank. Return 1 on error.
			if (Config_Enable_Adc_channel(channel_num, rank)) return 1;
			rank++;

		}
		channel_num++; // Keep track of which channel number we are checking
	}


	return 0;
}

static uint8_t Config_Set_Resolution(uint8_t resolution)
{
	switch (resolution)
	{


	case ADC_16_BITS:
		hadc1.Init.OversamplingMode = ENABLE;
		  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
		  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
		  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
		break;

	//case ADC_12_BITS:
	default:
		hadc1.Init.OversamplingMode = DISABLE;
		hadc1.Init.Resolution = ADC_RESOLUTION_12B;
		 hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
		break;


	}

	return 0;

}


static void ADC_Reinit()
{
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

void Config_Handler()
{
	uint8_t retVal;
	retVal = HAL_SPI_Receive(&hspi1,  RxBuffer, 2, 250);

	  if( retVal == HAL_ERROR)
	  {
		  /* Transfer error in transmission process */
		  Error_Handler();
	  }
	  else if (retVal == HAL_OK)
	  {
		  switch(RxBuffer[0])
		  {
			  case CMD_NOP:
				  HAL_SPI_Send_cmd(CMD_RESP_OK, CMD_NOP);
				  break;

			  case CMD_MEASURE_MODE:
				  // Re-init ADC
				  if (HAL_SPI_Send_cmd(CMD_RESP_OK, CMD_MEASURE_MODE) == HAL_OK)
				  {
					  ADC_Reinit();
					  NextState = MAIN_IDLE;
				  }
				  break;

			  case CMD_SET_RESOLUTION:
				  if (!Config_Set_Resolution(RxBuffer[1]))
				  {
					  HAL_SPI_Send_cmd(CMD_RESP_OK, CMD_SET_RESOLUTION);
				  } else {
					  HAL_SPI_Send_cmd(CMD_RESP_NOK, CMD_SET_RESOLUTION);
				  }

				  break;


			  case CMD_SET_SAMPLE_RATE:
			 				// receive setting
				if (!Config_Set_Sample_freq(RxBuffer[1]))
				{
					HAL_SPI_Send_cmd(CMD_RESP_OK, CMD_SET_SAMPLE_RATE);
				} else {
					HAL_SPI_Send_cmd(CMD_RESP_NOK, CMD_SET_SAMPLE_RATE);
				}
			 	break;

			  case CMD_SET_ADC_CHANNELS_ENABLED:
				  if (!Config_Set_Adc_channels(RxBuffer[1]))
				  {
					  HAL_SPI_Send_cmd(CMD_RESP_OK, CMD_SET_ADC_CHANNELS_ENABLED);
				  } else {
					  HAL_SPI_Send_cmd(CMD_RESP_NOK, CMD_SET_ADC_CHANNELS_ENABLED);
				  }
				  break;

//			  case CMD_CONFIG_SET_DAC_PWM:
//
//				  break;

			  default:
				  HAL_SPI_Send_cmd(CMD_RESP_NOK, CMD_UNKNOWN);

		  }
	  }

//	if (msgRx)
//	{
//		switch(RxBuffer[0])
//		{
//			case CMD_NOP:
//
//			break;
//
//			case CMD_MEASURE_MODE:
//			// Re-init ADC
//			ADC_Reinit();
//			if (Send_OK())
//			{
//				MainState = MAIN_IDLE;
//			}
//			break;
//
//
//
//			case CMD_SET_RESOLUTION:
//				if (Config_Set_Resolution(RxBuffer[1]))
//				{
//					Send_OK();
//				} else {
//					Send_NOK();
//				}
//
//			break;
//
//
//			case CMD_SET_SAMPLE_RATE:
//			// receive setting
//			if (Config_Set_Sample_freq(RxBuffer[1]))
//			{
//				Send_OK();
//			} else {
//				Send_NOK();
//			}
//			break;
//
//			//			  case CMD_CONFIG_SET_DAC_PWM:
//			//
//			//				  break;
//
//			default:
//			Send_NOK();
//		}
//	msgRx = 0;
//	}

}
void Idle_Handler()
{
	uint8_t retVal;

//	if (msgRx)
//	{
//		switch(RxBuffer[0])
//			  {
//				  //case CMD_ADC_START:
//					//if (Send_OK())
//					//{
//					//	MainState = MAIN_ADC_START;
//					//}
//
//				  case CMD_SETTINGS_MODE:
//					  if (Send_OK())
//					  {
//						MainState = MAIN_CONFIG;
//					  }
//
//				  break;
//
//				  case CMD_NOP:
//					  break;
//
//				  default:
//					Send_NOK();
//
//			  }
//		msgRx = 0;
//	}
//	 Blocking SPI read with 1000 clock cycles timeout.
	retVal = HAL_SPI_Receive(&hspi1,  RxBuffer, 2, 250);
	if( retVal == HAL_ERROR)
	{
	  /* Transfer error in transmission process */
	  Error_Handler();
	}
	else if (retVal == HAL_OK)
	{
	  switch(RxBuffer[0])
	  {
		  //case CMD_ADC_START:
			//if (Send_OK())
			//{
			//	MainState = MAIN_ADC_START;
			//}

		  case CMD_SETTINGS_MODE:
			  if (HAL_SPI_Send_cmd(CMD_RESP_OK, CMD_SETTINGS_MODE) == HAL_OK)
			  {
				NextState = MAIN_CONFIG;
			  }

		  break;

		  case CMD_NOP:
			  HAL_SPI_Send_cmd(CMD_RESP_OK, CMD_NOP);
			  break;

		  default:
			  HAL_SPI_Send_cmd(CMD_RESP_NOK, CMD_UNKNOWN);

	  }
	}
}

//uint8_t Config_Set_Sample_freq(uint8_t sampleFreq)
//{
//	return 1;
//}


//
//uint8_t Send_OK(void)
//{
//
//	t.t8[0] = RESP_OK;
//	t.t8[1] = 0x00;
//	HAL_StatusTypeDef errorcode;
//
//	errorcode = HAL_SPI_TransmitReceive_DMA(&hspi1, t.t8, RxBuffer, 2);
//
//	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_SET);
////	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);
//	if (errorcode == HAL_OK)
//	{
//		return 1;
//	} else {
//		return 0;
//	}
//}
//uint8_t Send_NOK(void)
//{
//
//	t.t8[0]= RESP_NOK;
//	t.t8[1] = 0x00;
//	HAL_StatusTypeDef errorcode;
//
//	errorcode = HAL_SPI_TransmitReceive_DMA(&hspi1, t.t8, RxBuffer, 2);
//	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_SET);
////	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);
//	if (errorcode == HAL_OK)
//	{
//		return 1;
//	} else {
//		return 0;
//	}
//}


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
