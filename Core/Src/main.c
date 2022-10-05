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
#define BUFFERSIZE 3*sizeof(uint16_t) // 3 channels * uint16
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
uint8_t MainState = MAIN_IDLE;
__IO uint8_t spiDone = 0;

static uint16_t  aADCxConvertedData[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	// Successfully transmitted values. Set data ready pin to low a
//	MainState = MAIN_ADC_START;
	spiDone = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  Error_Handler();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Conversion Complete & DMA Transfer Complete As Well
    // So The AD_RES Is Now Updated & Let's Move IT To The PWM CCR1
    // Update The PWM Duty Cycle With Latest ADC Conversion Result
	MainState = MAIN_SPI_START;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t aTxBuffer[BUFFERSIZE];
	uint8_t aRxBuffer[BUFFERSIZE];
	uint8_t i = 0;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(DATA_RDY_GPIO_Port, DATA_RDY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_RDY_DUP_GPIO_Port, DATA_RDY_DUP_Pin, GPIO_PIN_RESET);
  aTxBuffer[0] = 0x01;
  aTxBuffer[1] = 0x02;

  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(MainState)
		  {
		  	  case MAIN_IDLE:
		  		if (HAL_GPIO_ReadPin(ADC_EN_GPIO_Port, ADC_EN_Pin) != GPIO_PIN_SET )
		  		{
		  		  Idle_Handler(aTxBuffer, aRxBuffer);
		  		} else {
		  			MainState = MAIN_ADC_START;
		  		}
		  		  break;

		  	  case MAIN_CONFIG:
		  		  Config_Handler(aTxBuffer, aRxBuffer);
		  		  break;

		  	  case MAIN_ADC_START:
		  		  // When SPI is done sending/receiving, it sets the state to MAIN_IDLE. Quickly switch to MAIN_ADC_START in case we are still logging.
				  if (HAL_GPIO_ReadPin(ADC_EN_GPIO_Port, ADC_EN_Pin) != GPIO_PIN_SET )
				  {
					  MainState = MAIN_IDLE;

				  } else {
					  //	  		if (aRxBuffer[0] == CMD_ADC_EXIT)

	//	  		{
	//	  			if (Send_OK())
	//	  			{
	//	  				MainState = MAIN_IDLE;
	//	  			}
	//	  		} else
		  		// Check if we are still in ADC Enable mode, if not return to idle.
		  			HAL_GPIO_WritePin(DATA_RDY_GPIO_Port, DATA_RDY_Pin, GPIO_PIN_RESET);
		  			HAL_GPIO_WritePin(DATA_RDY_DUP_GPIO_Port, DATA_RDY_DUP_Pin, GPIO_PIN_RESET);
		  			if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)aADCxConvertedData, 3) == HAL_OK)
		  			{

		  				MainState = MAIN_ADC_CONVERTING;
		  			}
		  		}

		  		  break;

		  	  case MAIN_SPI_START:
		  		// Copy result
				for (i=0; i<(BUFFERSIZE/2); i++)
				{
		    		aTxBuffer[i] = (uint8_t) aADCxConvertedData[i];
		    		aTxBuffer[i+1] = (uint8_t) (aADCxConvertedData[i] >> 8);
	//				aTxBuffer[i] = i;
				}

				uint8_t retVal = HAL_SPI_TransmitReceive_DMA(&hspi1,  (uint8_t*) aTxBuffer, (uint8_t*) aRxBuffer,  BUFFERSIZE);
				if(retVal == HAL_OK)
				{
					MainState = MAIN_SPI_BUSY;
					HAL_GPIO_WritePin(DATA_RDY_GPIO_Port, DATA_RDY_Pin, GPIO_PIN_SET);
	//				HAL_GPIO_WritePin(SPI1_HANDSHAKE_DUP_GPIO_Port, SPI1_HANDSHAKE_DUP_Pin, GPIO_PIN_SET);
				}
	//			}  else { /* Transfer error in transmission process */
	//			  Error_Handler();
	//			}

				break;

		  	  case MAIN_SPI_BUSY:
		  	  case MAIN_ADC_CONVERTING:
		  		  if(spiDone)
		  		  {
		  			  spiDone = 0;
		  			  MainState = MAIN_ADC_START;
		  		  }
			  break;

		      default :
		        Error_Handler();
		      break;
		    }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
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
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
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
  HAL_GPIO_WritePin(GPIOB, DATA_RDY_Pin|DATA_RDY_DUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 ADC_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|ADC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AIN_RANGE_SELECT_CLK_Pin */
  GPIO_InitStruct.Pin = AIN_RANGE_SELECT_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AIN_RANGE_SELECT_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN_RANGE_SELECT_CLR_Pin AIN_PULLUP_SELECT_CLK_Pin AIN_PULLUP_SELECT_CLR_Pin */
  GPIO_InitStruct.Pin = AIN_RANGE_SELECT_CLR_Pin|AIN_PULLUP_SELECT_CLK_Pin|AIN_PULLUP_SELECT_CLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DATA_RDY_Pin DATA_RDY_DUP_Pin */
  GPIO_InitStruct.Pin = DATA_RDY_Pin|DATA_RDY_DUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Config_Handler(uint8_t* aTxBuffer, uint8_t* aRxBuffer)
{
	uint8_t retVal;
	retVal = HAL_SPI_Receive(&hspi1,  (uint8_t *)aRxBuffer, 1, 1000);
	  if( retVal == HAL_ERROR)
	  {
		  /* Transfer error in transmission process */
		  Error_Handler();
	  }
	  else if (retVal == HAL_OK)
	  {
		  switch(aRxBuffer[0])
		  {
			  case CMD_CONFIG_SET_SFREQ:
				// receive setting
				retVal = HAL_SPI_Receive(&hspi1,  (uint8_t *)aRxBuffer, 1, 1000);

				if (Config_Set_Sample_freq(retVal))
				{
					Send_OK();
				} else {
					Send_NOK();
				}


			  break;

			  case CMD_CONFIG_SET_ADC_BITS:

				  break;

			  case CMD_CONFIG_SET_DAC_PWM:

				  break;

			  case CMD_CONFIG_EXIT:
				  if (Send_OK())
				  {
					  MainState = MAIN_IDLE;
				  }
				  break;

			  case CMD_NO_CMD:
				  break;

			  default:
				 Send_NOK();

		  }
	  }
}

void ADC_Handler(void)
{

}

void Idle_Handler(uint8_t* aTxBuffer, uint8_t* aRxBuffer)
{
	uint8_t retVal;

	// Blocking SPI read with 1000 clock cycles timeout.
	retVal = HAL_SPI_Receive(&hspi1,  (uint8_t *)aRxBuffer, 1, 1000);
	if( retVal == HAL_ERROR)
	{
	  /* Transfer error in transmission process */
	  Error_Handler();
	}
	else if (retVal == HAL_OK)
	{
	  switch(aRxBuffer[0])
	  {
		  //case CMD_ADC_START:
			//if (Send_OK())
			//{
			//	MainState = MAIN_ADC_START;
			//}

		  break;

		  case CMD_CONFIG_START:
			  if (Send_OK())
			  {
				MainState = MAIN_CONFIG;
			  }

		  break;

		  case CMD_NO_CMD:
			  break;

		  default:
			Send_NOK();

	  }
	}
}

uint8_t Config_Set_Sample_freq(uint8_t sampleFreq)
{
	return 1;
}
uint8_t Send_OK(void)
{
	uint8_t t = RESP_OK;
	if (HAL_SPI_Transmit(&hspi1, &t, 1, 20000) == HAL_OK)
	{
		return 1;
	} else {
		return 0;
	}
}
uint8_t Send_NOK(void)
{
	uint8_t t = RESP_NOK;
		if (HAL_SPI_Transmit(&hspi1, &t, 1, 20000) == HAL_OK)
		{
			return 1;
		} else {
			return 0;
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
