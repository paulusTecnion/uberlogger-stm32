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
#include "string.h"
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


// One line of the spi buffer depends on at what stage of sending it is. For the first half of the ADC conversion we have:
// [start bytes][39*(1 year byte, 1 month byte, 1 date byte, 1 hour, 1 second byte, 4 subsecondsTime bytes) Time bytes][39*GPIO bytes][60*8channels*2 ADC bytes]
// For the second half we have :
// [39*8channels*2 ADC bytes][39*GPIO bytes][39*(1 year byte, 1 month byte, 1 date byte, 1 hour, 1 second byte, 4 subsecondsTime bytes) Time bytes][Stop bytes]
// So the SPI TX length is:
// 2 start/stop bytes   = 2
// 39*8*2 ADC           = 624
// 39 * 1 GPIO          = 39
// 39*(1+1+1+1+1+4) Time= 351
//  Total               = 1016 bytes

#define DATA_LINES_PER_SPI_TRANSACTION  70
#define ADC_VALUES_PER_SPI_TRANSACTION  DATA_LINES_PER_SPI_TRANSACTION*8 // Number of ADC uint16_t per transaction. This is 5 times 480 ADC values
#define ADC_BYTES_PER_SPI_TRANSACTION ADC_VALUES_PER_SPI_TRANSACTION*2
#define GPIO_BYTES_PER_SPI_TRANSACTION  DATA_LINES_PER_SPI_TRANSACTION*1
#define TIME_BYTES_PER_SPI_TRANSACTION  DATA_LINES_PER_SPI_TRANSACTION*12
#define START_STOP_NUM_BYTES            2

// Number of bytes when receiving data from the STM
#define STM_SPI_BUFFERSIZE_DATA_TX      (ADC_BYTES_PER_SPI_TRANSACTION + GPIO_BYTES_PER_SPI_TRANSACTION + TIME_BYTES_PER_SPI_TRANSACTION + START_STOP_NUM_BYTES)
#define STM_DATA_BUFFER_SIZE_PER_TRANSACTION (ADC_BYTES_PER_SPI_TRANSACTION + GPIO_BYTES_PER_SPI_TRANSACTION + TIME_BYTES_PER_SPI_TRANSACTION)

//#define STM_SPI_BUFFERSIZE_DATA_TX 1020
#define ADC_BUFFERSIZE_SAMPLES ADC_VALUES_PER_SPI_TRANSACTION*2
#define ADC_BUFFERSIZE_BYTES ADC_BUFFERSIZE_SAMPLES*2
#define GPIO_IO_BUFFERIZE_BYTES GPIO_BYTES_PER_SPI_TRANSACTION*2
#define TIME_BUFFERSIZE_BYTES TIME_BYTES_PER_SPI_TRANSACTION*2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t MainState = MAIN_IDLE, NextState = MAIN_IDLE;
uint8_t logging_en = 0;
uint8_t msgRx = 0;
uint8_t RxBuffer[1];

ADC_HandleTypeDef hadc1_bak;



//union _adc_result{
//	uint16_t t16[ADC_BUFFERSIZE_SAMPLES];
//	uint8_t t8[ADC_BUFFERSIZE_BYTES];
//};
//
//uint16_t  counter;
//
//union _adc_result adc_result;
//uint8_t gpio_result[GPIO_IO_BUFFERIZE_BYTES];
//
//// Don't touch order of struct!
//struct {
//	uint8_t adc[ADC_BUFFERSIZE_BYTES/2];
//	uint8_t gpio[GPIO_IO_BUFFERIZE_BYTES/2];
//} spi_buffer;



volatile uint16_t data_buffer_write_ptr = 0;
volatile uint32_t gpio_result_write_ptr = 0;
volatile uint32_t time_result_write_ptr = 0;

static uint8_t adc_is_half = 0;

static RTC_TimeTypeDef current_time;
static RTC_DateTypeDef current_date;

// Variable for retrieving the
s_date_time_t current_date_time;

uint16_t tim3_counter = 0;

typedef struct {
    uint8_t startByte[START_STOP_NUM_BYTES];
    uint16_t dataLen;
    s_date_time_t timeData[DATA_LINES_PER_SPI_TRANSACTION];
    uint8_t padding3;
    uint8_t padding4;
    uint8_t gpioData[GPIO_BYTES_PER_SPI_TRANSACTION];
    uint8_t adcData[ADC_BYTES_PER_SPI_TRANSACTION];
} spi_msg_1_t;

typedef struct {
    uint8_t adcData[ADC_BYTES_PER_SPI_TRANSACTION];
    uint8_t gpioData[GPIO_BYTES_PER_SPI_TRANSACTION];
    uint8_t padding1;
    uint8_t padding2;
    s_date_time_t timeData[DATA_LINES_PER_SPI_TRANSACTION];
    uint16_t dataLen;
    uint8_t stopByte[START_STOP_NUM_BYTES];
} spi_msg_2_t;

uint8_t data_buffer[sizeof(spi_msg_1_t) + sizeof(spi_msg_2_t)];

spi_msg_1_t * spi_msg_1_ptr = (spi_msg_1_t*) data_buffer;
spi_msg_2_t * spi_msg_2_ptr = (spi_msg_2_t*) (data_buffer + sizeof(spi_msg_1_t)) ;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
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

	 // Check which version of the timer triggered this callback and toggle LED
	HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &current_date, RTC_FORMAT_BCD);

	current_date_time.year = current_date.Year;
	current_date_time.month = current_date.Month;
	current_date_time.date = current_date.Date;
	current_date_time.hours = current_time.Hours;
	current_date_time.minutes = current_time.Minutes;
	current_date_time.seconds = current_time.Seconds;
	// Next line not 100% correct!
	current_date_time.subseconds = 1000 * (current_time.SecondFraction - current_time.SubSeconds) / (current_time.SecondFraction + 1);

	tim3_counter++;

	if (htim == &htim3 )
	  {
		// Are still in the first ADC half?
		if (!adc_is_half)
		{
//			// 0x50000411 = GPIOB, 2nd byte (GPIOB8 to GPIOB15)

//			memcpy((void*) (gpio_start_pointer_1 + gpio_result_write_ptr), (void*) 0x50000411, 1);
////			// Store time
//			memcpy((void*) (time_start_pointer_1 + time_result_write_ptr), &current_date_time, sizeof(current_date_time));
			spi_msg_1_ptr->gpioData[gpio_result_write_ptr] = (GPIOB->IDR >> 8);
			memcpy((void*)&spi_msg_1_ptr->timeData[gpio_result_write_ptr], &current_date_time, sizeof(s_date_time_t));
			spi_msg_1_ptr->dataLen = tim3_counter;
		} else { // If not, we fill the second part
//			memcpy((void*) (gpio_start_pointer_2 + gpio_result_write_ptr), (void*) 0x50000411, 1);
////
//			memcpy((void*) (time_start_pointer_2 + time_result_write_ptr), &current_date_time, sizeof(current_date_time));
			spi_msg_2_ptr->gpioData[gpio_result_write_ptr] = (GPIOB->IDR >> 8);
			memcpy((void*)&spi_msg_2_ptr->timeData[gpio_result_write_ptr], &current_date_time, sizeof(s_date_time_t));
			spi_msg_2_ptr->dataLen = tim3_counter;
		}
//
		gpio_result_write_ptr++;
//		time_result_write_ptr = time_result_write_ptr + sizeof(current_date_time);
//e
//		// to add: check for overrun
		if (gpio_result_write_ptr == GPIO_BYTES_PER_SPI_TRANSACTION)
		{
			gpio_result_write_ptr = GPIO_BYTES_PER_SPI_TRANSACTION-1;
		}
//
//		if (time_result_write_ptr <= TIME_BYTES_PER_SPI_TRANSACTION)
//		{
//			time_result_write_ptr = TIME_BYTES_PER_SPI_TRANSACTION-1;
//		}

	  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

		HAL_StatusTypeDef ret;
		adc_is_half = 1;
		gpio_result_write_ptr = 0;
		time_result_write_ptr = 0;
		tim3_counter=0;
		// Half way we have the pointers start at the beginning


		ret= HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)spi_msg_1_ptr,   sizeof(spi_msg_1_t));
		if (ret == HAL_OK)
			HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_SET);


}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

		HAL_StatusTypeDef ret;
		adc_is_half = 0;
		gpio_result_write_ptr = 0;
		time_result_write_ptr = 0;
		tim3_counter=0;


	//	uint8_t * ptr = data_buffer+(sizeof(data_buffer)/2);
		// At the last ADC sample, we have to send data from half-way the data_buffer size.

		ret = HAL_SPI_Transmit_DMA(&hspi1,  (uint8_t*)spi_msg_2_ptr,   sizeof(spi_msg_2_t));

		if (ret == HAL_OK)
			HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_SET);

}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{

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
	HAL_StatusTypeDef errorcode;

	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_SET);
	errorcode = HAL_SPI_TransmitReceive(&hspi1, t8, RxBuffer, 2, 2000);

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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  // Backup current adc settings
  hadc1_bak = hadc1;

  spi_msg_1_ptr->startByte[0] = 0xFF;
  spi_msg_1_ptr->startByte[1] = 0xFF;
  spi_msg_2_ptr->stopByte[0]= 0xFF;
  spi_msg_2_ptr->stopByte[1]= 0xFF;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  switch(MainState)
	  {
		  case MAIN_IDLE:
			  if (logging_en)
			  {
				  // Make sure we have the original ADC config put in place
				   hadc1 = hadc1_bak;
				  // Reinit ADC
				  ADC_Reinit();

				  // Start TIM3 and DMA conversion
				  HAL_TIM_Base_Start_IT(&htim3);
				  HAL_ADC_Start_DMA(
						  &hadc1,
						  // data_buffer + offset
						  (uint32_t*)(spi_msg_1_ptr->adcData),
						  ADC_BUFFERSIZE_SAMPLES);



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
//
				  HAL_TIM_Base_Stop_IT(&htim3);
				  HAL_ADC_Stop_DMA(&hadc1);
				  // Send final bytes

				  /* Part below does not work well yet */
//				  if (!adc_is_half)
//				  {
//					  	  HAL_StatusTypeDef ret = HAL_SPI_Transmit_DMA(&hspi1,  (uint8_t*)spi_msg_1_ptr,   sizeof(spi_msg_1_t));
//						if (ret == HAL_OK)
//							HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_SET);
//				  } else {
//					  	  HAL_StatusTypeDef ret = HAL_SPI_Transmit_DMA(&hspi1,  (uint8_t*)spi_msg_2_ptr,   sizeof(spi_msg_2_t));
//							if (ret == HAL_OK)
//								HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_SET);
//				  }


				  // Set ADC to single conversion measure mode
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x17;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
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

static void ADC_Set_Single_Acq()
{


	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	  hadc1.Init.LowPowerAutoWait = DISABLE;
	  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	  hadc1.Init.ContinuousConvMode = ENABLE;
	  hadc1.Init.NbrOfConversion = 8;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	  hadc1.Init.DMAContinuousRequests = DISABLE;
	  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	  hadc1.Init.OversamplingMode = DISABLE;
	  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;


}

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

	 case ADC_SAMPLE_RATE_2Hz:
	 		 // Reconfig the timer
		htim3.Init.Prescaler = 500;
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


void ADC_Reinit()
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
