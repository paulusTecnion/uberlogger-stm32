
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

  /*
 * MIT License
 *
 * Copyright (c) 2025 Tecnion Technologies
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
#include "spi_ctrl.h"
#include "iirfilter.h"
#include "adc_comp_lut.h"
#include "framing.h"
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
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim3_bak;
uint8_t MainState = MAIN_IDLE, NextState = MAIN_IDLE;
uint8_t logging_en = 0;
uint8_t msgRx = 0;
uint8_t cmd_buffer[ 20];
//spi_cmd_t cmd_buffer;

ADC_HandleTypeDef hadc1_bak;

uint8_t main_exit_config = 0;

volatile uint16_t data_buffer_write_ptr = 0;
volatile uint32_t time_result_write_ptr = 0;
volatile uint16_t ext_trigger_input = DIGITAL_IN_0_Pin;
uint8_t ext_trigger_input_value = 0;
uint8_t ext_trigger_input_value_debounced = 0;

static uint8_t _singleshot = 0;
uint8_t _trigger_mode = TRIGGER_MODE_CONTINUOUS; // indicates if trigger mode is disabled (TRIGGER_MODE_CONTINUOUS) or by external trigger (TRIGGER_MODE_EXTERNAL)
uint32_t _debounce_time_ext_input = 0;
uint32_t _debounce_prev_time = 0 ;
static RTC_TimeTypeDef current_time;
static RTC_DateTypeDef current_date;

// Variable for retrieving the
s_date_time_t current_date_time;

uint16_t tim3_counter = 0;
uint8_t tim14_event = 0;

log_mode_t logMode = LOGMODE_CSV;
uint8_t _data_lines_per_transaction = DATA_LINES_PER_SPI_TRANSACTION;

extern uint8_t spi_ctrl_state;
uint8_t overrun = 0;
uint8_t datardypin;
uint8_t busy = 0;
uint16_t adc16bBuffer[16];
uint16_t adc12Buffer[8*8];
uint16_t tbuffer[8];
uint16_t correctedAdc = 0;
uint16_t iirFilter[8];

adc_resolution_t adc_resolution = ADC_12_BITS;
adc_channel_range_t adc_voltage_range_g = ADC_RANGE_10V;
uint16_t adcCounter = 0;

extern lut_t * active_lut_table[NUM_ADC_CHANNELS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// From https://github.com/LonelyWolf/stm32/blob/master/stm32l-dosfs/RTC.c
// Convert epoch time to Date/Time structures



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim == &htim14)
	{
		// Disable interrupt
//		TIM14->DIER &= ~TIM_DIER_UIE;
//		CLEAR_BIT(TIM14->DIER, TIM_DIER_UIE);

		TIM14->CNT = 0;
		// Indicate timeout
		SET_BIT(spi_ctrl_state, SPI_CTRL_TX_TIMEOUT);
	}

	if (htim == &htim16)
	{
//		TIM16->DIER &= ~TIM_DIER_UIE;
//		CLEAR_BIT(TIM16->DIER, TIM_DIER_UIE);
		//		CLEAR_BIT(TIM14->DIER, TIM_DIER_UIE);
		TIM16->CNT = 0;
		// Indicate timeout
		SET_BIT(spi_ctrl_state, SPI_CTRL_RX_TIMEOUT);
	}

	if (htim == &htim3 )
	  {


		/* spec sec.7: behavior preserved verbatim, do not 'fix' in Phase 1 */
		if (busy)
		{
			Error_Handler();
		}
		busy = 1;


		 // Check which version of the timer triggered this callback and toggle LED
		// Should be RTC_FORMAT_BCD, but there's a bug in the HAL_RTC_Gettime function
		HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &current_date, RTC_FORMAT_BIN);


		current_date_time.year = current_date.Year;
		current_date_time.month = current_date.Month;
		current_date_time.date = current_date.Date;
		current_date_time.hours = current_time.Hours;
		current_date_time.minutes = current_time.Minutes;
		current_date_time.seconds = current_time.Seconds;
		// Next line not 100% correct!
		current_date_time.subseconds = 1000 * (current_time.SecondFraction - current_time.SubSeconds) / (current_time.SecondFraction + 1);

		// Deposit one sample line into the active half (buffer-writing owned by framing).
		frame_append_line(&current_date_time, (uint8_t)(GPIOB->IDR >> 8), iirFilter, adc_resolution);

		tim3_counter++;

	  }
	  busy = 0; // reset interrupt timeout
}



void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

		if (adc_resolution == ADC_12_BITS)
		{
			for (int i = 0; i<8; i++)
			{
				//  correct adc values for non-linearities
				iirFilter[i] = adc_comp(active_lut_table[i], &(adc12Buffer[i]));
			}

		} else {
			for (int i = 0; i<8; i++)
			{
				// First correct adc values for non-linearities
				correctedAdc = adc_comp(active_lut_table[i],&(adc16bBuffer[i]));
				// Then filter
				iir_filter(&correctedAdc, &(iirFilter[i]), i);
			}
		}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		
		if (adc_resolution == ADC_12_BITS)
		{
			for (int i = 0; i<8; i++)
			{
				// First correct adc values for non-linearities
				iirFilter[i] = adc_comp(active_lut_table[i], &(adc12Buffer[i+8*4]));
			}
		
		} else {
			for (int i = 0; i<8; i++)
			{
				// First correct adc values for non-linearities
				correctedAdc = adc_comp(active_lut_table[i], &(adc16bBuffer[i+8]));
				// Then filter
				iir_filter(&correctedAdc, &(iirFilter[i]), i);
			}
		}

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

void Adc_start()
{
	if (adc_resolution == ADC_12_BITS)
	{
		HAL_ADC_Start_DMA(
		&hadc1,
		(uint32_t*)(adc12Buffer),
		8*8);
	} else {
		HAL_ADC_Start_DMA(
		&hadc1,
		(uint32_t*)(adc16bBuffer),
		16);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */


  // Set MISO pin drive strenght to High speed (bit 8 and 9 = '10' (bit 9 = 1))
  GPIOB->OSPEEDR |= (0x0200);

  // Backup current adc settings
  hadc1_bak = hadc1;



  iir_init();


  HAL_TIM_Base_Start(&htim14);
  HAL_TIM_Base_Start(&htim16);


  // Turn off the interrupt flag for timer 14.
  CLEAR_BIT(TIM14->DIER, TIM_DIER_UIE);
  CLEAR_BIT(TIM16->DIER, TIM_DIER_UIE);

  frame_init();


  HAL_ADCEx_Calibration_Start(&hadc1);
  Adc_start();
  busy = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  spi_ctrl_loop();
//	  Config_Handler();
	  datardypin = HAL_GPIO_ReadPin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin);

	  busy = 0; // reset interrupt timeout

	  // forward trigger input to esp32
	  if (HAL_GPIO_ReadPin(GPIOB, ext_trigger_input))
	  {
	      // Input is high and hasn't been debounced yet
	      if (ext_trigger_input_value_debounced == 0)
	      {
	          // Start debouncing: set the previous time to the current time
	          _debounce_prev_time = HAL_GetTick();
	          ext_trigger_input_value_debounced = 1;  // Mark as debouncing
	      }

	      // Check if debounce time has passed
	      if ((HAL_GetTick() - _debounce_prev_time > _debounce_time_ext_input))
	      {
	    	  ext_trigger_input_value = 1;  // Set the output high
	      }
	  }
	  else
	  {
	      // Input is low, reset debouncing and immediately set output low
	      ext_trigger_input_value = 0;
	      ext_trigger_input_value_debounced = 0;  // Reset debouncing state
	      _debounce_prev_time = HAL_GetTick();    // Reset debounce timer
	  }

	  HAL_GPIO_WritePin(EXT_PIN_VALUE_GPIO_Port, EXT_PIN_VALUE_Pin, ext_trigger_input_value);

	  switch(MainState)
	  {
	  	  case MAIN_WAIT_FOR_TRIGGER:


	  		if (ext_trigger_input_value && logging_en)
	  		{
	  			// Debounce the input

	  				// start the ADC timer
	  			  tim3_counter = 0;
				  frame_reset();
				  time_result_write_ptr = 0;
				  TIM3->CNT = 0;
				  NextState = MAIN_LOGGING;
				  HAL_TIM_Base_Start_IT(&htim3);

	  		} else if (!logging_en){
	  			NextState = MAIN_IDLE;
	  		}

		  break;

	  	  case MAIN_LOGGING:

	  		// Forward the state of the external input to the ESP32 via EXT_PIN_VALUE_Pin
	  		// This way the ESP32 knows logging has stopped and data needs to be retrieved.

	  		{
				uint8_t *buf; uint16_t len;
				if (frame_take_ready(&buf, &len))
				{
	//			gpio_result_write_ptr = 0;
	//			time_result_write_ptr = 0;
					// Half way we have the pointers start at the beginning
//				if (READ_BIT(spi_ctrl_state,SPI_CTRL_SENDING))
//				{
//					overrun = 1;
//				}

					tim3_counter=0;

					spi_ctrl_send(buf, len);

				}
			}


	  	  if ((!logging_en || overrun) || (ext_trigger_input_value == 0 && _trigger_mode == TRIGGER_MODE_EXTERNAL))
		  {
//				  if (overrun)
//				  {
//					  HAL_GPIO_WritePin(DATA_OVERRUN_GPIO_Port , DATA_OVERRUN_Pin, SET);
//				  }
//				  overrun =0;
			  // reset the this variable to 0, since we expect that a "

			  HAL_TIM_Base_Stop_IT(&htim3);

			  // Delay of 50 ms, since signal ringing may cause a retrigger of LOGGING state
			  HAL_Delay(50);
			  // Set ADC to single conversion measure mode

			  if (ext_trigger_input_value == 0 && _trigger_mode == TRIGGER_MODE_EXTERNAL && logging_en)
			  {
				  NextState = MAIN_WAIT_FOR_TRIGGER;
			  } else {
				  NextState = MAIN_IDLE;
			  }
		  }
		  break;


		  case MAIN_IDLE:
			  // In case logging gets enabled and we are in continuous mode, start the ADC
			  if (logging_en && spi_ctrl_isIdle() && (_trigger_mode != TRIGGER_MODE_EXTERNAL))
			  {

				  tim3_counter = 0;
				  frame_reset();
				  time_result_write_ptr = 0;
				  // Start TIM3 and DMA conversion
				  TIM3->CNT = 0;

				  NextState = MAIN_LOGGING;

				  HAL_TIM_Base_Start_IT(&htim3);


			  } else if (logging_en && spi_ctrl_isIdle() && _trigger_mode == TRIGGER_MODE_EXTERNAL) {
				 // In this case we wait for the external trigger to become high
				  _debounce_prev_time = HAL_GetTick();
				  NextState = MAIN_WAIT_FOR_TRIGGER;

			  } else  {
				  // Check for events

				  if (spi_ctrl_msg_received())
				  {
					spi_cmd_t * cmd = (spi_cmd_t*)&cmd_buffer;
					spi_cmd_t resp;
					  switch(cmd->command)
					  {

						  case STM32_CMD_SETTINGS_MODE:
							  resp.command = STM32_CMD_SETTINGS_MODE;
							  resp.data = CMD_RESP_OK;
					//			  if (HAL_SPI_Send_cmd(STM32_CMD_SETTINGS_MODE, CMD_RESP_OK) == HAL_OK)
							  if (spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t)) == HAL_OK)
							  {

								HAL_ADC_Stop_DMA(&hadc1);

								NextState = MAIN_CONFIG;
							  }

						  break;

						  case STM32_CMD_SINGLE_SHOT_MEASUREMENT:

							  resp.command = STM32_CMD_SINGLE_SHOT_MEASUREMENT;
							  resp.data = CMD_RESP_OK;
					//			  if (HAL_SPI_Send_cmd(STM32_CMD_SINGLE_SHOT_MEASUREMENT, CMD_RESP_OK) == HAL_OK)
							  if (spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t)) == HAL_OK)
							  {
								  NextState = MAIN_SINGLE_SHOT;
							  }
							  break;

						  case STM32_CMD_SEND_LAST_ADC_BYTES:
						  {
							  uint8_t *buf; uint16_t len;
							  frame_take_last(&buf, &len, _singleshot, adc_resolution);
							  spi_ctrl_send(buf, len);
							  // Preserve legacy _singleshot reset: it only happened in the 16-bit (!adc_16b_is_half || _singleshot) branch; the 12-bit path never reset it here. Do NOT add a 12-bit reset without tracing the caller flow.
							  if (adc_resolution == ADC_16_BITS && (!frame_adc_16b_is_half() || _singleshot))
							  {
								  _singleshot = 0;
							  }
						  }


							  break;

						  case STM32_CMD_NOP:
					//			  HAL_SPI_Send_cmd(CMD_RESP_OK, CMD_NOP);
//							  resp.command = CMD_NOP;
//							  resp.data = CMD_RESP_OK;
//							  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));
							  break;

						  default:
					//			  HAL_SPI_Send_cmd(CMD_RESP_NOK, CMD_UNKNOWN);
							  resp.command = STM32_CMD_NOP;
							  resp.data = CMD_RESP_NOK;
							  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));

					  }

				  }
				  // No event occured, check for SPI messages
				  else if (spi_ctrl_isIdle())
				  {
					  spi_ctrl_receive(cmd_buffer, sizeof(spi_cmd_t));
				  }
			  }
			  break;

		  case MAIN_CONFIG:

			  if (  main_exit_config )
			  {
				  NextState = MAIN_IDLE;
				  Adc_start();
				  main_exit_config = 0 ;
				  break;
			  }
			  else if (spi_ctrl_msg_received())
			  {
				// Forward the message to the config handler
				Config_Handler((spi_cmd_t*)cmd_buffer);
				break;
			  }
			  // No event occurred, check for SPI messages
			  else if (spi_ctrl_isIdle())
			  {
				  spi_ctrl_receive(cmd_buffer, sizeof(spi_cmd_t));
			  }
			  break;

		  case MAIN_SINGLE_SHOT:
		  {
			  htim3_bak = htim3;
			  tim3_counter = 0;
			  frame_reset();
			  time_result_write_ptr = 0;
        _singleshot = 1;

			  TIM3->CNT = 0;

			  HAL_TIM_Base_Start_IT(&htim3);

			  // Set ADC to single conversion measure mode
			  NextState = MAIN_SINGLE_SHOT_AWAIT_RESULT;

			  break;
		  }

		  case MAIN_SINGLE_SHOT_AWAIT_RESULT:
			  if (spi_ctrl_isIdle())
			  {
				  spi_ctrl_receive(cmd_buffer, sizeof(spi_cmd_t));
			  }
			  // limit our acquisition to 3 samples
			  if (frame_write_ptr() >= 1 && frame_adc_ready())
			  {
				 HAL_TIM_Base_Stop_IT(&htim3);
				 }

				 NextState = MAIN_IDLE;

		  break;

	  }

	  if (NextState != MainState)
	  {
		  MainState = NextState;
	  }

  }// end of while
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV32;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
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
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
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
  hrtc.Init.AsynchPrediv = 15;
  hrtc.Init.SynchPrediv = 2047;
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
  // Check if RTC was already set before. If not, then set it with default value.
  if (__HAL_RTC_GET_FLAG(&hrtc, RTC_FLAG_INITS) == RESET) {
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 12;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 1;
  sDate.Year = 24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  }
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
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 333;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 64000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 333;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 64000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EXT_PIN_VALUE_GPIO_Port, EXT_PIN_VALUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIGITAL_IN_0_Pin DIGITAL_IN_1_Pin DIGITAL_IN_2_Pin DIGITAL_IN_3_Pin
                           DIGITAL_IN_4_Pin DIGITAL_IN_5_Pin */
  GPIO_InitStruct.Pin = DIGITAL_IN_0_Pin|DIGITAL_IN_1_Pin|DIGITAL_IN_2_Pin|DIGITAL_IN_3_Pin
                          |DIGITAL_IN_4_Pin|DIGITAL_IN_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_PIN_VALUE_Pin */
  GPIO_InitStruct.Pin = EXT_PIN_VALUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EXT_PIN_VALUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN_RANGE_SELECT_CLK_Pin AIN_RANGE_SELECT_CLR_Pin AIN_PULLUP_SELECT_CLR_Pin */
  GPIO_InitStruct.Pin = AIN_RANGE_SELECT_CLK_Pin|AIN_RANGE_SELECT_CLR_Pin|AIN_PULLUP_SELECT_CLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : AIN_PULLUP_SELECT_CLK_Pin */
  GPIO_InitStruct.Pin = AIN_PULLUP_SELECT_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AIN_PULLUP_SELECT_CLK_GPIO_Port, &GPIO_InitStruct);

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void ADC_Reinit()
{
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
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
