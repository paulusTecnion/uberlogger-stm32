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

#include <spi_ctrl.h>
#include "config.h"
//#include "msg.h"
#include "stm32g0xx_hal.h"
#include "iirfilter.h"
#include "adc_comp_lut.h"
#include "framing.h"
#include "app.h"

/* Build-visible guard: the two vendored ul_protocol.h copies (STM32 + ESP32)
 * must stay byte-identical; a stale copy bumps the version and fails here. */
_Static_assert(UL_PROTOCOL_VERSION == 2, "ul_protocol.h copies are out of sync");

//extern SPI_HandleTypeDef * hspi1;
extern ADC_HandleTypeDef hadc1;
//extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;

extern RTC_HandleTypeDef hrtc;

/* Task 8: config.c is the single owner of the parse-time settings written by the
 * config command handlers. Consumers read them via the config_*() accessors below
 * (declared in config.h) instead of externing the raw symbols.
 * adc_resolution stays main/app-owned & extern-shared on purpose: it is read in the
 * hot sample ISR (acquisition.c) where a function call would change ISR timing.
 * main_exit_config is an app.c runtime flag set here via app_set_exit_config(). */
extern adc_resolution_t adc_resolution;
static log_mode_t          logMode = LOGMODE_CSV;
static adc_channel_range_t adc_voltage_range_g = ADC_RANGE_10V;
static uint8_t             _trigger_mode = TRIGGER_MODE_CONTINUOUS; // TRIGGER_MODE_CONTINUOUS=disabled, TRIGGER_MODE_EXTERNAL=external trigger
static uint32_t            _debounce_time_ext_input = 0;
static volatile uint16_t   ext_trigger_input = DIGITAL_IN_0_Pin;

/* Trivial accessors (get-only) for the cross-module readers of the above. */
uint8_t  config_trigger_mode(void)            { return _trigger_mode; }
uint16_t config_ext_trigger_input(void)       { return ext_trigger_input; }
uint32_t config_debounce_time_ext_input(void) { return _debounce_time_ext_input; }

/* Low-power trigger settings (config-owned; spec §4.1). */
#define LP_SOURCE_ANALOG  0
#define LP_SOURCE_DIGITAL 1
static uint8_t  _lp_source     = LP_SOURCE_ANALOG;
static uint8_t  _lp_channel    = 1;
static uint16_t _lp_threshold  = 2048;
static uint8_t  _lp_edge       = 0;
static uint16_t _lp_duration_s = 10;

uint8_t  config_lp_source(void)     { return _lp_source; }
uint8_t  config_lp_channel(void)    { return _lp_channel; }
uint16_t config_lp_threshold(void)  { return _lp_threshold; }
uint8_t  config_lp_edge(void)       { return _lp_edge; }
uint16_t config_lp_duration_s(void) { return _lp_duration_s; }

uint8_t Config_set_lpConfig(uint8_t source, uint8_t channel, uint16_t threshold,
                            uint8_t edge, uint16_t duration_s)
{
	if (source > LP_SOURCE_DIGITAL) return 1;
	if (edge > 1) return 1;
	if (duration_s == 0) return 1;
	if (source == LP_SOURCE_ANALOG  && (channel < 1 || channel > 8)) return 1;
	if (source == LP_SOURCE_DIGITAL && (channel < 1 || channel > 6)) return 1;

	_lp_source = source;
	_lp_channel = channel;
	_lp_threshold = threshold;
	_lp_edge = edge;
	_lp_duration_s = duration_s;
	return 0;
}

void Config_Handler(spi_cmd_t *  cmd)
{
//	spi_cmd_t * cmd = (spi_cmd_t*) msg->message_payload;

		spi_cmd_t resp;


		switch(cmd->command)
		{
			  case STM32_CMD_NOP:
				  resp.command = STM32_CMD_NOP;
				  resp.data = CMD_RESP_OK;


				  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));

				  break;

			  case STM32_CMD_MEASURE_MODE:
				  // Re-init ADC
				  resp.command = STM32_CMD_MEASURE_MODE;
				  resp.data = CMD_RESP_OK;


				  if (spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t)) == HAL_OK)
				  {
					  ADC_Reinit();
					  for (int i = 0; i < NUM_ADC_CHANNELS; i++)
					  {
						  adc_set_lut(adc_voltage_range_g,  adc_resolution, i);
					  }

					  app_set_exit_config(1);
				  }
				  break;

			  case STM32_CMD_SET_RESOLUTION:
				  if (!Config_Set_Resolution(cmd->data))
				  {
					  resp.command = STM32_CMD_SET_RESOLUTION;
					  resp.data = CMD_RESP_OK;
				  } else {
					  resp.command = STM32_CMD_SET_RESOLUTION;
					  resp.data = CMD_RESP_NOK;
				  }

				  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));

				  break;


			  case STM32_CMD_SET_SAMPLE_RATE:
							// receive setting
				  resp.command = STM32_CMD_SET_SAMPLE_RATE;
				if (!Config_Set_Sample_freq(cmd->data))
				{
					resp.data = CMD_RESP_OK;

				} else {
					resp.data = CMD_RESP_NOK;

				}
				spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));

				break;

			  case STM32_CMD_SET_DATETIME:
				  resp.command = STM32_CMD_SET_DATETIME;

				  uint32_t epoch;

				  memcpy((void*)&epoch, (const void*)&cmd->data, sizeof(epoch));
				  if (!Config_Set_Time(epoch))
				  {
					  resp.data = CMD_RESP_OK;

				  } else {
					  resp.data = CMD_RESP_NOK;

				  }

				  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));

				  break;

			  case STM32_CMD_SET_LOGMODE:
				  resp.command = STM32_CMD_SET_LOGMODE;
				  if (!Config_set_logMode(cmd->data, cmd->data1))
				  {
					  resp.data = CMD_RESP_OK;
				  } else {
					  resp.data = CMD_RESP_NOK;
				  }

				  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));
				  break;

			  case STM32_CMD_SET_RANGE:
				  resp.command = STM32_CMD_SET_RANGE;
				  if (!Config_set_range((adc_channel_range_t)cmd->data))
				  {
					  resp.data = CMD_RESP_OK;
				  } else {
					  resp.data = CMD_RESP_NOK;
				  }

				  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));
				  break;

			  case STM32_CMD_SET_TRIGGER_MODE:
				  uint32_t debounceTime;

				  memcpy((void*)&debounceTime, (const void*)&cmd->data2, sizeof(debounceTime));

				  resp.command = STM32_CMD_SET_TRIGGER_MODE;
				  if (!Config_set_triggerMode(cmd->data, cmd->data1) &&
						  (!Config_set_debounceTime(debounceTime)))
				  {
					  resp.data = CMD_RESP_OK;
				  } else {
					  resp.data = CMD_RESP_NOK;
				  }

				  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));
				  break;



			  case STM32_CMD_SET_LP_CONFIG:
			  {
				  uint16_t lpThreshold, lpDuration;
				  memcpy((void*)&lpThreshold, (const void*)&cmd->data2, sizeof(lpThreshold));
				  memcpy((void*)&lpDuration,  (const void*)&cmd->data5, sizeof(lpDuration));

				  resp.command = STM32_CMD_SET_LP_CONFIG;
				  if (!Config_set_lpConfig(cmd->data, cmd->data1, lpThreshold,
						  cmd->data4, lpDuration))
				  {
					  resp.data = CMD_RESP_OK;
				  } else {
					  resp.data = CMD_RESP_NOK;
				  }
				  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));
				  break;
			  }

			  default:
				  resp.command = CMD_UNKNOWN;
				  resp.data = CMD_RESP_NOK;
				  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));

		  }

}


uint8_t Config_set_range(adc_channel_range_t range)
{
	// Range can be anything between 0 and 255
	adc_voltage_range_g = range;
	return 0;
}


uint8_t Config_set_logMode(uint8_t logtype, uint8_t data_lines_per_transaction)
{
	switch (logtype)
	{
	case LOGMODE_CSV:
	case LOGMODE_RAW:
		logMode = logtype;
//		_data_lines_per_transaction = data_lines_per_transaction;
		return 0;
		break;

	default:

		logMode = LOGMODE_UNKNOWN_TYPE;
		return 1;

	}

}

uint8_t Config_set_triggerMode(uint8_t mode, uint8_t gpio)
{
	if ((mode > TRIGGER_MODE_LOW_POWER) || ((gpio < 1) || (gpio > 6)))
		return 1;
	_trigger_mode = mode;

	switch (gpio)
	{
	case 1:
		ext_trigger_input = DIGITAL_IN_0_Pin;
		break;

	case 2:
		ext_trigger_input = DIGITAL_IN_1_Pin;
		break;

	case 3:
		ext_trigger_input = DIGITAL_IN_2_Pin;
		break;

	case 4:
		ext_trigger_input = DIGITAL_IN_3_Pin;
		break;

	case 5:
		ext_trigger_input = DIGITAL_IN_4_Pin;
		break;

	case 6:
		ext_trigger_input = DIGITAL_IN_5_Pin;
		break;
	}

	return 0;
}

uint8_t Config_set_debounceTime(uint32_t debounceTime)
{
	if (debounceTime > MAX_DEBOUNCE_TIME)
		return 1;

	_debounce_time_ext_input = debounceTime;
	return 0;
}

uint8_t Config_Set_Time(uint32_t epoch)
{

	RTC_TimeTypeDef time = {0};
	RTC_DateTypeDef date = {0};

	time.Hours = (epoch / 3600) % 24; // Extract hours (range: 0-23)
	time.Minutes = (epoch / 60) % 60; // Extract minutes (range: 0-59)
	time.Seconds = epoch % 60; // Extract seconds (range: 0-59)


	time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	time.StoreOperation = RTC_STOREOPERATION_RESET;


	// Step 2: Convert Unix timestamp to RTC date structure
	uint32_t days = epoch / 86400; // Number of days since January 1, 1970

	date.WeekDay = (days + 4) % 7; // Calculate the day of the week (0: Sunday, 1: Monday, ..., 6: Saturday)

	// Calculate the year, month, and day
	int16_t year = 1970;
	int16_t month = 1;
	int16_t day = 1;

	while (days >= 365) {
	    if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0) {
	        if (days >= 366) {
	            days -= 366;
	            year++;
	        }
	    } else {
	        days -= 365;
	        year++;
	    }
	}

	while (days > 0) {
	    int16_t daysInMonth = 0;
	    switch (month) {
	        case 1: case 3: case 5: case 7: case 8: case 10: case 12:
	            daysInMonth = 31;
	            break;
	        case 4: case 6: case 9: case 11:
	            daysInMonth = 30;
	            break;
	        case 2:
	            if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
	                daysInMonth = 29;
	            else
	                daysInMonth = 28;
	            break;
	    }

	    if (days >= daysInMonth) {
	        days -= daysInMonth;
	        month++;
	        if (month > 12) {
	            month = 1;
	            year++;
	        }
	    } else {
	        day += days;
	        days = 0;
	    }
	}

	date.Year = year - 2000;
	date.Month = month;
	date.Date = day;


	// Step 3: Set RTC date and time
	HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
	HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);

	return 0;
}

uint8_t Config_Set_Sample_freq(uint8_t sampleFreq)
{

	// Make sure timer3 has stopped
	HAL_TIM_Base_Stop_IT(&htim3);


	 htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	 htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	 htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	 // Please look at
	 // https://tecnionnl.sharepoint.com/:x:/s/uberlogger/EeEoN_zLy7BHslnFgKYobd4BH9o46vYH16z9PU2SE_CJCw?e=748AyK
	 // for the prescaler values when using 16 bit adc

	 // User must set the resolution before setting the sample rate!

	 if (adc_resolution == ADC_16_BITS)
	 {
		 hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
		  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		  hadc1.Init.ContinuousConvMode = ENABLE;
	 } else {
		hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
		hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		hadc1.Init.ContinuousConvMode = ENABLE;
	 }
	 // set the sample frequency for the iir filter
	 if (sampleFreq < ADC_SAMPLE_RATE_1Hz)
	 {
		 sampleFreq = ADC_SAMPLE_RATE_25Hz;
	 }

	iir_set_samplefreq(sampleFreq);


	 switch(sampleFreq)
	 {


		 case ADC_SAMPLE_RATE_1Hz:
			 // Reconfig the timer

			 frame_set_lines_per_transaction(1);
			 htim3.Init.Prescaler = 1000-1;
			 htim3.Init.Period = 64000 ;


			 break;

		 case ADC_SAMPLE_RATE_2Hz:
				 // Reconfig the timer

			 frame_set_lines_per_transaction(2);
			 htim3.Init.Prescaler = 500-1;
			 htim3.Init.Period = 64000 ;


			break;

		 case ADC_SAMPLE_RATE_5Hz:

			 frame_set_lines_per_transaction(5);
			 htim3.Init.Prescaler = 200-1;
			 htim3.Init.Period = 64000 ;
				break;

		 case ADC_SAMPLE_RATE_10Hz:

			 frame_set_lines_per_transaction(10);
			 htim3.Init.Prescaler = 100-1;
			 htim3.Init.Period = 64000;


			 break;

		 case ADC_SAMPLE_RATE_EVERY_3600S:
		 case ADC_SAMPLE_RATE_EVERY_600S:
		 case ADC_SAMPLE_RATE_EVERY_300S:
		 case ADC_SAMPLE_RATE_EVERY_60S:
		 case ADC_SAMPLE_RATE_EVERY_10S:
		 case ADC_SAMPLE_RATE_25Hz:


			 frame_set_lines_per_transaction(25);
			htim3.Init.Prescaler = 100-1;
			htim3.Init.Period = 25600;
			 break;

		 case ADC_SAMPLE_RATE_50Hz:
			 frame_set_lines_per_transaction(50);
			htim3.Init.Prescaler = 100-1;
			htim3.Init.Period = 12800;
	//		htim3.Init.Prescaler = 639;
	//		htim3.Init.Period = 1000;

			 break;

		 case 	ADC_SAMPLE_RATE_100Hz:
			 frame_set_lines_per_transaction(DATA_LINES_PER_SPI_TRANSACTION);
	//		if (is16bitmode)
	//		{

				// prescale 16
	//			ADC1_COMMON->CCR  |= ADC_CCR_PRESC_0;
	//			ADC1_COMMON->CCR  |= ADC_CCR_PRESC_1;
	//			ADC1_COMMON->CCR  |= ADC_CCR_PRESC_2;
	//			hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
	//		}

	//		htim3.Init.Prescaler = 10-1;
	//		htim3.Init.Period = 64000;
			htim3.Init.Prescaler = 639;
			htim3.Init.Period = 1000;
			 break;

		 case 	ADC_SAMPLE_RATE_250Hz:
			 frame_set_lines_per_transaction(DATA_LINES_PER_SPI_TRANSACTION);
	//		if (adc_resolution == 1)
	//		{
				// prescale 8
	//			ADC1_COMMON->CCR  |= ADC_CCR_PRESC_2;
	//			hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
	//		}

			htim3.Init.Prescaler = 255;
			htim3.Init.Period = 1000;

			 break;

	/* --- Phase 2 reference: candidate prescaler/period values for >250 Hz (see refactor spec docs/superpowers/specs/2026-06-05-uberlogger-stm32-refactor-design.md sec. 11) --- */
	//	 case ADC_SAMPLE_RATE_500Hz:
	//		 spi_lines_per_transaction = DATA_LINES_PER_SPI_TRANSACTION;
	//		htim3.Init.Prescaler = 127;
	//		htim3.Init.Period = 1000;
	//
	//
	//		 break;

	//	 case ADC_SAMPLE_RATE_1000Hz:
	//		 spi_lines_per_transaction = DATA_LINES_PER_SPI_TRANSACTION;
	//		 htim3.Init.Prescaler = 63;
	//		 htim3.Init.Period = 1000;
	//
	//	 break;

	//	 case ADC_SAMPLE_RATE_2000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 64000;
	//
	//		 break;
	//
	//	 case ADC_SAMPLE_RATE_4000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 16000;
	//
	//		 break;

	//	 case ADC_SAMPLE_RATE_2500Hz:
	//			 htim3.Init.Prescaler = 2-1;
	//			 htim3.Init.Period = 12800-1;
	//
	//			 break;


	//	 case ADC_SAMPLE_RATE_5000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 12800;
	//
	//		 break;

	//	 case ADC_SAMPLE_RATE_8000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 8000;
	//
	//		 break;
	//	 case ADC_SAMPLE_RATE_10000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 6400;
	//
	//		 break;
	//	 case ADC_SAMPLE_RATE_20000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 3200;
	//
	//		 break;
	//
	//	 case ADC_SAMPLE_RATE_40000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 1600;
	//
	//		 break;
	//
	//	 case ADC_SAMPLE_RATE_50000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 1280;
	//
	//		 break;
	//
	//	 case ADC_SAMPLE_RATE_100000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 640;
	//
	//		 break;

	//	 case ADC_SAMPLE_RATE_250000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 256;
	//
	//		 break;
	//
	//	 case ADC_SAMPLE_RATE_500000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 128;
	//
	//		 break;
	//
	//	 case ADC_SAMPLE_RATE_1000000Hz:
	//		 htim3.Init.Prescaler = 1;
	//		 htim3.Init.Period = 64;
	//
	//		 break;
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


uint8_t Config_Set_Adc_channels(uint8_t channels)
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


uint8_t Config_Set_Resolution(uint8_t resolution)
{
	switch (resolution)
	{


	case ADC_16_BITS:
		adc_resolution = ADC_16_BITS;
		hadc1.Init.OversamplingMode = ENABLE;
		hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
		hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
		hadc1.Init.ContinuousConvMode = DISABLE;
		hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
		hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;

		break;

	//case ADC_12_BITS:
	default:
		adc_resolution = ADC_12_BITS;
		hadc1.Init.OversamplingMode = DISABLE;
		hadc1.Init.Resolution = ADC_RESOLUTION_12B;
		hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
		hadc1.Init.ContinuousConvMode = DISABLE;
		hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
		hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
		break;


	}

	return 0;

}



