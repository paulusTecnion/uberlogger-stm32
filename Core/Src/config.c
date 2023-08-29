#include <spi_ctrl.h>
#include "config.h"
//#include "msg.h"
#include "stm32g0xx_hal.h"
#include "iirfilter.h"

extern SPI_HandleTypeDef * hspi1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;

extern RTC_HandleTypeDef hrtc;
extern uint8_t main_exit_config ;
extern log_mode_t logMode;
extern uint8_t _data_lines_per_transaction;
extern uint8_t is16bitmode;

void Config_Handler(spi_cmd_t *  cmd)
{
//	spi_cmd_t * cmd = (spi_cmd_t*) msg->message_payload;

		spi_cmd_t resp;


		switch(cmd->command)
		{
			  case CMD_NOP:
				  resp.command = CMD_NOP;
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
					  main_exit_config = 1;
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

			  case STM32_CMD_SET_ADC_CHANNELS_ENABLED:
				  resp.command = STM32_CMD_SET_ADC_CHANNELS_ENABLED;

				  if (!Config_Set_Adc_channels(cmd->data))
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


			  default:
				  resp.command = CMD_UNKNOWN;
				  resp.data = CMD_RESP_NOK;
				  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));


		  }

}



uint8_t Config_set_logMode(uint8_t logtype, uint8_t data_lines_per_transaction)
{
	switch (logtype)
	{
	case LOGMODE_CSV:
	case LOGMODE_RAW:
		logMode = logtype;
		_data_lines_per_transaction = data_lines_per_transaction;
		return 0;
		break;

	default:

		logMode = LOGMODE_UNKNOWN_TYPE;
		return 1;

	}

}

uint8_t Config_Set_Time(uint32_t epoch)
{

	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;

//	uint32_t tm;
//	uint32_t t1;
//	uint32_t a;
//	uint32_t b;
//	uint32_t c;
//	uint32_t d;
//	uint32_t e;
//	uint32_t m;
//	int16_t  year  = 0;
//	int16_t  month = 0;
//	int16_t  dow   = 0;
//	int16_t  mday  = 0;
//	int16_t  hour  = 0;
//	int16_t  min   = 0;
//	int16_t  sec   = 0;
//	uint64_t JD    = 0;
//	uint64_t JDN   = 0;
//
//	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day
//
//	JD  = ((epoch + 43200) / (86400 >>1 )) + (2440587 << 1) + 1;
//	JDN = JD >> 1;
//
//	tm = epoch; t1 = tm / 60; sec  = tm - (t1 * 60);
//	tm = t1;    t1 = tm / 60; min  = tm - (t1 * 60);
//	tm = t1;    t1 = tm / 24; hour = tm - (t1 * 24);
//
//
//	dow   = JDN % 7;
//	a     = JDN + 32044;
//	b     = ((4 * a) + 3) / 146097;
//	c     = a - ((146097 * b) / 4);
//	d     = ((4 * c) + 3) / 1461;
//	e     = c - ((1461 * d) / 4);
//	m     = ((5 * e) + 2) / 153;
//	mday  = e - (((153 * m) + 2) / 5) + 1;
//	month = m + 3 - (12 * (m / 10));
//	year  = (100 * b) + d - 4800 + (m / 10);
//
//	date.Year    = year - 2000;
//	date.Month   = month;
//	date.Date    = mday;
//	date.WeekDay = dow;
//	time.Hours   = hour;
//	time.Minutes = min;
//	time.Seconds = sec;
//
//
//	HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
//	HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);

	time.Hours = (epoch / 3600) % 24; // Extract hours (range: 0-23)
	time.Minutes = (epoch / 60) % 60; // Extract minutes (range: 0-59)
	time.Seconds = epoch % 60; // Extract seconds (range: 0-59)

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

	 if (is16bitmode)
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
	//  else {
	// 	 hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	// 	  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
	// 	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	// 	  hadc1.Init.ContinuousConvMode = DISABLE;
	//  }


	 // always set prescaler to 32, except when in 16 bit mode and having 100Hz or 250 Hz logging rate
//	 if (!(is16bitmode && (sampleFreq != ADC_SAMPLE_RATE_100Hz || sampleFreq != ADC_SAMPLE_RATE_250Hz)))
//	 {
//		 // Set to prescaler 32. See page 332 of tech reference
//		ADC1_COMMON->CCR  |= ADC_CCR_PRESC_3;
//	 }

	 // set the sample frequency for the iir filter

	iir_set_samplefreq(sampleFreq);



	 switch(sampleFreq)
	 {
	 case ADC_SAMPLE_RATE_1Hz:
		 // Reconfig the timer


		 htim3.Init.Prescaler = 1000-1;
		 htim3.Init.Period = 64000 ;


		 break;

	 case ADC_SAMPLE_RATE_2Hz:
	 		 // Reconfig the timer


		 htim3.Init.Prescaler = 500-1;
		 htim3.Init.Period = 64000 ;


	 	break;

	 case ADC_SAMPLE_RATE_5Hz:


		 htim3.Init.Prescaler = 200-1;
		 htim3.Init.Period = 64000 ;
		 	break;

	 case ADC_SAMPLE_RATE_10Hz:


		 htim3.Init.Prescaler = 100-1;
		 htim3.Init.Period = 64000;


		 break;

	 case ADC_SAMPLE_RATE_25Hz:


		htim3.Init.Prescaler = 100-1;
		htim3.Init.Period = 25600;
		 break;

	 case ADC_SAMPLE_RATE_50Hz:

		htim3.Init.Prescaler = 100-1;
		htim3.Init.Period = 12800;
//		htim3.Init.Prescaler = 639;
//		htim3.Init.Period = 1000;

		 break;

	 case 	ADC_SAMPLE_RATE_100Hz:
		if (is16bitmode)
		{

			// prescale 16
//			ADC1_COMMON->CCR  |= ADC_CCR_PRESC_0;
//			ADC1_COMMON->CCR  |= ADC_CCR_PRESC_1;
//			ADC1_COMMON->CCR  |= ADC_CCR_PRESC_2;
//			hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
		}

//		htim3.Init.Prescaler = 10-1;
//		htim3.Init.Period = 64000;
		htim3.Init.Prescaler = 639;
		htim3.Init.Period = 1000;
		 break;

	 case 	ADC_SAMPLE_RATE_250Hz:
		if (is16bitmode)
		{
			// prescale 8
//			ADC1_COMMON->CCR  |= ADC_CCR_PRESC_2;
//			hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
		}

		htim3.Init.Prescaler = 255;
		htim3.Init.Period = 1000;

		 break;

//	 case ADC_SAMPLE_RATE_500Hz:
//		if (use16bit)
//		{
//			// prescale 64
//			ADC1_COMMON->CCR  |= ADC_CCR_PRESC_0;
//			ADC1_COMMON->CCR  |= ADC_CCR_PRESC_3;
//		}
//
//		htim3.Init.Prescaler = 10-1;
//		htim3.Init.Period = 12800;
//
//
//		 break;

//	 case ADC_SAMPLE_RATE_1000Hz:
//		 htim3.Init.Prescaler = 1-1;
//		 htim3.Init.Period = 64000-1;
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
		is16bitmode = 1;
		hadc1.Init.OversamplingMode = ENABLE;
		hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
		hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
		hadc1.Init.ContinuousConvMode = DISABLE;
		hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
		hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;

		break;

	//case ADC_12_BITS:
	default:
		is16bitmode = 0;
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



