#include "config.h"
#include "msg.h"
#include "stm32g0xx_hal.h"

extern SPI_HandleTypeDef * hspi1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;

extern MessageQueue_t mainQ;

extern RTC_HandleTypeDef hrtc;
MessageQueue_t configQ;




void Config_Handler(spi_cmd_t *  cmd)
{
//	spi_cmd_t * cmd = (spi_cmd_t*) msg->message_payload;

		spi_cmd_t resp;


		switch(cmd->command)
		{
			  case CMD_NOP:
				  resp.command = CMD_NOP;
				  resp.data = CMD_RESP_OK;

				  spi_ctrl_send((uint8_t*)&resp, sizeof(resp));
				  //HAL_SPI_Send_cmd(CMD_RESP_OK, CMD_NOP);
				  break;

			  case STM32_CMD_MEASURE_MODE:
				  // Re-init ADC
				  resp.command = STM32_CMD_MEASURE_MODE;
				  resp.data = CMD_RESP_OK;


	//			  if (HAL_SPI_Send_cmd(STM32_CMD_MEASURE_MODE, CMD_RESP_OK) == HAL_OK)
				  if (spi_ctrl_send((uint8_t*)&resp, sizeof(resp)) == HAL_OK)
				  {
					  ADC_Reinit();
					  Message_t t;
					  t.event = EVENT_CONFIG_EXIT_HANDLER;
					  msgq_enqueue(&mainQ, t);
				  }
				  break;

			  case STM32_CMD_SET_RESOLUTION:
				  if (!Config_Set_Resolution(cmd->data))
				  {
					  resp.command = STM32_CMD_SET_RESOLUTION;
					  resp.data = CMD_RESP_OK;

	//				  HAL_SPI_Send_cmd(STM32_CMD_SET_RESOLUTION, CMD_RESP_OK);
				  } else {
					  resp.command = STM32_CMD_SET_RESOLUTION;
					  resp.data = CMD_RESP_NOK;

	//				  HAL_SPI_Send_cmd(STM32_CMD_SET_RESOLUTION, CMD_RESP_NOK);
				  }

				  spi_ctrl_send((uint8_t*)&resp, sizeof(resp));

				  break;


			  case STM32_CMD_SET_SAMPLE_RATE:
							// receive setting
				  resp.command = STM32_CMD_SET_SAMPLE_RATE;
				if (!Config_Set_Sample_freq(cmd->data))
				{
					resp.data = CMD_RESP_OK;
	//				HAL_SPI_Send_cmd(STM32_CMD_SET_SAMPLE_RATE, f);
				} else {
					resp.data = CMD_RESP_NOK;
	//				HAL_SPI_Send_cmd(STM32_CMD_SET_SAMPLE_RATE, CMD_RESP_NOK);
				}
				spi_ctrl_send((uint8_t*)&resp, sizeof(resp));

				break;

			  case STM32_CMD_SET_ADC_CHANNELS_ENABLED:
				  resp.command = STM32_CMD_SET_ADC_CHANNELS_ENABLED;

				  if (!Config_Set_Adc_channels(cmd->data))
				  {
					  resp.data = CMD_RESP_OK;
	//				  HAL_SPI_Send_cmd(STM32_CMD_SET_ADC_CHANNELS_ENABLED, CMD_RESP_OK);
				  } else {
					  resp.data = CMD_RESP_NOK;
	//				  HAL_SPI_Send_cmd(STM32_CMD_SET_ADC_CHANNELS_ENABLED, CMD_RESP_NOK);
				  }

				  spi_ctrl_send((uint8_t*)&resp, sizeof(resp));
				  break;

		//			  case STM32_CMD_CONFIG_SET_DAC_PWM:
		//
		//				  break;

			  default:
				  resp.command = CMD_UNKNOWN;
				  resp.data = CMD_RESP_NOK;
				  spi_ctrl_send((uint8_t*)&resp, sizeof(resp));
	//			  HAL_SPI_Send_cmd(CMD_RESP_NOK, CMD_UNKNOWN);

		  }

}

//static void ADC_Set_Single_Acq()
//{
//
//
//	  hadc1.Instance = ADC1;
//	  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
//	  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//	  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
//	  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
//	  hadc1.Init.LowPowerAutoWait = DISABLE;
//	  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
//	  hadc1.Init.ContinuousConvMode = ENABLE;
//	  hadc1.Init.NbrOfConversion = 8;
//	  hadc1.Init.DiscontinuousConvMode = DISABLE;
//	  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
//	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
//	  hadc1.Init.DMAContinuousRequests = DISABLE;
//	  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//	  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
//	  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
//	  hadc1.Init.OversamplingMode = DISABLE;
//	  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
//
//
//}

uint8_t Config_Set_Sample_freq(uint8_t sampleFreq)
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



