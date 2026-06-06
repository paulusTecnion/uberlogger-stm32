/*
 * app.c — the application state machine extracted verbatim from main.c.
 *
 * Holds the MainState/NextState machine (IDLE/CONFIG/LOGGING/SINGLE_SHOT/
 * WAIT_FOR_TRIGGER), the external-trigger debounce, the IDLE command dispatch,
 * and the two GPIO EXTI weak callbacks that drive logging_en.
 *
 * Phase 1, Task 7: behavior-preserving module extraction. The former while(1)
 * body is now app_run_once(); one pass per call. No logic was changed.
 */

#include "main.h"
#include "framing.h"
#include "acquisition.h"
#include "spi_ctrl.h"
#include "config.h"

/* TIM3 is owned by the CubeMX HAL handles in main.c; the state machine drives
 * it via HAL_TIM_Base_Start_IT/Stop_IT and the htim3_bak snapshot below. */
extern TIM_HandleTypeDef htim3;

/* ---- State-machine-owned globals (moved verbatim from main.c) ----
 * Globals still referenced by other TUs (config.c, acquisition.c, spi_ctrl.c,
 * framing.c) are kept with external linkage so their existing extern
 * declarations resolve here; purely loop-local ones are static.
 * Task 8 will consolidate ownership of the settings-ish globals.            */

static uint8_t MainState = MAIN_IDLE, NextState = MAIN_IDLE;
static uint8_t logging_en = 0;          /* app-private: set by the EXTI callbacks, read by the state machine */
static uint8_t msgRx = 0;
static uint8_t cmd_buffer[ 20];
//spi_cmd_t cmd_buffer;

static TIM_HandleTypeDef htim3_bak;

static uint8_t main_exit_config = 0;    /* app-owned; config.c sets via app_set_exit_config() */

static volatile uint16_t data_buffer_write_ptr = 0;
static volatile uint32_t time_result_write_ptr = 0;
/* ext_trigger_input / _trigger_mode / _debounce_time_ext_input are config-owned
 * settings now (config.c); read here via config_*() accessors. */
static uint8_t ext_trigger_input_value = 0;
static uint8_t ext_trigger_input_value_debounced = 0;

static uint8_t _singleshot = 0;
static uint32_t _debounce_prev_time = 0 ;

uint16_t tim3_counter = 0;              /* shared: acquisition.c externs this */
static uint8_t tim14_event = 0;

/* logMode / adc_voltage_range_g are config-owned settings now (config.c). */

static uint8_t datardypin;
static uint16_t tbuffer[8];

adc_resolution_t adc_resolution = ADC_12_BITS;            /* shared: acquisition.c/framing.c/config.c (hot-ISR read, stays extern) */
static uint16_t adcCounter = 0;


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


/* Setter so config.c can drive the app-owned CONFIG-exit flag without externing
 * the raw symbol. Trivial assignment; no side effects. */
void app_set_exit_config(uint8_t v)
{
	main_exit_config = v;
}


void app_init(void)
{
	/* One-time pre-loop state init. The state-machine globals are initialised
	 * at their definitions above (MainState=MAIN_IDLE, etc.); the hardware /
	 * module pre-loop setup (iir_init, TIM14/16 start, frame_init, acq_init,
	 * hadc1 backup) intentionally stays in main() to preserve init ordering. */
}


void app_run_once(void)
{
	  spi_ctrl_loop();
//	  Config_Handler();
	  datardypin = HAL_GPIO_ReadPin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin);

	  acq_clear_busy(); // reset interrupt timeout

	  // forward trigger input to esp32
	  if (HAL_GPIO_ReadPin(GPIOB, config_ext_trigger_input()))
	  {
	      // Input is high and hasn't been debounced yet
	      if (ext_trigger_input_value_debounced == 0)
	      {
	          // Start debouncing: set the previous time to the current time
	          _debounce_prev_time = HAL_GetTick();
	          ext_trigger_input_value_debounced = 1;  // Mark as debouncing
	      }

	      // Check if debounce time has passed
	      if ((HAL_GetTick() - _debounce_prev_time > config_debounce_time_ext_input()))
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
					tim3_counter=0;

					spi_ctrl_send(buf, len);

				}
			}


	  	  if ((!logging_en || frame_overrun()) || (ext_trigger_input_value == 0 && config_trigger_mode() == TRIGGER_MODE_EXTERNAL))
		  {
			  // Overrun release gate: when the ring fills, logging stops and returns to IDLE.

			  HAL_TIM_Base_Stop_IT(&htim3);

			  // Delay of 50 ms, since signal ringing may cause a retrigger of LOGGING state
			  HAL_Delay(50);
			  // Set ADC to single conversion measure mode

			  if (ext_trigger_input_value == 0 && config_trigger_mode() == TRIGGER_MODE_EXTERNAL && logging_en)
			  {
				  NextState = MAIN_WAIT_FOR_TRIGGER;
			  } else {
				  NextState = MAIN_IDLE;
			  }
		  }
		  break;


		  case MAIN_IDLE:
			  // In case logging gets enabled and we are in continuous mode, start the ADC
			  if (logging_en && spi_ctrl_isIdle() && (config_trigger_mode() != TRIGGER_MODE_EXTERNAL))
			  {

				  tim3_counter = 0;
				  frame_reset();
				  time_result_write_ptr = 0;
				  // Start TIM3 and DMA conversion
				  TIM3->CNT = 0;

				  NextState = MAIN_LOGGING;

				  HAL_TIM_Base_Start_IT(&htim3);


			  } else if (logging_en && spi_ctrl_isIdle() && config_trigger_mode() == TRIGGER_MODE_EXTERNAL) {
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

								acq_stop();

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
							  /* v2: frame_take_last delivers the whole partial frame in one
							   * transaction (no half model), so a single-shot completes here. */
							  if (_singleshot) { _singleshot = 0; }
						  }


							  break;

						  case STM32_CMD_GET_PROTOCOL_VERSION:
							  resp.command = STM32_CMD_GET_PROTOCOL_VERSION;
							  resp.data    = UL_PROTOCOL_VERSION;
							  spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));
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
				  acq_start();
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
			  // v2: !frame_at_line_zero() means >=1 line captured in the current frame.
			  if (!frame_at_line_zero())
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
}
