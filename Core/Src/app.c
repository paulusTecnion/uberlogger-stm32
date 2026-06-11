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

uint8_t overrun = 0;                    /* app-private, but kept extern: static-izing lets the compiler const-fold the only (read-only) use and drop a branch, shifting the image */
static uint8_t datardypin;
static uint16_t tbuffer[8];

adc_resolution_t adc_resolution = ADC_12_BITS;            /* shared: acquisition.c/framing.c/config.c (hot-ISR read, stays extern) */
static uint16_t adcCounter = 0;

/* --- Low-power digital trigger (spec §4.1) ---
 * lp_armed gates the EXTI callbacks below; lp_pin_edge_tick latches the edge
 * time so the main loop can confirm the level after the debounce window. */
static volatile uint8_t  lp_armed = 0;
static volatile uint8_t  lp_pin_edge_seen = 0;
static volatile uint32_t lp_pin_edge_tick = 0;

#define LP_USE_WFI        1     /* set 0 for debug sessions (live-watch etc.) */
#define LP_REARM_HOLDOFF_MS 1000

static uint8_t  lp_capturing = 0;
static uint32_t lp_capture_deadline = 0;
static uint32_t lp_rearm_after = 0;

/* Signal on the "safe" (non-trigger) side of the threshold? (edge semantics) */
static uint8_t lp_signal_on_safe_side(void)
{
	if (config_lp_source() == 0) /* analog */
	{
		uint16_t v = acq_last_sample(config_lp_channel() - 1);
		return (config_lp_edge() == 0) ? (v < config_lp_threshold())
		                               : (v > config_lp_threshold());
	}
	/* digital: safe = pin at the inactive level for the chosen edge */
	GPIO_PinState s = HAL_GPIO_ReadPin(GPIOB, config_ext_trigger_input());
	return (config_lp_edge() == 0) ? (s == GPIO_PIN_RESET) : (s == GPIO_PIN_SET);
}

static void lp_arm_digital(uint16_t pin, uint8_t edge)
{
	GPIO_InitTypeDef g = {0};
	g.Pin  = pin;
	g.Mode = (edge == 0) ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
	g.Pull = GPIO_NOPULL;
	lp_armed = 0;              /* gate the ISR latch during (re)configuration */
	lp_pin_edge_seen = 0;
	HAL_GPIO_Init(GPIOB, &g);  /* EXTI4_15_IRQn is already enabled (STM_ADC_EN) */
	/* HAL_GPIO_Init does not clear RPR1/FPR1: flush any stale pending edge so a
	 * pre-arm transition cannot fire the moment the caller sets lp_armed. */
	__HAL_GPIO_EXTI_CLEAR_RISING_IT(pin);
	__HAL_GPIO_EXTI_CLEAR_FALLING_IT(pin);
}

static void lp_disarm_digital(uint16_t pin)
{
	GPIO_InitTypeDef g = {0};
	g.Pin  = pin;
	g.Mode = GPIO_MODE_INPUT;  /* back to the plain-input config from MX_GPIO_Init */
	g.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &g);
	/* HAL_GPIO_Init with plain INPUT does NOT clear the EXTI config on G0 —
	 * without this the pin keeps interrupting on every edge after disarm
	 * (harmless but wasteful; final-review finding M-2). */
	CLEAR_BIT(EXTI->IMR1, (uint32_t)pin);
	CLEAR_BIT(EXTI->RTSR1, (uint32_t)pin);
	CLEAR_BIT(EXTI->FTSR1, (uint32_t)pin);
	lp_pin_edge_seen = 0;
}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == STM_ADC_EN_Pin)
	{
			logging_en = 1;
	}
	else if (lp_armed && GPIO_Pin == config_ext_trigger_input())
	{
		lp_pin_edge_seen = 1;
		lp_pin_edge_tick = HAL_GetTick();
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == STM_ADC_EN_Pin)
	{
		//HAL_SPI_DMAStop(&hspi1);

		logging_en = 0;

	}
	else if (lp_armed && GPIO_Pin == config_ext_trigger_input())
	{
		lp_pin_edge_seen = 1;
		lp_pin_edge_tick = HAL_GetTick();
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
	config_lp_bench_force(); /* no-op unless LP_BENCH_FORCE is defined */
}


/* Handle one ESP32 command while in an LP state (PRECHECK or ARMED).
 * Returns 1 if the command forces an exit to MAIN_CONFIG (caller must
 * disarm-if-armed + acq_stop() + set NextState = MAIN_CONFIG and break).
 * Returns 0 for NOP (silently consumed, no response) and for unknown commands
 * (NOK response sent). Mirrors MAIN_IDLE's dispatcher for LP-relevant commands.
 * Precondition: spi_ctrl_msg_received() returned true (flag already consumed). */
static uint8_t lp_dispatch_cmd(void)
{
	spi_cmd_t *cmd = (spi_cmd_t*)&cmd_buffer;
	spi_cmd_t resp;
	switch (cmd->command)
	{
		case STM32_CMD_SETTINGS_MODE:
			resp.command = STM32_CMD_SETTINGS_MODE;
			resp.data = CMD_RESP_OK;
			spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));
			return 1;

		case STM32_CMD_NOP:
			/* Silently consumed, no response — exact MAIN_IDLE parity. */
			return 0;

		case STM32_CMD_SEND_LAST_ADC_BYTES:
		{
			/* After an LP capture ends the ESP32 collects the final partial
			 * frame while we already sit in LP_PRECHECK/LP_ARMED. Serve the
			 * in-progress half with its TRUE pending line count: the legacy
			 * frame_take_last() leaves dataLen at the previous full frame's
			 * value, so a boundary-stop tail came back as a stale duplicate
			 * frame (found on the bench). _singleshot is not an LP concern. */
			uint8_t *buf; uint16_t len;
			frame_take_last_partial(&buf, &len, adc_resolution);
			spi_ctrl_send(buf, len);
			return 0;
		}

		default:
			resp.command = STM32_CMD_NOP;
			resp.data = CMD_RESP_NOK;
			spi_ctrl_send((uint8_t*)&resp, sizeof(spi_cmd_t));
			return 0;
	}
}


void app_run_once(void)
{
	  spi_ctrl_loop();
//	  Config_Handler();
	  datardypin = HAL_GPIO_ReadPin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin);

	  acq_clear_busy(); // reset interrupt timeout

	  // forward trigger input to esp32
	  if (config_trigger_mode() != TRIGGER_MODE_LOW_POWER)
	  {
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
	  }
	  else
	  {
	      HAL_GPIO_WritePin(EXT_PIN_VALUE_GPIO_Port, EXT_PIN_VALUE_Pin,
	                        lp_capturing ? GPIO_PIN_SET : GPIO_PIN_RESET);
	  }

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


	  	  if ((!logging_en || overrun)
	  			  || (ext_trigger_input_value == 0 && config_trigger_mode() == TRIGGER_MODE_EXTERNAL)
	  			  || (lp_capturing && (int32_t)(HAL_GetTick() - lp_capture_deadline) >= 0))
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

			  if (lp_capturing)
			  {
				  lp_capturing = 0;     /* EXT_PIN_VALUE drops on the next pass */
				  lp_rearm_after = HAL_GetTick() + LP_REARM_HOLDOFF_MS;
				  NextState = logging_en ? MAIN_LP_PRECHECK : MAIN_IDLE;
			  }
			  else if (ext_trigger_input_value == 0 && config_trigger_mode() == TRIGGER_MODE_EXTERNAL && logging_en)
			  {
				  NextState = MAIN_WAIT_FOR_TRIGGER;
			  } else {
				  NextState = MAIN_IDLE;
			  }
		  }
		  break;


		  case MAIN_IDLE:
			  // In case logging gets enabled and we are in continuous mode, start the ADC
			  if (logging_en && spi_ctrl_isIdle() && (config_trigger_mode() == TRIGGER_MODE_LOW_POWER))
			  {
				  lp_rearm_after = HAL_GetTick();   /* no holdoff on first arm */
				  NextState = MAIN_LP_PRECHECK;
			  }
			  else if (logging_en && spi_ctrl_isIdle() && (config_trigger_mode() != TRIGGER_MODE_EXTERNAL))
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

		  case MAIN_LP_PRECHECK:
			  /* Wait out the re-arm holdoff, then wait until the signal sits on
			   * the non-trigger side of the threshold (true crossing semantics),
			   * then arm the hardware trigger and go to sleep-armed.
			   * PRECHECK spins (no WFI) so it can poll acq_last_sample and
			   * service ESP32 commands that arrive during the holdoff/wait. */
			  if (!logging_en)
			  {
				  NextState = MAIN_IDLE;
				  break;
			  }

			  /* Service any ESP32 command that arrived during holdoff/wait. */
			  if (spi_ctrl_msg_received())
			  {
				  if (lp_dispatch_cmd())
				  {
					  /* SETTINGS_MODE: exit to config (nothing is armed yet) */
					  acq_stop();
					  NextState = MAIN_CONFIG;
				  }
				  /* else: NOP or unknown handled inside lp_dispatch_cmd(); stay in PRECHECK */
				  break;
			  }
			  if (spi_ctrl_isIdle())
			  {
				  spi_ctrl_receive(cmd_buffer, sizeof(spi_cmd_t));
			  }

			  if ((int32_t)(HAL_GetTick() - lp_rearm_after) >= 0 && lp_signal_on_safe_side())
			  {
				  if (config_lp_source() == 0)
				  {
					  acq_lp_arm_analog(config_lp_channel(), config_lp_threshold(), config_lp_edge());
					  lp_armed = 1;
				  }
				  else
				  {
					  lp_arm_digital(config_ext_trigger_input(), config_lp_edge());
					  /* Close the ISR gate before reading the pin: any transition that
					   * occurs after lp_armed=1 is captured by the ISR; a stale
					   * synthetic latch is harmless because debounce confirm re-checks
					   * the live level. */
					  lp_armed = 1;
					  GPIO_PinState s = HAL_GPIO_ReadPin(GPIOB, config_ext_trigger_input());
					  uint8_t already_active = (config_lp_edge() == 0) ? (s == GPIO_PIN_SET) : (s == GPIO_PIN_RESET);
					  if (already_active)
					  {
						  lp_pin_edge_seen = 1;
						  lp_pin_edge_tick = HAL_GetTick();
					  }
				  }
				  NextState = MAIN_LP_ARMED;
			  }
			  break;

		  case MAIN_LP_ARMED:
		  {
			  /* Step 1: logging_en exit — checked BEFORE consuming any message or
			   * trigger flag so that a simultaneous trigger + logging-stop does not
			   * silently discard the trigger read. */
			  if (!logging_en)
			  {
				  if (config_lp_source() == 0) acq_lp_disarm_analog();
				  else                         lp_disarm_digital(config_ext_trigger_input());
				  lp_armed = 0;
				  NextState = MAIN_IDLE;
				  break;
			  }

			  /* Step 2: handle any ESP32 command that arrived while armed.
			   * lp_dispatch_cmd() returns 1 for SETTINGS_MODE (caller must
			   * disarm + acq_stop + go to MAIN_CONFIG) and 0 for NOP/unknown.
			   * After handling a non-CONFIG command we fall through to step 3
			   * so a simultaneously latched trigger is not lost. */
			  uint8_t msg_dispatched = 0;
			  if (spi_ctrl_msg_received())
			  {
				  if (lp_dispatch_cmd())
				  {
					  /* SETTINGS_MODE: discard any pending trigger (intentional —
					   * user is reconfiguring) and exit LP entirely. */
					  if (config_lp_source() == 0) acq_lp_disarm_analog();
					  else                         lp_disarm_digital(config_ext_trigger_input());
					  lp_armed = 0;
					  acq_stop();
					  NextState = MAIN_CONFIG;
					  break;
				  }
				  /* Non-CONFIG command: responded inside lp_dispatch_cmd().
				   * Fall through to step 3 — check fired in the SAME pass. */
				  msg_dispatched = 1;
			  }

			  /* Step 3: evaluate trigger.
			   * acq_lp_triggered() is only called here, after any message has
			   * been consumed (or no message arrived), so a latched lp_awd_fired
			   * survives any number of message passes. */
			  uint8_t fired = 0;
			  if (config_lp_source() == 0)
			  {
				  fired = acq_lp_triggered();
			  }
			  else if (lp_pin_edge_seen)
			  {
				  /* Fix I2: read + clear atomically to avoid a fresh EXTI edge
				   * being wiped by the lp_pin_edge_seen = 0 clear below. */
				  __disable_irq();
				  GPIO_PinState s = HAL_GPIO_ReadPin(GPIOB, config_ext_trigger_input());
				  uint8_t active = (config_lp_edge() == 0) ? (s == GPIO_PIN_SET) : (s == GPIO_PIN_RESET);
				  if (!active)
					  lp_pin_edge_seen = 0;                  /* bounced away: re-wait */
				  __enable_irq();
				  /* debounce elapsed check is outside the critical section */
				  if (active && HAL_GetTick() - lp_pin_edge_tick >= config_debounce_time_ext_input())
					  fired = 1;
			  }

			  if (fired)
			  {
				  if (config_lp_source() == 0) acq_lp_disarm_analog();
				  else                         lp_disarm_digital(config_ext_trigger_input());
				  lp_armed = 0;

				  /* The armed-state command receive is still posted; without
				   * cancelling it spi_ctrl stays RECEIVING, every frame send
				   * returns HAL_BUSY (silently dropped by MAIN_LOGGING),
				   * DATA_RDY never rises and the ESP32 aborts the session
				   * with ERR_LOGGER_STM32_TIMEOUT. Found on the bench. */
				  spi_ctrl_cancel_receive();

				  lp_capturing = 1;
				  lp_capture_deadline = HAL_GetTick() + (uint32_t)config_lp_duration_s() * 1000u;

				  tim3_counter = 0;
				  frame_reset();
				  time_result_write_ptr = 0;
				  TIM3->CNT = 0;
				  NextState = MAIN_LOGGING;
				  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE); /* a pending UIF from the armed period would instantly append the stale pre-trigger sample */
				  HAL_TIM_Base_Start_IT(&htim3);
				  break;
			  }

			  /* Step 4: keep an SPI receive posted, then sleep until an IRQ.
			   * NOT on a pass that just dispatched a command: the response TX
			   * posted by lp_dispatch_cmd() puts HAL in BUSY_TX, and
			   * spi_ctrl_receive()'s error path would HAL_SPI_DMAStop() —
			   * aborting our own response after DATA_RDY already went high
			   * (final-review finding I-1). Repost next pass instead. */
			  if (!msg_dispatched && spi_ctrl_isIdle())
			  {
				  spi_ctrl_receive(cmd_buffer, sizeof(spi_cmd_t));
			  }
#if LP_USE_WFI
			  __WFI();   /* Sleep mode; SysTick/AWD/EXTI/SPI IRQs all wake us */
#endif
			  break;
		  }

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
}
