/*
 * acquisition.c — ADC + TIM3 sample path and the per-sample ISRs.
 *
 * Owns: the boot-time ADC calibrate/start path, the (re)start/stop DMA helpers,
 * and the HAL weak conversion callbacks. The TIM3 branch of the per-sample tick
 * (timestamp + frame_append_line) lives here too.
 *
 * HAL weak-callback ownership (spec §7, the link-time hazard):
 *   HAL_TIM_PeriodElapsedCallback is ONE weak symbol shared by TIM3/14/16.
 *   To keep exactly one strong definition, the whole callback lives HERE
 *   (acquisition.c owns TIM3) and forwards the TIM14/TIM16 SPI-watchdog cases
 *   to spi_ctrl_on_timeout_tick() so each branch stays with its owning module.
 *
 * The ~adc_16b_is_half toggle / the 12-bit-vs-16-bit memcpy split and the
 * `busy` reentrancy guard are intentional preserved smells (spec §7): kept
 * byte-for-byte, do not "fix".
 */

#include "acquisition.h"
#include "main.h"
#include "framing.h"
#include "iirfilter.h"
#include "adc_comp_lut.h"
#include "spi_ctrl.h"

/* CubeMX owns these handles in main.c; reach them via extern (matches config.c). */
extern ADC_HandleTypeDef hadc1;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim3;

/* These remain owned by main.c (state machine / config touch them). */
extern adc_resolution_t adc_resolution;
extern uint16_t tim3_counter;
extern lut_t * active_lut_table[NUM_ADC_CHANNELS];

/* Sample-path state, moved here as the single definition. */
static RTC_TimeTypeDef current_time;
static RTC_DateTypeDef current_date;
static s_date_time_t current_date_time;
static uint8_t busy = 0;
static uint16_t adc16bBuffer[16];
static uint16_t adc12Buffer[8*8];
static uint16_t correctedAdc = 0;
static uint16_t iirFilter[8];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	/* TIM14/TIM16 are the SPI watchdogs; forward to their owning module. */
	spi_ctrl_on_timeout_tick(htim);

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

void acq_start()
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

void acq_stop(void)
{
	HAL_ADC_Stop_DMA(&hadc1);
}

/* --- Low-power analog trigger (spec §4.1) -------------------------------- */
static volatile uint8_t lp_awd_fired = 0;

static const uint32_t lp_adc_chan_map[8] = {
	ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
	ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7
}; /* UI channel N -> ADC channel; verify physical AIN order on the bench (Task 7) */

uint16_t acq_last_sample(uint8_t idx)
{
	return iirFilter[idx & 0x07];
}

/* Single-consumer one-shot: only app_run_once() (main loop, no RTOS) calls this,
 * and the AWD ISR disables its own interrupt before setting the flag, so no
 * second fire can interleave with the read-clear. If a second consumer or an
 * RTOS is ever introduced, wrap this in __disable_irq()/__enable_irq(). */
uint8_t acq_lp_triggered(void)
{
	if (lp_awd_fired) { lp_awd_fired = 0; return 1; }
	return 0;
}

void acq_lp_arm_analog(uint8_t channel_1based, uint16_t threshold, uint8_t edge)
{
	ADC_AnalogWDGConfTypeDef awd = {0};

	if (adc_resolution == ADC_16_BITS)
		threshold >>= 4;  /* 16-bit mode: AWD compares DR's upper 12 bits; 12-bit mode needs no shift (already 12-bit, clamp below is a guard) */
	if (threshold > 0x0FFF)
		threshold = 0x0FFF;

	awd.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
	awd.WatchdogMode   = ADC_ANALOGWATCHDOG_SINGLE_REG;
	awd.Channel        = lp_adc_chan_map[(channel_1based - 1) & 0x07];
	awd.ITMode         = ENABLE;
	if (edge == 0) {              /* rising-above: out-of-window when DR > threshold */
		awd.HighThreshold = threshold;
		awd.LowThreshold  = 0;
	} else {                      /* falling-below: out-of-window when DR < threshold */
		awd.HighThreshold = 0x0FFF;
		awd.LowThreshold  = threshold;
	}

	/* AWD channel/mode bits require ADSTART=0: briefly stop, configure, restart. */
	acq_stop();

	/* Fix I3: boot config uses TIM3-TRGO as trigger source. If SET_SAMPLE_RATE
	 * has never been synced from the ESP32 (TIM3 not running), the AWD would
	 * never see a conversion. Force free-running mode so the AWD always fires.
	 * ADC_Reinit() calls HAL_ADC_Init() which re-programs the CFGR1 register
	 * without restarting DMA — safe here because ADSTART=0 after acq_stop(). */
	if (hadc1.Init.ExternalTrigConv != ADC_SOFTWARE_START
			|| hadc1.Init.ContinuousConvMode != ENABLE)
	{
		hadc1.Init.ExternalTrigConv     = ADC_SOFTWARE_START;
		hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		hadc1.Init.ContinuousConvMode   = ENABLE;
		ADC_Reinit();
	}

	lp_awd_fired = 0;
	HAL_ADC_AnalogWDGConfig(&hadc1, &awd);
	HAL_NVIC_SetPriority(ADC1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(ADC1_IRQn);
	acq_start();
}

void acq_lp_disarm_analog(void)
{
	__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_AWD1);
	lp_awd_fired = 0;
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
	/* One-shot: latch and silence until the next arm. */
	__HAL_ADC_DISABLE_IT(hadc, ADC_IT_AWD1);
	lp_awd_fired = 1;
}

/* Boot-time init: the HAL_ADCEx_Calibration_Start + acq_start + busy=1 block
 * from main()'s USER CODE 2. iir_init() is invoked by main() earlier in the
 * same block (kept there); ordering preserved at the call site. */
void acq_init(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1);
	acq_start();
	busy = 1;
}

/* main()'s loop historically clears the TIM3 reentrancy guard each iteration
 * (preserved verbatim, spec §7). */
void acq_clear_busy(void)
{
	busy = 0;
}
