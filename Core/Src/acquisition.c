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
