/* acquisition.h — owns the ADC + TIM3 sample path and the per-sample ISRs.
 * Phase 1: free-run continuous ADC + TIM3 decimation/timestamp tick (spec §2.1).
 * Phase 2 seam: free-run-vs-triggered and LUT/IIR placement live here (spec §5, §11). */
#ifndef _ACQUISITION_H
#define _ACQUISITION_H

#include "stdint.h"
#include "esp32_interface.h"

void acq_init(void);   /* calibrate ADC, start it (the boot-time Adc_start path) */
void acq_start(void);  /* (re)start ADC DMA per current resolution */
void acq_stop(void);   /* HAL_ADC_Stop_DMA */

/* The TIM3-ISR reentrancy guard `busy` now lives (static) in acquisition.c.
 * main()'s loop historically resets it every iteration (behavior preserved
 * verbatim, spec §7); expose a minimal accessor so that write survives the
 * move without re-exporting the global. */
void acq_clear_busy(void);

/* Low-power analog trigger: AWD1 on one channel, one-shot interrupt.
 * threshold is raw counts at the CURRENT resolution (16-bit values are
 * scaled >>4 internally: AWD thresholds are 12-bit, compared against the
 * post-oversampling result's upper 12 bits). */
void     acq_lp_arm_analog(uint8_t channel_1based, uint16_t threshold, uint8_t edge);
void     acq_lp_disarm_analog(void);
uint8_t  acq_lp_triggered(void);       /* read+clear the AWD one-shot flag */
uint16_t acq_last_sample(uint8_t idx); /* latest processed sample, channel 0-7 */

#endif
