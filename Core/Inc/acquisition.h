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

#endif
