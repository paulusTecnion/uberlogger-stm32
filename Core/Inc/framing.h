/* framing.h — owns the SPI message buffers, their exact byte layout, the
 * ping-pong/half bookkeeping, and the timestamp-coupling strategy.
 * Phase 1: byte-identical to the legacy twin-struct layout (refactor spec sec.5.1, sec.11). */
#ifndef _FRAMING_H
#define _FRAMING_H
#include "stdint.h"
#include "main.h"             /* s_date_time_t, spi_msg_1_t, spi_msg_2_t, layout #defines */
#include "esp32_interface.h"  /* adc_resolution_t, NUM_ADC_CHANNELS */

void     frame_init(void);    /* set start/stop bytes, zero buffers (boot) */
void     frame_reset(void);   /* reset write ptrs / halves / ready flags (logging start) */
void     frame_set_lines_per_transaction(uint8_t n);

/* Deposit one sample line (called from the TIM3 sample-tick ISR).
 * Returns 1 when a half just became ready to transmit, else 0. */
uint8_t  frame_append_line(const s_date_time_t *ts, uint8_t gpio,
                           const uint16_t *adc, adc_resolution_t res);

/* If a filled half is pending, hand back its buffer + byte length, clear the flag. Returns 1 if one was pending. */
uint8_t  frame_take_ready(uint8_t **buf, uint16_t *len);

/* For STM32_CMD_SEND_LAST_ADC_BYTES / single-shot: hand back the appropriate half.
 * Mirrors the legacy msg_1-vs-msg_2 selection EXACTLY. */
void     frame_take_last(uint8_t **buf, uint16_t *len, uint8_t singleshot, adc_resolution_t res);

/* Accessors so the state machine can read framing-owned bookkeeping without an extern.
 * (legacy code read these directly in main.c) */
uint8_t  frame_adc_16b_is_half(void);
uint8_t  frame_adc_ready(void);
uint32_t frame_write_ptr(void);
#endif
