/* framing.h — v2 SPI framing: owns the frame ring, the per-transaction base
 * timestamp, and the 17 B/line block layout (refactor + phase2a specs).
 * Phase-2A: per-transaction base timestamp; ESP32 reconstructs per-line time. */
#ifndef _FRAMING_H
#define _FRAMING_H
#include "stdint.h"
#include "ul_protocol.h"      /* ul_frame_hdr_t, UL_ADC_CH, flags, period table */
#include "esp32_interface.h"  /* adc_resolution_t */

/* Compile-time max lines per frame; runtime LINES_PER_FRAME (capacity) <= this. */
#define UL_LINES_MAX     70
/* Number of frame buffers in the ring (>=2). Tunable; see A4.
 * DEPTH=3 keeps total bss under the ~7600 B budget (DEPTH=4 measured 7656). */
#define UL_FRAME_DEPTH   3

void     frame_init(void);                         /* boot: zero ring, set markers */
void     frame_reset(void);                        /* logging start: reset ring/ptrs/overrun */
void     frame_set_lines_per_transaction(uint8_t n);   /* config: set capacity (<= UL_LINES_MAX) */

/* Begin a new frame if needed and append one sample line. Called from the TIM3
 * sample-tick ISR. On the first line of a frame, captures base via the supplied
 * epoch/subsec. gpio = GPIOB high byte; adc = 8 corrected/filtered u16; res
 * selects the flags bit. Returns 1 when a frame just became ready, else 0.
 * If no free ring slot exists, sets the overrun flag and drops the line. */
uint8_t  frame_append_line(uint32_t base_epoch, uint16_t base_subsec, uint8_t fs_code,
                           uint8_t gpio, const uint16_t *adc, adc_resolution_t res);

/* Hand back the oldest ready frame's buffer + on-wire byte length; advance the
 * read cursor and free the slot. Returns 1 if one was pending, else 0. */
uint8_t  frame_take_ready(uint8_t **buf, uint16_t *len);

/* For SEND_LAST/single-shot: hand back the current (partially) filled frame. */
void     frame_take_last(uint8_t **buf, uint16_t *len, uint8_t singleshot, adc_resolution_t res);

/* Overrun flag (set when the ring was full at append time). Cleared by frame_reset(). */
uint8_t  frame_overrun(void);

/* True iff the NEXT frame_append_line() will start a new frame (line position 0).
 * The TIM3 ISR uses this to read the RTC base exactly once per frame (single
 * source of truth — no separate mirror counter that could desync on overrun). */
uint8_t  frame_at_line_zero(void);

#endif
