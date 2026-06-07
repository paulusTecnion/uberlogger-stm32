/*
 * MIT License
 *
 * Copyright (c) 2025 Tecnion Technologies
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "framing.h"
#include "string.h"

/* One ring slot: a max-sized frame. Only [0..line_count) lines are valid. The
 * wire frame is CAPACITY-PACKED: the gpio block starts immediately after the adc
 * block of `capacity` lines, i.e. at byte offset 14 + capacity*16. It is NOT at a
 * fixed stride; the gpio[] field below only coincides with the wire position when
 * capacity == UL_LINES_MAX. The field is kept solely to size the slot. */
typedef struct {
    ul_frame_hdr_t hdr;
    uint16_t adc[UL_LINES_MAX * UL_ADC_CH];   /* aligned: hdr is 14 (even) */
    uint8_t  gpio[UL_LINES_MAX];
} ul_frame_buf_t;

_Static_assert(offsetof(ul_frame_buf_t, adc) == 14, "v2 adc block must follow the 14B header");

static ul_frame_buf_t ring[UL_FRAME_DEPTH];
static volatile uint8_t  wr_slot   = 0;   /* slot being filled */
static volatile uint8_t  rd_slot   = 0;   /* oldest ready slot */
static volatile uint8_t  made_cnt  = 0;   /* frames produced (written ONLY by the TIM3 ISR) */
static volatile uint8_t  taken_cnt = 0;   /* frames consumed (written ONLY by the main loop) */
static volatile uint8_t  line_idx  = 0;   /* line within wr_slot */
static volatile uint8_t  overrun   = 0;
static uint8_t capacity = DATA_LINES_PER_SPI_TRANSACTION;  /* default 70; reset in init */

/* Frames ready to send. made_cnt/taken_cnt are each single-writer; their
 * uint8 difference (true count always in [0, UL_FRAME_DEPTH]) is computed
 * with wrapping arithmetic, so no lock is needed (SPSC ring). */
static inline uint8_t frames_ready(void) { return (uint8_t)(made_cnt - taken_cnt); }

/* On-wire bytes for a frame of the current capacity. */
static inline uint16_t frame_wire_len(void) {
    return (uint16_t)(sizeof(ul_frame_hdr_t) + (uint32_t)capacity * UL_LINE_BYTES);
}

void frame_init(void) {
    memset(ring, 0, sizeof(ring));
    wr_slot = rd_slot = line_idx = overrun = 0;
    made_cnt = taken_cnt = 0;
    if (capacity == 0 || capacity > UL_LINES_MAX) capacity = UL_LINES_MAX;
}

void frame_reset(void) {
    wr_slot = rd_slot = line_idx = overrun = 0;
    made_cnt = taken_cnt = 0;
}

void frame_set_lines_per_transaction(uint8_t n) {
    if (n == 0) n = 1;
    if (n > UL_LINES_MAX) n = UL_LINES_MAX;
    capacity = n;
}

uint8_t frame_overrun(void)        { return overrun; }
uint8_t frame_at_line_zero(void)   { return (uint8_t)(line_idx == 0); }

uint8_t frame_append_line(uint32_t base_epoch, uint16_t base_subsec, uint8_t fs_code,
                          uint8_t gpio, const uint16_t *adc, adc_resolution_t res) {
    /* If the current slot is full of unsent frames, we cannot start/continue: overrun. */
    if (line_idx == 0) {
        if (frames_ready() >= UL_FRAME_DEPTH) { overrun = 1; return 0; }  /* ring full: drop */
        ul_frame_hdr_t *h = &ring[wr_slot].hdr;
        h->start[0] = UL_FRAME_START0; h->start[1] = UL_FRAME_START1;
        h->protocol_version = UL_PROTOCOL_VERSION;
        h->flags    = (res == ADC_16_BITS) ? UL_FLAG_RES16 : 0;
        h->base_epoch  = base_epoch;
        h->base_subsec = base_subsec;
        h->fs_code   = fs_code;
        h->capacity  = capacity;
        h->line_count = 0;
        h->pad = 0;
    }
    ul_frame_buf_t *f = &ring[wr_slot];
    memcpy(&f->adc[line_idx * UL_ADC_CH], adc, UL_ADC_CH * 2);  /* 8 x u16 */
    /* GPIO is capacity-packed on the wire: it starts immediately after the
     * adc block of `capacity` lines (byte offset 14 + capacity*16), NOT at the
     * fixed gpio[] array. For capacity < UL_LINES_MAX this lands in the unused
     * tail of adc[]; for capacity == UL_LINES_MAX it coincides with gpio[]. This
     * makes the first (14 + capacity*UL_LINE_BYTES) bytes a valid capacity-packed
     * frame matching the wire length and the ESP32/convert_raw decoder. */
    ((uint8_t *)f->adc)[(uint32_t)capacity * UL_ADC_CH * 2 + line_idx] = gpio;
    line_idx++;
    f->hdr.line_count = line_idx;

    if (line_idx >= capacity) {           /* frame full -> mark ready, advance ring */
        made_cnt++;
        wr_slot = (uint8_t)((wr_slot + 1) % UL_FRAME_DEPTH);
        line_idx = 0;
        return 1;
    }
    return 0;
}

uint8_t frame_take_ready(uint8_t **buf, uint16_t *len) {
    if (frames_ready() == 0) return 0;
    *buf = (uint8_t*)&ring[rd_slot];
    *len = frame_wire_len();
    rd_slot = (uint8_t)((rd_slot + 1) % UL_FRAME_DEPTH);
    taken_cnt++;
    return 1;
}

void frame_take_last(uint8_t **buf, uint16_t *len, uint8_t singleshot, adc_resolution_t res) {
    (void)singleshot; (void)res;
    /* Hand back the in-progress slot with its real line_count (partial frame). */
    ul_frame_buf_t *f = &ring[wr_slot];
    f->hdr.line_count = line_idx;
    *buf = (uint8_t*)f;
    *len = (uint16_t)(sizeof(ul_frame_hdr_t) + (uint32_t)capacity * UL_LINE_BYTES);
}
