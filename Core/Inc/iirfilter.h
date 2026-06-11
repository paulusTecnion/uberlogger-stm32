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

#ifndef __IIRFILTER_H
#define __IIRFILTER_H
#include "stdint.h"
#include "esp32_interface.h"



/** Apply one-pole error-feedback IIR low-pass to a single sample; input/output in raw ADC counts (0–65535 for 16-bit). */
void  iir_filter(uint16_t * input, uint16_t * output, uint8_t channel);
/** Set the filter cut-off via ADC_SAMPLE_RATE_* enum; returns 0 on success, 1 if sampleFreq is out of range. */
uint8_t iir_set_samplefreq(uint8_t sampleFreq);
/** Reset all per-channel filter state (y_state and 32-bit accumulators) to zero. */
void iir_reset();
/** Load the fixed-point IIR coefficients (one per supported sample rate). Must be called before iir_filter(). */
void iir_init();

#endif
