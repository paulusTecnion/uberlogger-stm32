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

#ifndef _CONFIG_H
#define _CONFIG_H
#include "main.h"
#include "events.h"

#include "esp32_interface.h"
#include "stdint.h"
#include "string.h"
//#include "msg.h"


void Config_Handler(spi_cmd_t *  cmd);
uint8_t Config_Set_Adc_channels(uint8_t channels);
uint8_t Config_set_range(adc_channel_range_t range);
uint8_t Config_Set_Resolution(uint8_t resolution);
uint8_t Config_Set_Sample_freq(uint8_t sampleFreq);
uint8_t Config_Set_Time(uint32_t epoch);
uint8_t Config_set_logMode(uint8_t logtype, uint8_t data_lines_per_transaction);
uint8_t Config_set_triggerMode(uint8_t mode, uint8_t gpio);
uint8_t Config_set_debounceTime(uint32_t debounceTime);

/* Task 8: read-only accessors for config-owned settings consumed by app.c.
 * Trivial getters; no validation, no side effects. */
uint8_t  config_trigger_mode(void);
uint16_t config_ext_trigger_input(void);
uint32_t config_debounce_time_ext_input(void);

/* Low-power trigger config (set via STM32_CMD_SET_LP_CONFIG). */
uint8_t  Config_set_lpConfig(uint8_t source, uint8_t channel, uint16_t threshold,
                             uint8_t edge, uint16_t duration_s);
uint8_t  config_lp_source(void);      /* 0=analog, 1=digital */
uint8_t  config_lp_channel(void);     /* 1-based: AIN 1-8 / DIO 1-6 */
uint16_t config_lp_threshold(void);   /* raw counts at the configured resolution */
uint8_t  config_lp_edge(void);        /* 0=rising-above, 1=falling-below */
uint16_t config_lp_duration_s(void);
void     config_lp_bench_force(void); /* no-op unless LP_BENCH_FORCE (config.c) */
#endif


