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
#endif


