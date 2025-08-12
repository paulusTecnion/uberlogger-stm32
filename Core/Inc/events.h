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

#ifndef _EVENTS_H
#define _EVENTS_H

#include "stdint.h"

typedef enum uint8_t {
	EVENT_ADC_IS_HALF= 0x00,
	EVENT_ADC_IS_FULL,
	EVENT_ADC_STOPPED,
	EVENT_ADC_OVERRUN,
	EVENT_CONFIG_ENTER_HANDLER,
	EVENT_CONFIG_EXIT_HANDLER,
	EVENT_SPI_MSG_RECEIVED,
	EVENT_SPI_MSG_SENT,
	EVENT_SPI_MSG_RX_TIMEOUT,
	EVENT_SPI_MSG_TX_TIMEOUT
} events_t;


#endif
