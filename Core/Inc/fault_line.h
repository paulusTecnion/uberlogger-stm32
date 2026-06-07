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
#ifndef _FAULT_LINE_H
#define _FAULT_LINE_H

#include "stm32g0xx_hal.h"

/* STM_USART_RX net = PA10 <-> ESP IO4. Repurposed STM->ESP fault line during
 * logging: driven push-pull, idle LOW, HIGH = fault (ring overrun or SPI tear).
 * Reverts to the ROM bootloader's USART1_RX automatically on the pre-flash STM
 * reset, so no explicit revert is needed here. PA10 has no other app function. */
#define FAULT_LINE_Pin        GPIO_PIN_10
#define FAULT_LINE_GPIO_Port  GPIOA

/* Configure PA10 as push-pull output, idle LOW. Call once at app init (GPIOA
 * clock is already enabled by MX_GPIO_Init). */
void fault_line_init_output(void);

/* Drive the line HIGH (latched until fault_line_clear). */
void fault_line_assert(void);

/* Drive the line LOW. Called at session (re)start. */
void fault_line_clear(void);

#endif /* _FAULT_LINE_H */
