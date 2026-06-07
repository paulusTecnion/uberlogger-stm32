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
#include "fault_line.h"

void fault_line_init_output(void)
{
    GPIO_InitTypeDef g = {0};
    /* GPIOA clock is enabled in MX_GPIO_Init() before app_init(). */
    HAL_GPIO_WritePin(FAULT_LINE_GPIO_Port, FAULT_LINE_Pin, GPIO_PIN_RESET);
    g.Pin   = FAULT_LINE_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FAULT_LINE_GPIO_Port, &g);
}

void fault_line_assert(void)
{
    HAL_GPIO_WritePin(FAULT_LINE_GPIO_Port, FAULT_LINE_Pin, GPIO_PIN_SET);
}

void fault_line_clear(void)
{
    HAL_GPIO_WritePin(FAULT_LINE_GPIO_Port, FAULT_LINE_Pin, GPIO_PIN_RESET);
}
