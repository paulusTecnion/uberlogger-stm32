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

/* adc_resolution remains owned by main.c (acquisition's concern, spec note for this task);
 * frame_take_ready mirrors the legacy LOGGING-state selection which read this global. */
extern adc_resolution_t adc_resolution;

/* --- module state (moved verbatim from main.c) --- */
static uint8_t data_buffer[sizeof(spi_msg_1_t) + sizeof(spi_msg_2_t)];

static spi_msg_1_t * spi_msg_slow_freq_1 = (spi_msg_1_t *)(data_buffer);
static spi_msg_2_t * spi_msg_slow_freq_2 = (spi_msg_2_t *)(data_buffer + sizeof(spi_msg_1_t));

static volatile uint32_t gpio_result_write_ptr = 0;
static uint8_t adc_is_half = 0, adc_16b_is_half = 0;
static uint8_t gpio_is_half = 0, gpio_ready = 0;
static uint8_t adc_ready = 0;

static uint8_t spi_lines_per_transaction = DATA_LINES_PER_SPI_TRANSACTION;

void frame_init(void)
{
	memset(data_buffer, 0, sizeof(data_buffer));

	spi_msg_slow_freq_1->startByte[0] = 0xFA;
	spi_msg_slow_freq_1->startByte[1] = 0xFB;

	spi_msg_slow_freq_2->stopByte[0] = 0xFB;
	spi_msg_slow_freq_2->stopByte[1] = 0xFA;
}

void frame_reset(void)
{
	adc_is_half = 0;
	adc_16b_is_half = 0;
	adc_ready = 0;
	gpio_result_write_ptr = 0;
	gpio_is_half = 0;
	gpio_ready = 0;
}

void frame_set_lines_per_transaction(uint8_t n)
{
	spi_lines_per_transaction = n;
}

uint8_t frame_adc_16b_is_half(void)
{
	return adc_16b_is_half;
}

uint8_t frame_adc_ready(void)
{
	return adc_ready;
}

uint32_t frame_write_ptr(void)
{
	return gpio_result_write_ptr;
}

uint8_t frame_append_line(const s_date_time_t *ts, uint8_t gpio,
                          const uint16_t *adc, adc_resolution_t res)
{
	uint8_t ready_now = 0;

	// Are still in the first ADC half?
	if (!gpio_is_half)
	{
		// 0x50000411 = GPIOB, 2nd byte (GPIOB8 to GPIOB15)
//			spi_msg_1_ptr->gpioData[gpio_result_write_ptr] = (GPIOB->IDR >> 8);
		spi_msg_slow_freq_1->gpioData[gpio_result_write_ptr] = gpio;
//			memcpy((void*)&spi_msg_1_ptr->timeData[gpio_result_write_ptr], &current_date_time, sizeof(s_date_time_t));
		memcpy((void*)&spi_msg_slow_freq_1->timeData[gpio_result_write_ptr], ts, sizeof(s_date_time_t));
//			spi_msg_1_ptr->dataLen = gpio_result_write_ptr +1 ;
		spi_msg_slow_freq_1->dataLen = gpio_result_write_ptr +1 ;
	} else { // If not, we fill the second part
//			spi_msg_2_ptr->gpioData[gpio_result_write_ptr] = (GPIOB->IDR >> 8);
		spi_msg_slow_freq_2->gpioData[gpio_result_write_ptr] = gpio;
//			memcpy((void*)&spi_msg_2_ptr->timeData[gpio_result_write_ptr], &current_date_time, sizeof(s_date_time_t));
		memcpy((void*)&spi_msg_slow_freq_2->timeData[gpio_result_write_ptr], ts, sizeof(s_date_time_t));
//			spi_msg_2_ptr->dataLen = gpio_result_write_ptr+1;
		spi_msg_slow_freq_2->dataLen = gpio_result_write_ptr +1 ;
	}
//


	// In case we are doing 16 bits, we manually need to copy data from the IIR filter buffer to the adc
	if (res == ADC_16_BITS)
	{
		/* spec sec.7: behavior preserved verbatim, do not 'fix' in Phase 1 */
		if (!adc_16b_is_half)
		{
			memcpy((uint8_t*)spi_msg_slow_freq_1->adcData + 2*8*gpio_result_write_ptr, adc, 8*2);
		} else {
			memcpy((uint8_t*)spi_msg_slow_freq_2->adcData + 2*8*gpio_result_write_ptr, adc, 8*2);
		}
	} else {
		/* spec sec.7: behavior preserved verbatim, do not 'fix' in Phase 1 */
		if (!adc_is_half)
		{
        // in 12 bit mode we copy from the buffer "iirFilter", but the actual IIR filter is not used in 12 bits mode.
//				memcpy((uint8_t*)spi_msg_1_ptr->adcData + 2*8*gpio_result_write_ptr, iirFilter, 8*2);
			memcpy((uint8_t*)spi_msg_slow_freq_1->adcData + 2*8*gpio_result_write_ptr, adc, 8*2);
		} else {
//				memcpy((uint8_t*)spi_msg_2_ptr->adcData + 2*8*gpio_result_write_ptr, iirFilter, 8*2);
			memcpy((uint8_t*)spi_msg_slow_freq_2->adcData + 2*8*gpio_result_write_ptr, adc, 8*2);
		}
	}


	gpio_result_write_ptr++;


	if (gpio_result_write_ptr >= spi_lines_per_transaction)
	{
		gpio_is_half = !gpio_is_half;
		gpio_ready = 1;

		// when in 16 bit mode, manually set adc_ready flag
		if (res == ADC_16_BITS)
		{
			adc_16b_is_half = ~adc_16b_is_half; /* spec sec.7: behavior preserved verbatim, do not 'fix' in Phase 1 */
			// adc_ready = 1;
		} else {
			adc_is_half = ~adc_is_half;
		}
		adc_ready = 1;
		ready_now = 1;
	}

	gpio_result_write_ptr = gpio_result_write_ptr % spi_lines_per_transaction;
	// if gpio_result_write_ptr is back to 0, we need to manually set the adc_16b_is_half byte

	return ready_now;
}

uint8_t frame_take_ready(uint8_t **buf, uint16_t *len)
{
	if (!(adc_ready && gpio_ready))
	{
		return 0;
	}

	/* TODO Phase 2: take resolution as a parameter like frame_take_last() instead of reading extern adc_resolution. */
	if (adc_resolution == ADC_16_BITS)
	{

		if (adc_16b_is_half)
		{
//					uint16_t * adcData = (uint16_t*)spi_msg_1_ptr->adcData;
			*buf = (uint8_t*)spi_msg_slow_freq_1; *len = sizeof(spi_msg_1_t);
		} else {
			*buf = (uint8_t*)spi_msg_slow_freq_2; *len = sizeof(spi_msg_2_t);
		}

	} else {

		if (adc_is_half)
		{
//					uint16_t * adcData = (uint16_t*)spi_msg_1_ptr->adcData;
			*buf = (uint8_t*)spi_msg_slow_freq_1; *len = sizeof(spi_msg_1_t);
//						spi_ctrl_send((uint8_t*)spi_msg_slow_freq_1, sizeof(spi_msg_slow_freq_t));
		} else {
			*buf = (uint8_t*)spi_msg_slow_freq_2; *len = sizeof(spi_msg_2_t);
//						spi_ctrl_send((uint8_t*)spi_msg_slow_freq_2, sizeof(spi_msg_slow_freq_t));
		}
	}

	adc_ready = 0;
	gpio_ready = 0;

	return 1;
}

void frame_take_last(uint8_t **buf, uint16_t *len, uint8_t singleshot, adc_resolution_t res)
{
	if (res == ADC_16_BITS)
	{
		if (!adc_16b_is_half || singleshot)
		{
			// adc_is_half == 1 means the last message sent was spi_msg_1
			// So we are now still writing in spi_msg_2.
			*buf = (uint8_t*)spi_msg_slow_freq_1; *len = sizeof(spi_msg_1_t);
		} else {
			*buf = (uint8_t*)spi_msg_slow_freq_2; *len = sizeof(spi_msg_2_t);
		}
	} else {
		if (!adc_is_half || singleshot)
		{
			// adc_is_half == 1 means the last message sent was spi_msg_1
			// So we are now still writing in spi_msg_2.
			*buf = (uint8_t*)spi_msg_slow_freq_1; *len = sizeof(spi_msg_1_t);
//									  spi_ctrl_send((uint8_t*)spi_msg_slow_freq_2, sizeof(spi_msg_slow_freq_t));
		} else {
			*buf = (uint8_t*)spi_msg_slow_freq_2; *len = sizeof(spi_msg_2_t);
//									  spi_ctrl_send((uint8_t*)spi_msg_slow_freq_1, sizeof(spi_msg_slow_freq_t));
		}
	}
}
