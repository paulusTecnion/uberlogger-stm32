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

/*
 * esp32_interface.h
 *
 *  Created on: 5 Dec 2022
 *      Author: Paulus
 */

#ifndef INC_ESP32_INTERFACE_H_
#define INC_ESP32_INTERFACE_H_

#define MAX_WIFI_SSID_LEN 50
#define MAX_WIFI_PASSW_LEN 20
#define NUM_ADC_CHANNELS 8
#define MAX_DEBOUNCE_TIME 60000

typedef enum adc_channel_e {
	ADC_CHANNEL_0_T = 0x00,
	ADC_CHANNEL_1_T = 0x01,
	ADC_CHANNEL_2_T = 0x02,
	ADC_CHANNEL_3_T = 0x03,
	ADC_CHANNEL_4_T = 0x04,
	ADC_CHANNEL_5_T = 0x05,
	ADC_CHANNEL_6_T = 0x06,
	ADC_CHANNEL_7_T = 0x07

} adc_channel_t;

typedef enum adc_channel_range_e
{
	ADC_RANGE_10V = 0,
	ADC_RANGE_60V = 1
} adc_channel_range_t;

typedef enum adc_channel_type_e
{
	ADC_CHANNEL_TYPE_NTC = 0,
	ADC_CHANNEL_TYPE_AIN = 1
} adc_channel_type_t;

typedef enum adc_resolution_e {
    ADC_12_BITS = 12,
    ADC_16_BITS = 16
} adc_resolution_t;

typedef enum adc_channel_enable_e {
	ADC_CHANNEL_DISABLED = 0,
	ADC_CHANNEL_ENABLED = 1
} adc_channel_enable_t;

typedef enum adc_sample_rate_e {
		ADC_SAMPLE_RATE_EVERY_3600S = 0, // once per hour
		ADC_SAMPLE_RATE_EVERY_600S, // once per 10 min
		ADC_SAMPLE_RATE_EVERY_300S, // once per 5 min
		ADC_SAMPLE_RATE_EVERY_60S, // once per 1 min
		ADC_SAMPLE_RATE_EVERY_10S, // once per 10 sec
	    ADC_SAMPLE_RATE_1Hz,
		ADC_SAMPLE_RATE_2Hz,
		ADC_SAMPLE_RATE_5Hz,
		ADC_SAMPLE_RATE_10Hz,
		ADC_SAMPLE_RATE_25Hz,
		ADC_SAMPLE_RATE_50Hz,
		ADC_SAMPLE_RATE_100Hz,
		ADC_SAMPLE_RATE_250Hz,
		// ADC_SAMPLE_RATE_500Hz,
		// ADC_SAMPLE_RATE_1000Hz,
		// ADC_SAMPLE_RATE_2500Hz,
		// ADC_SAMPLE_RATE_5000Hz,
		// ADC_SAMPLE_RATE_10000Hz,
	    ADC_SAMPLE_RATE_NUM_ITEMS
} adc_sample_rate_t;


typedef enum log_mode_e {
    LOGMODE_RAW = 0,
    LOGMODE_CSV,
	LOGMODE_UNKNOWN_TYPE
} log_mode_t;


// enum interface commands with ESP32
typedef enum  {
	STM32_CMD_NOP = 0x00,
	STM32_CMD_SETTINGS_MODE,
	STM32_CMD_SETTINGS_SYNC,
	STM32_CMD_MEASURE_MODE,
	STM32_CMD_SET_RESOLUTION,
	STM32_CMD_SET_SAMPLE_RATE,
	STM32_CMD_SET_ADC_CHANNELS_ENABLED,
	STM32_CMD_SET_DATETIME,
	STM32_CMD_SINGLE_SHOT_MEASUREMENT,
	STM32_CMD_SEND_LAST_ADC_BYTES,
	STM32_CMD_SET_LOGMODE,
	STM32_CMD_SET_RANGE,
	STM32_CMD_SET_TRIGGER_MODE,
	CMD_UNKNOWN
} spi_cmd_esp_t;

typedef struct {
    uint8_t command;
    uint8_t data;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;
    uint8_t data4;
    uint8_t data5;
    uint8_t data6;
} spi_cmd_t;

enum   {
	RESP_OK = 1,
	RESP_NOK
};

typedef enum {
	CMD_RESP_NOP = 0,
	CMD_RESP_OK,
	CMD_RESP_NOK
} spi_cmd_resp_t;



#endif /* INC_ESP32_INTERFACE_H_ */
