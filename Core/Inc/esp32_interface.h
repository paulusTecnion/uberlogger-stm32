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
    ADC_SAMPLE_RATE_1Hz = 1,
	ADC_SAMPLE_RATE_2Hz,
	ADC_SAMPLE_RATE_10Hz,
	ADC_SAMPLE_RATE_25Hz,
	ADC_SAMPLE_RATE_50Hz,
	ADC_SAMPLE_RATE_100Hz,
	ADC_SAMPLE_RATE_200Hz,
	ADC_SAMPLE_RATE_400Hz,
	ADC_SAMPLE_RATE_500Hz,
	ADC_SAMPLE_RATE_1000Hz,
	ADC_SAMPLE_RATE_2000Hz,
	ADC_SAMPLE_RATE_4000Hz,
	ADC_SAMPLE_RATE_5000Hz,
	ADC_SAMPLE_RATE_8000Hz,
	ADC_SAMPLE_RATE_10000Hz,
	ADC_SAMPLE_RATE_20000Hz,
	ADC_SAMPLE_RATE_40000Hz,
	ADC_SAMPLE_RATE_50000Hz,
	ADC_SAMPLE_RATE_100000Hz,
	ADC_SAMPLE_RATE_250000Hz,
	ADC_SAMPLE_RATE_500000Hz,
	ADC_SAMPLE_RATE_1000000Hz,
    ADC_SAMPLE_RATE_NUM_ITEMS
} adc_sample_rate_t;


typedef enum log_mode_e {
    LOGMODE_RAW = 0,
    LOGMODE_CSV
} log_mode_t;



struct  {
    adc_resolution_t adc_resolution;
    adc_sample_rate_t log_sample_rate; // can make this one out of fixed options
    uint8_t adc_channel_type; // indicate whether channel 0..7 are normal ADC (bit = 0) or NTC (bit = 1). LSB = channel 0, MSB = channel 7
    uint8_t adc_channels_enabled; // Indicate whether an ADC channel should be enabled or not. Each bit represents a channel. LSB = 0 channel 0 (Mask 0x01), MSB = channel 7 (Mask 0x80)
	uint8_t adc_channel_range; // Indicate what the range of channel 0..7 is -10V / +10 (bit = 0) or -60V / +60V (bit = 1)
	uint8_t logMode;
	char wifi_ssid[MAX_WIFI_SSID_LEN];
	char wifi_password[MAX_WIFI_PASSW_LEN];
	uint8_t wifi_channel;
} Settings_t;

#endif /* INC_ESP32_INTERFACE_H_ */
